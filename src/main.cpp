/* ========================================================================

   (C) Copyright 2025 by Sung Woo Lee, All Rights Reserved.

   This software is provided 'as-is', without any express or implied
   warranty. In no event will the authors be held liable for any damages
   arising from the use of this software.

   ======================================================================== */

#include <stdio.h>
#include <time.h>
#include <math.h>

#include "vendor/glad/glad.h"
#include "vendor/GLFW/glfw3.h"

#include "vendor/imgui/imgui.h"
#include "vendor/imgui/imgui_impl_glfw.h"
#include "vendor/imgui/imgui_impl_opengl3.h"

#include "types.h"
#include "core.h"
#include "scope_exit.h"
#include "intrinsics.h"
#include "math.h"
#include "geo.h"
#include "gl.h"

#define WINDOW_WIDTH    1000
#define WINDOW_HEIGHT   1000

void swap_vertex(Vertex *a, Vertex *b) {
    Vertex tmp = *a;
    *a = *b;
    *b = tmp;
}

void swap_u32(u32 *a, u32 *b) {
    u32 tmp = *a;
    *a = *b;
    *b = tmp;
}

u32 partition_colinear(u32 *colinear, u32 lo, u32 hi, Vertex *vertices, Vertex anchor) {
    u32 j, k = lo;

    for (j = lo + 1; j < hi; j++) {
        if (distance(anchor.position, vertices[colinear[j]].position) - distance(anchor.position, vertices[colinear[lo]].position) <= 0) {
            k++;
            swap_u32(colinear + k, colinear + j);
        }
    }

    swap_u32(colinear + lo, colinear + k);
    return k;
}

void quicksort_colinear(u32 *colinear, u32 lo, u32 hi, Vertex *vertices, Vertex anchor) {
    if (hi > lo + 1) {
        u32 mid = partition_colinear(colinear, lo, hi, vertices, anchor);
        quicksort_colinear(colinear, lo, mid, vertices, anchor);
        quicksort_colinear(colinear, mid + 1, hi, vertices, anchor);
    }
}

Vertex_Array jarvismarch(Vertex *vertices_, u32 vertexcount_) {
    u32 vertexcount = vertexcount_;
    Vertex *vertices = (Vertex *)malloc(vertexcount*sizeof(Vertex));
    SCOPE_EXIT(free(vertices));
    copyarray(vertices_, vertices, vertexcount);

    u32 *colinear = (u32 *)malloc(vertexcount*sizeof(u32));
    SCOPE_EXIT(free(colinear));
    u32 colinearcount = 0;

    Vertex_Array hull = {};
    hull.vertices = (Vertex *)malloc(vertexcount*sizeof(Vertex));
    hull.count = 0;

    if (vertexcount == 3) return hull;
    if (vertexcount < 3) return hull;

    u32 start = 0;
    Vertex sv = vertices[start];

    for (u32 i = 1; i < vertexcount; ++i) {
        Vertex v = vertices[i];
        if ((v.position.x < sv.position.x) ||
            ((v.position.x == sv.position.x) && (v.position.z < sv.position.z))) {
            start = i;
            sv = v;
        }
    }
    hull.vertices[hull.count++] = sv;
    swap_vertex(&vertices[start], &vertices[--vertexcount]);

    Vertex anchor = hull.vertices[0];

    u32 counter = 0;

    while (1) {
        if (counter == 2) {
            vertices[vertexcount++] = hull.vertices[0];
        }

        u32 end1 = 0;

        for (u32 end2 = 1; end2 < vertexcount; ++end2) {
            f32 relation = cross(normalize(vertices[end1].position - anchor.position), normalize(vertices[end2].position - anchor.position)).y;
            f32 eps = 0.00001f;

            if (relation > -eps && relation < eps) {
                colinear[colinearcount++] = end2;
            } else if (relation < 0.0f) {
                end1 = end2;
                colinearcount = 0;
            }
        }

        if (colinearcount) {
            colinear[colinearcount++] = end1;

            quicksort_colinear(colinear, 0, colinearcount, vertices, anchor);

            for (u32 i = 0; i < colinearcount; ++i) {
                hull.vertices[hull.count++] = vertices[colinear[i]];
            }

            anchor = vertices[colinear[colinearcount - 1]];

            for (u32 i = 0; i < colinearcount; ++i) {
                swap_vertex(&vertices[colinear[i]], &vertices[--vertexcount]);
            }

            colinearcount = 0;
        } else {
            hull.vertices[hull.count++] = vertices[end1];
            anchor = vertices[end1];
            swap_vertex(&vertices[end1], &vertices[--vertexcount]);
        }

        if (anchor.position == hull.vertices[0].position) {
            --hull.count;
            break;
        }

        ++counter;
    }

    return hull;
}

Vertex_Array triangulate_convex_hull(Vertex_Array *convexhull) {
    u32 trianglecount = convexhull->count - 2;
    Vertex_Array result;
    result.count = 0;
    result.vertices = (Vertex *)malloc(3*sizeof(Vertex)*trianglecount);

	for (u32 i = 2; i < convexhull->count; i++) {
		Vertex a = convexhull->vertices[0];
		Vertex b = convexhull->vertices[i - 1];
		Vertex c = convexhull->vertices[i];

		result.vertices[result.count++] = a;
		result.vertices[result.count++] = b;
		result.vertices[result.count++] = c;
	}

    ASSERT(trianglecount*3 == result.count);
    return result;
}

int get_bin(Vertex vertex, int ndiv) {
    f32 x = vertex.position.x;
    f32 z = vertex.position.z;
    int i = int(z*ndiv*0.999f);
    int j = int(x*ndiv*0.999f);
    int bin = (i % 2 == 0) ? i*ndiv+j+1 : (i+1)*ndiv-j;
    return bin;
}

u32 partition_bin(Vertex *vertices, u32 lo, u32 hi, int ndiv) {
    u32 j, k = lo;

    for (j = lo + 1; j < hi; j++) {
        if (get_bin(vertices[j], ndiv) - get_bin(vertices[lo], ndiv) <= 0) {
            k++;
            swap_vertex(vertices + k, vertices + j);
        }
    }

    swap_vertex(vertices + lo, vertices + k);
    return k;
}

void quicksort_bin(Vertex *vertices, u32 lo, u32 hi, int ndiv) {
    if (hi > lo + 1) {
        u32 mid = partition_bin(vertices, lo, hi, ndiv);
        quicksort_bin(vertices, lo, mid, ndiv);
        quicksort_bin(vertices, mid + 1, hi, ndiv);
    }
}

// @TODO: Correctness.
bool point_in_triangle(v2 p, v2 a, v2 b, v2 c) {
    v2 ab = b - a;
    v2 bc = c - b;
    v2 ca = a - c;
    v2 ap = p - a;
    v2 bp = p - b;
    v2 cp = p - c;

    v2 n1 = v2{ab.y, -ab.x};
    v2 n2 = v2{bc.y, -bc.x};
    v2 n3 = v2{ca.y, -ca.x};

    f32 s1 = ap.x*n1.x + ap.y*n1.y;
    f32 s2 = bp.x*n2.x + bp.y*n2.y;
    f32 s3 = cp.x*n3.x + cp.y*n3.y;

    f32 tolerance = 0.0001f;

    if ((s1 < 0 && s2 < 0 && s3 < 0) ||
        (s1 < tolerance && s2 < 0 && s3 < 0) ||
        (s2 < tolerance && s3 < 0 && s1 < 0) || 
        (s3 < tolerance && s1 < 0 && s2 < 0)) {
        return true;
    } else {
        return false;
    }
}

bool point_in_triangle(v3 p, v3 a, v3 b, v3 c) {
    v2 p_ = v2{p.x, p.z};
    v2 a_ = v2{a.x, a.z};
    v2 b_ = v2{b.x, b.z};
    v2 c_ = v2{c.x, c.z};
    return point_in_triangle(p_, a_, b_, c_);
}

Triangle Triangle_(v3 a, v3 b, v3 c) {
    Triangle result = {};
    result.e[0] = a;
    result.e[1] = b;
    result.e[2] = c;
    return result;
}

void push(Triangle_Stack *ts, Triangle t) {
    ASSERT(ts->count < ts->size);
    ts->data[ts->count++] = t;
}

void pop(Triangle_Stack *ts) {
    ASSERT(ts->count > 0);
    --ts->count;
}

int delaunay_edge(int (*adj)[3], int L, int K) {
    for (int e = 0; e < 3; ++e) {
        if (adj[L][e] == K) {
            return e;
            break;
        }
    }
    ASSERT(0);
    return -1;
}

void delaunay_triangulate(Vertex *vertices_, u32 count_) {
    // Alloc/Copy vertices.
    int count = (int)count_;
    Vertex *vertices = (Vertex *)malloc(sizeof(Vertex)*(count + 3)); // add 3 for super-triangle.
    copyarray(vertices_, vertices, count);

#if 1 // Assert remapping correctness.
    printf("\nVertices before mapping\n");
    for (int vi = 0; vi < count; ++vi) {
        printf("%f, %f, %f\n", vertices[vi].position.x, vertices[vi].position.y, vertices[vi].position.z);
    }
#endif

    // Normalize while keeping aspect ratio.
    f32 xmin =  F32_MAX;
    f32 xmax = -F32_MAX;
    f32 zmin =  F32_MAX;
    f32 zmax = -F32_MAX;

    for (int i = 0; i < count; ++i) {
        Vertex v = vertices[i];
        xmin = min(xmin, v.position.x);
        xmax = max(xmax, v.position.x);
        zmin = min(zmin, v.position.z);
        zmax = max(zmax, v.position.z);
    }

    f32 dmax = max(xmax - xmin, zmax - zmin);

    for (int i = 0; i < count; ++i) {
        Vertex *v = vertices + i;
        v->position.x = (v->position.x - xmin) / dmax;
        v->position.z = (v->position.z - zmin) / dmax;
    }

#if 1 // Assert remapping correctness.
    printf("\nVertices after mapping\n");
    for (int vi = 0; vi < count; ++vi) {
        printf("%f, %f, %f\n", vertices[vi].position.x, vertices[vi].position.y, vertices[vi].position.z);
    }
#endif

    // Sort by proximity.
    int ndiv = (int)(pow((f32)count, 0.25f) + 0.5f);
    quicksort_bin(vertices, 0, count, ndiv);

    // To ASSERT if vertices are sorted correctly.
#if 0
    for (u32 i = 1; i < count; ++i) {
        ASSERT(get_bin(vertices[i], ndiv) >= get_bin(vertices[i-1], ndiv));
    }
#endif

    // Add super-triangle to vertex array.
    vertices[count].position.x   = -100;
    vertices[count].position.y   = 0;
    vertices[count].position.z   = -100;
    vertices[count+1].position.x =  100;
    vertices[count+1].position.y = 0;
    vertices[count+1].position.z = -100;
    vertices[count+2].position.x = 0;
    vertices[count+2].position.y = 0;
    vertices[count+2].position.z = 100;
    count += 3;

    int maxtri = 2*(count-3);

    int (*tri)[3] = (int (*)[3])malloc(3*sizeof(int)*maxtri);
    tri[0][0] = count-3;
    tri[0][1] = count-2;
    tri[0][2] = count-1;

    int (*adj)[3] = (int (*)[3])malloc(3*sizeof(int)*maxtri);
    adj[0][0] = -1;
    adj[0][1] = -1;
    adj[0][2] = -1;

    int maxstk = (count-3);
    int *ts = (int *)malloc(maxstk*sizeof(int)); // @ROBUSTNESS: Sloan suggests #point is good enough for 10,000. Assertion required.
    SCOPE_EXIT(free(ts));
    int top = -1;

    int numtri = 1;

    // Iterate through points.
    for (int vi = 0; vi < count - 3; ++vi) {
        v3 p = vertices[vi].position;
        //int ti = numtri - 1;
        for (int ti = numtri - 1; ti >= 0; --ti) {
            if (point_in_triangle(p, vertices[tri[ti][0]].position, vertices[tri[ti][1]].position, vertices[tri[ti][2]].position)) {
                tri[ti+1][0] = vi;
                tri[ti+1][1] = tri[ti][2];
                tri[ti+1][2] = tri[ti][0];

                tri[ti+2][0] = vi;
                tri[ti+2][1] = tri[ti][0];
                tri[ti+2][2] = tri[ti][1];

                // Retrive adj info.
                int adj0 = adj[ti][0]; // A
                int adj1 = adj[ti][1]; // B
                int adj2 = adj[ti][2]; // C

                // Update adj info of surrounding triangles.
                if (adj2 >= 0) {
                    adj[adj2][delaunay_edge(adj, adj2, ti)] = ti + 1;
                }

                if (adj0 >= 0) {
                    adj[adj0][delaunay_edge(adj, adj0, ti)] = ti + 2;
                }

                // Update adj of new triangles.
                adj[ti][0] = ti+2;
                adj[ti][1] = adj1;
                adj[ti][2] = ti+1;

                adj[ti+1][0] = ti;
                adj[ti+1][1] = adj2;
                adj[ti+1][2] = ti+2;

                adj[ti+2][0] = ti+1;
                adj[ti+2][1] = adj0;
                adj[ti+2][2] = ti;

                // Move first point of T
                tri[ti][0] = vi;
                //tri[ti][1] = tri[ti][1];
                //tri[ti][2] = tri[ti][2];

                // Push newly added triangles to stack if has opposing triangle.
                for (int i = 0; i < 3; ++i) {
                    if (adj[ti + i][1] >= 0) {
                        ASSERT(top < maxstk-1);
                        ts[++top] = ti + i;
                    }
                }

                // @TODO: Something's off. Have to look into Lawson's paper.
                // @NOTE: Sloan follows Lawson's swapping algorithm from below.
                while (top >= 0) {
                    int L = ts[top];
                    int R = adj[L][1];
                    ASSERT(R >= 0);

                    int ERL = delaunay_edge(adj, R, L);
                    int ERA = (ERL % 3) + 1;
                    int ERB = (ERA % 3) + 1;

                    int P = tri[L][0];
                    int V1 = tri[R][ERL];
                    int V2 = tri[R][ERA];
                    int V3 = tri[R][ERB];

                    // Determine if a pair of adjacent triangles form a convex quadrilateral with the
                    // maximum minimum angle. Due to 'Cline' and 'Renka'.
                    // @TODO: typo...?
                    f32 xp = vertices[P].position.x;
                    f32 x1 = vertices[V1].position.x;
                    f32 x2 = vertices[V2].position.x;
                    f32 x3 = vertices[V3].position.x;

                    f32 zp = vertices[P].position.z;
                    f32 z1 = vertices[V1].position.z;
                    f32 z2 = vertices[V2].position.z;
                    f32 z3 = vertices[V3].position.z;

                    f32 x13 = x1 - x3;
                    f32 x23 = x2 - x3;
                    f32 x1p = x1 - xp;
                    f32 x2p = x2 - xp;

                    f32 z13 = z1 - z3;
                    f32 z23 = z2 - z3;
                    f32 z1p = z1 - zp;
                    f32 z2p = z2 - zp;

                    f32 cosa = x13*x23 + z13*z23;
                    f32 cosb = x2p*x1p + z2p*z1p;

                    bool swap;

                    if (cosa >= 0 && cosb >= 0) {
                        swap = false;
                    } else if (cosa < 0 && cosb < 0) {
                        swap = true;
                    } else {
                        f32 sina = x13*z23 - x23*z13;
                        f32 sinb = x2p*z1p - x1p*z2p;

                        f32 sinab = sina*cosb + sinb*cosa;

                        if (sinab < 0) {
                            swap = true;
                        } else {
                            swap = false;
                        }
                    }

                    if (swap) {
                        int A = adj[R][ERA];
                        int B = adj[R][ERB];
                        int C = adj[L][2];

                        // Update vertex and adjacency list for L.
                        tri[L][2] = V3;
                        adj[L][1] = A;
                        adj[L][2] = R;

                        // Update vertex and adjacency list for R.
                        tri[R][0] = P;
                        tri[R][1] = V3;
                        tri[R][2] = V1;
                        adj[R][0] = L;
                        adj[R][1] = B;
                        adj[R][2] = C;

                        // Push L-A and R-B on stack.
                        // Update adjacency lists for triangle A and C.
                        if (A >= 0) {
                            adj[A][delaunay_edge(adj, A, R)] = L;
                            ASSERT(top < maxstk-1);
                            ts[++top] = L;
                        }
                        if (B >= 0) {
                            ASSERT(top < maxstk-1);
                            ts[++top] = R;
                        }
                        if (C >= 0) {
                            adj[C][delaunay_edge(adj, C, L)] = R;
                        }
                    }

                    ASSERT(top >= 0);
                    --top;
                }

                break;
            }
        }

        // @TODO: adjust triangle index according to direction and sorted bin.
    }

    // Check consistency of triangulation.
    ASSERT(numtri == 2*(count-3)+1);

    // Remove triangles including super-triangle's vertex.
    int p[3];
    p[0] = count - 3;
    p[1] = count - 2;
    p[2] = count - 1;

    bool *flag = (bool *)malloc(sizeof(bool)*numtri);
    SCOPE_EXIT(free(flag));
    zeroarray(flag, numtri);

    for (int ti = 0; ti < numtri; ++ti) {
        bool match = false;
        for (int i = 0; i < 3 && !match; ++i) {
            for (int j = 0; j < 3 && !match; ++j) {
                if (p[i] == tri[ti][j]) {
                    flag[ti] = true;
                    match = true;
                }
            }
        }
    }

    // @NOTE: Just realloc?
    // Remove by swapping and decrementing counter.
    for (int ti = 0; ti < numtri; ++ti) {
        if (flag[ti]) {
            int tmp[3];
            copyarray(tri[ti], tmp, 3);
            copyarray(tri[numtri-1], tri[ti], 3);
            copyarray(tmp, tri[numtri-1], 3);
            --numtri;
        }
    }

    // Remap to original value.
    count-=3;
    for (int vi = 0; vi < count; ++vi) {
        vertices[vi].position.x = (vertices[vi].position.x * dmax) + xmin;
        vertices[vi].position.z = (vertices[vi].position.z * dmax) + zmin;
    }

#if 1 // Assert remapping correctness. printf("\n");
    printf("\nVertices restored.\n");
    for (int vi = 0; vi < count - 3; ++vi) {
        printf("%f, %f, %f\n", vertices[vi].position.x, vertices[vi].position.y, vertices[vi].position.z);
    }
#endif

#if 1
    printf("\n#Triangle: %d\n", numtri);
#endif

    // @TODO: Return vertices, triangle-indices, adj-info.
}

/*
 * Scene
 */
void generate_vertices(Vertex *vertices, u32 count) {
#if 0
    time_t t;
    time(&t);
    srand((u32)t);
    const f32 invrandmax = 1.0f/(f32)RAND_MAX;
    const f32 range = 0.9f;
    for (u32 i = 0; i < count; ++i) {
        for (u32 j = 0; j < 3; ++j) {
            f32 val = 2.0f * ((f32)rand() * invrandmax) - 1.0f;
            val*=range;
            vertices[i].position.e[j] = val;
        }
        vertices[i].position.y = 0.0f;
    }
#else
    vertices[0].position = v3{1,0,1};
    vertices[1].position = v3{3,0,4};
    vertices[2].position = v3{-2,0,3};
    vertices[3].position = v3{-2,0,2};
    vertices[4].position = v3{-1,0,-1};
    vertices[5].position = v3{-2,0,-3};
    vertices[6].position = v3{4,0,-2};
#endif
}

void restart(Vertex *vertices, u32 count, Vertex_Array *convexhull, Vertex_Array *triangulatedconvexhull) {
    generate_vertices(vertices, count);
    *convexhull = jarvismarch(vertices, count);

    *triangulatedconvexhull = triangulate_convex_hull(convexhull);
}

int main(void) {
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow *window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "UNITITLED", NULL, NULL);
    if (!window) {
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        return -1;
    }

    glViewport(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);
    glfwSetFramebufferSizeCallback(window, gl_framebuffer_resize_callback);

    GLuint vao, vio, vbo;

    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);

    glGenBuffers(1, &vio);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vio);


    const char *simple_vs = 
#include "shader/simple_vs.glsl"

    const char *simple_fs = 
#include "shader/simple_fs.glsl"

    const char *circle_vs = 
#include "shader/circle_vs.glsl"
    const char *circle_gs = 
#include "shader/circle_gs.glsl"
    const char *circle_fs = 
#include "shader/circle_fs.glsl"

    GLuint simpleshader = glcreateshader(simple_vs, simple_fs);
    GLuint circleshader = glcreateshader(circle_vs, circle_gs, circle_fs);

    u32 vertexcount = 7;
    Vertex *vertices = (Vertex *)malloc(sizeof(Vertex)*vertexcount);
    Vertex_Array hull;
    Vertex_Array triangulated;
    restart(vertices, vertexcount, &hull, &triangulated);

    delaunay_triangulate(vertices, 7);

    {
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LEQUAL);
    }

    bool drawconvexhull = false;
    bool drawtriangulatedconvexhull = false;

    while (!glfwWindowShouldClose(window)) 
    {
        glfwPollEvents();    

        {
            ImGui_ImplOpenGL3_NewFrame();
            ImGui_ImplGlfw_NewFrame();
            ImGui::NewFrame();
        }

        {
            ImGui::Begin("Control");
            ImGui::Checkbox("Draw Convex Hull", &drawconvexhull);
            ImGui::Checkbox("Draw Triangulated Convex Hull", &drawtriangulatedconvexhull);
            if (ImGui::Button("Restart")) {
                free(vertices);
                vertices = (Vertex *)malloc(sizeof(Vertex) * vertexcount);
                restart(vertices, vertexcount, &hull, &triangulated);
            }
            ImGui::End();
        }

        {
            glclearcolor(v4{0.1f,0.1f,0.1f,1.0f});
            glClearDepth(1.0f);
            glClear(GL_DEPTH_BUFFER_BIT);
        }

        {
            glUseProgram(circleshader);
            glEnableVertexAttribArray(0);
            glVertexAttribPointer(0, 3, GL_FLOAT, false, sizeof(Vertex), (GLvoid *)(offsetof(Vertex, position)));
            glBufferData(GL_ARRAY_BUFFER, vertexcount * sizeof(Vertex), vertices, GL_DYNAMIC_DRAW);
            glDrawArrays(GL_POINTS, 0, vertexcount);
            glDisableVertexAttribArray(0);
        }

        if (drawconvexhull) {
            glUseProgram(simpleshader);
            glEnableVertexAttribArray(0);
            glVertexAttribPointer(0, 3, GL_FLOAT, false, sizeof(Vertex), (GLvoid *)(offsetof(Vertex, position)));
            glBufferData(GL_ARRAY_BUFFER, hull.count * sizeof(Vertex), hull.vertices, GL_DYNAMIC_DRAW);
            glDrawArrays(GL_LINE_LOOP, 0, hull.count);
            glDisableVertexAttribArray(0);
        }

        if (drawtriangulatedconvexhull) {
            glUseProgram(simpleshader);
            glEnableVertexAttribArray(0);
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            glVertexAttribPointer(0, 3, GL_FLOAT, false, sizeof(Vertex), (GLvoid *)(offsetof(Vertex, position)));
            glBufferData(GL_ARRAY_BUFFER, triangulated.count * sizeof(Vertex), triangulated.vertices, GL_DYNAMIC_DRAW);
            glDrawArrays(GL_TRIANGLES, 0, triangulated.count);
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            glDisableVertexAttribArray(0);
        }

        {
            ImGui::Render();
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        }

        glfwSwapBuffers(window);
    }

    return 0;
}
