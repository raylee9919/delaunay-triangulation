/* ========================================================================

   (C) Copyright 2025 by Sung Woo Lee, All Rights Reserved.

   This software is provided 'as-is', without any express or implied
   warranty. In no event will the authors be held liable for any damages
   arising from the use of this software.

   ======================================================================== */

#include <stdlib.h>
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

unsigned int partition_colinear(u32 *colinear, u32 lo, u32 hi, Vertex *vertices, Vertex anchor) {
    u32 j, k = lo;

    for (j = lo + 1; j < hi; j++) {
        if (distance(anchor.position, vertices[colinear[j]].position) - distance(anchor.position, vertices[colinear[lo]].position) <= 0) {
            k++;
            unsigned int tmp = colinear[k];
            colinear[k] = colinear[j];
            colinear[j] = tmp;
        }
    }

    unsigned int tmp = colinear[k];
    colinear[k] = colinear[lo];
    colinear[lo] = tmp;

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
    u32 numtri = convexhull->count - 2;
    Vertex_Array result;
    result.count = 0;
    result.vertices = (Vertex *)malloc(3*sizeof(Vertex)*numtri);

	for (u32 i = 2; i < convexhull->count; i++) {
		Vertex a = convexhull->vertices[0];
		Vertex b = convexhull->vertices[i - 1];
		Vertex c = convexhull->vertices[i];

		result.vertices[result.count++] = a;
		result.vertices[result.count++] = b;
		result.vertices[result.count++] = c;
	}

    ASSERT(numtri*3 == result.count);
    return result;
}

u32 partition_bin(int *VIDX, u32 lo, u32 hi, int *BIN) {
    u32 j, k = lo;

    for (j = lo + 1; j < hi; j++) {
        if (BIN[VIDX[j]] - BIN[VIDX[lo]] <= 0) {
            k++;
            int tmp = VIDX[k];
            VIDX[k] = VIDX[j];
            VIDX[j] = tmp;
        }
    }

    int tmp = VIDX[k];
    VIDX[k] = VIDX[lo];
    VIDX[lo] = tmp;
    return k;
}

void quicksort_bin(int *VIDX, u32 lo, u32 hi, int *BIN) {
    if (hi > lo + 1) {
        u32 mid = partition_bin(VIDX, lo, hi, BIN);
        quicksort_bin(VIDX, lo, mid, BIN);
        quicksort_bin(VIDX, mid + 1, hi, BIN);
    }
}

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

inline float cross(v2 a, v2 b) {
    return (a.x*b.y-a.y*b.x);
}

// @NOTE: Colinear isn't considered on right.
bool point_on_right_line(v2 p, v2 a, v2 b) {
    v2 v1 = b-a;
    v2 v2 = p-a;
    float det = v1.x*v2.y - v1.y*v2.x;
    if (det < 0) return true;
    return false;
}

// @NOTE: Colinear cases aren't considered intersecting.
bool line_segments_intersect(v2 a1, v2 a2, v2 b1, v2 b2) {
    v2 p = a2-a1;
    v2 q1 = b1-a2;
    v2 q2 = b2-a2;
    float c1 = cross(p, q1);
    float c2 = cross(p, q2);
    if (c1*c2 >= 0) return false;

    v2 q = b2-b1;
    v2 p1 = a1-b2;
    v2 p2 = a2-b2;
    c1 = cross(q, p1);
    c2 = cross(q, p2);
    if (c1*c2 >= 0) return false;

    return true;
}

bool is_convex(v2 a, v2 b, v2 c, v2 d) {
    v2 s[4];
    s[0] = b-a;
    s[1] = c-b;
    s[2] = d-c;
    s[3] = a-d;

    float cp[4];
    for (int i = 0; i < 4; ++i) {
        int j = (i+1)%4;
        cp[i] = (s[i].x*s[j].y - s[i].y*s[j].x);
    }

    if (cp[0] < 0 && cp[1] < 0 && cp[2] < 0 && cp[3] < 0) return true;
    if (cp[0] > 0 && cp[1] > 0 && cp[2] > 0 && cp[3] > 0) return true;
    return false;
}

bool delaunay_bad(float xp, float x1, float x2, float x3, float yp, float y1, float y2, float y3) {
    // @NOTE: Determine if a pair of adjacent triangles form a convex quadrilateral with the maximum minimum angle.

    float x13 = x1 - x3;
    float x23 = x2 - x3;
    float x1p = x1 - xp;
    float x2p = x2 - xp;

    float y13 = y1 - y3;
    float y23 = y2 - y3;
    float y1p = y1 - yp;
    float y2p = y2 - yp;

    float cosa = x13*x23 + y13*y23;
    float cosb = x2p*x1p + y2p*y1p;

    bool result;

    if (cosa >= 0 && cosb >= 0) {
        result = false;
    } else if (cosa < 0 && cosb < 0) {
        result = true;
    } else {
        float sina = x13*y23 - x23*y13;
        float sinb = x2p*y1p - x1p*y2p;

        float sinab = sina*cosb + sinb*cosa;

        if (sinab < 0) {
            result = true;
        } else {
            result = false;
        }
    }

    return result;
}


typedef struct Delaunay_Triangulate_Result {
    int numtri;
    int (*tri)[3];
    int (*adj)[3];
} Delaunay_Triangulate_Result;

Delaunay_Triangulate_Result
delaunay_triangulate(Vertex *vertices, unsigned int vertexcount, int (*edges)[2], int edgecount) {
    int count = (int)vertexcount;

    int *VIDX = (int *)malloc(sizeof(int)*count);
    SCOPE_EXIT(free(VIDX));
    for (int i = 0; i < count; ++i) VIDX[i] = i;

    int *BIN = (int *)malloc(sizeof(int)*count);
    SCOPE_EXIT(free(BIN));

    v2 *positions = (v2 *)malloc(sizeof(v2)*(count+3));
    SCOPE_EXIT(free(positions));
    for (int i = 0; i < count; ++i) {
        positions[i].x = vertices[i].position.x;
        positions[i].y = vertices[i].position.z;
    }

    // @NOTE: Normalize while keeping aspect ratio.
    float xmin =  F32_MAX;
    float xmax = -F32_MAX;
    float ymin =  F32_MAX;
    float ymax = -F32_MAX;

    for (int i = 0; i < count; ++i) {
        float x = positions[i].x;
        float z = positions[i].y;
        xmin = min(xmin, x);
        xmax = max(xmax, x);
        ymin = min(ymin, z);
        ymax = max(ymax, z);
    }

    float invdmax = 1.0f / max(xmax - xmin, ymax - ymin);

    for (int i = 0; i < count; ++i) {
        positions[i].x = (positions[i].x - xmin) * invdmax;
        positions[i].y = (positions[i].y - ymin) * invdmax;
    }

    float invnxmax = 1.0f / ((xmax - xmin) * invdmax);
    float invnymax = 1.0f / ((ymax - ymin) * invdmax);

    // @NOTE: Sort by proximity.
    int ndiv = (int)(pow((float)count, 0.25f) + 0.5f);
    for (int k = 0; k < count; ++k) {
        float x = positions[k].x;
        float y = positions[k].y;
        int i = int(y*ndiv*0.99f*invnxmax);
        int j = int(x*ndiv*0.99f*invnymax);
        int bin = (i % 2 == 0) ? i*ndiv+j+1 : (i+1)*ndiv-j;
        BIN[k] = bin;
    }
    quicksort_bin(VIDX, 0, count, BIN);

    //for (int i = 0; i < count; ++i) {
    //    printf("%d (%.2f,%.2f)\n", VIDX[i], vertices[VIDX[i]].position.x, vertices[VIDX[i]].position.z);
    //}

    // Add super-triangle to vertex array.
    positions[count].x   = -100;
    positions[count].y   = -100;

    positions[count+1].x = 100;
    positions[count+1].y = -100;

    positions[count+2].x = 0;
    positions[count+2].y = 100;

    count += 3;

    int maxnumtri = 2*(count+3);

    int (*tri)[3] = (int (*)[3])malloc(3*sizeof(int)*maxnumtri);
    SCOPE_EXIT(free(tri));
    tri[0][0] = count-3;
    tri[0][1] = count-2;
    tri[0][2] = count-1;

    int (*adj)[3] = (int (*)[3])malloc(3*sizeof(int)*maxnumtri);
    SCOPE_EXIT(free(adj));
    adj[0][0] = -1;
    adj[0][1] = -1;
    adj[0][2] = -1;

    int maxstk = (count-3);
    int *ts = (int *)malloc(maxstk*sizeof(int)); // @NOTE: Sloan suggests #point is good enough for 10,000. Assertion required.
    SCOPE_EXIT(free(ts));
    int top = -1;

    int numtri = 1;

    for (int vii = 0; vii < count - 3; ++vii) {
        int p = VIDX[vii];
        //int ti = numtri - 1;
        for (int ti = numtri - 1; ti >= 0; --ti) {
            if (point_in_triangle(positions[p], positions[tri[ti][0]], positions[tri[ti][1]], positions[tri[ti][2]])) {
                tri[numtri][0] = p;
                tri[numtri][1] = tri[ti][1];
                tri[numtri][2] = tri[ti][2];

                tri[numtri+1][0] = p;
                tri[numtri+1][1] = tri[ti][2];
                tri[numtri+1][2] = tri[ti][0];

                // Retrive adj info.
                int A = adj[ti][0];
                int B = adj[ti][1];
                int C = adj[ti][2];

                // Update adj info of surrounding triangles.
                if (B >= 0) {
                    int EBT = delaunay_edge(adj, B, ti);
                    adj[B][EBT] = numtri;
                }

                if (C >= 0) {
                    int ECT = delaunay_edge(adj, C, ti);
                    adj[C][ECT] = numtri+1;
                }

                // Update adj of new triangles.
                adj[ti][0] = numtri+1;
                adj[ti][1] = A;
                adj[ti][2] = numtri;

                adj[numtri][0] = ti;
                adj[numtri][1] = B;
                adj[numtri][2] = numtri+1;

                adj[numtri+1][0] = numtri;
                adj[numtri+1][1] = C;
                adj[numtri+1][2] = ti;

                tri[ti][2] = tri[ti][1];
                tri[ti][1] = tri[ti][0];
                tri[ti][0] = p;

                // Push newly added triangles to stack if has opposing triangle.
                if (adj[ti][1] >= 0) {
                    ASSERT(top < maxstk-1);
                    ts[++top] = ti;
                }

                if (adj[numtri][1] >= 0) {
                    ASSERT(top < maxstk-1);
                    ts[++top] = numtri;
                }

                if (adj[numtri+1][1] >= 0) {
                    ASSERT(top < maxstk-1);
                    ts[++top] = numtri+1;
                }

                while (top >= 0) {
                    int L = ts[top--];
                    int R = adj[L][1];
                    ASSERT(R >= 0);

                    int ERL = delaunay_edge(adj, R, L);
                    int ERA = (ERL + 1) % 3;
                    int ERB = (ERL + 2) % 3;

                    int P  = tri[L][0];
                    int V1 = tri[R][ERL];
                    int V2 = tri[R][ERA];
                    int V3 = tri[R][ERB];

                    float xp = positions[P].x;
                    float x1 = positions[V1].x;
                    float x2 = positions[V2].x;
                    float x3 = positions[V3].x;

                    float yp = positions[P].y;
                    float y1 = positions[V1].y;
                    float y2 = positions[V2].y;
                    float y3 = positions[V3].y;

                    if (delaunay_bad(xp,x1,x2,x3,yp,y1,y2,y3)) {
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
                            int EAR = delaunay_edge(adj, A, R);
                            adj[A][EAR] = L;
                            ASSERT(top < maxstk-1);
                            ts[++top] = L;
                        }
                        if (B >= 0) {
                            ASSERT(top < maxstk-1);
                            ts[++top] = R;
                        }
                        if (C >= 0) {
                            int ECL = delaunay_edge(adj, C, L);
                            adj[C][ECL] = R;
                        }
                    }
                }

                numtri += 2;

                break;
            }
        }

        // @TODO: adjust triangle index according to direction and sorted bin.
    }

    // @NOTE: Check consistency of triangulation.
    ASSERT(numtri == 2*(count-3)+1);


    // @NOTE: Constrained Delaunay Triangulation.
    if (edges && edgecount) {
        int *tl = (int *)malloc(sizeof(int)*count*numtri);
        SCOPE_EXIT(free(tl));

        int *tlcount = (int *)malloc(sizeof(int)*count);
        SCOPE_EXIT(free(tlcount));
        zeroarray(tlcount, count);

        // Itersecting edges.
        int ielen = count*3;
        int (*ie)[2] = (int (*)[2])malloc(2*sizeof(int)*ielen);
        int ielo = 0;
        int iehi = 0;
        SCOPE_EXIT(free(ie));

        // New edges.
        int nelen = count*3;
        int (*ne)[2] = (int (*)[2])malloc(2*sizeof(int)*nelen);
        int nelo = 0;
        int nehi = 0;
        SCOPE_EXIT(free(ne));

        // Build triangle-list per vertex.
        int tlpitch = numtri;
        for (int T = 0; T < numtri; ++T) {
            for (int i = 0; i < 3; ++i) {
                int V = tri[T][i];
                *(tl + (V*tlpitch) + tlcount[V]++) = T;
            }
        }

        // @NOTE: Iterate through constraint edges.
        for (int cei = 0; cei < edgecount; ++cei) {
            int vi = edges[cei][0];
            int vj = edges[cei][1];

            // @NOTE: Find starting triangle by circling around vertex i.
            int T = -1;
            for (int t = 0; t < tlcount[vi]; ++t) {
                int tmpT = tl[vi*tlpitch + t];
                for (int v = 0; v < 3; ++v) {
                    int vk = tri[tmpT][v];
                    int vl = tri[tmpT][(v+1)%3];
                    if (line_segments_intersect(positions[vi], positions[vj], positions[vk], positions[vl])) {
                        T = tmpT;
                        break;
                    }
                }

                if (T != -1)
                    break;
            }
            ASSERT(T != -1);

            while (1) {
                for (int v = 0; v < 3; ++v) {
                    int vk = tri[T][v];
                    int vl = tri[T][(v+1)%3];

                    if (vk == vj || vl == vj) {
                        T = -1;
                        break;
                    }

                    // @TODO: Possible duplicate calcs.
                    if (point_on_right_line(positions[vj], positions[vk], positions[vl]) && line_segments_intersect(positions[vi], positions[vj], positions[vk], positions[vl])) {
                        ie[iehi][0] = vk;
                        ie[iehi][1] = vl;;
                        iehi = (iehi+1)%ielen;

                        T = adj[T][v];
                        break;
                    }
                }

                if (T == -1)
                    break;
            }

            // @NOTE Iterate intersecting edges (ie).
            while (ielo != iehi) {
                // @NOTE: Remove an edge from the list.
                int vk = ie[ielo][0];
                int vl = ie[ielo][1];

                int L = ie[ielo][0];
                int v = ie[ielo][1];

                ielo = (ielo+1)%ielen;

                int R = adj[L][v];

                int ERL = delaunay_edge(adj, R, L);
                int ERA = (ERL+1)%3;
                int ERB = (ERL+2)%3;

                int V2 = tri[L][v];
                int V1 = tri[L][(v+1)%3];
                int P = tri[L][(v+2)%3];
                int V3 = tri[R][(ERL+2)%3];

                if (!is_convex(positions[P], positions[V2], positions[V3], positions[V1])) {
                    // @NOTE: If strictly concave, put it back on.
                    ie[iehi][0] = L;
                    ie[iehi][1] = v;
                    iehi = (iehi+1)%ielen;
                } else {
                    // @NOTE: If convex, swap diagonal.
                    int A = adj[R][ERA];
                    int B = adj[R][ERB];
                    int C = adj[L][(v+1)%3];
                    int D = adj[L][(v+2)%3];

                    // Update vertex and adjacency list for L.
                    tri[L][0] = P;
                    tri[L][1] = V2;
                    tri[L][2] = V3;
                    adj[L][0] = D;
                    adj[L][1] = A;
                    adj[L][2] = R;

                    // Update vertex and adjacency list for R.
                    tri[R][0] = P;
                    tri[R][1] = V3;
                    tri[R][2] = V1;
                    adj[R][0] = L;
                    adj[R][1] = B;
                    adj[R][2] = C;

                    // Update adjacency lists for triangle A and C.
                    if (A >= 0) {
                        int EAR = delaunay_edge(adj, A, R);
                        adj[A][EAR] = L;
                    }
                    if (C >= 0) {
                        int ECL = delaunay_edge(adj, C, L);
                        adj[C][ECL] = R;
                    }

                    // @TODO: HOW TO STRUCTURE OUR EDGE-LIST!!!
                    // out of edges, I need to retrieve L and R, which constantly changes in the loop.
#if 0
                    // @NOTE: Update triangle-list.
                    //        Remove L from V1, Add L to V3
                    //        Remove R from V2, Add R to P 
                    for (int i = 0; i < tlcount[V1]; ++i) {
                        if (tl[V1*tlpitch + i] == L) {
                            tl[V1*tlpitch + i] = tl[V1*tlpitch + --tlcount[V1]];
                            break;
                        }
                    }
                    tl[V3*tlpitch + tlcount[V3]++] = L;

                    for (int i = 0; i < tlcount[V2]; ++i) {
                        if (tl[V2*tlpitch + i] == R) {
                            tl[V2*tlpitch + i] = tl[V2*tlpitch + --tlcount[V2]];
                            break;
                        }
                    }
                    tl[P*tlpitch + tlcount[P]++] = R;
#endif

                    // @NOTE: If still intersects, add to intersecting list.
                    int ELR = delaunay_edge(adj, L, R);
                    if (line_segments_intersect(positions[vi], positions[vj], positions[P], positions[V3])) {
                        ie[iehi][0] = L;
                        ie[iehi][1] = ELR;
                        iehi = (iehi+1)%ielen;
                    } else {
                        // @NOTE: If not, place it on a list of newly created edges.
                        ne[nehi][0] = L;
                        ne[nehi][1] = ELR;
                        nehi = (nehi+1)%nelen;
                    }
                }
            }

            // @NOTE: Iterate over newly created edges.
            while (nelo != nehi) {
                int L = ne[nelo][0];
                int v = ne[nelo][1];
                nelo = (nelo+1)%nelen;

                int vk = tri[L][v];
                int vl = tri[L][(v+1)%3];

                int R = adj[L][v];

                int ERL = delaunay_edge(adj, R, L);
                int ERA = (ERL+1)%3;
                int ERB = (ERL+2)%3;

                int V2 = tri[L][v];
                int V1 = tri[L][(v+1)%3];
                int P = tri[L][(v+2)%3];
                int V3 = tri[R][(ERL+2)%3];

                float xp = positions[P].x;
                float x1 = positions[V1].x;
                float x2 = positions[V2].x;
                float x3 = positions[V3].x;

                float yp = positions[P].y;
                float y1 = positions[V1].y;
                float y2 = positions[V2].y;
                float y3 = positions[V3].y;

                // @NOTE: If it's constrained edge, skip.
                if ((vk==vi && vl==vj) || (vk==vj && vl==vi)) {
                    continue;
                }

                // @NOTE: If delaunay-bad, swap.
                if (delaunay_bad(xp,x1,x2,x3,yp,y1,y2,y3)) {
                    int A = adj[R][ERA];
                    int B = adj[R][ERB];
                    int C = adj[L][(v+1)%3];
                    int D = adj[L][(v+2)%3];

                    // Update vertex and adjacency list for L.
                    int ELD = delaunay_edge(adj, L, D);
                    tri[L][ELD] = P;
                    tri[L][(ELD+1)%3] = V2;
                    tri[L][(ELD+2)%3] = V3;
                    adj[L][ELD] = D;
                    adj[L][(ELD+1)%3] = A;
                    adj[L][(ELD+2)%3] = R;

                    // Update vertex and adjacency list for R.
                    int ERB = delaunay_edge(adj, R, B);
                    tri[R][ERB] = V3;
                    tri[R][(ERB+1)%3] = V1;
                    tri[R][(ERB+2)%3] = P;
                    adj[R][ERB] = B;
                    adj[R][(ERB+1)%3] = C;
                    adj[R][(ERB+2)%3] = L;

                    // Update adjacency lists for triangle A and C.
                    if (A >= 0) {
                        int EAR = delaunay_edge(adj, A, R);
                        adj[A][EAR] = L;
                    }
                    if (C >= 0) {
                        int ECL = delaunay_edge(adj, C, L);
                        adj[C][ECL] = R;
                    }

                    // @NOTE: No need to update triangle-list since it isn't used afterwards.
                }
            }
        }
    }


    // @NOTE: Remove triangles including super-triangle's vertex.
    bool *rmflag = (bool *)malloc(sizeof(bool)*numtri);
    SCOPE_EXIT(free(rmflag));
    zeroarray(rmflag, numtri);

    int *trimap = (int *)malloc(sizeof(int)*count);
    SCOPE_EXIT(free(trimap));

    int result_numtri = numtri;
    for (int ti = 0; ti < numtri; ++ti) {
        if (tri[ti][0] >= count-3 || tri[ti][1] >= count-3 || tri[ti][2] >= count-3) {
            rmflag[ti] = 1;
            --result_numtri;
        }
    }

    int (*result_tri)[3] = (int (*)[3])malloc(sizeof(int)*3*result_numtri);
    int (*result_adj)[3] = (int (*)[3])malloc(sizeof(int)*3*result_numtri);

    int idx = 0;
    for (int T = 0; T < numtri; ++T) {
        if (rmflag[T]) {
            trimap[T] = -1;
        } else {
            result_tri[idx][0] = tri[T][0];
            result_tri[idx][1] = tri[T][1];
            result_tri[idx][2] = tri[T][2];

            trimap[T] = idx;

            ++idx;
        }
    }
    ASSERT(idx == result_numtri);

    for (int T = 0; T < numtri; ++T) {
        int idx = trimap[T];
        if (idx >= 0) {
            for (int i = 0; i < 3; ++i) {
                result_adj[idx][i] = trimap[adj[T][i]];
            }
        }
    }

 
    // @NOTE: Remap to original value.
    //count-=3;

    Delaunay_Triangulate_Result result = {};
    result.numtri = result_numtri;
    result.tri = result_tri;
    result.adj = result_adj;
    return result;
}

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
#endif
#if 0
    vertices[0].position = v3{1,0,1};
    vertices[1].position = v3{3,0,4};
    vertices[2].position = v3{-2,0,3};
    vertices[3].position = v3{-2,0,2};
    vertices[4].position = v3{-1,0,-1};
    vertices[5].position = v3{-2,0,-3};
    vertices[6].position = v3{4,0,-2};
#endif
#if 1
    vertices[0].position={0.0f,  0, 0.0f};
    vertices[1].position={-0.2f, 0, 0.1f};
    vertices[2].position={-0.8f, 0, -0.2f};
    vertices[3].position={-0.8f, 0, -0.5f};
    vertices[4].position={-0.3f, 0, 0.1f};
    vertices[5].position={ 0.7f, 0, -0.1f};
    vertices[6].position={ 0.7f, 0, -0.0f};
#endif
}

void restart(Vertex *vertices, u32 count, Vertex_Array *convexhull, Vertex_Array *triangulatedconvexhull) {
    generate_vertices(vertices, count);
    *convexhull = jarvismarch(vertices, count);
    *triangulatedconvexhull = triangulate_convex_hull(convexhull);
}

int main(void) {
    glfwSetErrorCallback(glfw_error_callback);
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow *window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "UNITITLED", NULL, NULL);
    if (!window) {
        return -1;
    }
    glfwMakeContextCurrent(window);
    //glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;

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

    const int edgecount = 1;
    int edges[edgecount][2];
    edges[0][0] = 2;
    edges[0][1] = 6;
    Delaunay_Triangulate_Result dln_result = delaunay_triangulate(vertices, vertexcount, edges, edgecount);

    bool drawconvexhull = false;
    bool drawtriangulatedconvexhull = false;
    bool drawdelaunay = false;
    float scale = 0.15f;
    GLuint circleshader_scale = glad_glGetUniformLocation(circleshader, "scale");
    GLuint simpleshader_scale = glad_glGetUniformLocation(simpleshader, "scale");
    GLuint simpleshader_color = glad_glGetUniformLocation(simpleshader, "color");

    {
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LEQUAL);
    }

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();    

        if (glfwGetWindowAttrib(window, GLFW_ICONIFIED) != 0) {
            ImGui_ImplGlfw_Sleep(10);
            continue;
        }

        {
            ImGui_ImplOpenGL3_NewFrame();
            ImGui_ImplGlfw_NewFrame();
            ImGui::NewFrame();
        }

        {
            ImGui::Begin("Control");
            ImGui::Checkbox("Draw Convex Hull", &drawconvexhull);
            ImGui::Checkbox("Draw Triangulated Convex Hull", &drawtriangulatedconvexhull);
            ImGui::Checkbox("Draw Constrained Delaunay Triangulation", &drawdelaunay);
            ImGui::DragFloat("Scale", &scale, 0.01f, 0.001f, 100.0f);
#if 1
            if (ImGui::Button("Restart")) {
                free(vertices);
                vertices = (Vertex *)malloc(sizeof(Vertex) * vertexcount);
                restart(vertices, vertexcount, &hull, &triangulated);

                free(dln_result.tri);
                free(dln_result.adj);
                dln_result = delaunay_triangulate(vertices, vertexcount, edges, edgecount);
            }
#endif
            ImGui::End();
        }
        ImGui::Render();

        {
            glclearcolor(v4{0.1f,0.1f,0.1f,1.0f});
            glClearDepth(1.0f);
            glClear(GL_DEPTH_BUFFER_BIT);
        }

        {
            glUseProgram(circleshader);
            glUniform1f(circleshader_scale, scale);
            glEnableVertexAttribArray(0);
            glVertexAttribPointer(0, 3, GL_FLOAT, false, sizeof(Vertex), (GLvoid *)(offsetof(Vertex, position)));
            glBufferData(GL_ARRAY_BUFFER, vertexcount * sizeof(Vertex), vertices, GL_DYNAMIC_DRAW);
            glDrawArrays(GL_POINTS, 0, vertexcount);
            glDisableVertexAttribArray(0);
        }

        if (drawconvexhull) {
            glUseProgram(simpleshader);
            glUniform1f(simpleshader_scale, scale);
            glUniform4f(simpleshader_color, 1.0f, 1.0f, 0.0f, 1.0f);
            glEnableVertexAttribArray(0);
            glVertexAttribPointer(0, 3, GL_FLOAT, false, sizeof(Vertex), (GLvoid *)(offsetof(Vertex, position)));
            glBufferData(GL_ARRAY_BUFFER, hull.count * sizeof(Vertex), hull.vertices, GL_DYNAMIC_DRAW);
            glDrawArrays(GL_LINE_LOOP, 0, hull.count);
            glDisableVertexAttribArray(0);
        }

        if (drawtriangulatedconvexhull) {
            glUseProgram(simpleshader);
            glUniform1f(simpleshader_scale, scale);
            glUniform4f(simpleshader_color, 1.0f, 1.0f, 0.0f, 1.0f);
            glEnableVertexAttribArray(0);
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            glVertexAttribPointer(0, 3, GL_FLOAT, false, sizeof(Vertex), (GLvoid *)(offsetof(Vertex, position)));
            glBufferData(GL_ARRAY_BUFFER, triangulated.count * sizeof(Vertex), triangulated.vertices, GL_DYNAMIC_DRAW);
            glDrawArrays(GL_TRIANGLES, 0, triangulated.count);
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            glDisableVertexAttribArray(0);
        }

        if (drawdelaunay) {
            glUseProgram(simpleshader);
            glUniform1f(simpleshader_scale, scale);
            glUniform4f(simpleshader_color, 1.0f, 1.0f, 0.0f, 1.0f);
            glEnableVertexAttribArray(0);
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            glVertexAttribPointer(0, 3, GL_FLOAT, false, sizeof(Vertex), (GLvoid *)(offsetof(Vertex, position)));
            glBufferData(GL_ARRAY_BUFFER, vertexcount * sizeof(Vertex), vertices, GL_DYNAMIC_DRAW);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, dln_result.numtri*3 * sizeof(int), dln_result.tri, GL_DYNAMIC_DRAW);
            glDrawElements(GL_TRIANGLES, dln_result.numtri*3, GL_UNSIGNED_INT, (void *)0);
            if (edges && edgecount) {
                glUniform4f(simpleshader_color, 1.0f, 0.0f, 1.0f, 1.0f);
                glBufferData(GL_ELEMENT_ARRAY_BUFFER, edgecount*2 * sizeof(int), edges, GL_DYNAMIC_DRAW);
                glDrawElements(GL_LINES, edgecount*2, GL_UNSIGNED_INT, (void *)0);
            }
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            glDisableVertexAttribArray(0);
        }

        {
            glUseProgram(0);
        }

        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();

        glfwDestroyWindow(window);
        glfwTerminate();
    }

    return 0;
}
