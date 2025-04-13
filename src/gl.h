/* ========================================================================

   (C) Copyright 2025 by Sung Woo Lee, All Rights Reserved.

   This software is provided 'as-is', without any express or implied
   warranty. In no event will the authors be held liable for any damages
   arising from the use of this software.

   ======================================================================== */




const char *g_shader_header = 
#include "shader/header.glsl"

    

void gl_framebuffer_resize_callback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}  

GLuint glcreateshader(const char *vsrc, const char *fsrc) {
    GLuint program = 0;

    if (glCreateShader) {
        GLuint vshader = glCreateShader(GL_VERTEX_SHADER);
        const GLchar *vunit[] = { g_shader_header, vsrc };
        glShaderSource(vshader, arraycount(vunit), (const GLchar **)vunit, 0);
        glCompileShader(vshader);

        GLuint fshader = glCreateShader(GL_FRAGMENT_SHADER);
        const GLchar *funit[] = { g_shader_header, fsrc };
        glShaderSource(fshader, arraycount(funit), (const GLchar **)funit, 0);
        glCompileShader(fshader);

        program = glCreateProgram();
        glAttachShader(program, vshader);
        glAttachShader(program, fshader);
        glLinkProgram(program);

        glValidateProgram(program);
        GLint linked = false;
        glGetProgramiv(program, GL_LINK_STATUS, &linked);
        if (!linked) {
            GLsizei stub;

            GLchar vlog[1024];
            glGetShaderInfoLog(vshader, sizeof(vlog), &stub, vlog);

            GLchar flog[1024];
            glGetShaderInfoLog(fshader, sizeof(flog), &stub, flog);

            GLchar plog[1024];
            glGetProgramInfoLog(program, sizeof(plog), &stub, plog);

            ASSERT(!"compile/link error.");
        }

        glDeleteShader(vshader);
        glDeleteShader(fshader);
    } else {
        // @TODO: Error-Handling.
    }
    
    return program;
}

GLuint glcreateshader(const char *vsrc, const char *gsrc, const char *fsrc) {
    GLuint program = 0;

    if (glCreateShader) 
    {
        GLuint vshader = glCreateShader(GL_VERTEX_SHADER);
        const GLchar *vunit[] = { g_shader_header, vsrc };
        glShaderSource(vshader, arraycount(vunit), (const GLchar **)vunit, 0);
        glCompileShader(vshader);

        GLuint gshader = glCreateShader(GL_GEOMETRY_SHADER);
        const GLchar *gunit[] = { g_shader_header, gsrc };
        glShaderSource(gshader, arraycount(gunit), (const GLchar **)gunit, 0);
        glCompileShader(gshader);

        GLuint fshader = glCreateShader(GL_FRAGMENT_SHADER);
        const GLchar *funit[] = { g_shader_header, fsrc };
        glShaderSource(fshader, arraycount(funit), (const GLchar **)funit, 0);
        glCompileShader(fshader);

        program = glCreateProgram();
        glAttachShader(program, vshader);
        glAttachShader(program, gshader);
        glAttachShader(program, fshader);
        glLinkProgram(program);

        glValidateProgram(program);
        GLint linked = false;
        glGetProgramiv(program, GL_LINK_STATUS, &linked);
        if (!linked) 
        {
            GLsizei stub;

            GLchar vlog[1024];
            glGetShaderInfoLog(vshader, sizeof(vlog), &stub, vlog);

            GLchar glog[1024];
            glGetShaderInfoLog(gshader, sizeof(glog), &stub, glog);

            GLchar flog[1024];
            glGetShaderInfoLog(fshader, sizeof(flog), &stub, flog);

            GLchar plog[1024];
            glGetProgramInfoLog(program, sizeof(plog), &stub, plog);

            ASSERT(!"compile/link error.");
        }

        glDeleteShader(vshader);
        glDeleteShader(gshader);
        glDeleteShader(fshader);
    } else {
        // @TODO: Error-Handling.
    }
    
    return program;
}

void glclearcolor(v4 color) {
    glClearColor(color.r, color.g, color.b, color.a);
    glClear(GL_COLOR_BUFFER_BIT);
}

