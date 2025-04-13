#ifndef GEO_H_
#define GEO_H_
/* ========================================================================

   (C) Copyright 2025 by Sung Woo Lee, All Rights Reserved.

   This software is provided 'as-is', without any express or implied
   warranty. In no event will the authors be held liable for any damages
   arising from the use of this software.

   ======================================================================== */


struct Vertex {
    v3 position;
};

struct Vertex_Array {
    Vertex *vertices;
    unsigned int count;
};

#endif // GEO_H_
