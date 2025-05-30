/* ========================================================================

   (C) Copyright 2025 by Sung Woo Lee, All Rights Reserved.

   This software is provided 'as-is', without any express or implied
   warranty. In no event will the authors be held liable for any damages
   arising from the use of this software.

   ======================================================================== */



#include <stdint.h>
#include <stddef.h>
#include <limits.h>
#include <float.h>

#define KB(value) (   value  * 1024ll)
#define MB(value) (KB(value) * 1024ll)
#define GB(value) (MB(value) * 1024ll)
#define TB(value) (GB(value) * 1024ll)

#define internal        static
#define global          static
#define local_persist   static

typedef int8_t int8;  
typedef int16_t int16; 
typedef int32_t int32; 
typedef int64_t int64; 

typedef int8_t s8;  
typedef int16_t s16; 
typedef int32_t s32; 
typedef int64_t s64; 

typedef int32 bool32;
typedef bool32 b32;

typedef uint8_t uint8;
typedef uint16_t uint16; 
typedef uint32_t uint32; 
typedef uint64_t uint64; 

typedef uint8_t u8;  
typedef uint16_t u16; 
typedef uint32_t u32; 
typedef uint64_t u64; 

typedef float f32; 
typedef double f64; 

typedef size_t mmm;
typedef uintptr_t umm;
typedef intptr_t  smm;


struct Buffer {
    umm count;
    u8 *data;
};
typedef Buffer String;


#define F32_MIN FLT_MIN
#define F32_MAX FLT_MAX
