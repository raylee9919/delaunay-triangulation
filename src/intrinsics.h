/* ========================================================================

   (C) Copyright 2025 by Sung Woo Lee, All Rights Reserved.

   This software is provided 'as-is', without any express or implied
   warranty. In no event will the authors be held liable for any damages
   arising from the use of this software.

   ======================================================================== */




#if _MSC_VER
  #include <intrin.h>
#elif __LINUX
  #include <x86intrin.h>
#endif



inline f32 pow(f32 x, f32 y) {
    __m128 val = _mm_pow_ps(_mm_set1_ps(x), _mm_set1_ps(y));
    f32 result = *(f32 *)&val;
    return result;
}

inline f32 cos(f32 x) {
    __m128 val = _mm_cos_ps(_mm_set1_ps(x));
    f32 result = *(f32 *)&val;
    return result;
}

inline f32 acos(f32 x) {
    __m128 val = _mm_acos_ps(_mm_set1_ps(x));
    f32 result = *(f32 *)&val;
    return result;
}

inline f32 sin(f32 x) {
    __m128 val = _mm_sin_ps(_mm_set1_ps(x));
    f32 result = *(f32 *)&val;
    return result;
}

inline f32 tan(f32 x) {
    __m128 val = _mm_tan_ps(_mm_set1_ps(x));
    f32 result = *(f32 *)&val;
    return result;
}

inline f32 sqrt(f32 x) {
    __m128 val = _mm_sqrt_ps(_mm_set1_ps(x));
    f32 result = *(f32 *)&val;
    return result;
}

inline s32 round_f32_to_s32(f32 x) {
    s32 result = _mm_cvtss_si32(_mm_set_ss(x));
    return result;
}

inline u32 round_f32_to_u32(f32 x) {
    s32 result = (u32)_mm_cvtss_si32(_mm_set_ss(x));
    return result;
}
