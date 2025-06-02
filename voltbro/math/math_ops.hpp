#pragma once
#if defined(STM32G474xx) || defined(STM32_G)

#include <cstdint>

#ifdef ARM_MATH_CM4
#include "stm32g4xx.h"
#ifdef __cplusplus
extern "C" {
#endif
#include "arm_math.h"
#ifdef __cplusplus
}
#endif
#else
#include "math.h"
#define arm_sqrt_f32(v, pOut) *(pOut) = sqrtf(v)
#define arm_cos_f32 cosf
#define arm_sin_f32 sinf
#endif

inline float fmaxf(float x, float y) {
    /// Returns maximum of x, y ///
    return (((x) > (y)) ? (x) : (y));
}

inline float fminf(float x, float y) {
    /// Returns minimum of x, y ///
    return (((x) < (y)) ? (x) : (y));
}

inline float fmaxf3(float x, float y, float z) {
    /// Returns maximum of x, y, z ///
    return (x > y ? (x > z ? x : z) : (y > z ? y : z));
}

inline float fminf3(float x, float y, float z) {
    /// Returns minimum of x, y, z ///
    return (x < y ? (x < z ? x : z) : (y < z ? y : z));
}

inline float roundf(float x) {
    /// Returns nearest integer ///

    return x < 0.0f ? ceilf(x - 0.5f) : floorf(x + 0.5f);
}

inline void limit_norm(float* x, float* y, float limit) {
    /// Scales the lenght of vector (x, y) to be <= limit ///
    float norm = 0;
    arm_sqrt_f32(*x * *x + *y * *y, &norm);
    // float norm = sqrt(*x * *x + *y * *y);
    if (norm > limit) {
        *x = *x * limit / norm;
        *y = *y * limit / norm;
    }
}

inline int float_to_uint(float x, float x_min, float x_max, uint8_t bits) {
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

inline float uint_to_float(int x_int, float x_min, float x_max, uint8_t bits) {
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

inline float mfmod(float x, float y) {
    return x - ((int)(x / y)) * y;
}


#endif
