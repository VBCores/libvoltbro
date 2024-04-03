#pragma once
#if defined(STM32G474xx) || defined(STM32_G)

#include <cstdint>

#ifdef ARM_MATH_CM4
#include "stm32g4xx.h"
#include "arm_math.h"
#else
#include "math.h"
#define arm_sqrt_f32(v, pOut) *(pOut) = sqrtf(v)
#define arm_cos_f32 cosf
#define arm_sin_f32 sinf
#endif

float fmaxf(float x, float y);
float fminf(float x, float y);
float fmaxf3(float x, float y, float z);
float fminf3(float x, float y, float z);
float roundf(float x);
void limit_norm(float* x, float* y, float limit);
int float_to_uint(float x, float x_min, float x_max, uint8_t bits);
float uint_to_float(int x_int, float x_min, float x_max, uint8_t bits);
float mfmod(float x, float y);

#endif
