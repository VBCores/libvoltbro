/*
 * math.h
 *
 *  Created on: Jan 25, 2023
 *      Author: igor
 */

#ifndef VBLIB_MATH_MATH_OPS_H_
#define VBLIB_MATH_MATH_OPS_H_

#ifdef __cplusplus
#include <cstdint>
extern "C" {
#else
#include <stdint.h>
#endif

#ifdef ARM_MATH_CM4
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

#ifdef __cplusplus
}
#endif

#endif /* VBLIB_MATH_MATH_OPS_H_ */
