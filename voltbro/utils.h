#pragma once

#ifdef ARM_MATH_CM4
#include "arm_math.h"
#else
#include "math.h"
#endif

#include "stdint.h"
#include "stdbool.h"

typedef uint16_t pin;
typedef uint32_t pwm_channel;
typedef uint32_t dac_channel;
typedef uint32_t tim_register;
typedef uint16_t encoder_data;

typedef uint32_t millis;
typedef uint64_t micros;

#define pi2 6.28318530718

#define force_inline __attribute__((always_inline)) static inline
#define arm_atomic(T) alignas(T) T

force_inline int64_t subtract_64(uint64_t first, uint64_t second) {
    uint64_t abs_diff = (first > second) ? (first - second): (second - first);
    return (first > second) ? (int64_t)abs_diff : -(int64_t)abs_diff;
}

#if defined(STM32G474xx) || defined(STM32_G)
#define CRITICAL_SECTION(code_blk)          \
    uint32_t primask_bit = __get_PRIMASK(); \
    __disable_irq();                        \
    code_blk __set_PRIMASK(primask_bit);

#define HAL_IMPORTANT(command) \
    if ((command) != HAL_OK) { \
        Error_Handler();       \
    }
#endif

#define EACH_N(_value, _counter, N, code_blk)           \
    if ((_value - _counter) >= (N)) {                   \
        code_blk                                        \
        _counter = _value;                              \
    }

#define EACH_N_MICROS(_value, _counter, N, code_blk) \
    int64_t diff_##_counter = subtract_64(_value, _counter);    \
    if (diff_##_counter >= (int64_t)N) {                        \
        code_blk                                     \
        _counter = _value;                           \
    }


#ifdef __cplusplus
extern "C" {
#endif

#define EPS 1e-10

inline uint32_t dac_value(double dac_voltage) {
    return floor(4095 * dac_voltage / 3.3);
}

inline bool is_close(float x, float y) {
    return fabsf(x - y) < EPS;
}

#ifdef __cplusplus
}
#endif
