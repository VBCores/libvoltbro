/*
 * utils.h
 *
 *  Created on: Oct 19, 2022
 *      Author: igor
 */

#ifndef VOLTBROLIB_UTILS_H_
#define VOLTBROLIB_UTILS_H_

#include "stdbool.h"
#if defined(STM32G474xx) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#else
#include "stm32f4xx_hal.h"
#endif

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
    assert_param(abs_diff <= INT64_MAX);
    return (first > second) ? (int64_t)abs_diff : -(int64_t)abs_diff;
}

#define CRITICAL_SECTION(code_blk)          \
    uint32_t primask_bit = __get_PRIMASK(); \
    __disable_irq();                        \
    code_blk __set_PRIMASK(primask_bit);

#define HAL_IMPORTANT(command) \
    if ((command) != HAL_OK) { \
        Error_Handler();       \
    }

#define EACH_N(_value, _counter, N, code_blk)           \
    if ((_value - _counter) >= (N)) {                   \
        code_blk                                        \
        _counter = _value;                              \
    }

#define EACH_N_MICROS(_value, _counter, N, code_blk) \
    int64_t diff = subtract_64(_value, _counter);    \
    if (diff >= (int64_t)N) {                        \
        code_blk                                     \
        _counter = _value;                           \
    }


#ifdef __cplusplus
extern "C" {
#endif

uint32_t dac_value(double dac_voltage);
bool is_close(float x, float y);

#ifdef __cplusplus
}
#endif

#endif /* VOLTBROLIB_UTILS_H_ */
