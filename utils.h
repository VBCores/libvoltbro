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

#define pi2 6.28318530718

#define force_inline __attribute__((always_inline)) static inline

#define CRITICAL_SECTION(code_blk)          \
    uint32_t primask_bit = __get_PRIMASK(); \
    __disable_irq();                        \
    code_blk __set_PRIMASK(primask_bit);

#define HAL_IMPORTANT(command) \
    if ((command) != HAL_OK) { \
        Error_Handler();       \
    }

#define MILLIS_COUNTER(counter) static uint32_t counter = 0;

#define EACH_N_MILLIS(N, counter, code_blk) \
    if ((tick - (counter)) >= (N)) {        \
        (counter) = tick;                   \
        code_blk                            \
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
