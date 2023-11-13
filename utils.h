/*
 * utils.h
 *
 *  Created on: Oct 19, 2022
 *      Author: igor
 */

#ifndef VOLTBROLIB_UTILS_H_
#define VOLTBROLIB_UTILS_H_

#if defined(STM32G474xx) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#else
#include "stm32f4xx_hal.h"
#endif

#include "o1heap/o1heap.h"

typedef uint16_t pin;
typedef uint32_t pwm_channel;
typedef uint32_t dac_channel;
typedef uint32_t tim_register;

extern GPIO_TypeDef* NotifyLED_GPIOx;
extern pin NotifyLED_PIN;

#define force_inline __attribute__((always_inline)) static inline

#define CRITICAL_SECTION(code_blk)          \
    uint32_t primask_bit = __get_PRIMASK(); \
    __disable_irq();                        \
    code_blk __set_PRIMASK(primask_bit);

#ifdef __cplusplus
extern "C" {
#endif

void blink_notify_led(int blinks, GPIO_TypeDef* GPIOx, pin GPIO_Pin);
void blink_notify(int blinks);

uint32_t dac_value(double dac_voltage);

#ifdef __cplusplus
}
#endif

#endif /* VOLTBROLIB_UTILS_H_ */
