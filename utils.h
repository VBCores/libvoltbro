/*
 * utils.h
 *
 *  Created on: Oct 19, 2022
 *      Author: igor
 */

#ifndef VOLTBROLIB_UTILS_H_
#define VOLTBROLIB_UTILS_H_

#ifdef STM32_G
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

#ifdef __cplusplus
extern "C" {
#endif

void blink_notify_led(int blinks, GPIO_TypeDef* GPIOx, pin GPIO_Pin);
void blink_notify(int blinks);

O1HeapInstance* heapInit(void** memoryArena, size_t heapSize);

uint32_t dac_value(double dac_voltage);

#ifdef __cplusplus
}
#endif

#endif /* VOLTBROLIB_UTILS_H_ */
