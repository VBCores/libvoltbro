/*
 * config.h
 *
 *  Created on: June 8, 2023
 *      Author: Igor Beschastnov
 */

#ifndef VBLIB_CONFIG_CONFIG_H_
#define VBLIB_CONFIG_CONFIG_H_

#if defined(STM32G474xx) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#else
#include "stm32f4xx_hal.h"
#endif

#include "utils.h"

#ifdef __cplusplus
#include <cstdint>
extern "C" {
#else
#include <stdbool.h>
#include <stdint.h>
#endif

typedef struct {
    pin pin;
    GPIO_TypeDef* port;
} PinInfo;

typedef struct {
    size_t pin_number;
    PinInfo pins[];
} ConfigPins;

uint16_t get_configured_id(ConfigPins* config);

#ifdef __cplusplus
}
#endif
#endif /* VBLIB_CONFIG_CONFIG_H_ */
