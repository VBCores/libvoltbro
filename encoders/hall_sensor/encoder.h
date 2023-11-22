/*
 * encoder.h
 *
 *  Created on: Mar 1, 2023
 *      Author: Igor Beschastnov
 */

#ifndef VBLIB_HALL_SENSOR_ENCODER_H_
#define VBLIB_HALL_SENSOR_ENCODER_H_

#if defined(STM32G474xx) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#else
#include "stm32f4xx_hal.h"
#endif

#include "encoders/generic.h"
#include "utils.h"

#ifdef __cplusplus
#include <cstdint>
extern "C" {
#else
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#endif

typedef enum {
    CA = 4,
    AB = 1,
    CB = 5,
    BC = 2,
    BA = 6,
    AC = 3,
} EncoderStep;

typedef struct {
    // Shared
    GEncoder common;
    // Hardware stuff, valid only for HALL_SIX_STEP mode, ignore otherwise
    bool newer_interrupt;
    // State machine
    int state_1;
    int state_2;
    int state_3;
    int8_t direction;
    int8_t increment;
    int8_t last_activated;
    EncoderStep step;
    const GPIO_TypeDef* pin_1_gpiox;
    const GPIO_TypeDef* pin_2_gpiox;
    const GPIO_TypeDef* pin_3_gpiox;
    const pin pin_1;
    const pin pin_2;
    const pin pin_3;
    EncoderStep sequence[6];
} HallSensor;

HallSensor* make_hall_sensor(
    bool inverted,
    GPIO_TypeDef* pin_1_gpiox,
    GPIO_TypeDef* pin_2_gpiox,
    GPIO_TypeDef* pin_3_gpiox,
    pin pin_1,
    pin pin_2,
    pin pin_3,
    EncoderStep* sequence
);

void make_hall_sensor_reserved(
    HallSensor* dest,
    bool inverted,
    GPIO_TypeDef* pin_1_gpiox,
    GPIO_TypeDef* pin_2_gpiox,
    GPIO_TypeDef* pin_3_gpiox,
    pin pin_1,
    pin pin_2,
    pin pin_3,
    EncoderStep* sequence
);
bool handle_hall_channel(HallSensor* encoder, pin channel);

#ifdef __cplusplus
}
#endif
#endif /* VBLIB_HALL_SENSOR_ENCODER_H_ */
