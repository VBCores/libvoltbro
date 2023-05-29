/*
 * encoder.h
 *
 *  Created on: Mar 1, 2023
 *      Author: Igor Beschastnov
 */

#ifndef VBLIB_INCREMENTAL_ENCODER_ENCODER_H_
#define VBLIB_INCREMENTAL_ENCODER_ENCODER_H_

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
    A = 0,   // CA (B  C') B->C
    AB = 1,  // CB (A' B)  B->A
    B = 2,   // AB (A' C)  C->A
    BC = 3,  // AC (B' C)  C->B
    C = 4,   // BC (B' A)  A->B
    CA = 5,  // CA (A  C') A->C
} EncoderStep;

typedef struct IncrementalEncoder {
    // Shared
    GEncoder common;
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
} IncrementalEncoder;

IncrementalEncoder* make_incr_encoder(
    uint16_t CPR,
    bool inverted,
    GPIO_TypeDef* pin_1_gpiox,
    GPIO_TypeDef* pin_2_gpiox,
    GPIO_TypeDef* pin_3_gpiox,
    pin pin_1,
    pin pin_2,
    pin pin_3
);

void make_incr_encoder_reserved(
    IncrementalEncoder* dest,
    uint16_t CPR,
    bool inverted,
    GPIO_TypeDef* pin_1_gpiox,
    GPIO_TypeDef* pin_2_gpiox,
    GPIO_TypeDef* pin_3_gpiox,
    pin pin_1,
    pin pin_2,
    pin pin_3
);
void handle_encoder_channel(IncrementalEncoder* encoder, pin channel);
void calc_encoder_step(IncrementalEncoder* encoder);

#ifdef __cplusplus
}
#endif
#endif /* VBLIB_INCREMENTAL_ENCODER_ENCODER_H_ */
