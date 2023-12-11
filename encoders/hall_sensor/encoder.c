/*
 * encoder.c
 *
 *  Created on: Mar 1, 2023
 *      Author: Igor Beschastnov
 */

#include "encoder.h"
#include "string.h"
#include "utils.h"

force_inline EncoderStep get_encoder_step(HallSensor* encoder) {
    return encoder->state_1 + encoder->state_2 * 2 + encoder->state_3 * 4;
}

force_inline bool get_pin_state(const GPIO_TypeDef* gpiox, pin p) {
    return HAL_GPIO_ReadPin((GPIO_TypeDef*)gpiox, p) == GPIO_PIN_SET ? true : false;
}

uint16_t nop(GEncoder* encoder) {
    return encoder->value;
}

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
) {
    HallSensor tmp_encoder = {
        .common =
            {.inverted = inverted,
             .is_electrical = true,
             .CPR = 6,
             .value = 0,
             .last_error = false,
             .elec_offset = 0,
             .get_angle = &nop,
             .revolutions = 0},
        .newer_interrupt = false,
        .state_1 = get_pin_state(pin_1_gpiox, pin_1),
        .state_2 = get_pin_state(pin_2_gpiox, pin_2),
        .state_3 = get_pin_state(pin_3_gpiox, pin_3),
        .pin_1_gpiox = pin_1_gpiox,
        .pin_2_gpiox = pin_2_gpiox,
        .pin_3_gpiox = pin_3_gpiox,
        .pin_1 = pin_1,
        .pin_2 = pin_2,
        .pin_3 = pin_3,
        .increment = inverted ? -1 : 1,
        .direction = 0,
        .last_activated = -1,
        .sequence = *sequence
    };
    tmp_encoder.step = get_encoder_step(&tmp_encoder);

    memcpy(dest, &tmp_encoder, sizeof(HallSensor));
}

HallSensor* make_hall_sensor(
    bool inverted,
    GPIO_TypeDef* pin_1_gpiox,
    GPIO_TypeDef* pin_2_gpiox,
    GPIO_TypeDef* pin_3_gpiox,
    pin pin_1,
    pin pin_2,
    pin pin_3,
    EncoderStep* sequence
) {
    HallSensor* encoder;
    CRITICAL_SECTION({ encoder = malloc(sizeof(HallSensor)); })
    if (encoder == NULL) {
        return NULL;
    }

    make_hall_sensor_reserved(
        encoder,
        inverted,
        pin_1_gpiox,
        pin_2_gpiox,
        pin_3_gpiox,
        pin_1,
        pin_2,
        pin_3,
        sequence
    );

    return encoder;
}

#ifdef DEBUG
int8_t activated_pin;
#endif
//#define TRUSTED_EXTI
bool handle_hall_channel(HallSensor* encoder, pin channel) {
#ifndef DEBUG
    uint8_t activated_pin;
#endif

#ifdef TRUSTED_EXTI
    if (encoder->pin_1 == channel) {
        activated_pin = 0;
    } else if (encoder->pin_2 == channel) {
        activated_pin = 1;
    } else {
        activated_pin = 2;
    }
#endif

#ifdef TRUSTED_EXTI
    *(&encoder->state_1 + activated_pin) = !*(&encoder->state_1 + activated_pin);
#else
    bool old_state_1 = encoder->state_1;
    bool old_state_2 = encoder->state_2;
    bool old_state_3 = encoder->state_3;
#endif
    encoder->state_1 = get_pin_state(encoder->pin_1_gpiox, encoder->pin_1);
    encoder->state_2 = get_pin_state(encoder->pin_2_gpiox, encoder->pin_2);
    encoder->state_3 = get_pin_state(encoder->pin_3_gpiox, encoder->pin_3);
#ifndef TRUSTED_EXTI
    if (old_state_1 != encoder->state_1) {
        activated_pin = 0;
    }
    else if (old_state_2 != encoder->state_2) {
        activated_pin = 1;
    }
    else if (old_state_3 != encoder->state_3) {
        activated_pin = 2;
    }
    else {
        activated_pin = -1;
    }
#endif

    if (encoder->last_activated == -1 || activated_pin == -1) {
        encoder->last_activated = activated_pin;
        encoder->step = get_encoder_step(encoder);
        return false;
    }

    int32_t value = (int32_t)encoder->common.value;
    bool positive_direction = false;
    switch (encoder->last_activated) {
        case 0:
            positive_direction = activated_pin == 2;
            break;
        case 1:
            positive_direction = activated_pin == 0;
            break;
        case 2:
            positive_direction = activated_pin == 1;
            break;
    }
    if (positive_direction) {
        encoder->direction = 1;
    } else {
        encoder->direction = -1;
    }
    value += encoder->increment * encoder->direction;
    encoder->last_activated = (int8_t)activated_pin;

    if (value >= encoder->common.CPR) {
        value = value - encoder->common.CPR;
    } else if (value < 0) {
        value = value + encoder->common.CPR;
    }
    bool has_changed = encoder->common.value != (uint16_t)value;
    encoder->common.value = (uint16_t)value;
    encoder->step = get_encoder_step(encoder);

    return has_changed;
}
