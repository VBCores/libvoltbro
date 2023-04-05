/*
 * encoder.c
 *
 *  Created on: Mar 1, 2023
 *      Author: Igor Beschastnov
 */

#include "encoder.h"
#include "string.h"
#include "utils.h"

force_inline EncoderStep get_encoder_step(IncrementalEncoder* encoder) {
    if (encoder->state_3) {
        if (encoder->state_1)
            return 1;
        if (encoder->state_2)
            return 5;
        return 0;
    }
    if (encoder->state_1) {
        if (encoder->state_2)
            return 3;
        return 2;
    }
    return 4;
    // return encoder->state_1 + encoder->state_2*2 + encoder->state_3*4;
}

force_inline bool get_pin_state(const GPIO_TypeDef* gpiox, pin p) {
    return HAL_GPIO_ReadPin((GPIO_TypeDef*)gpiox, p) == GPIO_PIN_SET ? true
                                                                     : false;
}

uint16_t nop(GEncoder* encoder) {
    return encoder->value;
}

void make_incr_encoder_reserved(
    IncrementalEncoder* dest,
    uint16_t CPR,
    bool inverted,
    GPIO_TypeDef* pin_gpiox,
    pin pin_1,
    pin pin_2,
    pin pin_3
) {
    IncrementalEncoder tmp_encoder = {
        .common =
            {.inverted = inverted,
             .CPR = CPR,
             .value = 0,
             .last_error = false,
             .elec_offset = 0,
             .get_angle = &nop},
        .state_1 = get_pin_state(pin_gpiox, pin_1),
        .state_2 = get_pin_state(pin_gpiox, pin_2),
        .state_3 = get_pin_state(pin_gpiox, pin_3),
        .pin_gpiox = pin_gpiox,
        .pin_1 = pin_1,
        .pin_2 = pin_2,
        .pin_3 = pin_3,
        .increment = inverted ? -1 : 1,
        .direction = 0,
        .last_activated = -1,
    };
    tmp_encoder.step = get_encoder_step(&tmp_encoder);

    memcpy(dest, &tmp_encoder, sizeof(IncrementalEncoder));
}

IncrementalEncoder* make_incr_encoder(
    uint16_t CPR,
    bool inverted,
    GPIO_TypeDef* pin_gpiox,
    pin pin_1,
    pin pin_2,
    pin pin_3
) {
    IncrementalEncoder* encoder = malloc(sizeof(IncrementalEncoder));
    if (encoder == NULL) {
        return NULL;
    }

    make_incr_encoder_reserved(
        encoder,
        CPR,
        inverted,
        pin_gpiox,
        pin_1,
        pin_2,
        pin_3
    );

    return encoder;
}

void handle_encoder_channel(IncrementalEncoder* encoder, pin channel) {
    uint8_t activated_pin;
    // bool state = get_pin_state(encoder->pin_gpiox, channel);

    if (encoder->pin_1 == channel) {
        activated_pin = 0;
    } else if (encoder->pin_2 == channel) {
        activated_pin = 1;
    } else {
        activated_pin = 2;
    }

    if (encoder->last_activated == -1) {
        encoder->last_activated = (int8_t)activated_pin;
        return;
    }
    //*(&encoder->state_1 + activated_pin) = state;  // TODO: test
    encoder->state_1 = get_pin_state(encoder->pin_gpiox, encoder->pin_1);
    encoder->state_2 = get_pin_state(encoder->pin_gpiox, encoder->pin_2);
    encoder->state_3 = get_pin_state(encoder->pin_gpiox, encoder->pin_3);

    int32_t value = (int32_t)encoder->common.value;
    if ((activated_pin == encoder->last_activated + 1) ||
        (encoder->last_activated == 2 && activated_pin == 0)) {
        value += encoder->increment;
        encoder->direction = 1 * encoder->increment;
    } else {
        value -= encoder->increment;
        encoder->direction = -1 * encoder->increment;
    }
    encoder->last_activated = (int8_t)activated_pin;

    if (value >= encoder->common.CPR) {
        value = value - encoder->common.CPR;
    } else if (value < 0) {
        value = value + encoder->common.CPR;
    }
    encoder->common.value = (uint16_t)value;
    encoder->step = get_encoder_step(encoder);
}
