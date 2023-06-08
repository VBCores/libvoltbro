/*
 * config.c
 *
 *  Created on: June 8, 2023
 *      Author: Igor Beschastnov
 */

#include "config.h"

force_inline uint8_t pin_state(PinInfo* pin_info) {
    GPIO_PinState state = HAL_GPIO_ReadPin(pin_info->port, pin_info->pin);
    return state == GPIO_PIN_SET ? 1 : 0;
}

uint16_t get_configured_id(ConfigPins* config) {
    uint16_t result = 0;
    for (int i = 0; i < config->pin_number; i++) {
        PinInfo pin_info = config->pins[i];
        uint8_t base_value = pin_state(&pin_info);
        uint8_t value = base_value << i;
        result += value;
    }
    return result;
}