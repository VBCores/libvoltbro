#pragma once

#if defined(STM32G474xx) || defined(STM32_G)
#include "stm32g4xx_hal.h"

#include <array>
#include <cstdint>

#include "voltbro/encoders/generic.h"
#include "voltbro/utils.hpp"

enum class EncoderStep: uint8_t {
    CA = 4,
    AB = 1,
    CB = 5,
    BC = 2,
    BA = 6,
    AC = 3
};


class HallSensor: public GenericEncoder {
private:
    int state_1;
    int state_2;
    int state_3;
    int8_t increment;
    EncoderStep step;
    int8_t direction = 0;
    int8_t last_activated = -1;
    const GPIO_TypeDef* pin_1_gpiox;
    const GPIO_TypeDef* pin_2_gpiox;
    const GPIO_TypeDef* pin_3_gpiox;
    const pin pin_1;
    const pin pin_2;
    const pin pin_3;
    std::array<EncoderStep, 6> sequence;

    __attribute__((always_inline)) inline EncoderStep get_encoder_step() {
        return EncoderStep(state_1 + state_2 * 2 + state_3 * 4);
    }
public:
    HallSensor(
        encoder_data CPR,
        bool is_inverted,
        GPIO_TypeDef* pin_1_gpiox,
        pin pin_1,
        GPIO_TypeDef* pin_2_gpiox,
        pin pin_2,
        GPIO_TypeDef* pin_3_gpiox,
        pin pin_3,
        std::array<EncoderStep, 6>&& sequence
    );

    EncoderStep get_step() const { return step; }
    bool handle_hall_channel(pin channel = (uint16_t)-1);

    void update_value() override {}
    inline encoder_data get_value() const override {
        return value;
    }
};

#endif