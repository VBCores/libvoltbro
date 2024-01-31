#pragma once

#include "stm32g4xx_hal.h"
#include "arm_math.h"

#include "utils.h"
#include "encoders/generic.h"

class DCIncrementalEncoder : public GenericEncoder {
private:
    TIM_HandleTypeDef* const encoder_tim;
    int32_t offset = 0;
    const encoder_data offset_incr;
public:
    DCIncrementalEncoder(
        TIM_HandleTypeDef* encoder_tim,
        encoder_data CPR,
        bool is_inverted = false
    ):
        GenericEncoder(CPR, is_inverted),
        encoder_tim(encoder_tim),
        offset_incr((UINT16_MAX + 1) % CPR)
    {};

    HAL_StatusTypeDef init() {
        return HAL_TIM_Encoder_Start(encoder_tim,TIM_CHANNEL_ALL);
    }

    inline encoder_data get_value() const override {
        uint32_t offset_val = value;
        offset_val += offset;
        return offset_val % CPR;
    }

    void update_value() override {
        encoder_data new_value = __HAL_TIM_GetCounter(encoder_tim);
        int diff = value - new_value;
        if (abs(diff) > INT16_MAX) { // INT16_MAX == UINT16_MAX / 2
            offset += copysign(offset_incr, diff);
        }
        value = new_value;
    }
};