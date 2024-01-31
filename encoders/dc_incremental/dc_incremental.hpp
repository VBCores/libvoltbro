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
    float processed_value;
    const uint16_t half_cpr;
public:
    DCIncrementalEncoder(
        TIM_HandleTypeDef* encoder_tim,
        encoder_data CPR,
        bool is_inverted = false
    ):
        GenericEncoder(CPR, is_inverted),
        encoder_tim(encoder_tim),
        offset_incr((UINT16_MAX + 1) % CPR),
        half_cpr(CPR / 2)
    {};

    HAL_StatusTypeDef init() {
        return HAL_TIM_Encoder_Start(encoder_tim,TIM_CHANNEL_ALL);
    }

    inline encoder_data get_value() const override {
        return processed_value;
    }

    void update_value(const LowPassFilter& filter) override {
        encoder_data new_value = __HAL_TIM_GetCounter(encoder_tim);
        encoder_data filtered_data = (uint16_t) filter(
            (float) value,
            (float) new_value
        );
        int diff = value - filtered_data;
        if (abs(diff) > INT16_MAX) { // INT16_MAX == UINT16_MAX / 2
            offset += copysign(offset_incr, diff);
        }
        value = new_value;

        uint32_t offset_val = value;
        offset_val += offset;
        int32_t new_processed_value = offset_val % CPR - electric_offset;
        if (new_processed_value < 0) {
            new_processed_value += CPR;
        }

        int32_t difference = processed_value - new_processed_value;
        if (abs(difference) > half_cpr) {
            if (difference > 0) {
                incr_revolutions();
            }
            else {
                decr_revolutions();
            }
        }

        processed_value = new_processed_value;
    }
};