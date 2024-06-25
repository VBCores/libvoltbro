#pragma once
#if defined(STM32G474xx) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#if defined(HAL_TIM_MODULE_ENABLED)

#include "basic.hpp"

constexpr float HALF_PI = PI / 2.0f;

class ServoMotor: public BasicServo {
public:
    ServoMotor(TIM_HandleTypeDef *htim, tim_register channel):
        BasicServo(htim, channel, 1000)
    {}

    float get_target_angle() {
        const float pulse = get_pulse();
        return pulse * PI - HALF_PI;
    }

    // -HALF_PI <= angle <= +HALF_PI
    void set_target_angle(float angle) {
        assert_param(angle >= -HALF_PI && angle <= +HALF_PI);
        set_pulse((angle + HALF_PI) / PI);
    }
};

#endif
#endif