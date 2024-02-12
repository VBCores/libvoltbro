#pragma once
#if defined(STM32G474xx) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#if defined(HAL_TIM_MODULE_ENABLED)

#include "arm_math.h"

class BLHeli_SController {
private:
    const uint32_t min_phase;
    TIM_HandleTypeDef * const htim;
    const uint32_t channel;
public:
    static constexpr uint32_t RECOMMENDED_DELAY = 1500;

    BLHeli_SController(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t period):
        min_phase(period / 20), htim(htim), channel(channel)
    {}

    void set_pulse(float pulse) {
        assert_param((pulse >= 0.0) && (pulse <= 1.0));
        const uint32_t adj = min_phase * pulse;
        __HAL_TIM_SET_COMPARE(htim, channel, min_phase + adj);
    }

    void start() {
        set_pulse(0);
        HAL_TIM_PWM_Start(htim, channel);
    }

    void calibration_sequence() {
        set_pulse(1.0f);
        HAL_Delay(6000);
        set_pulse(0.5f);
        HAL_Delay(500);
        set_pulse(0.0f);
        HAL_Delay(4000);
    }

    void arming_sequence() {
        const uint32_t delay = 20;
        const uint32_t segments = 1 * 100 / delay;

        const double quarter_pi = M_PI / 4;
        for (float t = 0; t < quarter_pi; t += quarter_pi / segments) {
            HAL_Delay(delay);
            set_pulse(sinf(t));
        }
        for (float t = 0; t < quarter_pi; t += quarter_pi / segments) {
            HAL_Delay(delay);
            set_pulse(sinf(quarter_pi - t));
        }
        set_pulse(0.0f);
        HAL_Delay(500);
    }
};

#endif
#endif