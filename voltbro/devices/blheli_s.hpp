#pragma once
#if defined(STM32G474xx) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#if defined(HAL_TIM_MODULE_ENABLED)

#include "array"
#include "memory"
#include "arm_math.h"

static constexpr uint32_t arming_delay = 20;
static constexpr uint32_t arming_segments = 1 * 1000 / arming_delay;
static constexpr double eighth_pi = M_PI / 8;

class BLHeli_SController {
private:
    const tim_register min_phase;
    TIM_HandleTypeDef * const htim;
    const tim_register channel;
    bool _is_on = false;
public:
    static constexpr millis RECOMMENDED_DELAY = 1500;

    BLHeli_SController(TIM_HandleTypeDef *htim, tim_register channel):
    //htim->Instance->ARR / (1000 / (160000000 / htim->Instance->PSC))
        min_phase(1000), htim(htim), channel(channel)
    {}

    void set_pulse(float pulse) {
        if (!_is_on) {
            // Защита от дурака
            __HAL_TIM_SET_COMPARE(htim, channel, min_phase);
            return;
        }
        assert_param((pulse >= 0.0) && (pulse <= 1.0));
        const uint32_t adj = (float)min_phase * pulse;
        __HAL_TIM_SET_COMPARE(htim, channel, min_phase + adj);
    }

    void stop() {
        set_pulse(0);
        _is_on = false;
    }

    float get_pulse() const {
        const tim_register cpr = __HAL_TIM_GET_COMPARE(htim, channel);
        const tim_register offset_cpr = cpr - min_phase;
        return (float)offset_cpr / (float)min_phase;
    }

    bool is_on() const {
        return _is_on;
    }

    void start() {
        _is_on = true;
        set_pulse(0);
        HAL_TIM_PWM_Start(htim, channel);
    }
};

inline void rising_arm(std::array<std::shared_ptr<BLHeli_SController>, 2>& controllers_iterable) {
    for (float t = 0; t < eighth_pi; t += eighth_pi / arming_segments) {
        for (auto& controller: controllers_iterable) {
            HAL_Delay(arming_delay);
            controller->set_pulse(sinf(t));
        }
    }
}

inline void falling_arm(std::array<std::shared_ptr<BLHeli_SController>, 2>& controllers_iterable) {
    for (float t = 0; t < eighth_pi; t += eighth_pi / arming_segments) {
        for (auto& controller: controllers_iterable) {
            HAL_Delay(arming_delay);
            controller->set_pulse(sinf(eighth_pi - t));
        }
    }
}

inline void rising_arm(std::shared_ptr<BLHeli_SController>& controller) {
    for (float t = 0; t < eighth_pi; t += eighth_pi / arming_segments) {
        HAL_Delay(arming_delay);
        controller->set_pulse(sinf(t));
    }
}

inline void falling_arm(std::shared_ptr<BLHeli_SController>& controller) {
    for (float t = 0; t < eighth_pi; t += eighth_pi / arming_segments) {
        HAL_Delay(arming_delay);
        controller->set_pulse(sinf(eighth_pi - t));
    }
}

inline void calibration_sequence(std::array<std::shared_ptr<BLHeli_SController>, 2>& controllers_iterable) {
    rising_arm(controllers_iterable);

    for (auto& controller: controllers_iterable) {
        controller->set_pulse(1.0f);
    }
    HAL_Delay(6000);
    for (auto& controller: controllers_iterable) {
        controller->set_pulse(0.5f);
    }
    HAL_Delay(500);
    for (auto& controller: controllers_iterable) {
        controller->set_pulse(0.0f);
    }
    HAL_Delay(4000);
}

inline void calibration_sequence(std::shared_ptr<BLHeli_SController>& controller) {
    //rising_arm(controller);

    controller->set_pulse(1.0f);
    HAL_Delay(6000);
    controller->set_pulse(0.5f);
    HAL_Delay(500);
    controller->set_pulse(0.0f);
    HAL_Delay(4000);
}

inline void arming_sequence(std::array<std::shared_ptr<BLHeli_SController>, 2>& controllers_iterable) {
    rising_arm(controllers_iterable);
    falling_arm(controllers_iterable);
    for (auto& controller: controllers_iterable) {
        controller->set_pulse(0.0f);
    }
    HAL_Delay(500);
}

inline void arming_sequence(std::shared_ptr<BLHeli_SController>& controller) {
    rising_arm(controller);
    falling_arm(controller);
    controller->set_pulse(0.0f);
    HAL_Delay(500);
}

#endif
#endif