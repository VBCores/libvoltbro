#pragma once
#if defined(STM32G474xx) || defined(STM32_G)

#include "stm32g4xx_hal.h"
#if defined(HAL_DAC_MODULE_ENABLED) && defined(HAL_TIM_MODULE_ENABLED)

#include <utility>
#include <memory>

#include "voltbro/utils.hpp"
#include "voltbro/encoders/generic.h"
#include "voltbro/motors/motor_commons.hpp"
#include "voltbro/math/dsp/low_pass_filter.hpp"

struct DCDriverConfig {
    const pin nSLEEP_pin;
    GPIO_TypeDef* const nSLEEP_GPIOx;

    const pwm_channel IN1_channel;
    const pwm_channel IN2_channel;
    TIM_HandleTypeDef* const timer;

    const dac_channel dac_channel_;
    DAC_HandleTypeDef* const dac;

    const double Rsense;

    const CommonDriverConfig common;
};

class DCMotorController: public AbstractMotor {
private:
    const DCDriverConfig config;
    GenericEncoder& encoder;
    bool is_using_brake;

    /* WARNING! Explicitly specify alignment for guaranteed atomic reads and writes. Explanation:
     * https://developer.arm.com/documentation/dui0375/g/C-and-C---Implementation-Details/Basic-data-types-in-ARM-C-and-C-- or https://stackoverflow.com/a/52785864
     * short version: all reads/writes to var are atomic if it is "self"-aligned (1/2/4 byte)
     * (Please, copy this comment to all variables that can be accessed concurrently - as a warning and a reminder) */
    arm_atomic(bool) _is_on = false;
    arm_atomic(float) Ipeak = 0;
    arm_atomic(float) angle = 0;

    float current_pwm;
public:
    DCMotorController(
        const DCDriverConfig&& driver,
        GenericEncoder& encoder,
        bool is_using_brake = false
    ):
        AbstractMotor(),
        config(std::move(driver)),
        encoder(encoder),
        is_using_brake(is_using_brake)
    {};

    HAL_StatusTypeDef init();
    HAL_StatusTypeDef set_state(bool);
    HAL_StatusTypeDef set_Ipeak(float);
    void update();
    void set_pulse(float pwm);

    bool is_on() const {
        return _is_on;
    }
    float get_angle() const {
        return angle;
    }
    float get_Ipeak() const {
        return Ipeak;
    }
    HAL_StatusTypeDef stop() {
        return set_state(false);
    }
    HAL_StatusTypeDef start() {
        return set_state(true);
    }
};

#endif
#endif
