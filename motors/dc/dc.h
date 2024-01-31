#pragma once

#include <utility>

#include "stm32g4xx_hal.h"

#include "utils.h"
#include "encoders/generic.h"
#include "regulators/pid/pid.h"
#include "motors/motor_commons.h"
#include "dsp/low_pass_filter.hpp"


struct DCDriverConfig {
    const bool is_small;

    const pin nSLEEP_pin;
    GPIO_TypeDef* const nSLEEP_GPIOx;

    const pwm_channel IN1_channel;
    const pwm_channel IN2_channel;
    TIM_HandleTypeDef* const timer;
    const tim_register max_pwm;

    const dac_channel dac_channel_;
    DAC_HandleTypeDef* const dac;

    const double Rsense;

    const CommonDriverConfig common;
};

class DCMotorController {
private:
    const DCDriverConfig config;
    PIDRegulator regulator;
    const LowPassFilter angle_filter;
    const LowPassFilter speed_filter;

    bool is_on = false;
    float speed = 0;
    float target_speed = 0;
    float Ipeak = 0;
public:
    DCMotorController(
        const DCDriverConfig&& driver,
        PIDConfig&& config,
        float angle_filter = 1,
        float speed_filter = 1
    ):
        config(driver),
        regulator(std::forward<PIDConfig>(config)),
        angle_filter(angle_filter),
        speed_filter(speed_filter)
    {};

    HAL_StatusTypeDef init();
    HAL_StatusTypeDef stop();
    HAL_StatusTypeDef start();
    HAL_StatusTypeDef set_state(bool);
    HAL_StatusTypeDef set_Ipeak(float);
    void set_target_speed(float);
    void regulate(GenericEncoder& encoder, float dt) const;
};