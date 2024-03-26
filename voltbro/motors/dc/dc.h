#pragma once
#if defined(STM32G474xx) || defined(STM32_G)

#include "stm32g4xx_hal.h"
#if defined(HAL_DAC_MODULE_ENABLED) && defined(HAL_TIM_MODULE_ENABLED)

#include <utility>

#include "voltbro/utils.hpp"
#include "voltbro/encoders/generic.h"
#include "voltbro/regulators/pid/pid.h"
#include "voltbro/motors/motor_commons.h"
#include "voltbro/dsp/low_pass_filter.hpp"

struct DCDriverConfig {
    const pin nSLEEP_pin;
    GPIO_TypeDef* const nSLEEP_GPIOx;

    const pwm_channel IN1_channel;
    const pwm_channel IN2_channel;
    TIM_HandleTypeDef* const timer;
    const tim_register min_pwm;
    const tim_register max_pwm;

    const dac_channel dac_channel_;
    DAC_HandleTypeDef* const dac;

    const double Rsense;

    const CommonDriverConfig common;
};

class DCMotorController: public AbstractMotor {
private:
    const DCDriverConfig config;
    PIDRegulator regulator;
    bool is_using_brake;

    /* WARNING! Explicitly specify alignment for guaranteed atomic reads and writes. Explanation:
     * https://developer.arm.com/documentation/dui0375/g/C-and-C---Implementation-Details/Basic-data-types-in-ARM-C-and-C-- or https://stackoverflow.com/a/52785864
     * short version: all reads/writes to var are atomic if it is "self"-aligned (1/2/4 byte)
     * (Please, copy this comment to all variables that can be accessed concurrently - as a warning and a reminder) */
    arm_atomic(bool) _is_on = false;
    arm_atomic(float) Ipeak = 0;
    arm_atomic(float) angle = 0;
    arm_atomic(float) speed = 0;
    arm_atomic(float) speed_error = 0;
    arm_atomic(float) target_speed = 0;

    float current_pwm;
public:
    DCMotorController(
        const DCDriverConfig&& driver,
        PIDConfig&& config,
        float angle_filter = 1,
        float speed_filter = 1,
        bool is_using_brake = false
    ):
        AbstractMotor(angle_filter, speed_filter),
        config(driver),
        regulator(std::forward<PIDConfig>(config)),
        is_using_brake(is_using_brake)
    {};

    HAL_StatusTypeDef init();
    HAL_StatusTypeDef stop();
    HAL_StatusTypeDef start();
    HAL_StatusTypeDef set_state(bool);
    HAL_StatusTypeDef set_Ipeak(float);
    void set_target_speed(float);

    void update_angle(GenericEncoder& speed_encoder);
    void update_speed(const float dt);
    void regulate(float dt) override;

    bool is_on() const {
        return _is_on;
    }
    float get_angle() const {
        return angle;
    }
    float get_speed() const {
        return speed;
    }
    float get_target_speed() const {
        return target_speed;
    }

    float get_Ipeak() const {
        return Ipeak;
    }

    PIDConfig get_config() {
        return regulator.get_config();
    }
    void update_pid_config(PIDConfig&& new_config) {
        regulator.update_config(std::forward<PIDConfig&&>(new_config));
    }
};

#endif
#endif
