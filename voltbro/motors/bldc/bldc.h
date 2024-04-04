#pragma once

#if defined(STM32G474xx) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#ifdef HAL_TIM_MODULE_ENABLED

#include <cstdint>

#include "voltbro/math/math_ops.h"
#include "voltbro/regulators/pid/pid.h"
#include "voltbro/utils.hpp"
#include "voltbro/encoders/hall_sensor/hall_sensor.h"
#include "inverter.hpp"
#include "../motor_commons.h"

enum class DrivePhase: uint8_t { PHASE_A = 0, PHASE_B = 1, PHASE_C = 2 };

struct ControlConfig {
    bool predict_change = false;
    bool detect_stall = false;

    float sampling_interval;
    float stall_timeout;
    float stall_tolerance;

    float velocity_target = 0;
    float voltage_target = 0;
    float current_limit;
    float user_current_limit;

    float speed_mult = 1;
    float electric_mult = 1;
    float PWM_mult = 1;
    uint16_t max_PWM_per_s = 0;

    PIDRegulator velocity_regulator;
    PIDRegulator electric_regulator;
};

struct DriveInfo {
    const float torque_const;
    const float speed_const;
    float max_current;
    float stall_current;
    const float supply_voltage;

    const pin L_PINS[3];
    const pin en_pin;
    GPIO_TypeDef* const en_port;
    const CommonDriverConfig common;
};

class BLDCController: public AbstractMotor {
protected:
    ControlConfig control_config;
    DriveInfo drive_info;
    Inverter inverter;
    const int32_t full_pwm;
    TIM_HandleTypeDef* const htim;

    arm_atomic(float) shaft_angle;
    arm_atomic(float) shaft_velocity;
    arm_atomic(bool) _is_on;
    arm_atomic(bool) is_stalling;
    uint16_t DQs[3] = {0, 0, 0};
public:
    BLDCController(
        DriveInfo&& drive_info,
        ControlConfig&& control_config,
        TIM_HandleTypeDef* htim,
        ADC_HandleTypeDef* hadc,
        float angle_filter = 1,
        float speed_filter = 1
    ):
        AbstractMotor(angle_filter, speed_filter),
        control_config(std::move(control_config)),
        drive_info(std::move(drive_info)),
        inverter(hadc),
        full_pwm(htim->Instance->ARR),
        htim(htim)
    {}

    void detect_stall(double passed_time_abs);
    void quit_stall() {
        control_config.current_limit = control_config.user_current_limit;
        is_stalling = false;
    }

    __attribute__((always_inline)) void flow_direction(DrivePhase from, DrivePhase to, int16_t pwm) {
        uint16_t actual_pwm = abs(pwm);
        if (pwm < 0) {
            DrivePhase tmp = to;
            to = from;
            from = tmp;
        }
        DrivePhase off;
        if (DrivePhase::PHASE_A != from && DrivePhase::PHASE_A != to)
            off = DrivePhase::PHASE_A;
        if (DrivePhase::PHASE_B != from && DrivePhase::PHASE_B != to)
            off = DrivePhase::PHASE_B;
        if (DrivePhase::PHASE_C != from && DrivePhase::PHASE_C != to)
            off = DrivePhase::PHASE_C;

        HAL_GPIO_WritePin(GPIOB, drive_info.L_PINS[to_underlying(off)], GPIO_PIN_RESET);
        DQs[to_underlying(off)] = 0;

        HAL_GPIO_WritePin(GPIOB, drive_info.L_PINS[to_underlying(from)], GPIO_PIN_SET);
        DQs[to_underlying(from)] = actual_pwm;

        HAL_GPIO_WritePin(GPIOB, drive_info.L_PINS[to_underlying(to)], GPIO_PIN_SET);
        DQs[to_underlying(to)] = 0;
    }

    __attribute__((always_inline)) float calc_elec_theta(float encoder_data, uint16_t pulses_per_pair) {
        float theta = pi2 * (mfmod(encoder_data, pulses_per_pair) / (float)pulses_per_pair) - PI;
        if (theta < 0) {
            theta += pi2;
        }
        return theta;
    }

    bool is_on() const {
        return _is_on;
    }
    HAL_StatusTypeDef stop();
    HAL_StatusTypeDef start();
    HAL_StatusTypeDef set_state(bool);

    float get_angle() const {
        return shaft_angle;
    }
    float get_velocity() const {
        return shaft_velocity;
    }

    inline void set_pwm() {
        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, DQs[0]);
        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, DQs[1]);
        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, DQs[2]);
    }
};

void step_to_phases(EncoderStep step, DrivePhase& first, DrivePhase& second);
force_inline float get_current(Inverter& inverter, DrivePhase current_relative) {
    switch (current_relative) {
        case DrivePhase::PHASE_A:
            return -inverter.get_A();
        case DrivePhase::PHASE_B:
            return -inverter.get_B();
        case DrivePhase::PHASE_C:
            return -inverter.get_C();
    }
}

#endif
#endif