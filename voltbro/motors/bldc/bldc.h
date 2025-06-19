#pragma once

#if defined(STM32G4) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#if defined(HAL_TIM_MODULE_ENABLED) && defined(HAL_ADC_MODULE_ENABLED)

#include <cstdint>

#include "voltbro/utils.hpp"
#include "voltbro/math/math_ops.hpp"
#include "voltbro/math/regulators/pid.hpp"
#include "voltbro/encoders/hall_sensor/hall_sensor.h"
#include "voltbro/devices/gpio.hpp"
#include "voltbro/devices/inverter.hpp"
#include "../motor_commons.hpp"

enum class SetPointType { VELOCITY, TORQUE, POSITION, VOLTAGE };

enum class DrivePhase: uint8_t { PHASE_A = 0, PHASE_B = 1, PHASE_C = 2 };

struct ControlConfig {
    bool predict_change = false;
    bool detect_stall = false;

    float sampling_interval;
    float stall_timeout;
    float stall_tolerance;

    SetPointType point_type;
    float target = 0;

    float current_limit;
    float user_current_limit = 0;
    float user_torque_limit = 0;

    float speed_mult = 1;
    float electric_mult = 1;
    float PWM_mult = 1;
    uint16_t max_PWM_per_s = 0;

    PIDRegulator main_regulator;
    PIDRegulator aux_regulator;
};

struct DriveInfo {
    const float torque_const;
    const float speed_const;
    float max_current;
    float max_torque;
    float stall_current;
    float stall_timeout;
    float stall_tolerance;
    const float supply_voltage;

    const GpioPin l_pins[3];
    const GpioPin en_pin;
    const CommonDriverConfig common;
};

/**
 * Base for BLDC motors. Controlls PWM, coil switching and current sensing
 */
class BLDCController: public AbstractMotor {
protected:
    DriveInfo drive_info;
    Inverter inverter;
    const int32_t full_pwm;
    TIM_HandleTypeDef* const htim;

    arm_atomic(float) shaft_angle;
    arm_atomic(float) shaft_velocity;
    arm_atomic(float) shaft_torque = 0;
    arm_atomic(float) voltage_target = 0;
    arm_atomic(bool) _is_on;
    arm_atomic(bool) is_stalling;
    uint16_t DQs[3] = {0, 0, 0};
public:
    BLDCController(
        float user_current_limit,
        DriveInfo&& drive_info,
        TIM_HandleTypeDef* htim,
        ADC_HandleTypeDef* hadc
    ):
        AbstractMotor(),
        drive_info(std::move(drive_info)),
        inverter(hadc),
        full_pwm(htim->Instance->ARR),
        htim(htim)
    {
        assert_param(user_current_limit <= drive_info.max_current && user_current_limit >= 0);
        if (is_close(user_current_limit, 0)) {
            user_current_limit = drive_info.max_current;
        }
        AbstractMotor::user_current_limit = current_limit = user_current_limit;
    }

    inline bool set_current_limit(float current_limit) {
        if (current_limit > drive_info.max_current || current_limit <= 0) {
            return false;
        }
        user_current_limit = current_limit;
        return true;
    }
    /*
    inline bool set_torque_limit(float torque_limit) {
        if (torque_limit > drive_info.max_torque || torque_limit <= 0) {
            return false;
        }
        control_config.user_torque_limit = torque_limit;
        return true;
    }
    inline bool set_angle_point(float angle) {
        if (angle < 0 || angle > pi2) {
            return false;
        }
        control_config.point_type = SetPointType::POSITION;
        control_config.target = angle;
        return true;
    }
    inline bool set_velocity_point(float velocity) {
        control_config.point_type = SetPointType::VELOCITY;
        control_config.target = velocity;
        return true;
    }
    inline bool set_torque_point(float torque) {
        control_config.point_type = SetPointType::TORQUE;
        control_config.target = torque;
        return true;
    }
    */
    virtual bool set_voltage_point(float voltage) {
        voltage_target = voltage;
        return true;
    }
    const DriveInfo& get_info() const {
        return drive_info;
    }
    const Inverter& get_inverter() const {
        return inverter;
    }
    bool is_on() const {
        return _is_on;
    }
    float get_angle() const {
        return shaft_angle;
    }
    float get_velocity() const {
        return shaft_velocity;
    }

    void detect_stall(double passed_time_abs);
    void quit_stall();
    HAL_StatusTypeDef init() override;
    HAL_StatusTypeDef stop() override;
    HAL_StatusTypeDef start() override;
    HAL_StatusTypeDef set_state(bool) override;

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

        drive_info.l_pins[to_underlying(off)].reset();
        DQs[to_underlying(off)] = 0;

        drive_info.l_pins[to_underlying(from)].set();
        DQs[to_underlying(from)] = actual_pwm;

        drive_info.l_pins[to_underlying(to)].set();
        DQs[to_underlying(to)] = 0;
    }

    __attribute__((always_inline)) float calc_elec_theta(float encoder_data, uint16_t pulses_per_pair) {
        float theta = pi2 * (mfmod(encoder_data, pulses_per_pair) / (float)pulses_per_pair) - PI;
        if (theta < 0) {
            theta += pi2;
        }
        return theta;
    }

    __attribute__((always_inline)) void set_pwm() {
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
