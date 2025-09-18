#pragma once

#if defined(STM32G4) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#if defined(HAL_TIM_MODULE_ENABLED) && defined(HAL_ADC_MODULE_ENABLED)

#include <cstdint>
#include <optional>
#include <array>

#include "voltbro/utils.hpp"
#include "voltbro/math/math_ops.hpp"
#include "voltbro/math/regulators/pid.hpp"
#include "voltbro/encoders/hall_sensor/hall_sensor.h"
#include "voltbro/devices/gpio.hpp"
#include "voltbro/devices/inverter.hpp"
#include "../motor_commons.hpp"

enum class SetPointType: uint8_t  {
    VELOCITY = 0,
    TORQUE = 1,
    POSITION = 2,
    VOLTAGE = 3,
    UNIVERSAL = 4
};

enum class DrivePhase: uint8_t { PHASE_A = 0, PHASE_B = 1, PHASE_C = 2 };

struct DriveInfo {
    const float torque_const;
    float max_current;
    float max_torque;
    float stall_current;
    float stall_timeout;
    float stall_tolerance;
    float calibration_voltage;

    const std::optional<std::array<GpioPin, 3>> l_pins = std::nullopt;
    const GpioPin en_pin;
    const CommonDriverConfig common;
};


/*
 * Base for BLDC motors. Controls PWM, stall detection, current sensing, limits
 */
class BLDCController: public AbstractMotor {
protected:
    const DriveInfo drive_info;
    BaseInverter& inverter;
    const int32_t full_pwm;
    TIM_HandleTypeDef* const htim;

    arm_atomic(SetPointType) point_type = SetPointType::VOLTAGE;
    arm_atomic(float) target = 0;

    arm_atomic(float) shaft_angle;
    arm_atomic(float) shaft_velocity;
    arm_atomic(float) shaft_torque = 0;
    arm_atomic(bool) _is_on;
    arm_atomic(bool) is_stalling;
    uint16_t DQs[3] = {0, 0, 0};
public:
    BLDCController(
        const DriveLimits& limits,
        const DriveInfo& drive_info,
        TIM_HandleTypeDef* htim,
        BaseInverter& inverter
    ):
        AbstractMotor(),
        drive_info(drive_info),
        inverter(inverter),
        full_pwm(htim->Instance->ARR),
        htim(htim)
    {
        // TODO: fix limits setup from config
        if (!set_limits(limits)) {
            exit(-1);
        }
    }

    virtual bool check_limits(const DriveLimits& limits) override {
        if (
            (limits.user_current_limit > drive_info.max_current) ||
            (limits.user_torque_limit > drive_info.max_torque)
        ) {
            return false;
        }
        return AbstractMotor::check_limits(limits);
    }
    virtual HAL_StatusTypeDef apply_limits() override {
        if (drive_limits.user_current_limit <= 0) {
            drive_limits.user_current_limit = drive_info.max_current;
        }
        if (drive_limits.user_torque_limit <= 0) {
            drive_limits.user_torque_limit = drive_info.max_torque;
        }
        return AbstractMotor::apply_limits();
    }

    virtual bool set_angle_point(float angle) {
        if (angle < 0 || angle > pi2) {
            return false;
        }
        point_type = SetPointType::POSITION;
        target = angle;
        return true;
    }
    virtual bool set_velocity_point(float velocity) {
        point_type = SetPointType::VELOCITY;
        target = velocity;
        return true;
    }
    virtual bool set_torque_point(float torque) {
        point_type = SetPointType::TORQUE;
        target = torque;
        return true;
    }
    virtual bool set_voltage_point(float voltage) {
        point_type = SetPointType::VOLTAGE;
        target = voltage;
        return true;
    }
    const DriveInfo& get_info() const {
        return drive_info;
    }
    const BaseInverter& get_inverter() const {
        return inverter;
    }
    bool is_on() const {
        return _is_on;
    }

    virtual float get_angle() const {
        return shaft_angle + drive_info.common.user_angle_offset;
    }
    virtual float get_velocity() const {
        return shaft_velocity;
    }
    virtual float get_voltage() const {
        return 0; // TODO
    }
    virtual float get_torque() const {
        return 0; // TODO
    }

    void detect_stall(double passed_time_abs);
    void quit_stall();
    HAL_StatusTypeDef init() override;
    HAL_StatusTypeDef stop() override;
    HAL_StatusTypeDef start() override;
    HAL_StatusTypeDef set_state(bool) override;

    virtual void set_pwm() {
        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, DQs[0]);
        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, DQs[1]);
        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, DQs[2]);
    }

    virtual ~BLDCController() = default;
};

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
