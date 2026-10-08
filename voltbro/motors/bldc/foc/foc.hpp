#pragma once
#include <cstdint>
#include "voltbro/encoders/generic.h"
#if defined(STM32G4) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#if defined(HAL_TIM_MODULE_ENABLED) && defined(HAL_ADC_MODULE_ENABLED) && defined(HAL_CORDIC_MODULE_ENABLED)

#include <algorithm>
#include <array>
#include <cmath>
#include <string_view>
#include <utility>

#include "../bldc.h"
#include "servo_control.hpp"
#include "voltbro/profiling.hpp"
#include "voltbro/math/regulators/pid.hpp"

#define USE_CALIBRATION_ARRAY
inline constexpr float MAX_BOARD_CURRENT = 30.0f; // Board current ceiling, A.
constexpr size_t CALIBRATION_BUFF_SIZE = 2048;
using __non_const_calib_array_t = std::array<int, CALIBRATION_BUFF_SIZE>;
using calibration_array_t = const __non_const_calib_array_t;

struct __attribute__((packed)) CalibrationData {
    static constexpr uint32_t TYPE_ID = 0x89ABCDEF;
    bool was_calibrated;
    bool is_encoder_inverted;
    uint16_t ppair_counter;
    int meas_elec_offset;
    uint32_t type_id = 0;
#ifdef USE_CALIBRATION_ARRAY
    __non_const_calib_array_t calibration_array;
#endif

    CalibrationData() {
        reset();
        type_id = 0;  // to distinguish between uninitialized and reset data
    }

    void reset() {
        type_id = TYPE_ID;
        was_calibrated = false;
        is_encoder_inverted = false;
        ppair_counter = 0;
        meas_elec_offset = 0;
#ifdef USE_CALIBRATION_ARRAY
        calibration_array.fill(0);
#endif
    }
};

struct FOCTarget {
    float torque = 0.0f;
    float angle = 0.0f;
    float velocity = 0.0f;
    float angle_kp = 0.0f;
    float velocity_kp = 0.0f;
};

struct FiltersConfig {
    float expected_a;
    float g1;
    float g2;
    float g3;
    float I_lpf_coefficient;
};


/**
 * Field oriented control.
 */
class FOC: public BLDCController  {
protected:
    float T;
    ServoInputConfig servo_input_config;
    std::optional<ServoCommand> servo_command;
    std::optional<ServoTrajectoryStorage> servo_traj_generator;
    uint32_t control_tick = 0;
    uint32_t servo_reference_epoch = 0; // FOC tick represented by the current generator state, including its horizon.
    uint8_t servo_reference_ticks = 0;
    uint8_t servo_integral_ticks = 0;
    bool servo_reference_initialize = false;
    float raw_rotor_angle = 0; // Calibrated mechanical rotor angle, rad in [0, 2*pi).
    struct FilterState {
        float rotor_angle = 0.0f; // Posterior mechanical rotor angle, rad.
        float rotor_velocity = 0.0f; // Posterior rotor angular velocity, rad/s.
        float residual_acceleration = 0.0f; // Acceleration beyond expected_a, rad/s^2.
        bool initialized = false;
    } filter_state;
    float elec_angle = 0;
    float I_Q = 0;
    calibration_array_t* lookup_table = nullptr;
    GenericEncoder& encoder;
    const FiltersConfig filters_config;
    FOCTarget foc_target;
    PIDRegulator q_reg;
    PIDRegulator d_reg;
    PIDRegulator servo_pos_reg;
    PIDRegulator servo_vel_reg;

    void reset_control() override {
        servo_reference_epoch = 0;
        servo_reference_ticks = servo_integral_ticks = 0;
        servo_reference_initialize = false;
        servo_pos_reg.reset();
        servo_vel_reg.reset();
        foc_target = {};
    }

    float servo_torque();

    /** Continue the reference; on mode entry use measured position and a nearby reference velocity. */
    TrajectoryState servo_initial_state(float tolerance, bool continuing = false) {
        if (continuing && servo_traj_generator) return get_trajectory(*servo_traj_generator).get_state();
        TrajectoryState initial{get_angle(), get_velocity()};
        if (servo_traj_generator) {
            const float velocity = get_trajectory(*servo_traj_generator).get_velocity();
            if (std::fabs(velocity - initial.velocity) <= tolerance) initial.velocity = velocity;
        }
        return initial;
    }

    void apply_kalman();
    void update_angle();
    virtual void update_shaft_angle();

    void set_windings_calibration(float current_angle);
public:
    virtual void calibrate(CalibrationData& calibration_data, std::byte* additional_buffer, size_t buffer_size,
                           void (*progress)(int done, int total) = nullptr);
    void apply_calibration(CalibrationData& calibration_data) {
        // Replace config parameters with the ones loaded from EEPROM
        // Very ugly hack, but it will work for now
        const_cast<int&>(encoder.electric_offset) = calibration_data.meas_elec_offset;
#ifdef USE_CALIBRATION_ARRAY
        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Waddress-of-packed-member"
        // TODO: verify that this is safe, but it works so probably fine?
        lookup_table = &(calibration_data.calibration_array);
        #pragma GCC diagnostic pop
#endif
        const_cast<bool&>(encoder.is_inverted) = calibration_data.is_encoder_inverted;
        filter_state = {}; // Calibration changes the measurement reference frame.
    }

    FOC(
        float T,
        FiltersConfig&& filters_config_,
        PIDConfig&& q_config,
        PIDConfig&& d_config,
        const DriveRuntimeConfig& drive_runtime_config,
        const DriveInfo& drive_info,
        TIM_HandleTypeDef* htim,
        GenericEncoder& encoder,
        BaseInverter& inverter
    ):
        BLDCController(
            drive_runtime_config,
            drive_info,
            htim,
            inverter
        ),
        T(T),
        encoder(encoder),
        filters_config(filters_config_),
        q_reg(std::move(q_config)),
        d_reg(std::move(d_config))
        {}

    float get_electric_angle() {
        return elec_angle;
    }
    float get_working_current() {
        return I_Q;
    }

    bool set_foc_point(FOCTarget&& target) {
        if (
            !is_torque_target_valid(target.torque) ||
            !is_angle_target_valid(target.angle) ||
            !is_velocity_target_valid(target.velocity) ||
            !std::isfinite(target.angle_kp) || !std::isfinite(target.velocity_kp)
        ) {
            return false;
        }
        CRITICAL_SECTION({
            servo_traj_generator.reset();
            servo_command.reset();
            if (point_type != SetPointType::UNIVERSAL) reset_control();
            point_type = SetPointType::UNIVERSAL;
            foc_target = std::move(target);
        })
        return true;
    }
    void reset_servo_input() {
        CRITICAL_SECTION({
            servo_traj_generator.reset();
            servo_command.reset();
            servo_reference_ticks = servo_integral_ticks = 0;
            servo_reference_epoch = 0;
            servo_reference_initialize = false;
        })
    }
    /** Apply settings while retaining a continuing reference and its time axis. */
    [[gnu::noinline, gnu::optimize("Os")]] bool set_servo_input_config(ServoInputConfig config) {
        if (!std::isfinite(config.velocity_planning_tolerance) || config.velocity_planning_tolerance < 0) return false;
        std::optional<ServoCommand> command;
        TrajectoryState initial;
        uint32_t epoch;
        CRITICAL_SECTION({
            command = servo_command;
            initial = servo_initial_state(config.velocity_planning_tolerance, servo_traj_generator.has_value());
            epoch = servo_traj_generator ? servo_reference_epoch : control_tick;
        })
        auto next = command ? make_servo_trajectory(command->type, config) : std::nullopt;
        if (next && !get_trajectory(*next).start(initial, command->value)) return false;
        CRITICAL_SECTION({
            servo_input_config = config;
            if (next) {
                get_trajectory(*next).on_update(
                    servo_traj_generator ? &get_trajectory(*servo_traj_generator) : nullptr,
                    static_cast<float>(control_tick - epoch + servo_reference_ticks + 1U) * T);
                servo_traj_generator = std::move(next);
                servo_reference_epoch = control_tick + servo_reference_ticks + 1U;
            }
        })
        return true;
    }
    /** Snapshot state/time, prepare outside the critical section, then publish for the next reference tick. */
    [[gnu::noinline, gnu::optimize("Os")]] bool set_servo_command(uint8_t type, float value, bool indexed = false, uint8_t index = 0) {
        if (!std::isfinite(value)) return false;
        SetPointType controller_type;
        switch (type) {
            case VELOCITY_DIRECT: case VELOCITY_RAMP:
                if (!is_velocity_target_valid(value)) return false;
                controller_type = SetPointType::VELOCITY;
                break;
            case POSITION_DIRECT: case POSITION_FILTER: case POSITION_POLY:
                if (!is_angle_target_valid(value)) return false;
                controller_type = SetPointType::POSITION;
                break;
            case TORQUE_DIRECT:
                if (!is_torque_target_valid(value)) return false;
                controller_type = SetPointType::TORQUE;
                break;
            case VOLTAGE_DIRECT:
                controller_type = SetPointType::VOLTAGE;
                break;
            default: return false;
        }
        std::optional<ServoCommand> previous;
        ServoInputConfig config;
        TrajectoryState initial;
        uint32_t epoch;
        CRITICAL_SECTION({
            previous = servo_command;
            config = servo_input_config;
            const bool continuing = previous && previous->type == type && servo_traj_generator;
            initial = servo_initial_state(config.velocity_planning_tolerance, continuing);
            epoch = continuing ? servo_reference_epoch : control_tick;
        })
        if (previous) {
            const bool same_content = previous->type == type && previous->value == value;
            if (indexed && previous->has_index && previous->index == index) return same_content;
            if (!indexed && !previous->has_index && same_content) return true;
        }
        const bool changed_mode = !previous || previous->type != type;
        VB_PROFILE_BEGIN(plan_start)
        auto next = make_servo_trajectory(type, config);
        if (next && !get_trajectory(*next).start(initial, value)) return false;
        VB_PROFILE_RECORD_IF(type == POSITION_POLY, servo_schedule_profile.plans,
                             servo_schedule_profile.plan_max, plan_start)
        CRITICAL_SECTION({
            if (point_type != controller_type) reset_control();
            if (changed_mode) servo_reference_ticks = 0;
            if (next) {
                get_trajectory(*next).on_update(
                    !changed_mode && servo_traj_generator ? &get_trajectory(*servo_traj_generator) : nullptr,
                    static_cast<float>(control_tick - epoch + servo_reference_ticks + 1U) * T);
            }
            servo_traj_generator = std::move(next);
            if (servo_traj_generator) servo_reference_epoch = control_tick + servo_reference_ticks + 1U;
            if (changed_mode) servo_reference_initialize = type == VELOCITY_RAMP;
            servo_command = (ServoCommand{type, value, indexed, index});
            point_type = controller_type;
            if (controller_type == SetPointType::TORQUE || controller_type == SetPointType::VOLTAGE) {
                target = value * get_direction_multiplier();
            } else if (changed_mode || type == POSITION_DIRECT || type == VELOCITY_DIRECT) {
                target = value;
                if (servo_traj_generator) {
                    const auto state = get_trajectory(*servo_traj_generator).get_state();
                    target = controller_type == SetPointType::VELOCITY ? state.velocity : state.position;
                }
            }
        })
        return true;
    }
    PIDConfig get_servo_config(SetPointType type) const {
        return (type == SetPointType::POSITION ? servo_pos_reg : servo_vel_reg).get_config();
    }
    void update_servo_config(SetPointType type, PIDConfig config) {
        auto& regulator = type == SetPointType::POSITION ? servo_pos_reg : servo_vel_reg;
        CRITICAL_SECTION({
            const auto active = regulator.get_config();
            if (active.kp != config.kp || active.ki != config.ki || active.kd != config.kd) {
                regulator.update_config(config.kp, config.ki, config.kd);
                regulator.reset();
                if (point_type == type) servo_integral_ticks = 0;
            }
        })
    }
    void update_q_config(PIDConfig&& new_config) {
        q_reg.update_config(std::move(new_config));
    }
    void update_d_config(PIDConfig&& new_config) {
        d_reg.update_config(std::move(new_config));
    }
    void update_control_config(PIDConfig&& new_config) {
        update_servo_config(SetPointType::POSITION, new_config);
        update_servo_config(SetPointType::VELOCITY, new_config);
    }
    const GenericEncoder& get_encoder() const {
        return encoder;
    }

    HAL_StatusTypeDef init() override {
        HAL_StatusTypeDef result = BLDCController::init();
        if (result != HAL_OK) {
            return result;
        }

        return encoder.init();
    }
    void update() override;
    virtual void update_sensors();
};

#endif
#endif
