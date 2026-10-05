#if defined(STM32G4) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#if defined(HAL_TIM_MODULE_ENABLED) && defined(HAL_ADC_MODULE_ENABLED) && defined(HAL_CORDIC_MODULE_ENABLED)

#include "foc.hpp"

#include "arm_math.h"
#include "stm32g4xx_ll_cordic.h"

#include "voltbro/math/transform.hpp"

void FOC::update_angle() {
    encoder.update_value();

    #ifndef MONITOR
    encoder_data raw_value;
    #endif
    raw_value = encoder.get_value();
    int offset_value = (int)raw_value - encoder.electric_offset;
    if (lookup_table != nullptr) {
        offset_value -= (*lookup_table)[raw_value >> 3];
    }
    static const float cpr_offset = (float)encoder.CPR / (2.0f * drive_info.common.ppairs);
    offset_value -= cpr_offset;
    if(offset_value > (encoder.CPR - 1)) {
        offset_value -= encoder.CPR;
    }
    else if( offset_value < 0 ) {
        offset_value += encoder.CPR;
    }

    raw_rotor_angle = offset_value * (pi2 / (float)encoder.CPR);
}

/** Estimate rotor motion with a fixed-gain constant-acceleration observer.
 * State is posterior to the previous measurement. Predict to this sample,
 * then correct with its shortest angular innovation. Gains g1/g2/g3 have
 * units 1, 1/s, 1/s^2. expected_a is a known acceleration input; the state
 * estimates its residual. T is the sampling period in seconds.
 * Publish the current-sample prior, preserving the existing output convention;
 * the correction contributes to the next prediction, not a future PWM horizon.
 * Model reference: Bellini, Bifaretti, Costantini, "A digital speed filter for
 * motion control drives with a low resolution position encoder", 2003.
 */
void FOC::apply_kalman() {
    VB_PROFILE_DETAIL_BEGIN()
    // Seed the absolute angle from the first sample; assume rest until motion is observed.
    float predicted_angle = raw_rotor_angle; // Current-sample prior rotor angle, rad.
    float predicted_velocity = 0.0f; // Current-sample prior rotor velocity, rad/s.
    if (!filter_state.initialized) {
        filter_state.rotor_angle = raw_rotor_angle;
        filter_state.rotor_velocity = 0.0f;
        filter_state.residual_acceleration = 0.0f;
        filter_state.initialized = true;
    } else {
        // Predict from the previous posterior under constant total acceleration.
        const float acceleration = filter_state.residual_acceleration + filters_config.expected_a;
        predicted_angle = filter_state.rotor_angle + filter_state.rotor_velocity * T +
                          acceleration * (T * T) / 2.0f;
        predicted_velocity = filter_state.rotor_velocity + acceleration * T;
        predicted_angle = mfmod(predicted_angle, pi2);
        if (predicted_angle < 0.0f) predicted_angle += pi2;

        // Update using this sample's prediction.
        // The nearest angular branch assumes prediction error is less than pi radians.
        float innovation = raw_rotor_angle - predicted_angle; // Wrapped angle residual, rad.
        if (innovation < -PI) innovation += pi2;
        else if (innovation > PI) innovation -= pi2;
    VB_PROFILE_DETAIL_SPLIT(kalman_profile.start)
        filter_state.rotor_angle = predicted_angle + filters_config.g1 * innovation;
        filter_state.rotor_velocity = predicted_velocity + filters_config.g2 * innovation;
        filter_state.residual_acceleration += filters_config.g3 * innovation;
    }
    VB_PROFILE_DETAIL_SPLIT(kalman_profile.mid)

    // Convert rotor mechanical radians to electrical phase and output-shaft velocity.
    const float electrical_period = pi2 / static_cast<float>(drive_info.common.ppairs);
    elec_angle = drive_info.common.ppairs * mfmod(predicted_angle, electrical_period);
    shaft_velocity = predicted_velocity / drive_info.common.gear_ratio;
    VB_PROFILE_DETAIL_END(kalman_profile.end, t_start)
    VB_PROFILE_DETAIL_END(kalman_profile.total, start_total)
}

void FOC::update_shaft_angle() {
    static float prev_rotor_angle = raw_rotor_angle;
    static int32_t rotor_turns = 0;
    float travel = raw_rotor_angle - prev_rotor_angle;
    prev_rotor_angle = raw_rotor_angle;
    if (travel < -PI) {
        rotor_turns += 1;
    } else if (travel > PI) {
        rotor_turns -= 1;
    }
    float rotor_angle_unwrapped = (float)rotor_turns * pi2 + raw_rotor_angle;
    shaft_angle = rotor_angle_unwrapped / drive_info.common.gear_ratio;
}

void FOC::update_sensors() {
    VB_PROFILE_DETAIL_BEGIN()
    inverter.update();
    VB_PROFILE_DETAIL_SPLIT(sensors_profile.inverter)
    update_angle();
    VB_PROFILE_DETAIL_SPLIT(sensors_profile.angle)
    apply_kalman();
    update_shaft_angle();
    VB_PROFILE_DETAIL_END(sensors_profile.kalman, t_start)
    VB_PROFILE_DETAIL_END(sensors_profile.total, start_total)
}


/** Compute motor-side torque using position PID or velocity PI with clamping anti-windup. */
float FOC::servo_torque() {
    VB_PROFILE_COUNT(servo_schedule_profile.calls)
    const bool position = point_type == SetPointType::POSITION;
    auto& regulator = position ? servo_pos_reg : servo_vel_reg;
    if (servo_traj_generator) {
        if (servo_reference_ticks == 0) {
            VB_PROFILE_BEGIN(reference_start)
            VB_PROFILE_COUNT(servo_schedule_profile.references)
            auto& generator = get_trajectory(*servo_traj_generator);
            if (servo_reference_initialize) {
                generator.on_activate({get_angle(), get_velocity()});
            }
            target = generator.step(8 * T);
            servo_reference_initialize = false;
            servo_reference_ticks = 8;
            VB_PROFILE_END(servo_schedule_profile.reference_max, reference_start)
        }
        --servo_reference_ticks;
    }
    const bool update_integral = ++servo_integral_ticks == 5;
    if (update_integral) servo_integral_ticks = 0;
    const float error = target - (position ? get_angle() : get_velocity());

    // Respect both torque limits and the current actually available, including stall derating.
    const float gear_ratio_f = static_cast<float>(drive_info.common.gear_ratio);
    const float torque_per_amp = drive_info.torque_const;
    float limit = std::min(drive_info.max_torque / gear_ratio_f, 30.0f * torque_per_amp);
    // Compare all bounds in motor-torque units, without converting current
    // to output torque and back through the gear ratio on every tick.
    if (is_symmetric_limit_set(drive_runtime_config.user_torque_limit)) {
        limit = std::min(limit, drive_runtime_config.user_torque_limit / gear_ratio_f);
    }
    if (is_symmetric_limit_set(drive_runtime_config.user_current_limit)) {
        limit = std::min(limit, drive_runtime_config.user_current_limit * torque_per_amp);
    }
    if (is_symmetric_limit_set(drive_runtime_config.current_limit)) {
        limit = std::min(limit, drive_runtime_config.current_limit * torque_per_amp);
    }

    // D acts on measured velocity, so a position-target step has no derivative kick.
    VB_PROFILE_BEGIN(integral_start)
    const float response = regulator.regulation_with_derivative(error, T, -limit, limit,
                                                 position ? -get_velocity() : 0.0f, update_integral);
    VB_PROFILE_RECORD_IF(update_integral, servo_schedule_profile.integrals,
                           servo_schedule_profile.integral_max, integral_start)
    return response;
}

void FOC::update() {
    ++control_tick;
    VB_PROFILE_DETAIL_BEGIN()
    update_sensors();
    VB_PROFILE_DETAIL_END(foc_profile.sensors, t_start)

    // calculate sin and cos of electrical angle with the help of CORDIC.
    // convert electrical angle from float to q31. electrical theta should be [-pi, pi]
    int32_t ElecTheta_q31 = (int32_t)((elec_angle / PI - 1.0f) * 2147483648.0f);
    // load angle value into CORDIC. Input value is in PIs!
    LL_CORDIC_WriteData(CORDIC, ElecTheta_q31);

    // the values are negative to level out [-pi, pi] representation of electrical angle at the CORDIC input
    struct {
        int32_t cosOutput = -(int32_t)LL_CORDIC_ReadData(CORDIC);  // Read cosine
        int32_t sinOutput = -(int32_t)LL_CORDIC_ReadData(CORDIC);  // Read sine
    } elec_angles_q31;
    struct {
        float c;
        float s;
    } elec_angles;
    //arm_q31_to_float(&elec_angles_q31.cosOutput, &elec_angles.c, 2);

    elec_angles.c = (float32_t)elec_angles_q31.cosOutput / 2147483648.0f;  // convert to float from q31
    elec_angles.s = (float32_t)elec_angles_q31.sinOutput / 2147483648.0f;  // convert to float from q31

    #ifndef MONITOR
    float V_d, V_q;
    static float I_D = 0;
    #endif
    VB_PROFILE_DETAIL_START()
    // LPF for motor current
    float tempD, tempQ;
    // dq0 transform on currents
    dq0(elec_angles.s, elec_angles.c, inverter.get_A(), inverter.get_B(), inverter.get_C(), &tempD, &tempQ);
    const float diff_D = I_D - tempD;
    const float diff_Q = I_Q - tempQ;

    I_D = I_D - (filters_config.I_lpf_coefficient * diff_D);
    I_Q = I_Q - (filters_config.I_lpf_coefficient * diff_Q);
    VB_PROFILE_DETAIL_END(foc_profile.currents, t_start)

    const float gear_ratio_f = static_cast<float>(drive_info.common.gear_ratio);
    const float busV = inverter.get_busV();

    shaft_torque = I_Q * drive_info.torque_const * gear_ratio_f;

    if (point_type == SetPointType::VOLTAGE) {
        V_d = 0;
        V_q = target;
    }
    else {
        #ifndef MONITOR
        float i_d_error, i_q_error, d_response, q_response, i_q_set;
        #endif

        i_d_error = -I_D;
        d_response = d_reg.regulation(i_d_error, T, busV);
        V_d = std::clamp(d_response, -busV, busV);

        i_q_set = 0.0f;
    VB_PROFILE_DETAIL_START()
        if (point_type == SetPointType::UNIVERSAL) {
            #ifdef MONITOR
            value_foc_p = foc_target.angle;
            value_foc_v = foc_target.velocity;
            value_foc_p_kp = foc_target.angle_kp;
            value_foc_v_kp = foc_target.velocity_kp;
            value_foc_t = foc_target.torque;
            #endif
            i_q_set = get_direction_multiplier() / drive_info.torque_const * (
                foc_target.angle_kp * (foc_target.angle - get_angle()) +
                foc_target.velocity_kp * (foc_target.velocity - get_velocity()) +
                (foc_target.torque / gear_ratio_f)
            );
        }
        else if (point_type == SetPointType::TORQUE) {
            i_q_set = target / drive_info.torque_const / gear_ratio_f;
        }
        else {
            const float controller_response = servo_torque();
            #ifdef MONITOR
            control_error_glob = target - (point_type == SetPointType::POSITION ? get_angle() : get_velocity());
            controller_response_glob = controller_response;
            #endif
            i_q_set = controller_response * get_direction_multiplier() / drive_info.torque_const;
        }

        const float abs_max_current_from_torque = (drive_info.max_torque / drive_info.torque_const / gear_ratio_f);
        if (fabs(i_q_set) > fabs(abs_max_current_from_torque)) {
            i_q_set = copysign(abs_max_current_from_torque, i_q_set);
        }
        if (
            drive_runtime_config.current_limit > 0.0f &&
            (fabs(i_q_set) > fabs(drive_runtime_config.current_limit))
        ) {
            i_q_set = copysign(drive_runtime_config.current_limit, i_q_set);
        }
        // absolute limit on currents defined by the hardware safe operation region
        if (fabs(i_q_set) > 30.0f) {
            i_q_set = copysign(30.0f, i_q_set);
        }

        i_q_error = i_q_set - I_Q;
        q_response = q_reg.regulation(i_q_error, T, busV);
        V_q = q_response;
    VB_PROFILE_DETAIL_END(foc_profile.outer_loop, t_start)
    }

    limit_norm(&V_d, &V_q, busV);
    VB_PROFILE_DETAIL_START()
    float v_u = 0, v_v = 0, v_w = 0;
    float dtc_u = 0, dtc_v = 0, dtc_w = 0;

    // inverse dq0 transform on voltages
    abc(elec_angles.s, elec_angles.c, V_d, V_q, &v_u, &v_v, &v_w);
    // space vector modulation
    svm(busV, v_u, v_v, v_w, &dtc_u, &dtc_v, &dtc_w);

    DQs[0] = (uint16_t)(float(full_pwm + 1) * dtc_u);
    DQs[1] = (uint16_t)(float(full_pwm + 1) * dtc_v);
    DQs[2] = (uint16_t)(float(full_pwm + 1) * dtc_w);

    set_pwm();
    VB_PROFILE_DETAIL_END(foc_profile.pwm, t_start)
    VB_PROFILE_DETAIL_END(foc_profile.total, start_total)
}


#endif
#endif
