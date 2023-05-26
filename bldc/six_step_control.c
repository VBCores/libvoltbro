/*
 * bldc.c
 *
 *  Created on: Mar 23, 2023
 *      Author: Igor Beschastnov,
 */
#include "bldc.h"
#include "six_step_control.h"

#include "arm_math.h"

#include "encoders/AS5048A/AS5048A.h"
#include "encoders/incremental_encoder/encoder.h"
#include "math/transform.h"

extern AS5048AConfig as5048a_config;
extern void Error_Handler();

const float step_size = pi2 / 6;

#ifdef REPORT_STATE
SixStepState SSState;
#endif

#ifdef DEBUG
static float elec_speed;
static float elec_predicted;
#endif
force_inline float observer(DriverState* driver, const IncrementalEncoder* encoder, float dt) {
#ifndef DEBUG
    float elec_speed;
#endif
    elec_speed = driver->vel_SetP * driver->gear_ratio * driver->ppairs;
    float value = driver->ElecTheta + elec_speed * dt;
    if (value > pi2) {
        value -= pi2;
    }
#ifdef REPORT_STATE
    SSState.elec_speed = elec_speed;
    SSState.elec_predicted = value;
#endif

    return value;
}

// very stupid
force_inline float speed_to_voltage(float speed) {
    return (speed + copysignf(1.0f, speed)) / 16.5f;
}

#ifdef DEBUG
uint16_t filtered_data;
int abs_diff;
float speed_error;
float Vd;
double pid_signal;
#endif
#define ABS_ANGLES
void six_step_control(
    DriverState* driver,
    GEncoder* encoder,
    InverterState* inverter,
    PIDConfig* pid,
    uint16_t* DQA,
    uint16_t* DQB,
    uint16_t* DQC
) {
    if (!driver->is_on) {
        return;
    }
    static EncoderStep last_step = -1;

    IncrementalEncoder* inc_encoder = (IncrementalEncoder*)encoder;
    calc_encoder_step(inc_encoder);
    const EncoderStep step = inc_encoder->step;

    // Guards
    if (inc_encoder->step < 0) {
        return;
    }

#ifndef DEBUG
    float speed_error
#endif
    // SPEED AND SHAFT ANGLE
#ifdef ABS_ANGLES
    as5048a_config.common.value = as5048a_config.common.get_angle((GEncoder*)&as5048a_config);
    const uint16_t abs_value = as5048a_config.common.value;

    static uint32_t ticks_since_sample_abs = 0;
    float passed_time_abs = (float)ticks_since_sample_abs * driver->T;
    if (passed_time_abs > driver->sampling_interval) {
        ticks_since_sample_abs = 0;

        static uint16_t prev_abs_value = -1;  // initial bigger than CPR
        int32_t aligned_encoder_data =
            abs_value - as5048a_config.common.elec_offset;
        if (aligned_encoder_data < 0) {
            aligned_encoder_data += as5048a_config.common.CPR;
        }
        if (prev_abs_value > as5048a_config.common.CPR) {
            prev_abs_value = aligned_encoder_data;
        }
#ifndef DEBUG
        uint16_t filtered_data;
#endif
        const uint16_t half_cpr = as5048a_config.common.CPR / 2;
        if (abs(prev_abs_value - aligned_encoder_data) > half_cpr) {
            prev_abs_value = aligned_encoder_data;
        } else {
            filtered_data = (uint16_t)lp_filter(
                driver->encoder_filtering,
                (float)prev_abs_value,
                (float)aligned_encoder_data
            );
#ifndef DEBUG
            int abs_diff;
#endif
            abs_diff = filtered_data - prev_abs_value;
            if (abs_diff < -half_cpr) {
                abs_diff += as5048a_config.common.CPR;
            } else if (abs_diff > half_cpr) {
                abs_diff -= as5048a_config.common.CPR;
            }
            prev_abs_value = filtered_data;

            driver->ShaftAngle =
                pi2 * ((float)filtered_data / (float)as5048a_config.common.CPR);

            static float prev_velocity = 0;
            const float travel =
                pi2 * ((float)abs_diff / (float)as5048a_config.common.CPR);
            float new_speed = travel / driver->sampling_interval;
            driver->ShaftVelocity =
                lp_filter(driver->speed_filtering, prev_velocity, new_speed);
            prev_velocity = driver->ShaftVelocity;

            // PID
            speed_error = driver->vel_SetP - driver->ShaftVelocity;
            regulation(pid, speed_error, passed_time_abs);
#ifndef DEBUG
            double pid_signal;
#endif
            //pid_signal = speed_to_voltage(pid->signal);
            pid_signal = pid->signal / 20; // amplifier

            // 2v/s max
            const float max_v_per_sample = 2.0f * driver->sampling_interval;
            if (fabs(pid_signal) > max_v_per_sample) {
                pid_signal = copysign(max_v_per_sample, pid_signal);
            }
#ifndef DEBUG
            static float Vd = 0
#endif
            Vd -= pid_signal;
            if (fabsf(Vd) > driver->max_V) {
                Vd = copysign(driver->max_V, Vd);
            }
#ifdef REPORT_STATE
            SSState.Vd = Vd;
#endif
        }
    } else {
        ticks_since_sample_abs += 1;
    }
#endif

    // ELECTRIC ANGLE
    static uint32_t ticks_since_change_incr = 0;
    ticks_since_change_incr += 1;
    float passed_time_incr = (float)ticks_since_change_incr * driver->T;
    //const double time_for_step = (pi2 / driver->vel_SetP) / (double)driver->gear_ratio / (double)driver->ppairs / 6.0;
#ifndef DEBUG
    float elec_predicted;
#endif
    if (step == last_step) {
        if (driver->predict_change) {
            elec_predicted = observer(driver, inc_encoder, passed_time_incr);
        }
    } else {
        driver->ElecTheta = (float)(step * step_size);
        elec_predicted = driver->ElecTheta;

        last_step = step;
        ticks_since_change_incr = 0;
    }

    // DRIVER CONTROL
    float elec_angle;
    if (driver->predict_change) {
        elec_angle = elec_predicted;
    }
    else {
        elec_angle = driver->ElecTheta;
    }

    float sf = arm_sin_f32(elec_angle);
    float cf = arm_cos_f32(elec_angle);

    float v_a = 0;
    float v_b = 0;
    float v_c = 0;

    abc(sf, cf, 0, Vd, &v_a, &v_b, &v_c);

    *DQA = 1000 + (int16_t)(1000.0f * v_a);
    *DQB = 1000 + (int16_t)(1000.0f * v_b);
    *DQC = 1000 + (int16_t)(1000.0f * v_c);
}