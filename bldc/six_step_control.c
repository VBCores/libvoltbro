/*
 * bldc.c
 *
 *  Created on: Mar 23, 2023
 *      Author: Igor Beschastnov,
 */
#include "bldc.h"

#include "arm_math.h"

#include "encoders/AS5048A/AS5048A.h"
#include "encoders/incremental_encoder/encoder.h"
#include "math/transform.h"

extern AS5048AConfig as5048a_config;

extern void Error_Handler();

static uint32_t ticks_since_sample_abs = 0;

static EncoderStep last_step = -1;
static uint32_t ticks_since_change_incr = 0;
static float elec_speed = 0.5f;
static float elec_theta_obs;

force_inline void
observer(DriverState* driver, const IncrementalEncoder* encoder, float dt) {
    elec_theta_obs = driver->ElecTheta + elec_speed * dt * encoder->direction;
    if (elec_theta_obs > pi2) {
        elec_theta_obs -= pi2;
    }
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
    const IncrementalEncoder* inc_encoder = (IncrementalEncoder*)encoder;
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
        as5048a_config.common.value =
        as5048a_config.common.get_angle((GEncoder*)&as5048a_config);
    const uint16_t abs_value = as5048a_config.common.value;
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
            double regl = regulation(pid, speed_error, passed_time_abs);
            if (fabs(regl) > driver->vel_SetP) {
                regl = copysign(driver->vel_SetP, regl);  // TODO: use??
            }
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
            /**/
            // Vd = -0.5f;
            if (fabsf(Vd) > 1.0f) {
                Vd = copysign(1.0f, Vd);
            }
        }
    } else {
        ticks_since_sample_abs += 1;
    }
#endif

    // ELECTRIC ANGLE
    const float step_size = pi2 / 6;

    ticks_since_change_incr += 1;
    float passed_time_incr = (float)ticks_since_change_incr * driver->T;
    if (step == last_step) {
        if (driver->predict_change) {
            observer(driver, inc_encoder, passed_time_incr);
        }
    } else {
        if (last_step != (EncoderStep)-1) {
            //elec_speed = step_size / passed_time_incr;
            elec_speed = (driver->vel_SetP / driver->gear_ratio) / driver->ppairs;
        }
        driver->ElecTheta = (float)(step * step_size);
        last_step = step;
        ticks_since_change_incr = 0;
    }

    // DRIVER CONTROL
    // float target_angle = driver->ElecTheta + pi2 * (90.0f / 360.0f);
    float target_angle = elec_theta_obs;// + pi2 * (90.0f / 360.0f);

    float sf = arm_sin_f32(target_angle);
    float cf = arm_cos_f32(target_angle);

    float v_a = 0;
    float v_b = 0;
    float v_c = 0;

    abc(sf, cf, 0, Vd, &v_a, &v_b, &v_c);

    *DQA = 1000 + (int16_t)(1000.0f * v_a);
    *DQB = 1000 + (int16_t)(1000.0f * v_b);
    *DQC = 1000 + (int16_t)(1000.0f * v_c);
}