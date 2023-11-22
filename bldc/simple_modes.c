/*
 * simple_modes.c
 *
 *  Created on: June 6, 2023
 *      Author: Igor Beschastnov,
 */

#include "bldc.h"

#include "math/math_ops.h"
#include "math/transform.h"

volatile float calib_elec_angle = 0;

void current_mode(
    DriverControl* controller,
    DriveInfo* driver,
    InverterState* inverter,
    uint16_t* dqa,
    uint16_t* dqb,
    uint16_t* dqc
) {
    // float target_angle = 0.01f;  // actually, elec angle
    float target_angle = calib_elec_angle;
    float sf = arm_sin_f32(target_angle);
    float cf = arm_cos_f32(target_angle);

    // set the electrical angle to -pi, rotor should be pointing at +pi
    const float const_Vd = -0.5f;
    const float const_Vq = 0.0f;

    float DVA, DVB, DVC;

    // inverse dq0 transform on voltages
    abc(sf, cf, const_Vd, const_Vq, &DVA, &DVB, &DVC);

    *dqa = 1000 + (int16_t)(1000.0f * DVA);
    *dqb = 1000 + (int16_t)(1000.0f * DVB);
    *dqc = 1000 + (int16_t)(1000.0f * DVC);
}

void rotate_mode(
    DriverControl* controller,
    DriveInfo* driver,
    InverterState* inverter,
    uint16_t* dqa,
    uint16_t* dqb,
    uint16_t* dqc
) {
    float sf = arm_sin_f32(calib_elec_angle);
    float cf = arm_cos_f32(calib_elec_angle);

    const float const_Vd = -0.3f;
    const float const_Vq = 0.0f;

    float DVA, DVB, DVC;

    // inverse dq0 transform on voltages
    abc(sf, cf, const_Vd, const_Vq, &DVA, &DVB, &DVC);

    *dqa = 1000 + (int16_t)(1000.0f * DVA);
    *dqb = 1000 + (int16_t)(1000.0f * DVB);
    *dqc = 1000 + (int16_t)(1000.0f * DVC);
}


#define SEQUENCE_LEN 6*4
EncoderStep flows[SEQUENCE_LEN];
EncoderStep hall_sequence_array[SEQUENCE_LEN];

void hall_sequence(
    HallSensor* encoder,
    DriverControl* controller,
    DriveInfo* driver,
    InverterState* inverter,
    uint16_t* dqa,
    uint16_t* dqb,
    uint16_t* dqc
) {
    static bool is_finished = false;
    static uint32_t last_time_ms = 0;
    static EncoderStep target_step = 1;
    static size_t i = 0;
    static uint32_t calibration_pwm = 200;
    if (is_finished) {
        return;
    }
    uint32_t current_time_ms = HAL_GetTick();
    if (current_time_ms - last_time_ms > 1000) {
        if (last_time_ms == 0) {
            last_time_ms = current_time_ms;
            flows[i] = target_step;
        }
        else {
            hall_sequence_array[i] = encoder->step;
            i++;
            if (i >= SEQUENCE_LEN) {
                // Breakpoint location
                is_finished = true;
                *dqa = *dqb = *dqc = 0;
                calibration_pwm = 0;
            }
            target_step += 1;
            if (target_step > AC) {
                target_step = CA;
            }
            flows[i] = target_step;
            last_time_ms = current_time_ms;
        }
    }
    uint16_t* DQs[3] = {dqa, dqb, dqc};

    DrivePhase first, second;
    step_to_phases(target_step, &first, &second);
    flow_direction(driver, first, second, DQs, calibration_pwm);
}