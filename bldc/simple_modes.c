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

    *dqa = 1000 + (int16_t) (1000.0f * DVA);
    *dqb = 1000 + (int16_t) (1000.0f * DVB);
    *dqc = 1000 + (int16_t) (1000.0f * DVC);
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

    *dqa = 1000 + (int16_t) (1000.0f * DVA);
    *dqb = 1000 + (int16_t) (1000.0f * DVB);
    *dqc = 1000 + (int16_t) (1000.0f * DVC);
}