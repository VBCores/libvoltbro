/*
 * stupid_control.c
 *
 *  Created on: Jan 24, 2023
 *      Author: Igor Beschastnov
 */

#include "bldc.h"
#include "math.h"
#include "math/math_ops.h"
#include "math/transform.h"

extern float calib_elec_angle;

#ifdef DEBUG
static volatile float const_Vq;
#endif
void stupid_control(DriverState* driver, InverterState* inverter, PIDConfig* pid, uint16_t* dqa, uint16_t* dqb, uint16_t* dqc) {
    float s = arm_sin_f32(driver->ElecTheta);
    float c = arm_cos_f32(driver->ElecTheta);

    const float const_Vd = 0.0f;
#ifndef DEBUG
    const float const_Vq;
#endif
    /*
    double vq = -pid->signal;
    if (fabs(vq) > 1) {
        vq = copysign(1, vq);
    }
    const_Vq = (float)vq;/**/
    const_Vq = -0.4f;

    float DVA, DVB, DVC;

    abc(s, c, const_Vd, const_Vq, &DVA, &DVB, &DVC); //inverse dq0 transform on voltages

    *dqa = 1000 + (int16_t)(1000.0f*DVA);
    *dqb = 1000 + (int16_t)(1000.0f*DVB);
    *dqc = 1000 + (int16_t)(1000.0f*DVC);
}
