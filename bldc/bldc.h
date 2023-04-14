/*
 * bldc.h
 *
 *  Created on: Jan 24, 2023
 *      Author: Igor Beschastnov
 */

#ifndef VBLIB_BLDC_BLDC_H_
#define VBLIB_BLDC_BLDC_H_

#ifdef STM32_G
#include "stm32g4xx_hal.h"
#else
#include "stm32f4xx_hal.h"
#endif

#ifdef HAL_TIM_MODULE_ENABLED

#include "math/math_ops.h"

#ifdef __cplusplus
#include <cstdint>
extern "C" {
#else
#include <stdbool.h>
#include <stdint.h>
#endif

#include "controllers/pid.h"
#include "encoders/generic.h"
#include "utils.h"

#define FOC 0
#define CLBR 1
#define ROTATE 2
#define CURRENT 3
#define CURRENT_STEP 4
#define STUPID_CONTROL 5
#define SIX_STEP_CONTROL 6
#define NO_ACTION 7
#define TESTS

typedef struct {
    bool is_on;
    bool predict_change;
    uint8_t State;  // Device state such as FOC, Calibration etc*/
    float encoder_filtering;
    float speed_filtering;
    float sampling_interval;
    uint16_t pulses_per_pair;

    float ShaftAngle;     // device's output shaft mechanical angle, rad
    float ShaftVelocity;  // device's output shaft angular velocity, rad/s
    int32_t RotorTurns;   // motor's rotor rotation number, int
    float ElecTheta;      // motor's electrical angle, electrical rad

    // dynamically set FOC setpoints and gains
    float mech_SetP;
    float mech_gain;
    float vel_SetP;
    float vel_gain;
    float torq_SetP;

    const float T;  // encoder value sampling period, s
    // motor physical properties
    const uint32_t gear_ratio;
    const uint32_t ppairs;
    const float torque_const;
    float max_torque;
} DriverState;

typedef struct {
    float I_A;
    float I_B;
    float I_C;
    float I_D;
    float I_Q;
    float busV;
} InverterState;

void CalculateAnglesSimple(
    DriverState* Driver,
    GEncoder* encoder,
    PIDConfig* pid
);
void CalculateAngles(DriverState* Driver, GEncoder* encoder);
void ProcessADC(InverterState* inverter, const uint32_t ADC_buf[]);
void motor_control(
    DriverState* driver,
    InverterState* inverter,
    PIDConfig* pid,
    const uint32_t ADC_buf[],
    GEncoder* encoder,
    TIM_HandleTypeDef* htim
);

#define pi2 (2.0f * PI)

force_inline float
calc_elec_theta(float encoder_data, uint16_t pulses_per_pair) {
    float theta =
        pi2 * (mfmod(encoder_data, pulses_per_pair) / (float)pulses_per_pair) -
        PI;
    if (theta < 0) {
        theta += pi2;
    }
    return theta;
}

force_inline float lp_filter(float beta, float yprev, float input) {
    return (float)((1.0f - beta) * yprev + beta * input);
}

#ifdef __cplusplus
}
#endif
#endif
#endif /* VBLIB_BLDC_BLDC_H_ */
