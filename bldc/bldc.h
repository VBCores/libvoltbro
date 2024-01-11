/*
 * bldc.h
 *
 *  Created on: Jan 24, 2023
 *      Author: Igor Beschastnov
 */

#ifndef VBLIB_BLDC_BLDC_H_
#define VBLIB_BLDC_BLDC_H_

#if defined(STM32G474xx) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#else
#include "stm32f4xx_hal.h"
#endif

#if defined(HAL_TIM_MODULE_ENABLED) && defined(ARM_MATH)
#define BLDC_ENABLED
#endif
#ifdef BLDC_ENABLED

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

typedef enum { CALIBRATE, ROTATE, CURRENT, SIX_STEP_CONTROL, NO_ACTION } ControlMode;

typedef struct {
    bool is_on;
    bool is_stalling;

    uint16_t pulses_per_pair;

    float shaft_angle;     // device's output shaft mechanical angle, rad
    float shaft_velocity;  // device's output shaft angular velocity, rad/s
    int32_t rotor_turns;   // motor's rotor rotation number, int
    float elec_theta;      // motor's electrical angle, electrical rad

    // motor physical properties
    const uint32_t gear_ratio;
    const uint32_t ppairs;
    float torque_const;
    float speed_const;
    float max_current;
    float stall_current;
} DriveInfo;

typedef struct {
    bool predict_change;
    bool detect_stall;
    ControlMode mode;

    float encoder_filtering;
    float speed_filtering;
    float sampling_interval;
    float stall_timeout;
    float stall_tolerance;

    float velocity_target;
    float current_limit;
    float user_current_limit;

    float speed_mult;
    float I_mult;
    float PWM_mult;
    uint16_t max_PWM_per_s;

    const double T;  // encoder value sampling period, s

    PIDConfig velocity_regulator;
    PIDConfig current_regulator;
} DriverControl;

typedef struct {
    float I_A;
    float I_B;
    float I_C;
    float I_D;
    float I_Q;
    float busV;

    float I_A_offset;
    float I_B_offset;
    float I_C_offset;
} InverterState;

void motor_control(
    DriverControl* controller,
    DriveInfo* drive,
    InverterState* inverter,
    const uint32_t ADC_buf[],
    GEncoder* encoder,
    TIM_HandleTypeDef* htim
);

// process_ADC.c
void process_ADC(InverterState* inverter, const uint32_t ADC_buf[]);

#define CONTROL_FUNC_ARGS                                                                \
    DriverControl *controller, DriveInfo *drive, InverterState *inverter, uint16_t *dqa, \
        uint16_t *dqb, uint16_t *dqc
// six_step_control.c
void six_step_control(GEncoder* encoder, CONTROL_FUNC_ARGS);
void quit_stall(DriveInfo* drive, DriverControl* controller);
// simple_modes.c
void current_mode(CONTROL_FUNC_ARGS);
void rotate_mode(CONTROL_FUNC_ARGS);

#define pi2 (2.0f * PI)

force_inline float calc_elec_theta(float encoder_data, uint16_t pulses_per_pair) {
    float theta = pi2 * (mfmod(encoder_data, pulses_per_pair) / (float)pulses_per_pair) - PI;
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
