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
#include "encoders/hall_sensor/encoder.h"
#include "utils.h"

typedef enum { HALL_SEQUENCE, ROTATE, CURRENT, SIX_STEP_CONTROL, HALL_SIX_STEP_CONTROL, NO_ACTION } ControlMode;
typedef enum { SET_VELOCITY, SET_VOLTAGE } SetPointType;

typedef struct {
    bool is_on;
    bool is_stalling;

    uint16_t pulses_per_pair;

    float shaft_angle;     // device's output shaft mechanical angle, rad
    float shaft_velocity;  // device's output shaft angular velocity, rad/s
    int32_t rotor_turns;   // motor's rotor revolutions number, int
    float elec_theta;      // motor's electrical angle, electrical rad

    // motor physical properties
    const uint32_t gear_ratio;
    const uint32_t ppairs;
    const float torque_const;
    const float speed_const;
    float max_current;
    float stall_current;
    const float supply_voltage;
    const int32_t full_pwm;

    const pin L_PINS[3];
} DriveInfo;

typedef struct {
    bool predict_change;
    bool detect_stall;
    ControlMode mode;
    SetPointType point_type;

    float encoder_filtering;
    float speed_filtering;
    float sampling_interval;
    float stall_timeout;
    float stall_tolerance;

    float velocity_target;
    float voltage_target;
    float current_limit;
    float user_current_limit;

    float speed_mult;
    float electric_mult;
    float PWM_mult;
    uint16_t max_PWM_per_s;

    const double T;  // control loop period, sec

    PIDConfig velocity_regulator;
    PIDConfig electric_regulator;
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
    HallSensor* sensor,
    GEncoder* speed_encoder,
    TIM_HandleTypeDef* htim
);

typedef enum { PHASE_A = 0, PHASE_B = 1, PHASE_C = 2 } DrivePhase;

// process_ADC.c
void process_ADC(InverterState* inverter, const uint32_t ADC_buf[]);

// common.c
void step_to_phases(EncoderStep step, DrivePhase* first, DrivePhase* second);
void calculate_angles(DriveInfo* drive, DriverControl* controller, GEncoder* speed_encoder);
void calculate_speed(DriveInfo* drive, DriverControl* controller, float dt);
void detect_stall(DriveInfo* drive, DriverControl* controller, double passed_time_abs);
void quit_stall(DriveInfo* drive, DriverControl* controller);

#define CONTROL_FUNC_ARGS                                                                \
    DriverControl *controller, DriveInfo *drive, InverterState *inverter, uint16_t *dqa, \
        uint16_t *dqb, uint16_t *dqc
// six_step_control.c
void six_step_control(HallSensor* sensor, GEncoder* speed_encoder, CONTROL_FUNC_ARGS);
// hall_six_step_control.c
void hall_six_step_control_callback(HallSensor* sensor, DriverControl *controller, DriveInfo *drive, InverterState *inverter, float dt);
void hall_six_step_control(HallSensor* sensor, CONTROL_FUNC_ARGS);
// simple_modes.c
void current_mode(CONTROL_FUNC_ARGS);
void rotate_mode(CONTROL_FUNC_ARGS);
void hall_sequence(HallSensor* encoder, CONTROL_FUNC_ARGS);

#define pi2 (2.0f * PI)

force_inline float get_current(InverterState* inverter, DrivePhase current_relative) {
    return -*(&inverter->I_A + current_relative);
}

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

// inline to avoid copying
force_inline void flow_direction(DriveInfo* drive, DrivePhase from, DrivePhase to, uint16_t* DQs[3], int16_t pwm) {
    uint16_t actual_pwm = abs(pwm);
    if (pwm < 0) {
        DrivePhase tmp = to;
        to = from;
        from = tmp;
    }
    DrivePhase off;
    if (PHASE_A != from && PHASE_A != to)
        off = PHASE_A;
    if (PHASE_B != from && PHASE_B != to)
        off = PHASE_B;
    if (PHASE_C != from && PHASE_C != to)
        off = PHASE_C;
    HAL_GPIO_WritePin(GPIOB, drive->L_PINS[off], GPIO_PIN_RESET);

    HAL_GPIO_WritePin(GPIOB, drive->L_PINS[from], GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, drive->L_PINS[to], GPIO_PIN_SET);
    *DQs[to] = 0;
    *DQs[from] = actual_pwm;
}


#ifdef __cplusplus
}
#endif
#endif
#endif /* VBLIB_BLDC_BLDC_H_ */
