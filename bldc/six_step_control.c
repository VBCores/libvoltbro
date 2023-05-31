/*
 * six_step_control.c
 *
 *  Created on: May 30, 2023
 *      Author: Igor Beschastnov,
 */
#include "bldc.h"

#include "arm_math.h"

#include "encoders/incremental_encoder/encoder.h"
#include "encoders/AS5048A/AS5048A.h"

#include "report.h"

extern void Error_Handler();
extern AS5048AConfig as5048a;

BLDCReport bldc_report;

// TODO: move to config
pin L_PINS[3] = {GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15};

typedef enum {
    PHASE_A = 0,
    PHASE_B = 1,
    PHASE_C = 2
} DrivePhase;

// inline to avoid copying
force_inline void flow_direction(DrivePhase from, DrivePhase to, uint16_t* DQs[3], int16_t pwm) {
    uint16_t actual_pwm = abs(pwm);
    if (pwm < 0) {
        DrivePhase tmp = to;
        to = from;
        from = tmp;
    }
    DrivePhase off;
    if (PHASE_A != from && PHASE_A != to) off = PHASE_A;
    if (PHASE_B != from && PHASE_B != to) off = PHASE_B;
    if (PHASE_C != from && PHASE_C != to) off = PHASE_C;
    HAL_GPIO_WritePin(GPIOB, L_PINS[off], GPIO_PIN_RESET);

    HAL_GPIO_WritePin(GPIOB, L_PINS[from], GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, L_PINS[to], GPIO_PIN_SET);
    *DQs[to] = 0;
    *DQs[from] = actual_pwm;
}

void calculate_angles(DriverState* driver);
void calculate_speed(DriverState* driver);
int16_t get_control(DriverState* driver, PIDConfig* pid, double passed_time_abs, int16_t pwm);

#define USE_SPEEDS
#ifdef DEBUG
int16_t PWM;
uint32_t ticks_since_sample_abs = 0;
#endif
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

    IncrementalEncoder* inc_encoder = (IncrementalEncoder*)encoder;
    // Guards
    if (inc_encoder->step < 0) {
        return;
    }
    // for convenience
    const EncoderStep step = inc_encoder->step;

#ifndef DEBUG
    static uint16_t PWM;
    static uint32_t ticks_since_sample_abs = 0;
#endif
    // refresh encoder
#ifdef USE_SPEEDS
    as5048a.common.value = as5048a.common.get_angle((GEncoder*)&as5048a);
    calculate_angles(driver);

    double passed_time_abs = ticks_since_sample_abs * driver->T;
    if (passed_time_abs > driver->sampling_interval) {
        calculate_speed(driver);
       PWM = get_control(driver, pid, passed_time_abs, PWM);
        ticks_since_sample_abs = 0;
    } else {
        ticks_since_sample_abs += 1;
    }
#else
    PWM = 150;
#endif
    uint16_t* DQs[3] = {DQA, DQB, DQC};
    switch (step) {
        case AB:
            flow_direction(PHASE_C, PHASE_B, DQs, PWM);
            break;
        case AC:
            flow_direction(PHASE_A, PHASE_B, DQs, PWM);
            break;
        case BC:
            flow_direction(PHASE_A, PHASE_C, DQs, PWM);
            break;
        case BA:
            flow_direction(PHASE_B, PHASE_C, DQs, PWM);
            break;
        case CA:
            flow_direction(PHASE_B, PHASE_A, DQs, PWM);
            break;
        case CB:
            flow_direction(PHASE_C, PHASE_A, DQs, PWM);
            break;
    }
}

#ifdef DEBUG
uint16_t filtered_data;
float speed_error;

double pid_signal;
#endif
void calculate_angles(DriverState* driver) {
#ifndef DEBUG
    float speed_error
#endif
    const uint16_t abs_value = as5048a.common.value;

    static uint16_t prev_abs_value = -1;  // initial bigger than CPR
    int32_t aligned_encoder_data = abs_value - as5048a.common.elec_offset;
    if (aligned_encoder_data < 0) {
        aligned_encoder_data += as5048a.common.CPR;
    }
    if (prev_abs_value > as5048a.common.CPR) {
        prev_abs_value = aligned_encoder_data;
    }
#ifndef DEBUG
    uint16_t filtered_data;
#endif
    const uint16_t half_cpr = as5048a.common.CPR / 2;
    if (abs(prev_abs_value - aligned_encoder_data) > half_cpr) {
        prev_abs_value = aligned_encoder_data;
        return;
    }

    filtered_data = (uint16_t)lp_filter(
        driver->encoder_filtering,
        (float)prev_abs_value,
        (float)aligned_encoder_data
    );
    prev_abs_value = filtered_data;

    driver->ShaftAngle = pi2 * ((float)filtered_data / (float)as5048a.common.CPR);
}

#ifdef DEBUG
float travel;
#endif
void calculate_speed(DriverState* driver) {
    static float prev_velocity = 0;
    static float prev_angle = -1;

    if (prev_angle < 0) {
        prev_angle = driver->ShaftAngle;
        return;
    }

#ifndef DEBUG
    float travel
#endif
    travel = driver->ShaftAngle - prev_angle;
    if (travel < -PI) {
        travel += pi2;
    }
    else if (travel > PI) {
        travel -= pi2;
    }

    float new_speed = travel / driver->sampling_interval;
    driver->ShaftVelocity = lp_filter(driver->speed_filtering, prev_velocity, new_speed);

    prev_velocity = driver->ShaftVelocity;
    prev_angle = driver->ShaftAngle;
}

// TODO: make configurable
#define MAX_PWM_PER_S 800.0f
#define PID_DIV 20.0f
#define MAX_PWM 1999
int16_t get_control(DriverState* driver, PIDConfig* pid, double passed_time_abs, int16_t pwm) {
    // PID
    speed_error = driver->vel_SetP - driver->ShaftVelocity;
    regulation(pid, speed_error, passed_time_abs);
#ifndef DEBUG
    double pid_signal;
#endif
    pid_signal = pid->signal; // amplifier

    const float max_change_per_sample = MAX_PWM_PER_S * driver->sampling_interval;
    if (fabs(pid_signal) > max_change_per_sample) {
        pid_signal = copysign(max_change_per_sample, pid_signal);
    }

    pwm -= (int16_t)pid_signal;
    if (abs(pwm) > MAX_PWM) {
        pwm = copysign(MAX_PWM, pwm);
    }
#ifdef REPORT_STATE
    // TODO
#endif
    return pwm;
}
