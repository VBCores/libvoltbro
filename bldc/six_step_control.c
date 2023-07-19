/*
 * six_step_control.c
 *
 *  Created on: May 30, 2023
 *      Author: Igor Beschastnov,
 */
#include "bldc.h"

#if defined(HAL_TIM_MODULE_ENABLED) && defined(HAL_ADC_MODULE_ENABLED)

#include "arm_math.h"

#include "encoders/AS5048A/AS5048A.h"
#include "encoders/incremental_encoder/encoder.h"

#include "report.h"

extern void Error_Handler();
extern AS5048AConfig as5048a;

BLDCReport bldc_report;

// TODO: move to config
pin L_PINS[3] = {GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15};

typedef enum { PHASE_A = 0, PHASE_B = 1, PHASE_C = 2 } DrivePhase;

// inline to avoid copying
force_inline void flow_direction(DrivePhase from, DrivePhase to, uint16_t* DQs[3], int16_t pwm) {
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
    HAL_GPIO_WritePin(GPIOB, L_PINS[off], GPIO_PIN_RESET);

    HAL_GPIO_WritePin(GPIOB, L_PINS[from], GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, L_PINS[to], GPIO_PIN_SET);
    *DQs[to] = 0;
    *DQs[from] = actual_pwm;
}

void calculate_angles(DriveInfo* drive, DriverControl* controller);
void calculate_speed(DriveInfo* drive, DriverControl* controller);
void detect_stall(DriveInfo* drive, DriverControl* controller, double passed_time_abs);

int16_t get_control(
    DriveInfo* driver,
    DriverControl* controller,
    InverterState* inverter,
    double passed_time_abs,
    DrivePhase current_relative,
    int16_t pwm
);

#define USE_CONTROL
#ifdef DEBUG
int16_t PWM;
uint32_t ticks_since_sample_abs = 0;
DrivePhase first;
DrivePhase second;
#endif
void six_step_control(
    GEncoder* encoder,
    DriverControl* controller,
    DriveInfo* drive,
    InverterState* inverter,
    uint16_t* DQA,
    uint16_t* DQB,
    uint16_t* DQC
) {
    if (!drive->is_on) {
        return;
    }

    IncrementalEncoder* inc_encoder = (IncrementalEncoder*)encoder;
    // for convenience
    const EncoderStep step = inc_encoder->step;

#ifndef DEBUG
    static uint16_t PWM;
    static uint32_t ticks_since_sample_abs = 0;
    static DrivePhase from;
    static DrivePhase to;
#endif

    uint16_t* DQs[3] = {DQA, DQB, DQC};
    switch (step) {
        case AB:
            first = PHASE_A;
            second = PHASE_B;
            break;
        case AC:
            first = PHASE_A;
            second = PHASE_C;
            break;
        case BC:
            first = PHASE_B;
            second = PHASE_C;
            break;
        case BA:
            first = PHASE_B;
            second = PHASE_A;
            break;
        case CA:
            first = PHASE_C;
            second = PHASE_A;
            break;
        case CB:
            first = PHASE_C;
            second = PHASE_B;
            break;
    }

#ifdef USE_CONTROL
    as5048a.common.value = as5048a.common.get_angle((GEncoder*)&as5048a);
    calculate_angles(drive, controller);

    double passed_time_abs = ticks_since_sample_abs * controller->T;
    if (passed_time_abs > controller->sampling_interval) {
        calculate_speed(drive, controller);
        detect_stall(drive, controller, passed_time_abs);
        PWM = get_control(drive, controller, inverter, passed_time_abs, first, PWM);

        ticks_since_sample_abs = 0;
    } else {
        ticks_since_sample_abs += 1;
    }
#else
    PWM = 1000;
#endif

    // TODO: fix?
    int16_t actual_pwm = PWM;
    if (!as5048a.common.inverted) {
        // due to current flow in reverse direction to voltage
        actual_pwm = -actual_pwm;
    }
    flow_direction(first, second, DQs, actual_pwm);
#ifdef REPORT_STATE
    bldc_report.PWM = abs(actual_pwm);
#endif
}

#ifdef DEBUG
uint16_t filtered_data;
#endif
void calculate_angles(DriveInfo* drive, DriverControl* controller) {
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

    filtered_data = (uint16_t
    )lp_filter(controller->encoder_filtering, (float)prev_abs_value, (float)aligned_encoder_data);
    prev_abs_value = filtered_data;

    drive->shaft_angle = pi2 * ((float)filtered_data / (float)as5048a.common.CPR);
}

#ifdef DEBUG
float travel;
#endif
void calculate_speed(DriveInfo* drive, DriverControl* controller) {
    static float prev_velocity = 0;
    static float prev_angle = -1;

    if (prev_angle < 0) {
        prev_angle = drive->shaft_angle;
        return;
    }

#ifndef DEBUG
    float travel
#endif
        travel = drive->shaft_angle - prev_angle;
    if (travel < -PI) {
        travel += pi2;
    } else if (travel > PI) {
        travel -= pi2;
    }

    float new_speed = travel / controller->sampling_interval;
    drive->shaft_velocity = lp_filter(controller->speed_filtering, prev_velocity, new_speed);

    prev_velocity = drive->shaft_velocity;
    prev_angle = drive->shaft_angle;
}

force_inline float get_current(InverterState* inverter, DrivePhase current_relative) {
    return -*(&inverter->I_A + current_relative);
}

#define MAX_PWM 1999
#ifdef DEBUG
double control_signal;
float speed_error;
float I_error;
double I_signal;
double new_I_signal;
double S_signal;
#endif
int16_t get_control(
    DriveInfo* driver,
    DriverControl* controller,
    InverterState* inverter,
    double passed_time_abs,
    DrivePhase current_relative,
    int16_t pwm
) {
#ifndef DEBUG
    double control_signal;
    float speed_error;
    float I_error;
    double I_signal;
    double new_I_signal;
    double S_signal;
#endif
    /*
     * Как это работает:
     * 1) Получаем сигнал для скорости S_sig
     * 2) Из него получаем изменение тока: I_err = S_sig * Kt
     * 3) Получаем сигнал для тока I_sig
     * 4) I_target = max(I_lim, I_now + I_sig);
     * 5) new_I_sig = I_target - I_now
     * 6) изменение PWM_new = PWM_prev + new_I_sig * coeff
     */

    speed_error = controller->velocity_target - driver->shaft_velocity;
    S_signal = regulation(&controller->velocity_regulator, speed_error, passed_time_abs);

    I_error = (S_signal * controller->speed_mult) * driver->torque_const;

    float I_now = get_current(inverter, current_relative);
    float I_out;
    if (fabsf(I_error) > 0.01) {
        I_out = I_error;
    }
    else {
        I_out = (-I_now)/120;
    }

    I_signal = regulation(&controller->current_regulator, I_out, passed_time_abs);

    float I_target = I_now + I_signal * controller->I_mult;
    if (fabs(I_target) > controller->current_limit) {
        I_target = copysign(controller->current_limit, I_target);
    }

    new_I_signal = I_target - I_now;
    control_signal = new_I_signal * controller->PWM_mult;

    // PWM guards
    const float max_change_per_sample = controller->max_PWM_per_s * controller->sampling_interval;
    if (fabs(control_signal) > max_change_per_sample) {
        control_signal = copysign(max_change_per_sample, control_signal);
    }

    // amplifying minimal signal
    int16_t pwm_diff = (int16_t)control_signal;
    if (pwm_diff == 0 && fabs(control_signal) >= 0.01) {
        pwm_diff = (int16_t)copysign(1.0, control_signal);
    }

    pwm += pwm_diff;
    if (abs(pwm) > MAX_PWM) {
        pwm = copysign(MAX_PWM, pwm);
    }
    return pwm;
}

void quit_stall(DriveInfo* drive, DriverControl* controller) {
    controller->current_limit = controller->user_current_limit;
    drive->is_stalling = false;
}

#ifdef DEBUG
double cur_time = 0;
double stall_start_time = 0;
#endif
void detect_stall(DriveInfo* drive, DriverControl* controller, double passed_time_abs) {
#ifndef DEBUG
    static double cur_time = 0;
    static double stall_start_time = 0;
    static bool is_stalling = false;
#endif
    cur_time += passed_time_abs;

    if (fabsf(drive->shaft_velocity) < controller->stall_tolerance) {
        if (!drive->is_stalling) {
            drive->is_stalling = true;
            // drive->is_bursting = true;
            stall_start_time = cur_time;
        }
    } else {
        quit_stall(drive, controller);
    }

    if (drive->is_stalling) {
        // TODO: burst
        // if (drive->is_bursting) {
        if ((cur_time - stall_start_time) > controller->stall_timeout) {
            controller->current_limit = drive->stall_current;
            // drive->is_bursting = false;
        }
    }
}

#endif
