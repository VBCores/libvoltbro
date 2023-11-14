/*
 * six_step_control.c
 *
 *  Created on: May 30, 2023
 *      Author: Igor Beschastnov,
 */
#include "bldc.h"

#include "arm_math.h"

#include "encoders/AS5048A/AS5048A.h"
#include "encoders/incremental_encoder/encoder.h"

#include "report.h"

extern void Error_Handler();

BLDCReport bldc_report;

int16_t get_control(
    DriveInfo* driver,
    DriverControl* controller,
    InverterState* inverter,
    double passed_time_abs,
    DrivePhase current_relative,
    int16_t pwm
);

//#define USE_CONTROL
#ifdef DEBUG
int16_t PWM = 100;
uint32_t ticks_since_sample_abs = 0;
DrivePhase first;
DrivePhase second;
#endif
void six_step_control(
    GEncoder* encoder,
    GEncoder* speed_encoder,
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

#ifndef DEBUG
    static uint16_t PWM;
    static uint32_t ticks_since_sample_abs = 0;
    static DrivePhase from;
    static DrivePhase to;
#endif

    uint16_t* DQs[3] = {DQA, DQB, DQC};
    step_to_phases(inc_encoder->step, &first, &second);

    speed_encoder->value = speed_encoder->get_angle(speed_encoder);
    calculate_angles(drive, controller, speed_encoder);

    double passed_time_abs = ticks_since_sample_abs * controller->T;
    if (passed_time_abs > controller->sampling_interval) {
        calculate_speed(drive, controller, -1);
        detect_stall(drive, controller, passed_time_abs);
        PWM = get_control(drive, controller, inverter, passed_time_abs, first, PWM);

        ticks_since_sample_abs = 0;
    } else {
        ticks_since_sample_abs += 1;
    }
#ifndef USE_CONTROL
    PWM = 200;
#endif

    // TODO: fix?
    int16_t actual_pwm = PWM;
    if (!speed_encoder->inverted) {
        // due to current flow in reverse direction to voltage
        actual_pwm = -actual_pwm;
    }
    flow_direction(drive, first, second, DQs, actual_pwm);
#ifdef REPORT_STATE
    bldc_report.PWM = abs(actual_pwm);
#endif
}

force_inline float get_current(InverterState* inverter, DrivePhase current_relative) {
    return -*(&inverter->I_A + current_relative);
}

#ifdef DEBUG
static double control_signal;
static float speed_error;
static float I_error;
static double I_signal;
static double new_I_signal;
static double S_signal;
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
     * 1) Получаем сигнал для скорости S_sig. Считаем полученный сигнал требованием к увеличению момента, ньютон-метры
     * 2) Из него получаем изменение тока: I_err = S_sig * Kv (Нм * (А / Нм) = А)
     * 3) Получаем сигнал для тока I_sig (безразмерный сигнал)
     * 4) I_target = max(I_lim, I_now + I_sig);
     * 5) new_I_sig = I_target - I_now (безразмерный сигнал, хотя казалось бы амперы)
     * 6) изменение PWM_new = PWM_prev + new_I_sig * coeff
     */

    speed_error = controller->velocity_target - driver->shaft_velocity;
    S_signal = regulation(&controller->velocity_regulator, speed_error, passed_time_abs);

    I_error = (S_signal * controller->speed_mult) * driver->speed_const;

    float I_now = get_current(inverter, current_relative);
    float I_out;
    if (fabsf(I_error) > 0.01) {
        I_out = I_error;
    }
    else {
        I_out = (-I_now)/120;
    }

    I_signal = regulation(&controller->electric_regulator, I_out, passed_time_abs);

    float I_target = I_now + I_signal * controller->electric_mult;
    if (fabs(I_target) > controller->current_limit) {
        I_target = copysign(controller->current_limit, I_target);
    }

    new_I_signal = I_target - I_now;
    control_signal = (driver->full_pwm / driver->supply_voltage) * new_I_signal;

    // PWM guards
    const float max_change_per_sample = controller->max_PWM_per_s * controller->sampling_interval;
    if (fabs(control_signal) > max_change_per_sample) {
        control_signal = copysign(max_change_per_sample, control_signal);
    }

    // amplifying minimal signal
    int16_t pwm_diff = (int16_t)control_signal * controller->PWM_mult;
    if (pwm_diff == 0 && fabs(control_signal) >= 0.01) {
        pwm_diff = (int16_t)copysign(1.0, control_signal);
    }

    pwm += pwm_diff;
    const uint32_t MAX_PWM = driver->full_pwm * 0.95f;
    if (abs(pwm) > MAX_PWM) {
        pwm = copysign(MAX_PWM, pwm);
    }
    return pwm;
}
