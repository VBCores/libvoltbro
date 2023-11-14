/*
 * hall_six_step_control.c
 *
 *  Created on: November 13, 2023
 *      Author: Igor Beschastnov,
 */
#include "bldc.h"

float clamp_velocity = 6.14;

static int16_t local_pwm = 100;

void hall_six_step_control_callback(IncrementalEncoder* encoder, DriverControl *controller, DriveInfo *drive, InverterState *inverter, float dt) {
    //calculate_speed(drive, controller, dt);
}

#define USE_CONTROL
#ifdef DEBUG
float speed_error;
double signal;
#endif
void hall_six_step_control(
    IncrementalEncoder* encoder,
    DriverControl* controller,
    DriveInfo* drive,
    InverterState* inverter,
    uint16_t* DQA,
    uint16_t* DQB,
    uint16_t* DQC
) {
#ifndef DEBUG
    static uint16_t PWM;
#endif
    if (!drive->is_on) {
        return;
    }

    uint16_t* DQs[3] = {DQA, DQB, DQC};
    DrivePhase first, second;
    step_to_phases(encoder->step, &first, &second);

    calculate_angles(drive, controller, (GEncoder*)encoder);

    static uint32_t ticks_since_sample_abs = 0;
    double passed_time_abs = ticks_since_sample_abs * controller->T;
    if (passed_time_abs > controller->sampling_interval) {
        calculate_speed(drive, controller, -1.0f);
#ifndef DEBUG
        float speed_error;
        double signal;
#endif
        speed_error = controller->velocity_target - drive->shaft_velocity;
        signal = regulation(&controller->velocity_regulator, speed_error, passed_time_abs);

        // amplifying minimal signal
        int16_t pwm_diff = (int16_t)signal * controller->PWM_mult;
        if (pwm_diff == 0 && fabs(signal) >= 0.01) {
            pwm_diff = (int16_t)copysign(1.0, signal);
        }

        int16_t new_pwm = local_pwm;
        new_pwm += pwm_diff;
        const uint32_t MAX_PWM = drive->full_pwm * 0.95f;
        if (abs(new_pwm) > MAX_PWM) {
            new_pwm = copysign(MAX_PWM, new_pwm);
        }

        local_pwm = new_pwm;

        ticks_since_sample_abs = 0;
    } else {
        ticks_since_sample_abs += 1;
    }

#ifndef USE_CONTROL
    local_pwm = 300;
#endif

    int16_t actual_pwm = local_pwm;
    if (!encoder->common.inverted) {
        // due to current flow in reverse direction to voltage
        actual_pwm = -actual_pwm;
    }
    flow_direction(drive, first, second, DQs, actual_pwm);
}