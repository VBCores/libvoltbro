/*
 * hall_six_step_control.c
 *
 *  Created on: November 13, 2023
 *      Author: Igor Beschastnov,
 */
#include "bldc.h"

int16_t local_pwm = 100;

const float outlier_threshold = pi2 * 10;
void hall_six_step_control_callback(HallSensor* encoder, DriverControl *controller, DriveInfo *drive, InverterState *inverter, float dt) {
    const uint16_t abs_value = encoder->common.value;

    static uint16_t prev_abs_value = -1;  // initial bigger than CPR
    if (prev_abs_value > encoder->common.CPR) {
        prev_abs_value = abs_value;
    }

    int32_t difference = prev_abs_value - abs_value;
    prev_abs_value = abs_value;
    if ( abs(difference) > (encoder->common.CPR / 2) ) {
        if (difference > 0) {
            encoder->common.revolutions += 1;
        }
        else {
            encoder->common.revolutions -= 1;
        }
    }

    float current_circle_part = (float)abs_value / encoder->common.CPR;

    const float circle_part_per_revolution = 1.0f / drive->ppairs;
    int32_t revolutions = encoder->common.revolutions % (int16_t)drive->ppairs;
    if (revolutions < 0) {
        revolutions += drive->ppairs;
    }
    float current_cycles = revolutions;
    float full_circle_part = (current_cycles + current_circle_part) * circle_part_per_revolution;

    drive->shaft_angle = pi2 * full_circle_part;

    float prev_velocity = drive->shaft_velocity;
    calculate_speed(drive, controller, dt);
    if (fabsf(drive->shaft_velocity) > outlier_threshold) {
        drive->shaft_velocity = prev_velocity;
    }
}

#define USE_CONTROL
#ifdef DEBUG
float speed_error;
double control_signal;
#endif
void hall_six_step_control(
    HallSensor* encoder,
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
handle_hall_data:
    step_to_phases(encoder->step, &first, &second);
    encoder->newer_interrupt = false;

    static uint32_t ticks_since_sample_abs = 0;
    double passed_time_abs = ticks_since_sample_abs * controller->T;
    if (passed_time_abs > controller->sampling_interval) {
#ifndef DEBUG
        float speed_error;
        double signal;
#endif
        int16_t new_pwm = local_pwm;

        if (controller->point_type == SET_VELOCITY) {
            float current = fabsf(get_current(inverter, first));
            float I_err = current - drive->max_current;
            if (I_err > 0) {
                control_signal = -I_err * controller->electric_mult;
            }
            else {
                speed_error = controller->velocity_target - drive->shaft_velocity;
                control_signal = regulation(&controller->velocity_regulator, speed_error, passed_time_abs);
            }
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

            new_pwm += pwm_diff;
        }
        else if (controller->point_type == SET_VOLTAGE) {
            new_pwm = drive->full_pwm / drive->supply_voltage * controller->voltage_target;
        }

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
    local_pwm = 100;
#endif

    int16_t actual_pwm = local_pwm;
    if (encoder->common.inverted) {
        actual_pwm = -actual_pwm;
    }
    if(encoder->newer_interrupt) {
        goto handle_hall_data;
    }
    flow_direction(drive, first, second, DQs, actual_pwm);
}
