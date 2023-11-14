/*
 * common.c
 *
 *  Created on: November 10, 2023
 *      Author: Igor Beschastnov,
 */
#include "bldc.h"

#include "arm_math.h"

#include "encoders/AS5048A/AS5048A.h"
#include "encoders/incremental_encoder/encoder.h"

extern void Error_Handler();

void step_to_phases(EncoderStep step, DrivePhase* first, DrivePhase* second) {
    switch (step) {
        case AB:
            *first = PHASE_A;
            *second = PHASE_B;
            break;
        case AC:
            *first = PHASE_A;
            *second = PHASE_C;
            break;
        case BC:
            *first = PHASE_B;
            *second = PHASE_C;
            break;
        case BA:
            *first = PHASE_B;
            *second = PHASE_A;
            break;
        case CA:
            *first = PHASE_C;
            *second = PHASE_A;
            break;
        case CB:
            *first = PHASE_C;
            *second = PHASE_B;
            break;
    }
}

#ifdef DEBUG
uint16_t filtered_data;
#endif
void calculate_angles(DriveInfo* drive, DriverControl* controller, GEncoder* speed_encoder) {
#ifndef DEBUG
    float speed_error
#endif
    const uint16_t abs_value = speed_encoder->value;

    static uint16_t prev_abs_value = -1;  // initial bigger than CPR
    int32_t aligned_encoder_data = abs_value - speed_encoder->elec_offset;
    if (aligned_encoder_data < 0) {
        aligned_encoder_data += speed_encoder->CPR;
    }
    if (prev_abs_value > speed_encoder->CPR) {
        prev_abs_value = aligned_encoder_data;
    }
#ifndef DEBUG
    uint16_t filtered_data;
#endif
    const uint16_t half_cpr = speed_encoder->CPR / 2;
    int32_t difference = prev_abs_value - aligned_encoder_data;
    if (abs(difference) > half_cpr) {
        if (difference > 0) {
            speed_encoder->revolutions += 1;
        }
        else {
            speed_encoder->revolutions -= 1;
        }
        prev_abs_value = aligned_encoder_data;
    }

    filtered_data = (uint16_t) lp_filter(
        controller->encoder_filtering,
        (float) prev_abs_value,
        (float) aligned_encoder_data
    );
    prev_abs_value = filtered_data;

    float current_circle_part = (float)filtered_data / (float)speed_encoder->CPR;
    float full_circle_part = current_circle_part;
    if (speed_encoder->is_electrical) {
        float circle_part_per_revolution = 1.0f / drive->ppairs;
        float current_cycles = speed_encoder->revolutions % drive->ppairs;
        full_circle_part = (current_cycles + current_circle_part) * circle_part_per_revolution;
    }

    drive->shaft_angle = pi2 * full_circle_part;
}

#ifdef DEBUG
float travel;
float dt_;
#endif
void calculate_speed(DriveInfo* drive, DriverControl* controller, float dt) {
#ifdef DEBUG
    dt_ = dt;
#endif
    static float prev_velocity = 0;
    static float prev_angle = -1;

    if (prev_angle < 0) {
        prev_angle = drive->shaft_angle;
        return;
    }

#ifndef DEBUG
    float travel;
#endif
    travel = drive->shaft_angle - prev_angle;
    if (travel < -PI) {
        travel += pi2;
    } else if (travel > PI) {
        travel -= pi2;
    }

    if (dt <= 0) {
        dt = controller->sampling_interval;
    }
    float new_speed = travel / dt;
    drive->shaft_velocity = lp_filter(controller->speed_filtering, prev_velocity, new_speed);

    prev_velocity = drive->shaft_velocity;
    prev_angle = drive->shaft_angle;
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
