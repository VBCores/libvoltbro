/*
 * hall_six_step_control.c
 *
 *  Created on: November 13, 2023
 *      Author: Igor Beschastnov,
 */
#include "bldc.h"

static int16_t local_pwm = 100;

void hall_six_step_control_callback(IncrementalEncoder* encoder, DriverControl *controller, DriveInfo *drive, InverterState *inverter, float dt) {
    // VERY roughly
    calculate_angles(drive, controller, (GEncoder*)encoder);
    calculate_speed(drive, controller, dt);
}

//#define USE_CONTROL
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

    delay_micros(5);
#ifndef USE_CONTROL
    // for testing
    local_pwm = 200;
#endif

    int16_t actual_pwm = local_pwm;
    if (!encoder->common.inverted) {
        // due to current flow in reverse direction to voltage
        actual_pwm = -actual_pwm;
    }
    flow_direction(drive, first, second, DQs, actual_pwm);
}