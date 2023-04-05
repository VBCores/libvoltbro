/*
 * simple_angles.c
 *
 *  Created on: Mar 23, 2023
 *      Author: Igor Beschastnov,
 */

#include "bldc.h"

#include "math/math_ops.h"

void predict_angles(DriverState* driver, GEncoder* encoder, float passed_time) {
    float new_encoder_value =
        ((float)encoder->value +
         (float)encoder->CPR * ((driver->ShaftVelocity * passed_time) / pi2));
    if (new_encoder_value >= encoder->CPR) {
        new_encoder_value = new_encoder_value - encoder->CPR;
    } else if (new_encoder_value < 0) {
        new_encoder_value = new_encoder_value + encoder->CPR;
    }

    driver->ElecTheta =
        calc_elec_theta(new_encoder_value, driver->pulses_per_pair);
    driver->ShaftAngle = pi2 * (new_encoder_value / (float)encoder->CPR);
}

static uint16_t prev_data = 0;
static uint16_t last_value = 0;
static uint16_t ticks_since_change = 0;
void CalculateAnglesSimple(
    DriverState* driver,
    GEncoder* encoder,
    PIDConfig* pid
) {
    float passed_time = (float)ticks_since_change * driver->T;

    ticks_since_change += 1;
    if (last_value == encoder->value) {
        if (driver->predict_change) {
            predict_angles(driver, encoder, passed_time);
        }
        return;
    }

    int32_t encoder_data = encoder->value;

    int32_t aligned_encoder_data = encoder_data - encoder->elec_offset;
    if (aligned_encoder_data < 0) {
        aligned_encoder_data += encoder->CPR;
    }

    uint16_t filtered_data;
    if (driver->encoder_filtering > 0.01f) {  // driver->encoder_filtering != 0
        filtered_data = (uint16_t)lp_filter(
            driver->encoder_filtering,
            (float)prev_data,
            (float)aligned_encoder_data
        );
        prev_data = filtered_data;
    } else {
        filtered_data = aligned_encoder_data;
    }

    driver->ElecTheta = calc_elec_theta(filtered_data, driver->pulses_per_pair);

    float prev_angle = driver->ShaftAngle;
    driver->ShaftAngle = pi2 * ((float)filtered_data / (float)encoder->CPR);
    float diff = driver->ShaftAngle - prev_angle;
    if (diff < -3.14) {
        diff += pi2;
    } else if (diff > 3.14) {
        diff -= pi2;
    }
    float new_velocity = diff / passed_time;
    driver->ShaftVelocity =
        lp_filter(0.5f, driver->ShaftVelocity, new_velocity);

    last_value = encoder->value;
    ticks_since_change = 0;
}
