#include "motor_commons.h"
#include "arm_math.h"

#ifdef DEBUG
uint16_t filtered_data;
#endif
float calculate_angles(
    const CommonDriverConfig& drive,
    const LowPassFilter& filter,
    GenericEncoder& speed_encoder
) {
#ifndef DEBUG
    float speed_error
#endif
    const uint16_t abs_value = speed_encoder.get_value();

    static uint16_t prev_abs_value = -1;  // initial bigger than CPR
    int32_t aligned_encoder_data = abs_value - speed_encoder.electric_offset;
    if (aligned_encoder_data < 0) {
        aligned_encoder_data += speed_encoder.CPR;
    }
    if (prev_abs_value > speed_encoder.CPR) {
        prev_abs_value = aligned_encoder_data;
    }
#ifndef DEBUG
    uint16_t filtered_data;
#endif
    const uint16_t half_cpr = speed_encoder.CPR / 2;
    int32_t difference = prev_abs_value - aligned_encoder_data;
    if (abs(difference) > half_cpr) {
        if (difference > 0) {
            speed_encoder.incr_revolutions();
        }
        else {
            speed_encoder.decr_revolutions();
        }
        prev_abs_value = aligned_encoder_data;
    }

    filtered_data = (uint16_t) filter(
        (float) prev_abs_value,
        (float) aligned_encoder_data
    );
    prev_abs_value = filtered_data;

    float current_circle_part = (float)filtered_data / (float)speed_encoder.CPR;
    float full_circle_part = current_circle_part;
    if (speed_encoder.is_electrical) {
        float circle_part_per_revolution = 1.0f / drive.ppairs;
        float current_cycles = speed_encoder.get_revolutions() % drive.ppairs;
        full_circle_part = (current_cycles + current_circle_part) * circle_part_per_revolution;
    }

    return pi2 * full_circle_part;
}

float calculate_speed(const float shaft_angle, const LowPassFilter& filter, const float dt) {
    static float prev_velocity = 0;
    static float prev_angle = -1;

    if (prev_angle < 0) {
        prev_angle = shaft_angle;
        return 0;
    }

    float travel = shaft_angle - prev_angle;
    if (travel < -PI) {
        travel += pi2;
    } else if (travel > PI) {
        travel -= pi2;
    }

    float new_speed = travel / dt;

    float velocity = filter(prev_velocity, new_speed);

    prev_velocity = velocity;
    prev_angle = shaft_angle;

    return velocity;
}