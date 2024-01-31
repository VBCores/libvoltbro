#include "motor_commons.h"
#include "arm_math.h"

float AbstractMotor::calculate_angle(
    const CommonDriverConfig& drive,
    GenericEncoder& speed_encoder
) const {
    float current_circle_part = (float)speed_encoder.get_value() / (float)speed_encoder.CPR;
    float full_circle_part = current_circle_part;
    if (speed_encoder.is_electrical) {
        float circle_part_per_revolution = 1.0f / drive.ppairs;
        float current_cycles = speed_encoder.get_revolutions() % drive.ppairs;
        full_circle_part = (current_cycles + current_circle_part) * circle_part_per_revolution;
    }

    return pi2 * full_circle_part;
}

float AbstractMotor::calculate_speed(const float shaft_angle, const float dt) const {
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

    float velocity = speed_filter(prev_velocity, new_speed);

    prev_velocity = velocity;
    prev_angle = shaft_angle;

    return velocity;
}