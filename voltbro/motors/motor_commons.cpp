#if defined(STM32G474xx) || defined(STM32_G)

#include "motor_commons.h"
#ifdef ARM_MATH
#include <arm_math.h>
#else
#include <math.h>
#endif

float AbstractMotor::calculate_angle(
    const CommonDriverConfig& drive,
    GenericEncoder& speed_encoder
) const {
    float current_circle_part = (float)speed_encoder.get_value() / (float)speed_encoder.CPR;
    if (speed_encoder.is_electrical) {
        float circle_part_per_revolution = 1.0f / drive.ppairs;
        int32_t revolutions = speed_encoder.get_revolutions() % (int16_t)drive.ppairs;
        if (revolutions < 0) {
            revolutions += drive.ppairs;
        }
        current_circle_part = (revolutions + current_circle_part) * circle_part_per_revolution;
    }

    return current_circle_part * pi2;
}

float AbstractMotor::calculate_speed(const float shaft_angle, const float dt) const {
    static float prev_velocity = 0;
    static float prev_angle = -pi2 - 2;

    if (prev_angle < (-pi2 - 1)) {
        prev_angle = shaft_angle;
        return 0;
    }

    float filtered_angle = angle_filter(prev_angle,shaft_angle);

    float travel = filtered_angle - prev_angle;
    if (travel < -PI) {
        travel += pi2;
    } else if (travel > PI) {
        travel -= pi2;
    }

    float new_speed = travel / dt;

    float velocity = speed_filter(prev_velocity, new_speed);

    prev_velocity = velocity;
    prev_angle = filtered_angle;

    return velocity;
}

#endif
