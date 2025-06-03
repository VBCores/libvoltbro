#pragma once

#include <utility>
#include <algorithm>

#ifdef ARM_MATH_CM4
#include <arm_math.h>
#else
#include <math.h>
#endif

struct PIDConfig {
    float multiplier = 1.0;
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float integral_error_lim = 10000.0f;
    float tolerance = 0.0f;
    float max_output = 10000.0f;  // 10k is a reasonable limit for most applications, can be changed if needed
    float min_output = -10000.0f;  // -10k is a reasonable limit for most applications, can be changed if needed
};

class PIDRegulator  {
private:
    float signal = 0.0f;
    float integral_error = 0.0f;
    float prev_error = 0.0f;
    PIDConfig config;
public:
    // expect copy-elision
    explicit PIDRegulator(PIDConfig&& config) : config(std::move(config)) {}

    float regulation(float error, float dt, bool zero_in_threshold=false) {
        if (zero_in_threshold && config.tolerance != 0 && (fabs(error) <= config.tolerance)) {
            signal = 0;
            integral_error = 0;
            return 0;
        }

        float error_diff = error - prev_error;
        prev_error = error;

        if (isnan(integral_error)) {
            integral_error = 0;
            // TODO: warn?
        }
        float new_integral_error = integral_error + error * dt;
        if (fabs(new_integral_error) > config.integral_error_lim) {
            new_integral_error = copysign(config.integral_error_lim, new_integral_error);
        }
        integral_error = new_integral_error;

        signal = config.multiplier * (
             config.kp * error + config.ki * integral_error +
             config.kd * error_diff / dt
         );

        return std::clamp(signal, config.min_output, config.max_output);
    }

    void update_config(PIDConfig&& new_config) {
        config = std::move(new_config);
    }

    PIDConfig get_config() {
        return config;
    }
};
