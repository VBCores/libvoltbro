#pragma once

#include <utility>
#include <algorithm>
#include <tuple>
#include <limits>
#include "voltbro/utils.hpp"

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
    float integral_error_lim = std::numeric_limits<float>::max();
    float tolerance = 0.0f;
    float max_output = std::numeric_limits<float>::max();
    float min_output = -std::numeric_limits<float>::max();
};

class PIDRegulator  {
private:
    float signal = 0.0f;
    float integral_error = 0.0f;
    float prev_error = 0.0f;
    float pending_integral_error = 0.0f; // Accumulated error * dt awaiting the next integral update.
    PIDConfig config;
protected:
    std::tuple<float, float> get_raw_signal_and_intergal(float error, float dt) {
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

        float kd_part = 0.0f;
        if (!is_close(config.kd, 0.0f) && !is_close(dt, 0.0f, 1e-8f)) {
            kd_part = config.kd * error_diff / dt;
        }
        return {
            config.multiplier * config.kp * error + config.ki * new_integral_error + kd_part,
            new_integral_error
        };
    }
public:
    void reset() {
        signal = integral_error = prev_error = pending_integral_error = 0.0f;
    }

    explicit PIDRegulator(PIDConfig&& config) : config(std::move(config)) {}
    PIDRegulator() {}

    FORCE_INLINE float get_integral_error() const {
        return integral_error;
    }

    FORCE_INLINE void set_integral_error(float new_integral) {
        pending_integral_error = 0.0f;
        integral_error = std::clamp(new_integral, -config.integral_error_lim, config.integral_error_lim);
    }

    FORCE_INLINE void update_config(float kp, float ki, float kd) {
        pending_integral_error = 0.0f;
        config.kp = kp;
        config.ki = ki;
        config.kd = kd;
    }

    /** Derivative from successive errors; multiplier scales P only. Configured
     * I-error/output limits and optional tolerance apply; saturation blocks I updates.
     */
    float regulation(float error, float dt, bool zero_in_threshold=false) {
        if (zero_in_threshold && config.tolerance != 0.0f && (fabs(error) <= config.tolerance)) {
            signal = 0.0f;
            integral_error = 0.0f;
        }
        else {
            auto [raw_signal, new_integral_error] = get_raw_signal_and_intergal(error, dt);
            // anti-windup - not updating integral error if the signal is saturated
            if (raw_signal < config.max_output && raw_signal > config.min_output) {
                integral_error = new_integral_error;
            }
            signal = std::clamp(raw_signal, config.min_output, config.max_output);
        }
        return signal;
    }

    /** Explicit bounds gate integration; final output uses configured output bounds.
     * Other terms and the optional tolerance follow the configured-bounds overload.
     */
    float regulation(float error, float dt, float lower_limit, float upper_limit, bool zero_in_threshold=false) {
        if (zero_in_threshold && config.tolerance != 0.0f && (fabs(error) <= config.tolerance)) {
            signal = 0.0f;
            integral_error = 0.0f;
        }
        else {
            auto [raw_signal, new_integral_error] = get_raw_signal_and_intergal(error, dt);
            // anti-windup - not updating integral error if the signal is saturated
            if (raw_signal < upper_limit && raw_signal > lower_limit) {
                integral_error = new_integral_error;
            }
            signal = std::clamp(raw_signal, config.min_output, config.max_output);
        }
        return signal;
    }

    float regulation(float error, float dt, float limit, bool zero_in_threshold=false) {
        return regulation(
            error,
            dt,
            -limit,
            limit,
            zero_in_threshold
        );
    }

    void update_config(PIDConfig&& new_config) {
        pending_integral_error = 0.0f;
        config = std::move(new_config);
    }
    /** PID using an externally supplied error derivative and dynamic output bounds.
     * Uses kp, ki and kd; multiplier, tolerance and configured I/output limits are unused.
     * lower_limit/upper_limit bound both the I contribution and the final output.
     * Every call accumulates error * dt; update_integral commits it with conditional
     * anti-windup. Ki == 0 clears I; tighter bounds also constrain retained I between commits.
     */
    float regulation_with_derivative(float error, float dt, float lower_limit, float upper_limit, float derivative, bool update_integral = true) {
        const float pd = config.kp * error + config.kd * derivative;
        float integral = 0.0f;
        if (config.ki == 0.0f) {
            integral_error = pending_integral_error = 0.0f;
        } else {
            pending_integral_error += error * dt;
            const float unbounded_integral = config.ki * integral_error;
            integral = std::clamp(unbounded_integral, lower_limit, upper_limit);
            if (update_integral) {
                const float increment = config.ki * pending_integral_error;
                pending_integral_error = 0.0f;
                const float candidate = std::clamp(integral + increment, lower_limit, upper_limit);
                const float raw = pd + candidate;
                // Block integration into saturation, but allow unwinding while saturated.
                if ((raw <= upper_limit || increment < 0.0f) && (raw >= lower_limit || increment > 0.0f)) {
                    integral = candidate;
                }
                integral_error = integral / config.ki;
            } else if (integral != unbounded_integral) {
                // A tighter live limit must also constrain the retained I state.
                integral_error = integral / config.ki;
            }
        }
        prev_error = error;
        signal = std::clamp(pd + integral, lower_limit, upper_limit);
        return signal;
    }

    PIDConfig get_config() const {
        return config;
    }
};
