#include "pid.h"

#include <arm_math.h>

double PIDRegulator::regulation(double error, double dt) {
    if (fabs(error) <= config.tolerance) {
        signal = 0;
        integral_error = 0;
        return 0;
    }

    double error_diff = fabs(error) - fabs(prev_error);
    prev_error = error;

    double new_integral_error = integral_error + error * dt;
    if (fabs(new_integral_error) > config.integral_error_lim) {
        new_integral_error = copysign(config.integral_error_lim, integral_error);
    }
    integral_error = new_integral_error;

    signal = config.p_gain * error + config.i_gain * integral_error +
             config.d_gain * error_diff / dt;

    return signal;
}