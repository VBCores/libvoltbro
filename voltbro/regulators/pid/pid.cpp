#include "pid.h"

#if defined(STM32G474xx) || defined(STM32_G)
#include "stm32g4xx.h"
#endif
#ifdef ARM_MATH
#include <arm_math.h>
#else
#include <math.h>
#endif

float PIDRegulator::regulation(float error, float dt, bool zero_in_threshold) {
    if (zero_in_threshold && fabs(error) <= config.tolerance) {
        signal = 0;
        integral_error = 0;
        return 0;
    }

    float error_diff = error - prev_error;
    prev_error = error;

    if (isnan(integral_error)) {
        integral_error = 0;
    }
    double new_integral_error = integral_error + error * dt;
    if (fabs(new_integral_error) > config.integral_error_lim) {
        new_integral_error = copysign(config.integral_error_lim, integral_error);
    }
    integral_error = new_integral_error;

    signal = config.multiplier * (
         config.p_gain * error + config.i_gain * integral_error +
         config.d_gain * error_diff / dt
     );

    return signal;
}
