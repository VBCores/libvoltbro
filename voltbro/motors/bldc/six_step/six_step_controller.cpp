#include "six_step_controller.h"
#if defined(HAL_TIM_MODULE_ENABLED) && defined(HAL_ADC_MODULE_ENABLED)

int16_t local_pwm = 0;

void SixStepController::hall_six_step_control_callback() {
    shaft_angle = calculate_angle(drive_info.common, hall_sensor);
}

#define USE_CONTROL
#ifdef DEBUG
float speed_error;
float control_signal;
#endif
void SixStepController::regulate(float _) {
#ifndef DEBUG
    static uint16_t PWM;
#endif
    if (!_is_on) {
        return;
    }

    DrivePhase first, second;
handle_hall_data:
    step_to_phases(hall_sensor.get_step(), first, second);

    static uint32_t ticks_since_sample_abs = 0;
    float passed_time_abs = ticks_since_sample_abs * T;
    if (passed_time_abs > control_config.sampling_interval) {
#ifndef DEBUG
        float speed_error;
        float control_signal;
#endif
        int16_t new_pwm = local_pwm;

        if (control_config.point_type == SetPointType::VELOCITY) {
            float current = fabsf(get_current(inverter, first));
            float I_err = current - drive_info.max_current;
            if (I_err > 0) {
                control_signal = -I_err * control_config.electric_mult;
            }
            else {
                speed_error = control_config.target - shaft_velocity;
                control_signal = control_config.main_regulator.regulation(speed_error, passed_time_abs);
            }
            // PWM guards
            const float max_change_per_sample = control_config.max_PWM_per_s * control_config.sampling_interval;
            if (fabs(control_signal) > max_change_per_sample) {
                control_signal = copysign(max_change_per_sample, control_signal);
            }

            // amplifying minimal signal
            int16_t pwm_diff = (int16_t)control_signal * control_config.PWM_mult;
            if (pwm_diff == 0 && fabs(control_signal) >= 0.01) {
                pwm_diff = (int16_t)copysign(1.0, control_signal);
            }

            new_pwm += pwm_diff;
        }
        else if (control_config.point_type == SetPointType::VOLTAGE) {
            new_pwm = full_pwm / drive_info.supply_voltage * control_config.target;
        }

        const uint32_t MAX_PWM = full_pwm * 0.95f;
        if ( ((uint16_t)abs(new_pwm)) > MAX_PWM ) {
            new_pwm = copysign(MAX_PWM, new_pwm);
        }
        local_pwm = new_pwm;

        ticks_since_sample_abs = 0;
    } else {
        ticks_since_sample_abs += 1;
    }

#ifndef USE_CONTROL
    local_pwm = 300;
#endif

    int16_t actual_pwm = local_pwm;
    if (hall_sensor.is_inverted) {
        actual_pwm = -actual_pwm;
    }
    flow_direction(first, second, actual_pwm);

    set_pwm();
}

#endif