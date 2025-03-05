#include "six_step_controller.h"
#if defined(HAL_TIM_MODULE_ENABLED) && defined(HAL_ADC_MODULE_ENABLED)

#define USE_CONTROL

void SixStepController::update() {
    if (!_is_on) {
        return;
    }

    DrivePhase first, second;
    step_to_phases(hall_sensor.get_step(), first, second);

    int16_t new_pwm = full_pwm / drive_info.supply_voltage * voltage_target;

    const uint32_t MAX_PWM = full_pwm * 0.95f;
    if ( ((uint16_t)abs(new_pwm)) > MAX_PWM ) {
        new_pwm = copysign(MAX_PWM, new_pwm);
    }

#ifndef USE_CONTROL
    local_pwm = 300;
#endif

    if (hall_sensor.is_inverted) {
        new_pwm = -new_pwm;
    }
    flow_direction(first, second, new_pwm);

    set_pwm();
}

#endif
