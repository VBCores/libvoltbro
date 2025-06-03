#include "six_step_controller.h"
#if defined(HAL_TIM_MODULE_ENABLED) && defined(HAL_ADC_MODULE_ENABLED)

#define USE_CONTROL

void SixStepController::update() {
    if (!_is_on) {
        return;
    }

    DrivePhase first, second;
    step_to_phases(hall_sensor.get_step(), first, second);

    // TODO: check and report if point_type is not voltage?
    int16_t new_pwm = full_pwm / drive_info.supply_voltage * target;

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

void step_to_phases(EncoderStep step, DrivePhase& first, DrivePhase& second) {
    switch (step) {
        case EncoderStep::AB:
            first = DrivePhase::PHASE_A;
            second = DrivePhase::PHASE_B;
            break;
        case EncoderStep::AC:
            first = DrivePhase::PHASE_A;
            second = DrivePhase::PHASE_C;
            break;
        case EncoderStep::BC:
            first = DrivePhase::PHASE_B;
            second = DrivePhase::PHASE_C;
            break;
        case EncoderStep::BA:
            first = DrivePhase::PHASE_B;
            second = DrivePhase::PHASE_A;
            break;
        case EncoderStep::CA:
            first = DrivePhase::PHASE_C;
            second = DrivePhase::PHASE_A;
            break;
        case EncoderStep::CB:
            first = DrivePhase::PHASE_C;
            second = DrivePhase::PHASE_B;
            break;
    }
}

#endif
