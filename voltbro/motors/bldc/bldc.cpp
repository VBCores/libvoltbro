#include "bldc.h"
#if defined(STM32G474xx) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#if defined(HAL_TIM_MODULE_ENABLED) && defined(HAL_ADC_MODULE_ENABLED)

HAL_StatusTypeDef BLDCController::stop() {
    HAL_GPIO_WritePin(
        drive_info.en_port,
        drive_info.en_pin,
        GPIO_PIN_RESET
    );
    _is_on = false;
    return HAL_OK;
}

HAL_StatusTypeDef BLDCController::start() {
    inverter.start();
    for (int i = 0; i < 3; i++) {
        HAL_GPIO_WritePin(
                drive_info.en_port,
                drive_info.en_pin,
                GPIO_PIN_RESET
        );
        HAL_Delay(10);
        HAL_GPIO_WritePin(
                drive_info.en_port,
                drive_info.en_pin,
                GPIO_PIN_SET
        );
        HAL_Delay(10);
    }
    quit_stall();
    _is_on = true;
    return HAL_OK;
}

HAL_StatusTypeDef BLDCController::set_state(bool state) {
    if (state) {
        return start();
    }
    else {
        return stop();
    }
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

#ifdef DEBUG
double cur_time = 0;
double stall_start_time = 0;
#endif
void BLDCController::detect_stall(double passed_time_abs) {
#ifndef DEBUG
    static double cur_time = 0;
    static double stall_start_time = 0;
#endif
    cur_time += passed_time_abs;

    if (fabsf(shaft_velocity) < control_config.stall_tolerance) {
        if (!is_stalling) {
            is_stalling = true;
            // is_bursting = true;
            stall_start_time = cur_time;
        }
    } else {
        quit_stall();
    }

    if (is_stalling) {
        // TODO: burst
        // if (drive->is_bursting) {
        if ((cur_time - stall_start_time) > control_config.stall_timeout) {
            control_config.current_limit = drive_info.stall_current;
            // drive->is_bursting = false;
        }
    }
}

#endif
#endif