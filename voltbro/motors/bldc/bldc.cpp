#include "bldc.h"
#if defined(STM32G4) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#if defined(HAL_TIM_MODULE_ENABLED) && defined(HAL_ADC_MODULE_ENABLED)

HAL_StatusTypeDef BLDCController::init() {
    if (drive_info.l_pins.has_value()) {
        for (auto pin : drive_info.l_pins.value()) {
            pin.set();
        }
    }

    HAL_StatusTypeDef result = HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1); // A-phase
    if (result != HAL_OK) {
        return result;
    }
    result = HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2); // B-phase
    if (result != HAL_OK) {
        return result;
    }
    result = HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3); // C-phase
    if (result != HAL_OK) {
        return result;
    }

    inverter.start();
    return HAL_OK;
}

HAL_StatusTypeDef BLDCController::stop() {
    drive_info.en_pin.reset();
    _is_on = false;
    return HAL_OK;
}

HAL_StatusTypeDef BLDCController::start() {
    /*
    for (int i = 0; i < 3; i++) {
        drive_info.en_pin.reset();
        HAL_Delay(10);
        drive_info.en_pin.set();
        HAL_Delay(10);
    }
    */
    drive_info.en_pin.set();
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

    if (fabsf(shaft_velocity) < drive_info.stall_tolerance) {
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
        if ((cur_time - stall_start_time) > drive_info.stall_timeout) {
            drive_limits.current_limit = drive_info.stall_current;
            // drive->is_bursting = false;
        }
    }
}

void BLDCController::quit_stall() {
    drive_limits.current_limit = drive_limits.user_current_limit;
    is_stalling = false;
}

#endif
#endif
