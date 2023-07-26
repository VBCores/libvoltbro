/*
 * bldc.c
 *
 *  Created on: Jan 24, 2023
 *      Author: Igor Beschastnov,
 */

#include "bldc.h"

#ifdef HAL_TIM_MODULE_ENABLED

#ifdef DEBUG
static uint16_t DQA = 0;
static uint16_t DQB = 0;
static uint16_t DQC = 0;
#endif

void motor_control(
    DriverControl* controller,
    DriveInfo* driver,
    InverterState* inverter,
    const uint32_t ADC_buf[],
    GEncoder* encoder,
    TIM_HandleTypeDef* htim
) {
#ifdef DEBUG
    DQA = 0;
    DQB = 0;
    DQC = 0;
#else
    uint_16t DQA = 0;
    uint_16t DQB = 0;
    uint_16t DQC = 0;
#endif

    process_ADC(inverter, ADC_buf);

    switch (controller->mode) {
        case CURRENT:
            current_mode(controller, driver, inverter, &DQA, &DQB, &DQC);
            break;
        case ROTATE:
            rotate_mode(controller, driver, inverter, &DQA, &DQB, &DQC);
            break;
        case SIX_STEP_CONTROL:
            six_step_control(encoder, controller, driver, inverter, &DQA, &DQB, &DQC);
            break;
        case CALIBRATE:
            // TODO
        case NO_ACTION:
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
            DQA = DQB = DQC = 0;
            break;
    }

    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, DQA);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, DQB);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, DQC);

    encoder->value = encoder->get_angle(encoder);
}

#endif
