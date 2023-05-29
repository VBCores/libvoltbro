/*
 * six_step_control.c
 *
 *  Created on: Mar 23, 2023
 *      Author: Igor Beschastnov,
 */
#include "bldc.h"

#include "arm_math.h"

#include "encoders/incremental_encoder/encoder.h"

extern void Error_Handler();

extern TIM_HandleTypeDef htim1;
// TODO: remove
#define INLA_Pin GPIO_PIN_13
#define INLA_GPIO_Port GPIOB
#define INLB_Pin GPIO_PIN_14
#define INLB_GPIO_Port GPIOB
#define INLC_Pin GPIO_PIN_15
#define INLC_GPIO_Port GPIOB

pin L_PINS[3] = {INLA_Pin, INLB_Pin, INLC_Pin};

typedef enum {
    PHASE_A = 0,
    PHASE_B = 1,
    PHASE_C = 2
} DrivePhase;

void flow_direction(DrivePhase from, DrivePhase to, uint16_t* DQ_FROM, uint16_t* DQ_TO) {
    DrivePhase off;
    if (PHASE_A != from && PHASE_A != to) off = PHASE_A;
    if (PHASE_B != from && PHASE_B != to) off = PHASE_B;
    if (PHASE_C != from && PHASE_C != to) off = PHASE_C;
    HAL_GPIO_WritePin(GPIOB, L_PINS[off], GPIO_PIN_RESET);

    HAL_GPIO_WritePin(GPIOB, L_PINS[from], GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, L_PINS[to], GPIO_PIN_SET);
    *DQ_TO = 1999;
    *DQ_FROM = 1400;
}

void six_step_control(
    DriverState* driver,
    GEncoder* encoder,
    InverterState* inverter,
    PIDConfig* pid,
    uint16_t* DQA,
    uint16_t* DQB,
    uint16_t* DQC
) {
    if (!driver->is_on) {
        return;
    }
    static EncoderStep last_step = -1;

    IncrementalEncoder* inc_encoder = (IncrementalEncoder*)encoder;
    const EncoderStep step = inc_encoder->step;

    // Guards
    if (inc_encoder->step < 0) {
        return;
    }

    uint16_t* modulation;
    pin L_PIN;
    GPIO_TypeDef* L_PORT;
    switch (step) {
        case A:
            L_PIN = INLC_Pin;
            L_PORT = INLC_GPIO_Port;
            modulation = DQB;
            *DQA = *DQC = 0;
            break;
        case AB:
            L_PIN = INLA_Pin;
            L_PORT = INLA_GPIO_Port;
            modulation = DQB;
            *DQA = *DQC = 0;
            break;
        case B:
            L_PIN = INLA_Pin;
            L_PORT = INLA_GPIO_Port;
            modulation = DQC;
            *DQA = *DQB = 0;
            break;
        case BC:
            L_PIN = INLB_Pin;
            L_PORT = INLB_GPIO_Port;
            modulation = DQC;
            *DQA = *DQB = 0;
            break;
        case C:
            L_PIN = INLB_Pin;
            L_PORT = INLB_GPIO_Port;
            modulation = DQA;
            *DQB = *DQC = 0;
            break;
        case CA:
            L_PIN = INLC_Pin;
            L_PORT = INLC_GPIO_Port;
            modulation = DQA;
            *DQB = *DQC = 0;
            break;
    }

    if (L_PIN != INLA_Pin) {
        HAL_GPIO_WritePin(INLA_GPIO_Port, INLA_Pin, GPIO_PIN_RESET);
    }
    if (L_PIN != INLB_Pin) {
        HAL_GPIO_WritePin(INLA_GPIO_Port, INLB_Pin, GPIO_PIN_RESET);
    }
    if (L_PIN != INLC_Pin) {
        HAL_GPIO_WritePin(INLC_GPIO_Port, INLC_Pin, GPIO_PIN_RESET);
    }
    HAL_GPIO_WritePin(L_PORT, L_PIN, GPIO_PIN_SET);

    *modulation = 1100;
}
