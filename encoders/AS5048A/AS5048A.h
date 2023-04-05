/*
 * AS5048A.h
 *
 *  Created on: Jan 16, 2023
 *      Author: igor
 */

#ifndef VBLIB_AS5048A_AS5048A_H_
#define VBLIB_AS5048A_AS5048A_H_

#ifdef STM32_G
#include "stm32g4xx_hal.h"
#else
#include "stm32f4xx_hal.h"
#endif

#ifdef HAL_SPI_MODULE_ENABLED

#include "encoders/generic.h"
#include "utils.h"

typedef enum AS5048ARegister {
    DIAG_AGC = 0x3FFD,
    MAGNITUDE = 0x3FFE,
    ANGLE = 0x3FFF,
    CLEAR_ERROR_FLAG = 0x0001,
    PROGRAMMING_CONTROL = 0x0003,
    NOP = 0x0000
} AS5048ARegister;

#ifdef __cplusplus
#include <cstdint>
extern "C" {
#else
#include <stdbool.h>
#include <stdint.h>
#endif

typedef struct AS5048AConfig {
    // Shared
    GEncoder common;
    // Specific
    GPIO_TypeDef* const SPI_SS_GPIOx;
    const pin SPI_SS;
    SPI_HandleTypeDef* spi;
} AS5048AConfig;

AS5048AConfig* make_AS5048A_config(
    bool inverted,
    uint16_t CPR,
    uint16_t elec_offset,
    SPI_HandleTypeDef* spi,
    GPIO_TypeDef* SPI_SS_GPIOx,
    pin SPI_SS
);
void make_AS5048A_config_reserved(
    AS5048AConfig* dest,
    bool inverted,
    uint16_t CPR,
    uint16_t elec_offset,
    SPI_HandleTypeDef* spi,
    GPIO_TypeDef* SPI_SS_GPIOx,
    pin SPI_SS
);
uint16_t AS5048A_read(AS5048AConfig* config, AS5048ARegister reg);
uint16_t AS5048A_get_angle(AS5048AConfig* config);

// ~200 ns delay for the SPI communication
#pragma optimize = s none
__attribute__((always_inline)) static inline void CS_delay(void) {
    uint8_t k = 32;
    while (k--) {
        __asm__("nop");
    }
}

__attribute__((always_inline)) static inline void AS5048A_start_transaction(
    AS5048AConfig* config
) {
    CS_delay();
    HAL_GPIO_WritePin(config->SPI_SS_GPIOx, config->SPI_SS, GPIO_PIN_RESET);
}

__attribute__((always_inline)) static inline void AS5048A_end_transaction(
    AS5048AConfig* config
) {
    HAL_GPIO_WritePin(config->SPI_SS_GPIOx, config->SPI_SS, GPIO_PIN_SET);
}

#ifdef __cplusplus
}
#endif
// Consts

extern int16_t lookup[128];

static const uint8_t AS5048A_AGC_FLAG = 0xFF;
static const uint8_t AS5048A_ERROR_PARITY_FLAG = 0x04;
static const uint8_t AS5048A_ERROR_COMMAND_INVALID_FLAG = 0x02;
static const uint8_t AS5048A_ERROR_FRAMING_FLAG = 0x01;

static const uint16_t AS5048A_DIAG_COMP_HIGH = 0x2000;
static const uint16_t AS5048A_DIAG_COMP_LOW = 0x1000;
static const uint16_t AS5048A_DIAG_COF = 0x0800;
static const uint16_t AS5048A_DIAG_OCF = 0x0400;

static const double AS5048A_MAX_VALUE = 8191.0;
static const uint16_t AS5048A_ERROR_BIT = 0x4000;

#endif /* HAL_SPI_MODULE_ENABLED */
#endif /* VBLIB_AS5048A_AS5048A_H_ */
