#include "AS5048A.h"
#ifdef HAL_SPI_MODULE_ENABLED

static const uint8_t AS5048A_AGC_FLAG = 0xFF;
static const uint8_t AS5048A_ERROR_PARITY_FLAG = 0x04;
static const uint8_t AS5048A_ERROR_COMMAND_INVALID_FLAG = 0x02;
static const uint8_t AS5048A_ERROR_FRAMING_FLAG = 0x01;

static const uint16_t AS5048A_DIAG_COMP_HIGH = 0x2000;
static const uint16_t AS5048A_DIAG_COMP_LOW = 0x1000;
static const uint16_t AS5048A_DIAG_COF = 0x0800;
static const uint16_t AS5048A_DIAG_OCF = 0x0400;

static const float AS5048A_MAX_VALUE = 8191.0f;
static const uint16_t AS5048A_ERROR_BIT = 0x4000;

HAL_StatusTypeDef AS5048A::spi_transmit_command(uint16_t command) {
    return HAL_SPI_Transmit(spi, (uint8_t*)&command, 1, 1000);
}

uint8_t AS5048A::getParity(uint16_t n) {
    uint8_t parity = 0;
    while (n) {
        parity = !parity;
        n = n & (n - 1);
    }
    return parity;
}

uint16_t AS5048A::spi_transmit_command_receive(uint16_t command) {
    uint16_t response;
    HAL_StatusTypeDef tr_status =
            HAL_SPI_TransmitReceive(spi, (uint8_t*)&command, (uint8_t*)&response, 1, 1000);
    if (tr_status != HAL_OK) {
        // TODO?
    }
    return response;
}

uint16_t AS5048A::get_command(AS5048ARegister reg) {
    uint16_t command = 0x4000;  // PAR=0 R/W=R
    command = command | (uint16_t)reg;

    if (getParity(command)) {
        command = command | 0x8000;  // set parity bit 1
    } else {
        command = command | 0x0000;  // set parity bit 0
    }

    return command;
}

uint16_t AS5048A::read(AS5048ARegister reg) {
    uint16_t command = get_command(reg);

    start_transaction();
    HAL_StatusTypeDef transmit_status = spi_transmit_command(command);
    // TODO? handle status
    end_transaction();

    CS_delay();

    start_transaction();
    uint16_t response = spi_transmit_command_receive((uint16_t)AS5048ARegister::NOP);
    end_transaction();

    if (response & AS5048A_ERROR_BIT) {
        last_error = true;
    } else {
        last_error = false;
    }

    // Return the data, stripping the parity and error bits
    return response & ~0xC000;
}

uint16_t AS5048A::get_angle() {
    uint16_t angle = read(AS5048ARegister::ANGLE);
    if (is_inverted) {
        angle = CPR - angle;
    }
    return angle;
}

#endif