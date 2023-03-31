/*
 * AS5048A.c
 *
 *  Created on: Jan 16, 2023
 *      Author: Igor Beschastnov
 */

#include <stdlib.h>
#include <string.h>
#include "AS5048A.h"
#ifdef HAL_SPI_MODULE_ENABLED

uint16_t TxBuffer;

void make_AS5048A_config_reserved(
    AS5048AConfig* dest,
    bool inverted,
    uint16_t CPR,
    uint16_t elec_offset,
    SPI_HandleTypeDef* spi,
    GPIO_TypeDef* SPI_SS_GPIOx,
    pin SPI_SS
) {
    AS5048AConfig tmp_config = {
        .common          = {
            .CPR         = CPR,
            .inverted    = inverted,
            .value       = 0,
            .last_error  = false,
            .elec_offset = elec_offset,
            .get_angle   = (uint16_t(*)(GEncoder*)) &AS5048A_get_angle
        },
        .spi             = spi,
        .SPI_SS          = SPI_SS,
        .SPI_SS_GPIOx    = SPI_SS_GPIOx,
    };

    memcpy(dest, &tmp_config, sizeof(AS5048AConfig));
}

AS5048AConfig* make_AS5048A_config(
    bool inverted,
    uint16_t CPR,
    uint16_t elec_offset,
    SPI_HandleTypeDef* spi,
    GPIO_TypeDef* SPI_SS_GPIOx,
    pin SPI_SS
) {
    AS5048AConfig* config = malloc(sizeof(AS5048AConfig));
    if (config == NULL) {
        return NULL;
    }

    make_AS5048A_config_reserved(
        config,
        inverted,
        CPR,
        elec_offset,
        spi,
        SPI_SS_GPIOx,
        SPI_SS
    );

    return config;
}

HAL_StatusTypeDef spi_transmit_command(AS5048AConfig* config, uint16_t command) {
    return HAL_SPI_Transmit(
        config->spi,
        (uint8_t *)&command,
        1,
        1000
    );
}

uint16_t spi_transmit_command_receive(AS5048AConfig* config, uint16_t command) {
    uint16_t response;
    HAL_StatusTypeDef tr_status = HAL_SPI_TransmitReceive(
        config->spi,
        (uint8_t *)&command,
        (uint8_t *)&response,
        1,
        1000
    );
    if (tr_status != HAL_OK) {
        // TODO?
    }
    return response;
}

uint8_t getParity(uint16_t n)
{
    uint8_t parity = 0;
    while (n) {
        parity = !parity;
        n = n & (n - 1);
    }
    return parity;
}

uint16_t get_command(AS5048ARegister reg) {
    uint16_t command = 0x4000;  // PAR=0 R/W=R
    command = command | reg;

    if( getParity(command) ) {
        command = command | 0x8000;  //set parity bit 1
    }
    else {
        command = command | 0x0000;  //set parity bit 0
    }

    return command;
}

uint16_t AS5048A_read(AS5048AConfig* config, AS5048ARegister reg) {
    uint16_t command = get_command(reg);

    AS5048A_start_transaction(config);
    HAL_StatusTypeDef transmit_status = spi_transmit_command(config, command);
    // TODO? handle status
    AS5048A_end_transaction(config);

    AS5048A_start_transaction(config);
    uint16_t response = spi_transmit_command_receive(config, NOP);
    AS5048A_end_transaction(config);

    if (response & AS5048A_ERROR_BIT) {
        config->common.last_error = true;
    }
    else {
        config->common.last_error = false;
    }

    //Return the data, stripping the parity and error bits
    return response & ~0xC000;
}

uint16_t AS5048A_get_angle(AS5048AConfig* config) {
    uint16_t angle = AS5048A_read(config, ANGLE);
    if (config->common.inverted) {
        angle = config->common.CPR - angle;
    }
    return angle;
}

#endif
