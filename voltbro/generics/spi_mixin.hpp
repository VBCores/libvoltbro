#pragma once

#if defined(STM32G4) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#ifdef HAL_SPI_MODULE_ENABLED

#include "voltbro/utils.hpp"

class SPIMixin {
protected:
    SPI_HandleTypeDef* const spi;

    explicit SPIMixin(SPI_HandleTypeDef* spi): spi(spi) {}

    void spi_transmit_only(uint16_t command) {
        HAL_SPI_Transmit(
            spi,
            reinterpret_cast<uint8_t*>(&command),
            1,
            HAL_MAX_DELAY
        );
    }

    uint16_t spi_transmit_command_receive(uint16_t command) {
        uint16_t resp = 0;
        HAL_SPI_TransmitReceive(
            spi,
            reinterpret_cast<uint8_t*>(&command),
            reinterpret_cast<uint8_t*>(&resp),
            1,
            HAL_MAX_DELAY
        );
        return resp;
    }
};

#endif
#endif
