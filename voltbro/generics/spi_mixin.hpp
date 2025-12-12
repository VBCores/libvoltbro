#pragma once

#if defined(STM32G4) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#ifdef HAL_SPI_MODULE_ENABLED
#include "stm32g4xx_ll_spi.h"

#include "voltbro/utils.hpp"

class SPIMixin {
protected:
    SPI_HandleTypeDef* const spi;

    explicit SPIMixin(SPI_HandleTypeDef* spi): spi(spi) {}

    void spi_transmit_only(uint16_t command) {
        const auto spix = spi->Instance;

        for (uint16_t i = 0; i < 2; i++) {
            /* Wait until TX buffer empty */
            while (!LL_SPI_IsActiveFlag_TXE(spix)) {
            }

            /* Write next byte to send */
            uint8_t tx = reinterpret_cast<uint8_t*>(&command)[i];
            LL_SPI_TransmitData8(spix, tx);
        }

        while (LL_SPI_IsActiveFlag_BSY(spix)) {
        }
    }

    uint16_t spi_transmit_command_receive(uint16_t command) {
        /*
        uint16_t response;
        const auto spix = spi->Instance;

        for (uint16_t i = 0; i < 2; i++) {
            while (!LL_SPI_IsActiveFlag_TXE(spix)) {
            }

            uint8_t tx = reinterpret_cast<uint8_t*>(&command)[i];
            LL_SPI_TransmitData8(spix, tx);

            while (!LL_SPI_IsActiveFlag_RXNE(spix)) {
            }

            uint8_t rx = LL_SPI_ReceiveData8(spix);
            reinterpret_cast<uint8_t*>(&response)[i] = rx;
        }

        while (LL_SPI_IsActiveFlag_BSY(spix)) {
        }
        return response;
        */
        SPI_TypeDef *spix = spi->Instance;

        while (!LL_SPI_IsActiveFlag_TXE(spix)) {}
        LL_SPI_TransmitData16(spix, command);

        while (!LL_SPI_IsActiveFlag_RXNE(spix)) {}
        uint16_t resp = LL_SPI_ReceiveData16(spix);

        while (LL_SPI_IsActiveFlag_BSY(spix)) {}
        return resp;
    }
};

#endif
#endif
