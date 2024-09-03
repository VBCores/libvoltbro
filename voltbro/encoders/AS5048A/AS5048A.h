#pragma once

#if defined(STM32G474xx) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#ifdef HAL_SPI_MODULE_ENABLED

#include <cstdint>

#include "voltbro/encoders/generic.h"
#include "voltbro/utils.hpp"

enum class AS5048ARegister {
    DIAG_AGC = 0x3FFD,
    MAGNITUDE = 0x3FFE,
    ANGLE = 0x3FFF,
    CLEAR_ERROR_FLAG = 0x0001,
    PROGRAMMING_CONTROL = 0x0003,
    NOP = 0x0000
};

// ~200 ns delay for the SPI communication
__attribute__((always_inline, optimize("O0"))) static inline void CS_delay(void) {
    uint8_t k = 32;
    while (k--) {
        __asm__("nop");
    }
}

class AS5048A: public GenericEncoder {
private:
    GPIO_TypeDef* const SPI_SS_GPIOx;
    const pin SPI_SS;
    SPI_HandleTypeDef* const spi;

    HAL_StatusTypeDef spi_transmit_command(uint16_t command);
    static uint8_t getParity(uint16_t n);
    uint16_t spi_transmit_command_receive(uint16_t command);
    uint16_t get_command(AS5048ARegister reg);
    uint16_t read(AS5048ARegister reg);
    encoder_data get_angle();
public:
    AS5048A(
        GPIO_TypeDef* const SPI_SS_GPIOx,
        const pin SPI_SS,
        SPI_HandleTypeDef* spi,
        encoder_data CPR,
        bool is_inverted = false,
        bool is_electrical = false,
        encoder_data electric_offset = 0
    ):
        GenericEncoder(CPR, is_inverted, is_inverted, electric_offset),
        SPI_SS_GPIOx(SPI_SS_GPIOx),
        SPI_SS(SPI_SS),
        spi(spi)
    {
        value = (encoder_data)-1;
    };

    __attribute__((always_inline)) inline void start_transaction() {
        HAL_GPIO_WritePin(SPI_SS_GPIOx, SPI_SS, GPIO_PIN_RESET);
    }

    __attribute__((always_inline)) inline void end_transaction() {
        HAL_GPIO_WritePin(SPI_SS_GPIOx, SPI_SS, GPIO_PIN_SET);
    }

    void update_value() override {
        encoder_data new_value = get_angle();

        if (value == (encoder_data)-1) {
            value = new_value;
            return;
        }

        int32_t diff = (int32_t)value - (int32_t)new_value;
        const encoder_data half_cpr = CPR / 2;
        if (abs(diff) > half_cpr) {
            if (diff < 0) {
                decr_revolutions();
            }
            else {
                incr_revolutions();
            }
        }

        value = new_value;
    }
    inline encoder_data get_value() const override {
        return value;
    }
};

#endif
#endif