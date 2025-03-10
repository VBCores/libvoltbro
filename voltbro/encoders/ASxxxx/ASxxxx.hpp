#pragma once

#if defined(STM32G474xx) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#ifdef HAL_SPI_MODULE_ENABLED

#include <cstdint>

#include "voltbro/encoders/generic.h"
#include "voltbro/utils.hpp"
#include "voltbro/devices/gpio.hpp"

constexpr uint16_t CYCLES_100NS_160Mhz = 16;

__attribute__((optimize("O1"))) static inline void delay_cpu_cycles(uint16_t cycles) {
    /* Reference:
     https://developer.arm.com/documentation/ddi0439/b/Programmers-Model/Instruction-set-summary/Cortex-M4-instructions?lang=en
     *
     * // 6 тактов на (cycles - 8) / 5
       sub     r3, r0, #5         // 1 такт
       ldr     r2, .L6            // 2 такта
       smull   r1, r2, r3, r2     // 1 такт
       asr     r3, r3, #31        // 1 такт
       rsb     r3, r3, r2, asr #1 // 1 такт
     *
     * // 2 такта на стартовую проверку
       ands    r3, r3, #255       // 1 такт
       bxeq    lr                 // 1 такт ("Conditional branch completes in a single cycle if the
     branch is not taken.")
     *
     * // ~5 тактов на цикл
       .L3:
       nop                       // 1 такт
       sub     r3, r3, #1        // 1 такт
       ands    r3, r3, #255      // 1 такт
       bne     .L3               // 1 + 1-3 такта, в среднем 2(3?)
     *
     * Всего 5 тактов на цикл + 8 в начале.
     */
    constexpr uint8_t SETUP_CYCLES = 8;
    constexpr uint8_t LOOP_CYCLES = 5;

    uint8_t real_cycles = (cycles - SETUP_CYCLES) / LOOP_CYCLES;
    while (real_cycles--) {
        // NOLINTBEGIN(hicpp-no-assembler)
        __asm__("nop");
        // NOLINTEND(hicpp-no-assembler)
    }
}


template<class ASxxxxParams>
class ASxxxx: public GenericEncoder {
private:
    GpioPin spi_ss;
    SPI_HandleTypeDef* const spi;

    HAL_StatusTypeDef spi_transmit_command(uint16_t command) {
        return HAL_SPI_Transmit(spi, (uint8_t*)&command, 1, 1000);
    }

    __attribute__((always_inline)) inline void start_transaction() {
        spi_ss.reset();
    }

    __attribute__((always_inline)) inline void end_transaction() {
        spi_ss.set();
    }

    uint16_t spi_transmit_command_receive(uint16_t command) {
        uint16_t response;
        HAL_StatusTypeDef transmit_status =
                HAL_SPI_TransmitReceive(spi, (uint8_t*)&command, (uint8_t*)&response, 1, 1000);
        if (transmit_status != HAL_OK) {
            // TODO?
        }
        return response;
    }

    uint16_t get_command(ASxxxxParams reg) {
        uint16_t command = to_underlying(ASxxxxParams::COM_READ);
        command = command | (uint16_t)reg;

        uint8_t parity = 0;
        uint16_t n = command;
        while (n) {
            parity = !parity;
            n = n & (n - 1);
        }

        ASxxxxParams parity_bit;
        if (parity) {
            parity_bit = ASxxxxParams::PARITY_BIT_1;
        } else {
            parity_bit = ASxxxxParams::PARITY_BIT_0;
        }
        command = command | to_underlying(parity_bit);

        return command;
    }

    uint16_t read(ASxxxxParams reg) {
        uint16_t command = get_command(reg);

        start_transaction();
        HAL_StatusTypeDef transmit_status = spi_transmit_command(command);
        if (transmit_status != HAL_OK) {
            // TODO?
        }
        end_transaction();

        delay_cpu_cycles(to_underlying(ASxxxxParams::CS_DELAY_CYCLES));

        start_transaction();
        uint16_t response = spi_transmit_command_receive((uint16_t)ASxxxxParams::REG_NOP);
        end_transaction();

        last_error = (response & to_underlying(ASxxxxParams::ERROR_BIT));

        // Return the data, stripping the parity and error bits
        return response & ~to_underlying(ASxxxxParams::CLEAR_ERROR_AND_PARITY);
    }

    encoder_data get_angle() {
        uint16_t angle = read(ASxxxxParams::REG_ANGLE);
        if (is_inverted) {
            angle = CPR - angle;
        }
        return angle;
    }

public:
    ASxxxx(
        GpioPin&& spi_ss,
        SPI_HandleTypeDef* spi,
        encoder_data CPR,
        bool is_inverted = false,
        encoder_data electric_offset = 0
    ):
        GenericEncoder(CPR, is_inverted, false, electric_offset),
        spi_ss(std::move(spi_ss)),
        spi(spi)
    {
        value = (encoder_data)-1;
    };

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
};

#endif
#endif
