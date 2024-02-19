#pragma once
#if defined(STM32G474xx) || defined(STM32_G)

#include "stm32g4xx_hal.h"
#if defined(HAL_DAC_MODULE_ENABLED) && defined(HAL_TIM_MODULE_ENABLED)

#include <functional>

#include "voltbro/utils.hpp"

class EEPROM {
private:
    const uint16_t device_id = 0x50;
    const uint16_t page_size = 64;
    I2C_HandleTypeDef* i2c;

    using eeprom_operation = std::function<HAL_StatusTypeDef(
        I2C_HandleTypeDef*, uint16_t, uint16_t,
        uint16_t, uint8_t*, uint16_t, uint32_t
    )>;

    HAL_StatusTypeDef memory_op(eeprom_operation operation, uint8_t* bytes, size_t size, uint16_t address) {
        uint16_t page_number = 0;

        HAL_StatusTypeDef result = HAL_OK;
        while( size > page_size )
        {
            result = static_cast<HAL_StatusTypeDef>(result & operation(
                i2c,
                device_id << 1,
                address + page_size * page_number,
                I2C_MEMADD_SIZE_16BIT,
                bytes + page_size * page_number,
                page_size,
                100
            ));
            page_number += 1;
            size -= page_size;
            delay();
        }
        result = static_cast<HAL_StatusTypeDef>(result & operation(
            i2c,
            device_id << 1,
            address + page_size * page_number,
            I2C_MEMADD_SIZE_16BIT,
            bytes + page_size * page_number,
            size,
            100
        ));
        return result;
    }

public:
    explicit EEPROM(I2C_HandleTypeDef* i2c): i2c(i2c) {};

    void __attribute__((optimize("O0"))) delay() {
        // TODO: 4 is empirical value, probably needs fix
        HAL_Delay(4);
    }

    bool is_connected(void) {
        return HAL_I2C_IsDeviceReady(
            i2c,
            device_id << 1,
            2,
            100
        ) == HAL_OK;
    }

    template <typename T>
    HAL_StatusTypeDef write(T* obj, uint16_t address) {
        auto bytes = reinterpret_cast<uint8_t*>(obj);
        size_t size_to_write = sizeof(T);
        return memory_op(HAL_I2C_Mem_Write, bytes, size_to_write, address);
    }

    template <typename T>
    HAL_StatusTypeDef read(T* obj, uint16_t address) {
        auto bytes = reinterpret_cast<uint8_t *>(obj);
        size_t size_to_read = sizeof(T);
        return memory_op(HAL_I2C_Mem_Read, bytes, size_to_read, address);
    }
};


#endif
#endif
