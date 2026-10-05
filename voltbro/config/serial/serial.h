#pragma once

#include "usart.h"
#include <nanoprintf.h>
#include <algorithm>
#include <cstdarg>
#include <cstddef>
#include <cstdint>

class UARTResponseAccumulator {
private:
    const size_t max_size;
    size_t pos;
    char* buffer;
    UART_HandleTypeDef* huart;
    const bool blocking;
public:
    UARTResponseAccumulator(UART_HandleTypeDef* huart, char* buffer, size_t max_size, bool blocking = false) :
        max_size(max_size), pos(0), buffer(buffer), huart(huart), blocking(blocking) {}

    ~UARTResponseAccumulator() {
        if (pos > 0) {
            HAL_UART_Transmit_DMA(huart, reinterpret_cast<uint8_t*>(buffer), pos);
        }

    }

    void append(const char* fmt, ...) {
        if (pos + 1 >= max_size) return;

        va_list args;
        va_start(args, fmt);
        int written = npf_vsnprintf(buffer + pos, max_size - pos, fmt, args);
        va_end(args);

        if (written > 0) {
            pos += std::min(static_cast<size_t>(written), max_size - pos - 1);
            if (blocking) {
                // Config dumps run in thread mode and exceed the shared TX buffer in total.
                if (HAL_UART_Transmit(huart, reinterpret_cast<uint8_t*>(buffer), pos, 100) != HAL_OK) {
                    Error_Handler();
                }
                pos = 0;
            }
        }
    }
};
