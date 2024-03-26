#pragma once

#if defined(STM32G474xx) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#include <cstdint>

class Inverter {
private:
    const uint32_t* const ADC_buffer;

    float I_A_offset = 0;
    float I_B_offset = 0;
    float I_C_offset = 0;
    float I_A = 0;
    float I_B = 0;
    float I_C = 0;
    float busV = 0;
public:
    Inverter(const uint32_t* ADC_buffer):
        ADC_buffer(ADC_buffer)
        {}

    float get_A() const { return I_A; }
    float get_B() const { return I_B; }
    float get_C() const { return I_C; }
    float get_busV() const { return busV; }

    void start() {
    }

    void update() {
        const float shunt_res = 0.045f;  // 0.045 Ohm shunt resistance
        const float op_amp_gain = 1.0f;  // current sensor gain
        I_A = ((3.3f * (float)ADC_buffer[1] / (16.0f * 4096.0f)) - I_A_offset) /
              (shunt_res * op_amp_gain);
        I_B = ((3.3f * (float)ADC_buffer[2] / (16.0f * 4096.0f)) - I_B_offset) /
              (shunt_res * op_amp_gain);
        I_C = ((3.3f * (float)ADC_buffer[3] / (16.0f * 4096.0f)) - I_C_offset) /
              (shunt_res * op_amp_gain);
        busV = 12.0f * 3.3f * ADC_buffer[0] / 4096.0f / 16.0f;  // drivers input voltage
    }
};

#endif