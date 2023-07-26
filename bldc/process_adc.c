/*
 * process_ADC.c
 *
 *  Created on: Mar 23, 2023
 *      Author: Igor Beschastnov,
 */

#include "bldc.h"

void process_ADC(InverterState* inverter, const uint32_t ADC_buf[]) {
    const float shunt_res = 0.045f;  // 0.045 Ohm shunt resistance
    const float op_amp_gain = 1.0f;  // current sensor gain
    inverter->I_A = ((3.3f * (float)ADC_buf[1] / (16.0f * 4096.0f)) - inverter->I_A_offset) /
                    (shunt_res * op_amp_gain);
    inverter->I_B = ((3.3f * (float)ADC_buf[2] / (16.0f * 4096.0f)) - inverter->I_B_offset) /
                    (shunt_res * op_amp_gain);
    inverter->I_C = ((3.3f * (float)ADC_buf[3] / (16.0f * 4096.0f)) - inverter->I_C_offset) /
                    (shunt_res * op_amp_gain);
    inverter->busV = 12.0f * 3.3f * ADC_buf[0] / 4096.0f / 16.0f;  // drivers input voltage
}