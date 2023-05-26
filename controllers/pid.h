/*
 * pid.h
 *
 *  Created on: Nov 28, 2022
 *      Author: Igor Beschastnov
 */

#ifndef VOLTBROLIB_CONTROLLERS_PID_H_
#define VOLTBROLIB_CONTROLLERS_PID_H_

#include <stdbool.h>
#ifdef STM32_G
#include "stm32g4xx_hal.h"
#else
#include "stm32f4xx_hal.h"
#endif
#include "utils.h"

#ifdef __cplusplus
#include <cstdint>
extern "C" {
#else
#include <stdint.h>
#endif

typedef struct PIDConfig {
    double p_gain;
    double i_gain;
    double d_gain;
    double integral_error;
    double prev_error;
    double integral_error_lim;
    double signal;
} PIDConfig;

PIDConfig* make_pid_config(double p_gain, double i_gain, double d_gain);

void make_pid_config_reserved(
    PIDConfig* dest,
    double p_gain,
    double i_gain,
    double d_gain
);

double regulation(PIDConfig* config, double error, double dt);

#ifdef __cplusplus
}
#endif

#endif /* VOLTBROLIB_CONTROLLERS_PID_H_ */
