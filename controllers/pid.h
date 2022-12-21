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

typedef struct PIDConfig {
    const double p_gain;
    const double i_gain;
    const double d_gain;
    double integral_error;
    double prev_error;
    double integral_error_lim;
} PIDConfig;

#ifdef __cplusplus
extern "C" {
#endif

PIDConfig *create_pid_config(
    double p_gain,
    double i_gain,
    double d_gain
);

double regulation(PIDConfig *config, double error, uint32_t dt);

#ifdef __cplusplus
}
#endif

#endif /* VOLTBROLIB_CONTROLLERS_PID_H_ */
