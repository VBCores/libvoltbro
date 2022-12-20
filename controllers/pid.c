/*
 * pid.c
 *
 *  Created on: Nov 28, 2022
 *      Author: Igor Beschastnov
 */

#include <stdlib.h>
#include <string.h>

#include "pid.h"

PIDConfig* create_pid_config(
    double p_gain,
    double i_gain,
    double d_gain
) {
    PIDConfig* config = malloc(sizeof(PIDConfig));
    if (config == NULL) {
        return NULL;
    }

    PIDConfig tmp_config = {
        .p_gain                = p_gain,
        .i_gain                = i_gain,
        .d_gain                = d_gain,
        .integral_error        = 0,
        .prev_error            = 0,
        .integral_error_lim    = 1000.0,
    };

    memcpy(config, &tmp_config, sizeof(PIDConfig));

    return config;
}

double regulation(PIDConfig* config, double error, uint32_t dt) {
    double error_diff = error - config->prev_error;
    config->prev_error = error;
    double integral_error = config->integral_error + error * dt;
    if (integral_error > config->integral_error_lim) {
        integral_error = config->integral_error_lim;
    }
    config->integral_error = integral_error;
    return config->p_gain*error + config->i_gain*config->integral_error + config->d_gain*error_diff/dt;
}
