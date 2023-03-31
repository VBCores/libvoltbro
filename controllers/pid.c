/*
 * pid.c
 *
 *  Created on: Nov 28, 2022
 *      Author: Igor Beschastnov
 */

#include <stdlib.h>
#include <string.h>

#include "pid.h"

PIDConfig* make_pid_config(
    double p_gain,
    double i_gain,
    double d_gain
) {
    PIDConfig* config = malloc(sizeof(PIDConfig));
    if (config == NULL) {
        return NULL;
    }

    make_pid_config_reserved(config, p_gain, i_gain, d_gain);

    return config;
}

void make_pid_config_reserved(
        PIDConfig* dest,
        double     p_gain,
        double     i_gain,
        double     d_gain
) {
    PIDConfig tmp_config = {
        .p_gain                = p_gain,
        .i_gain                = i_gain,
        .d_gain                = d_gain,
        .integral_error        = 0,
        .prev_error            = 0,
        .integral_error_lim    = 1000.0,
        .signal                = 0.1,  // TODO: fix! a hack
    };

    memcpy(dest, &tmp_config, sizeof(PIDConfig));
}

double regulation(PIDConfig* config, double error, double dt) {
    double error_diff = error - config->prev_error;
    config->prev_error = error;

    double integral_error = config->integral_error + error * dt;
    if (integral_error > config->integral_error_lim) {
        integral_error = config->integral_error_lim;
    }
    config->integral_error = integral_error;

    config->signal = config->p_gain*(error + config->i_gain*config->integral_error + config->d_gain*error_diff/dt);
    return config->signal;
}

#ifdef __cplusplus
}
#endif
