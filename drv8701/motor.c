/*
 * motor.c
 *
 *  Created on: Oct 19, 2022
 *      Author: igor
 */

#ifdef HAL_DAC_MODULE_ENABLED
#ifdef HAL_TIM_MODULE_ENABLED

#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "motor.h"

const double CIRCLE_RADIANS = M_PI * 2;

DriverConfig* create_driver_config(
    encoder_data pulses_per_revolution,
    uint32_t microseconds_per_revolution,
    pin nSLEEP_pin,
    GPIO_TypeDef* nSLEEP_GPIOx,
    pwm_channel IN1_CHANNEL,
    pwm_channel IN2_CHANNEL,
    TIM_HandleTypeDef* timer,
    dac_channel dac_channel_,
    DAC_HandleTypeDef* dac
) {
    DriverConfig* config;
    CRITICAL_SECTION({ config = malloc(sizeof(DriverConfig)); })
    if (config == NULL) {
        return NULL;
    }

    DriverConfig tmp_config = {
        .pulses_per_revolution = pulses_per_revolution,
        .microseconds_per_revolution = microseconds_per_revolution,
        .nSLEEP_pin = nSLEEP_pin,
        .nSLEEP_GPIOx = nSLEEP_GPIOx,
        .IN1_channel = IN1_CHANNEL,
        .IN2_channel = IN2_CHANNEL,
        .timer = timer,
        .dac = dac,
        .dac_channel_ = dac_channel_,
        .target_radian_speed = 0,
        .pid_config = NULL,
        .Ichop = -1,
        .is_small = false,
    };
    // Fixed for our driver
    tmp_config.Av = 20;
    tmp_config.Rsense = 0.005;
    tmp_config.Voff = 0.0;
    memcpy(config, &tmp_config, sizeof(DriverConfig));

    config->period = __HAL_TIM_GET_AUTORELOAD(timer);
    config->speed = 0;

    return config;
}

int motor_init(DriverConfig* config) {
    HAL_TIM_PWM_Start(config->timer, config->IN1_channel);
    HAL_TIM_PWM_Start(config->timer, config->IN2_channel);
    HAL_DAC_Start(config->dac, config->dac_channel_);
    return motor_stop(config);
}

int motor_set_speed(DriverConfig* config, double speed) {
    if (speed == 0) {
        return motor_stop(config);
    }
    if (speed > 1) {
        speed = 1.0;
    }
    pwm_channel channel_on =
        (speed < 0) ? config->IN1_channel : config->IN2_channel;
    pwm_channel channel_off =
        (speed < 0) ? config->IN2_channel : config->IN1_channel;
    tim_register compare = (tim_register)(config->period * fabs(speed));
    __HAL_TIM_SET_COMPARE(config->timer, channel_on, compare);
    __HAL_TIM_SET_COMPARE(config->timer, channel_off, 0);

    config->speed = speed;

    return 1;
}

int motor_tim_set_period(DriverConfig* config, tim_register period) {
    if (period < 1) {
        errno = EINVAL;
        return -1;
    }
    __HAL_TIM_SET_AUTORELOAD(config->timer, period);
    config->period = period;
    return motor_set_speed(config, config->speed);
}

int motor_stop(DriverConfig* config) {
    HAL_GPIO_WritePin(config->nSLEEP_GPIOx, config->nSLEEP_pin, GPIO_PIN_RESET);
    config->is_stopped = true;
    return 1;
}

int motor_start(DriverConfig* config) {
    HAL_GPIO_WritePin(config->nSLEEP_GPIOx, config->nSLEEP_pin, GPIO_PIN_SET);
    config->is_stopped = false;
    return 1;
}

double motor_get_radian_speed(DriverConfig* config) {
    return config->target_radian_speed;
}

int motor_set_radian_speed(DriverConfig* config, double radianSpeed) {
    config->target_radian_speed = radianSpeed;
    return motor_set_speed(config, radian_speed_to_pwm(config, radianSpeed));
}

double
get_motor_error(DriverConfig* config, int32_t encoder_diff, uint32_t dt) {
    return config->target_radian_speed -
           get_motor_actual_speed(config, encoder_diff, dt);
}

double Vref_from_Ichop(DriverConfig* config, double Ichop) {
    return Ichop * (config->Av * config->Rsense) + config->Voff;
}

int motor_set_Ichop(DriverConfig* config, double Ichop) {
    if (Ichop < 0) {
        return -1;
    }
    bool error = false;
    double vref = Vref_from_Ichop(config, Ichop);

    // if Rsense is small and required Ichop is so low that Vref is lost in
    // "noise"
    if (vref < 0.04 && Ichop > 0) {
        // then set Vref higher so motor keeps running but return error;
        vref = 0.05;
        error = true;
    }

    uint32_t dac_val = dac_value(vref);
    HAL_StatusTypeDef status = HAL_DAC_SetValue(
        config->dac,
        config->dac_channel_,
        DAC_ALIGN_12B_R,
        dac_val
    );
    if (status == HAL_OK) {
        config->Ichop = Ichop;
    }
    return status == HAL_OK && !error;
}

#endif
#endif