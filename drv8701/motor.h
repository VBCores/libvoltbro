/*
 * motor.h
 *
 *  Created on: Oct 19, 2022
 *      Author: igor
 */

#ifndef VOLTBROLIB_DRV8701_MOTOR_H_
#define VOLTBROLIB_DRV8701_MOTOR_H_

#include <stdbool.h>
#ifdef STM32_G
#include "stm32g4xx_hal.h"
#else
#include "stm32f4xx_hal.h"
#endif
#include "utils.h"
#include "controllers/pid.h"

typedef uint16_t encoder_data;

typedef struct DriverConfig {
    bool is_stopped;
    bool is_small;
    const encoder_data pulses_per_revolution;
    const pin nSLEEP_pin;
    const pwm_channel IN1_channel;
    const pwm_channel IN2_channel;
    const dac_channel dac_channel_;
    tim_register period;
    uint32_t microseconds_per_revolution;
    GPIO_TypeDef* const nSLEEP_GPIOx;
    TIM_HandleTypeDef* const timer;
    DAC_HandleTypeDef* const dac;
    double speed;
    double target_radian_speed;
    double Av;
    double Rsense;
    double Voff;
    double Ichop;
    PIDConfig* pid_config;
} DriverConfig;

#ifdef __cplusplus
extern "C" {
#endif

DriverConfig* create_driver_config(
    encoder_data      pulses_per_revolution,
    uint32_t          microseconds_per_revolution,
    pin               nSLEEP_pin,
    GPIO_TypeDef*     nSLEEP_GPIOx,
    pwm_channel       IN1_CHANNEL,
    pwm_channel       IN2_CHANNEL,
    TIM_HandleTypeDef *timer,
    dac_channel       dac_channel_,
    DAC_HandleTypeDef *dac
);
int           motor_init(DriverConfig* config);
int           motor_set_speed(DriverConfig* config, double speed);
int           motor_stop(DriverConfig* config);
int           motor_start(DriverConfig* config);
int           motor_tim_set_period(DriverConfig* config, tim_register period);
double        motor_get_radian_speed(DriverConfig* config);
int           motor_set_radian_speed(DriverConfig* config, double radianSpeed);
double        get_motor_error(DriverConfig* config, int32_t encoder_diff, uint32_t dt);
double        Vref_from_Ichop(DriverConfig* config, double Ichop);
int           motor_set_Ichop(DriverConfig* config, double Ichop);

extern const double CIRCLE_RADIANS;

// Following functions do simple calculations that compiler probably optimizes into a series of register operations,
// and they are called in performance critical places, so they are forced to be inlined

__attribute__((always_inline)) static inline double get_motor_actual_speed(
    DriverConfig* config,
    int32_t encoder_diff,
    uint32_t dt
) {
    double radians_passed = ((double)encoder_diff / config->pulses_per_revolution) * CIRCLE_RADIANS;
    double radian_speed = radians_passed / dt * 1000;
    return radian_speed;
}

__attribute__((always_inline)) static inline double radian_speed_to_pwm(DriverConfig* config, double radian_speed) {
    double part_of_speed = config->microseconds_per_revolution / ((CIRCLE_RADIANS / radian_speed) * 1000);
    if (config->is_small) {
        // If motor is "small", at very low voltages (e.g. < 0.5Vmax) it almost doesn't move at all - not enough current
        // So we must move PWM range from 0-1 to 0.5-1
        // It DOES work with 0-1 but PID signal is a lot smoother like this (again, for small motors)
        return (part_of_speed / 2) + 0.5;
    }
    return part_of_speed;
}

#ifdef __cplusplus
}
#endif

#endif /* VOLTBROLIB_DRV8701_MOTOR_H_ */
