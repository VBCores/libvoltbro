#if defined(STM32G474xx) || defined(STM32_G)
#include "dc.h"
#if defined(HAL_DAC_MODULE_ENABLED) && defined(HAL_TIM_MODULE_ENABLED)

#include "arm_math.h"

HAL_StatusTypeDef DCMotorController::stop() {
    return set_state(false);
}

HAL_StatusTypeDef DCMotorController::start() {
    return set_state(true);
}

HAL_StatusTypeDef DCMotorController::set_state(bool state) {
    HAL_GPIO_WritePin(
        config.nSLEEP_GPIOx,
        config.nSLEEP_pin,
        state ? GPIO_PIN_SET : GPIO_PIN_RESET
    );
    _is_on = state;
    return HAL_OK;
}

HAL_StatusTypeDef DCMotorController::set_Ipeak(float new_Ipeak) {
    Ipeak = new_Ipeak;
    double Vref = Ipeak * config.Rsense * 10 * 2;
    uint32_t dac_val = dac_value(Vref);
    return HAL_DAC_SetValue(
        config.dac,
        config.dac_channel_,
        DAC_ALIGN_12B_R,
        dac_val
    );
}

void DCMotorController::set_target_speed(float new_target_speed) {
    target_speed = new_target_speed;

}

void DCMotorController::regulate(float dt) {
    if (is_close(target_speed, 0)) {
        tim_register zero_pwm = 0;
        if (is_using_brake) {
            // Active brake
            zero_pwm = config.max_pwm;
        }
        __HAL_TIM_SET_COMPARE(config.timer, config.IN1_channel, zero_pwm);
        __HAL_TIM_SET_COMPARE(config.timer, config.IN2_channel, zero_pwm);
        current_pwm = 0;
        return;
    }

    float regulation = regulator.regulation(speed_error, dt);
    current_pwm += regulation;
    if (abs(current_pwm) > config.max_pwm) {
        current_pwm = copysign(config.max_pwm, current_pwm);
    }
    else if (abs(current_pwm) < config.min_pwm) {
        current_pwm = copysign(config.min_pwm, target_speed);
    }

    if (current_pwm < 0) {
        __HAL_TIM_SET_COMPARE(config.timer, config.IN2_channel, (uint16_t)abs(current_pwm));
        __HAL_TIM_SET_COMPARE(config.timer, config.IN1_channel, 0);
    }
    else {
        __HAL_TIM_SET_COMPARE(config.timer, config.IN2_channel, 0);
        __HAL_TIM_SET_COMPARE(config.timer, config.IN1_channel, (uint16_t)abs(current_pwm));
    }
}

HAL_StatusTypeDef DCMotorController::init() {
    HAL_DAC_Start(config.dac, config.dac_channel_);
    auto status = HAL_TIM_PWM_Start(config.timer, config.IN1_channel);
    if (status != HAL_OK) return status;
    status = HAL_TIM_PWM_Start(config.timer, config.IN2_channel);
    __HAL_TIM_SET_COMPARE(config.timer, config.IN1_channel, 0);
    __HAL_TIM_SET_COMPARE(config.timer, config.IN2_channel, 0);
    if (status != HAL_OK) return status;
    return stop();
}

void DCMotorController::update_angle(GenericEncoder &speed_encoder) {
    speed_encoder.update_value();
    float motor_angle = calculate_angle(config.common, speed_encoder);

    const float rads_per_rev = pi2 / config.common.gear_ratio;
    int revolutions = speed_encoder.get_revolutions() % config.common.gear_ratio;
    float base_angle = 0;
    if (revolutions < 0) {
        base_angle = (config.common.gear_ratio + revolutions) * rads_per_rev;
    }
    else {
        base_angle = revolutions * rads_per_rev;
    }
    angle = base_angle + (motor_angle / config.common.gear_ratio);
}

void DCMotorController::update_speed(const float dt) {
    speed = calculate_speed(angle, dt);
    speed_error = target_speed - speed;
}

#endif
#endif
