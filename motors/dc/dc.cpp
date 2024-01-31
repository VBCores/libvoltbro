#include "dc.h"
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
    double Vref = Ipeak * config.Rsense * 10;
    uint32_t dac_val = dac_value(4095 * Vref / 3.3);
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

void DCMotorController::regulate(GenericEncoder& encoder, float dt) const{
    encoder.update_value(angle_filter);
    float motor_angle = calculate_angle(config.common, encoder);

    const float rads_per_rev = pi2 / config.common.gear_ratio;
    int revolutions = encoder.get_revolutions() % config.common.gear_ratio;
    float base_angle = 0;
    if (revolutions < 0) {
        base_angle = (config.common.gear_ratio + revolutions) * rads_per_rev;
    }
    else {
        base_angle = revolutions * rads_per_rev;
    }
    angle = base_angle + (motor_angle / config.common.gear_ratio);
    speed = calculate_speed(angle, dt);
}

HAL_StatusTypeDef DCMotorController::init() {
    auto status = HAL_TIM_PWM_Start(config.timer, config.IN1_channel);
    if (status != HAL_OK) return status;
    status = HAL_TIM_PWM_Start(config.timer, config.IN2_channel);
    if (status != HAL_OK) return status;
    return stop();
}
