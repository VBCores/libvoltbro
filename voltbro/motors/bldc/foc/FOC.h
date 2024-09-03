#pragma once
#if defined(STM32G474xx) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#if defined(HAL_TIM_MODULE_ENABLED) && defined(HAL_CORDIC_MODULE_ENABLED)

#include "../bldc.h"

#include "voltbro/encoders/AS5048A/AS5048A.h"

enum class FOCMode {
    NORMAL,
    PI_CURRENT
};

using calibration_array_t = const std::array<int, 1024>;

class FOC: public BLDCController  {
private:
    const FOCMode mode;
    AS5048A encoder;
    float T;
    float raw_elec_angle = 0;
    float elec_angle = 0;
    calibration_array_t& lookup_table;

    void apply_kalman();
    void update_angle();
#ifdef ENABLE_CALIBRATION
    void set_windings_calibration();
    void reset_to_zero(float d_delta);
#endif
public:
#ifdef ENABLE_CALIBRATION
    void calibrate();
#endif
    FOC(
        float T,
        DriveInfo&& drive_info,
        ControlConfig&& control_config,
        AS5048A&& encoder,
        TIM_HandleTypeDef* htim,
        ADC_HandleTypeDef* hadc,
        calibration_array_t& lookup_table,
        FOCMode mode = FOCMode::NORMAL
    ):
        BLDCController(
            std::forward<DriveInfo>(drive_info),
            std::forward<ControlConfig>(control_config),
            htim,
            hadc,
            1,
            1
        ),
        mode(mode),
        encoder(std::forward<AS5048A>(encoder)),
        T(T),
        lookup_table(lookup_table)
        {}

    void init() {
        for (auto pin : drive_info.L_PINS) {
            HAL_GPIO_WritePin(GPIOB, pin, GPIO_PIN_SET);
        }
        HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1); // A-phase
        HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2); // B-phase
        HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3); // C-phase
    }
    void regulate(float _ = 0) override;

    float get_torque() const {
        return shaft_torque;
    }

    const AS5048A& get_encoder() {
        return encoder;
    }
};

#endif
#endif