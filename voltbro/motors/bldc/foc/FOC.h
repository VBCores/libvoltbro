#pragma once
#if defined(STM32G474xx) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#ifdef HAL_TIM_MODULE_ENABLED

#include "../bldc.h"

#include "voltbro/encoders/AS5048A/AS5048A.h"

class FOC: public BLDCController  {
private:
    AS5048A encoder;
    float T;
    float elec_angle = 0;

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
        float angle_filter = 1,
        float speed_filter = 1
    ):
        BLDCController(
            std::forward<DriveInfo>(drive_info),
            std::forward<ControlConfig>(control_config),
            htim,
            hadc,
            angle_filter,
            speed_filter
        ),
        encoder(std::forward<AS5048A>(encoder)),
        T(T)
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
};



#endif
#endif