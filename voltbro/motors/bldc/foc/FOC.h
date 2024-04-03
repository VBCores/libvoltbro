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

    float kalman_velocity(float new_angle);
public:
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

    void callback() override;
};

#endif
#endif