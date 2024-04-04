#pragma once
#if defined(STM32G474xx) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#ifdef HAL_TIM_MODULE_ENABLED

#include <memory>

#include "../bldc.h"

enum class SetPointType { VELOCITY, VOLTAGE };

class SixStepController: public BLDCController {
private:
    HallSensor& hall_sensor;
    const SetPointType point_type;

    const float T;  // control loop period, sec
public:
    SixStepController(
        float T,
        SetPointType point_type,
        DriveInfo&& drive_info,
        ControlConfig&& control_config,
        TIM_HandleTypeDef* htim,
        ADC_HandleTypeDef* hadc,
        HallSensor& hall_sensor
    ):
        BLDCController(
            std::forward<DriveInfo>(drive_info),
            std::forward<ControlConfig>(control_config),
            htim,
            hadc
        ),
        hall_sensor(hall_sensor),
        point_type(point_type),
        T(T)
    {}

    void hall_six_step_control_callback();
    void regulate(float _ = 0) override;
};

#endif
#endif
