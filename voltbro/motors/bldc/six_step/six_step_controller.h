#pragma once
#if defined(STM32G4) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#if defined(HAL_TIM_MODULE_ENABLED) && defined(HAL_ADC_MODULE_ENABLED)

#include <memory>

#include "../bldc.h"

/**
 * Simple controller with 6-step switching logic.
 */
class SixStepController: public BLDCController {
private:
    HallSensor& hall_sensor;
public:
    SixStepController(
        float user_current_limit,
        DriveInfo&& drive_info,
        TIM_HandleTypeDef* htim,
        ADC_HandleTypeDef* hadc,
        HallSensor& hall_sensor
    ):
        BLDCController(
            user_current_limit,
            std::move(drive_info),
            htim,
            hadc
        ),
        hall_sensor(hall_sensor)
    {}

    void update() override;
};

#endif
#endif
