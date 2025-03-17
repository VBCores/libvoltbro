#pragma once
#if defined(STM32G474xx) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#if defined(HAL_TIM_MODULE_ENABLED) && defined(HAL_CORDIC_MODULE_ENABLED)

#include "../bldc.h"

#include "voltbro/encoders/generic.h"

enum class FOCMode {
    NORMAL,
    PI_CURRENT
};

using calibration_array_t = const std::array<int, 1024>;

class FOC: public BLDCController  {
private:
    FOCMode mode;
    GenericEncoder& encoder;
    float T;
    float raw_elec_angle = 0;
    float elec_angle = 0;
    //calibration_array_t& lookup_table;

    void apply_kalman();
    void update_angle();

    void set_windings_calibration(float current_angle);
    float reset_to_zero(float start_angle, float d_delta);
public:
    void update_sensors();
    void calibrate();

    FOC(
        float T,
        float user_current_limit,
        DriveInfo&& drive_info,
        TIM_HandleTypeDef* htim,
        ADC_HandleTypeDef* hadc,
        GenericEncoder& encoder,
        //calibration_array_t& lookup_table,
        FOCMode mode = FOCMode::NORMAL
    ):
        BLDCController(
            user_current_limit,
            std::move(drive_info),
            htim,
            hadc
        ),
        mode(mode),
        encoder(encoder),
        T(T)
        //lookup_table(lookup_table)
        {}

    float get_electric_angle() {
        return elec_angle;
    }

    void update() override;
};

#endif
#endif
