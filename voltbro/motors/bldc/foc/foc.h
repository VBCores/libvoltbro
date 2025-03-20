#pragma once
#if defined(STM32G474xx) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#if defined(HAL_TIM_MODULE_ENABLED) && defined(HAL_CORDIC_MODULE_ENABLED)

#include <string_view>

#include "../bldc.h"
#include "voltbro/encoders/generic.h"

enum class FOCMode {
    NORMAL,
    PI_CURRENT
};

constexpr size_t CALIBRATION_BUFF_SIZE = 1024;
using __non_const_calib_array_t = std::array<int, CALIBRATION_BUFF_SIZE>;
using calibration_array_t = const __non_const_calib_array_t;
struct CalibrationData {
    static constexpr uint32_t TYPE_ID = 0x89ABCDEF;
    uint32_t type_id;
    bool was_calibrated = false;
    bool is_encoder_inverted;
    uint16_t ppair_counter;
    uint16_t meas_elec_offset;
    __non_const_calib_array_t calibration_array;
};

class FOC: public BLDCController  {
private:
    FOCMode mode;
    GenericEncoder& encoder;
    float T;
    float raw_elec_angle = 0;
    float elec_angle = 0;
    calibration_array_t* lookup_table = nullptr;

    void apply_kalman();
    void update_angle();

    void set_windings_calibration(float current_angle);
    float reset_to_zero(float start_angle, float d_delta);
    bool check_if_inverted(float start_angle, float d_delta);
public:
    void calibrate(CalibrationData* calibration_data);
    void apply_calibration(CalibrationData* calibration_data) {
        // Replace config parameters with the ones loaded from EEPROM
        // Very ugly hack, but it will work for now
        const_cast<int&>(encoder.electric_offset) = calibration_data->meas_elec_offset;
        lookup_table = &(calibration_data->calibration_array);
        const_cast<bool&>(encoder.is_inverted) = calibration_data->is_encoder_inverted;
    }

    void update_sensors();

    FOC(
        float T,
        float user_current_limit,
        DriveInfo&& drive_info,
        TIM_HandleTypeDef* htim,
        ADC_HandleTypeDef* hadc,
        GenericEncoder& encoder,
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
        {}

    float get_electric_angle() {
        return elec_angle;
    }

    void update() override;
};

#endif
#endif
