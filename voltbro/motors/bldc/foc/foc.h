#pragma once
#if defined(STM32G4) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#if defined(HAL_TIM_MODULE_ENABLED) && defined(HAL_CORDIC_MODULE_ENABLED)

#include <string_view>

#include "../bldc.h"
#include "voltbro/encoders/generic.h"
#include "voltbro/math/regulators/pid.hpp"

constexpr size_t CALIBRATION_BUFF_SIZE = 1024;
using __non_const_calib_array_t = std::array<int, CALIBRATION_BUFF_SIZE>;
using calibration_array_t = const __non_const_calib_array_t;
struct CalibrationData {
    static constexpr uint32_t TYPE_ID = 0x33FEDCBA;
    uint32_t type_id;
    bool was_calibrated = false;
    bool is_encoder_inverted;
    uint16_t ppair_counter;
    uint16_t meas_elec_offset;
    #ifdef USE_CALIBRATION_ARRAY
    __non_const_calib_array_t calibration_array;
    #endif

    void reset() {
        type_id = TYPE_ID;
        was_calibrated = false;
        is_encoder_inverted = false;
        ppair_counter = 0;
        meas_elec_offset = 0;
        #ifdef USE_CALIBRATION_ARRAY
        calibration_array.fill(0);
        #endif
    }
};

struct KalmanConfig {
    float expected_a;
    float g1;
    float g2;
    float g3;
};

/**
 * Field oriented control.
 */
class FOC: public BLDCController  {
protected:
    GenericEncoder& encoder;
    PIDRegulator q_reg;
    PIDRegulator d_reg;
    PIDRegulator control_reg;
    const KalmanConfig kalman_config;
    float T;
    float raw_elec_angle = 0;
    float elec_angle = 0;
    calibration_array_t* lookup_table = nullptr;
    bool is_limited = false;

    void apply_kalman();
    void update_angle();

    void set_windings_calibration(float current_angle);
    float reset_to_zero(float start_angle, float d_delta);
    bool check_if_inverted(float start_angle, float d_delta);
public:
    virtual void calibrate(CalibrationData& calibration_data);
    void apply_calibration(CalibrationData& calibration_data) {
        // Replace config parameters with the ones loaded from EEPROM
        // Very ugly hack, but it will work for now
        const_cast<int&>(encoder.electric_offset) = calibration_data.meas_elec_offset;
        #ifdef USE_CALIBRATION_ARRAY
        lookup_table = &(calibration_data.calibration_array);
        #endif
        const_cast<bool&>(encoder.is_inverted) = calibration_data.is_encoder_inverted;
    }

    FOC(
        float T,
        KalmanConfig&& kalman_config,
        PIDConfig&& control_config,
        PIDConfig&& q_config,
        PIDConfig&& d_config,
        const DriveLimits& drive_limits,
        const DriveInfo& drive_info,
        TIM_HandleTypeDef* htim,
        GenericEncoder& encoder,
        BaseInverter& inverter
    ):
        BLDCController(
            drive_limits,
            drive_info,
            htim,
            inverter
        ),
        encoder(encoder),
        T(T),
        control_reg(std::move(control_config)),
        q_reg(std::move(q_config)),
        d_reg(std::move(d_config)),
        kalman_config(std::move(kalman_config))
        {}

    float get_electric_angle() {
        return elec_angle;
    }

    void update_q_config(PIDConfig&& new_config) {
        q_reg.update_config(std::move(new_config));
    }
    void update_d_config(PIDConfig&& new_config) {
        d_reg.update_config(std::move(new_config));
    }
    void update_control_config(PIDConfig&& new_config) {
        control_reg.update_config(std::move(new_config));
    }
    const GenericEncoder& get_encoder() const {
        return encoder;
    }

    HAL_StatusTypeDef init() override {
        HAL_StatusTypeDef result = BLDCController::init();
        if (result != HAL_OK) {
            return result;
        }

        return encoder.init();
    }
    void update() override;
    virtual void update_sensors();
};

#endif
#endif
