#pragma once
#if defined(STM32G4) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#if defined(HAL_TIM_MODULE_ENABLED) && defined(HAL_CORDIC_MODULE_ENABLED)

#include "../foc/foc.h"


struct HardLimits {
    float max_speed = NAN;
    float max_torque = NAN;
    float angle_offset = NAN;
    float min_angle = NAN;
    float max_angle = NAN;
};

class VBInverter: public BaseInverter {
private:
    const volatile uint32_t ADC_1_buffer[3] = {};
    ADC_HandleTypeDef* hadc_1;
    const volatile uint32_t ADC_2_buffer[2] = {};
    ADC_HandleTypeDef* hadc_2;

    float stator_temperature = 0;
    float mcu_temperature = 0;
public:
    VBInverter(ADC_HandleTypeDef* hadc_1, ADC_HandleTypeDef* hadc_2):
        hadc_1(hadc_1), hadc_2(hadc_2) {};

    float get_stator_temperature() const { return stator_temperature; }
    float get_mcu_temperature() const { return mcu_temperature; }

    float read_raw_A() const {
        return 3.3f * (float)ADC_1_buffer[0] / (1.0f*4096.0f);
    }
    float read_raw_B() const {
        return 3.3f * (float)ADC_2_buffer[0] / (1.0f*4096.0f);
    }
    float read_raw_V() const {
        return 16.0f * 3.3f * (float)ADC_1_buffer[1] / (1.0f*4096.0f);
    }
    float read_raw_T() const {
        return (float)ADC_1_buffer[2]*1.1f;
    }

    void start() override{
        if (has_started()) {
            return;
        }
        is_started = true;

        HAL_ADCEx_Calibration_Start(hadc_1, ADC_SINGLE_ENDED);
        HAL_ADCEx_Calibration_Start(hadc_2, ADC_SINGLE_ENDED);
        HAL_ADC_Start_DMA(hadc_1, const_cast<uint32_t *>(ADC_1_buffer), 3);
        HAL_ADC_Start_DMA(hadc_2, const_cast<uint32_t *>(ADC_2_buffer), 2);

        // Record offset;
        int cycles = 64;
        for (int i = 0; i < cycles; i++) {
            I_A_offset += read_raw_A();
            I_B_offset += read_raw_B();
            HAL_Delay(1);
        }
        I_A_offset /= (float)cycles;
        I_B_offset /= (float)cycles;
    }

    void update() override {
        if (!has_started()) {
            return;
        }

        const float shunt_res = 0.003f;
        const float op_amp_gain = 20.0f;
        const float conv_factor = shunt_res * op_amp_gain;

        I_A = (read_raw_A() - I_A_offset ) / conv_factor;
        I_B = (read_raw_B() - I_B_offset ) / conv_factor;
        busV = read_raw_V();
        I_C = -I_A - I_B;

        const float TS_CAL1_TEMP = 30u;
        const float TS_CAL2_TEMP = 130u;
        volatile float TS_CAL1 = (float)*(uint16_t*)0x1FFF75A8;
        volatile float TS_CAL2 = (float)*(uint16_t*)0x1FFF75CA;
        mcu_temperature = (TS_CAL2_TEMP - TS_CAL1_TEMP) * (read_raw_T() - TS_CAL1) / ( TS_CAL2 - TS_CAL1 ) + TS_CAL1_TEMP;

        float thermistor = 1.0f / (4095.0f / ADC_2_buffer[1] - 1);
        float steinhart = logf(thermistor);
        steinhart /= 3950.0f;
        steinhart += 1.0 / (25.0f + 273.15);
        steinhart = 1.0 / steinhart;
        stator_temperature = steinhart - 273.15;
    }
};


class VBDrive: public FOC {
public:
    VBDrive(
        float T,
        KalmanConfig&& kalman_config,
        PIDConfig&& control_config,
        PIDConfig&& q_config,
        PIDConfig&& d_config,
        const DriveLimits& drive_limits,
        const DriveInfo& drive_info,
        TIM_HandleTypeDef* htim,
        GenericEncoder& encoder,
        VBInverter& inverter
    ):
        FOC(
            T,
            std::move(kalman_config),
            std::move(control_config),
            std::move(q_config),
            std::move(d_config),
            drive_limits,
            drive_info,
            htim,
            encoder,
            inverter
        )
        {}

        //void update() override {};

        HAL_StatusTypeDef init() override {
            HAL_StatusTypeDef result = FOC::init();
            if (result != HAL_OK) {
                return result;
            }
            result = HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_1); // AN-phase
            if (result != HAL_OK) {
                return result;
            }
            result = HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_2); // BN-phase
            if (result != HAL_OK) {
                return result;
            }
            result = HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_3); // CN-phase
            return result;
        }

        virtual void set_pwm() override {
            // limiting duty cycle to give ADC time to sample current reading
            // RM0440 21.4.12
            for (size_t index = 0; index < 3; index++) {
                if (DQs[index] > 1760) {
                    DQs[index] = 1760;
                }
            }

            __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, DQs[0]);
            __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, DQs[1]);
            __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, DQs[2]);
        }

        void calibrate(CalibrationData& calibration_data) override {
            uint16_t ppair_roll_counter = 0;
            std::array<encoder_data, 28> offset_samples = {};  // TODO: make size dynamic
            float current_angle = 0.0f;
            const float d_delta = 0.25f * (float)drive_info.common.ppairs * pi2 / (float)CALIBRATION_BUFF_SIZE;

            calibration_data.is_encoder_inverted = check_if_inverted(current_angle, d_delta);
            const_cast<bool&>(encoder.is_inverted) = calibration_data.is_encoder_inverted;

            current_angle = reset_to_zero(current_angle, d_delta);
            auto advance_angle = [this, &current_angle, d_delta](float step) {
                current_angle += step;
                set_windings_calibration(current_angle);
                HAL_Delay(5);
                update_angle();
            };
            auto step_forward_a_bit = [this, &advance_angle](float step) {
                for (int i = 0; i < 256; i++) {
                    advance_angle(step);
                }
            };

            uint32_t timestamp = HAL_GetTick();
            float old_angle = raw_elec_angle;

            while( (old_angle - raw_elec_angle) < PI || (HAL_GetTick() < timestamp + 100) ) {
                old_angle = raw_elec_angle;
                advance_angle(d_delta);
                if( fabs( mfmod(current_angle, 2.0f*PI ) - PI ) < d_delta/2.0f ) {
                    offset_samples[ppair_roll_counter] = encoder.get_value();
                    ppair_roll_counter++;
                }
            }

            step_forward_a_bit(d_delta);
            current_angle = reset_to_zero(current_angle, d_delta);

            timestamp = HAL_GetTick();
            old_angle = raw_elec_angle;
            while( (old_angle - raw_elec_angle) > -PI || (HAL_GetTick() < timestamp + 100) ) {
                old_angle = raw_elec_angle;
                advance_angle(-d_delta);
                if( fabs( mfmod(current_angle, 2.0f*PI ) - PI ) < d_delta/2.0f ) {
                    offset_samples[ppair_roll_counter] = encoder.get_value();
                    ppair_roll_counter++;
                }
            }

            uint16_t measured_elec_offset = 0;
            for( int i = 0; i < drive_info.common.ppairs; i++ ) {
                int16_t enc_angle = offset_samples[i];
                measured_elec_offset += (encoder.CPR / drive_info.common.ppairs) - (enc_angle % (encoder.CPR / drive_info.common.ppairs));
            }
            measured_elec_offset /= drive_info.common.ppairs;
            calibration_data.meas_elec_offset = measured_elec_offset;
        }
};
#endif
#endif
