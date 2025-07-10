#include "foc.h"
#if defined(HAL_TIM_MODULE_ENABLED) && defined(HAL_ADC_MODULE_ENABLED) && defined(HAL_CORDIC_MODULE_ENABLED)

#include "voltbro/math/transform.hpp"

#ifdef DEBUG
static volatile float current_target_angle = 0;
static volatile encoder_data current_encoder_value = 0;
#define update_angle(); \
    update_angle(); current_encoder_value = encoder.get_value();
#endif

void FOC::set_windings_calibration(float target_angle) {
    #ifdef DEBUG
    current_target_angle = target_angle;
    #endif
    float s = arm_sin_f32(target_angle);
    float c = arm_cos_f32(target_angle);

    const float V_d = -drive_info.calibration_voltage;
    const float V_q = 0.0f;

    float DVA, DVB, DVC;

    //inverse dq0 transform on voltages
    abc(s, c, V_d, V_q, &DVA, &DVB, &DVC);

    DQs[0] = 1000 + (int16_t)(1000.0f*DVA);
    DQs[1] = 1000 + (int16_t)(1000.0f*DVB);
    DQs[2] = 1000 + (int16_t)(1000.0f*DVC);

    set_pwm();
}

bool FOC::check_if_inverted(float start_angle, float d_delta) {
    float current_angle = start_angle;

    float old_angle = raw_elec_angle;
    float diff = 0;
    size_t step_counter = 0;
    size_t decrease_counter = 0;
    size_t increase_counter = 0;
    while(step_counter < 200) {
        step_counter += 1;
        diff = old_angle - raw_elec_angle;
        if (diff > 0) {
            // Angle is decreasing
            decrease_counter += 1;
        }
        else {
            increase_counter += 1;
        }

        old_angle = raw_elec_angle;
        current_angle += d_delta;
        set_windings_calibration(current_angle);
        HAL_Delay(5);
        update_angle();
    }

    return decrease_counter >= increase_counter;
}

float FOC::reset_to_zero(float start_angle, float d_delta) {
    float current_angle = start_angle;

    float old_angle = raw_elec_angle;
    while((old_angle - raw_elec_angle) < PI) {
        old_angle = raw_elec_angle;
        current_angle += d_delta;
        set_windings_calibration(current_angle);
        HAL_Delay(5);
        update_angle();
    }

    return raw_elec_angle;
}

void FOC::calibrate(CalibrationData& calibration_data) {
    uint16_t offset_samples[CALIBRATION_BUFF_SIZE] = {0};
    float CalBuf_Fwd[CALIBRATION_BUFF_SIZE] = {0};
    float CalBuf_Bckwd[CALIBRATION_BUFF_SIZE] = {0};

    float current_angle = 0;
    float d_delta = 0.25f * (float)drive_info.common.ppairs * pi2 / (float)CALIBRATION_BUFF_SIZE;

    calibration_data.is_encoder_inverted = check_if_inverted(current_angle, d_delta);
    const_cast<bool&>(encoder.is_inverted) = calibration_data.is_encoder_inverted;

    current_angle = reset_to_zero(current_angle, d_delta);
    float d_offset = current_angle;

    uint32_t timestamp = HAL_GetTick();
    float old_angle = raw_elec_angle;
    // rotate in positive direction until you make full mechanical rotation
    // f_elec_angle < 2*PI
    while( (old_angle - raw_elec_angle) < PI || (HAL_GetTick() < timestamp + 100) ) {
        old_angle = raw_elec_angle;

        uint16_t index = (uint16_t)( (float)CALIBRATION_BUFF_SIZE*raw_elec_angle / (2.0f*PI) );
        CalBuf_Fwd[ index ] = ( current_angle - d_offset ) / drive_info.common.ppairs - raw_elec_angle;
        // current algorithm sometimes leaves empty values in calibration buffer
        // this part fills the upcoming array member with the current value, which is overwritten in case of successful new reading
        if( index < CALIBRATION_BUFF_SIZE ) {
            CalBuf_Fwd[ index + 1 ] = CalBuf_Fwd[ index ];
        }

        current_angle += d_delta;

        set_windings_calibration(current_angle);
        HAL_Delay(5);
        update_angle();

        if( fabs( mfmod( current_angle, 2.0f*PI ) - PI ) < d_delta / 2.0f ) {
            offset_samples[calibration_data.ppair_counter] = encoder.get_value();
            calibration_data.ppair_counter += 1;
        }
    }

    for(int i = 0; i < 256; i++) {
        current_angle += d_delta;
        set_windings_calibration(current_angle);
        HAL_Delay(5);
        update_angle();
    }

    // NOTE: ZERO
    current_angle = reset_to_zero(current_angle, d_delta);

    d_offset = current_angle - (2.0f*PI) * drive_info.common.ppairs;

    timestamp = HAL_GetTick();
    old_angle = raw_elec_angle;
    // rotate in negative direction until you return to the beginning
    while ( (old_angle - raw_elec_angle) > -PI || (HAL_GetTick() < timestamp + 100) ) {
        old_angle = raw_elec_angle;

        uint16_t index = (uint16_t)( (float)CALIBRATION_BUFF_SIZE*raw_elec_angle / (2.0f*PI) );
        CalBuf_Bckwd[index] = (current_angle - d_offset) / drive_info.common.ppairs - raw_elec_angle;
        // current algorithm sometimes leaves empty values in calibration buffer
        // this part fills the upcoming array member with the current value, which is overwritten in case of successful new reading
        if(index > 0) {
            CalBuf_Bckwd[index - 1] = CalBuf_Bckwd[index];
        }

        current_angle -= d_delta;

        set_windings_calibration(current_angle);
        HAL_Delay(5);
        update_angle();

        if( fabs( mfmod( current_angle, 2.0f*PI ) - PI ) < d_delta/2.0f ) {
            offset_samples[calibration_data.ppair_counter] = encoder.get_value();
            calibration_data.ppair_counter += 1;
        }
    }

    for( int i = 0; i < drive_info.common.ppairs; i++ ) {
        int16_t enc_angle = offset_samples[i];
        calibration_data.meas_elec_offset += (encoder.CPR / drive_info.common.ppairs) - (enc_angle % (encoder.CPR / drive_info.common.ppairs));
    }
    calibration_data.meas_elec_offset /= drive_info.common.ppairs;

    // TODO: calculate calibration curve here
}

#endif
