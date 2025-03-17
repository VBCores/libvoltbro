#include "foc.h"
#if defined(HAL_TIM_MODULE_ENABLED) && defined(HAL_ADC_MODULE_ENABLED)

#include "voltbro/math/transform.hpp"

void FOC::set_windings_calibration(float target_angle) {
    float s = arm_sin_f32(target_angle);
    float c = arm_cos_f32(target_angle);

    const float V_d = -0.2f;
    const float V_q = 0.0f;

    float DVA, DVB, DVC;

    //inverse dq0 transform on voltages
    abc(s, c, V_d, V_q, &DVA, &DVB, &DVC);

    DQs[0] = 1000 + (int16_t)(1000.0f*DVA);
    DQs[1] = 1000 + (int16_t)(1000.0f*DVB);
    DQs[2] = 1000 + (int16_t)(1000.0f*DVC);

    set_pwm();
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

#define cal_buf_len 1024
uint16_t ppair_counter = 0;
uint16_t meas_elec_offset = 0;
uint16_t offset_samples[cal_buf_len] = {0};
float CalBuf_Fwd[cal_buf_len] = {0};
float CalBuf_Bckwd[cal_buf_len] = {0};

void FOC::calibrate() {
    // calculate angle step based on data array length
    float d_delta = 0.25f*(float)drive_info.common.ppairs * pi2 / (float)cal_buf_len;

    float current_angle = 0;
    // NOTE: ZERO
    current_angle = reset_to_zero(current_angle, d_delta);

    float d_offset = current_angle;

    uint32_t timestamp = HAL_GetTick();
     float old_angle = raw_elec_angle;
    // rotate in positive direction until you make full mechanical rotation
    // f_elec_angle < 2*PI
    while( (old_angle - raw_elec_angle) < PI || HAL_GetTick() < timestamp + 100 ) {
        old_angle = raw_elec_angle;

        uint16_t index = (uint16_t)( (float)cal_buf_len*raw_elec_angle / (2.0f*PI) );
        CalBuf_Fwd[ index ] = ( current_angle - d_offset ) / drive_info.common.ppairs - raw_elec_angle;

        // current algorithm sometimes leaves empty values in calibration buffer
        // this part fills the upcoming array member with the current value, which is overwritten in case of successful new reading
        if( index < cal_buf_len ) {
            CalBuf_Fwd[ index + 1 ] = CalBuf_Fwd[ index ];
        }

        current_angle += d_delta;

        set_windings_calibration(current_angle);
        HAL_Delay(5);
        update_angle();

        if( fabs( mfmod( current_angle, 2.0f*PI ) - PI ) < d_delta / 2.0f ) {
            offset_samples[ppair_counter] = encoder.get_value();
            ppair_counter++;
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
    while ( (old_angle - raw_elec_angle) > -PI || HAL_GetTick() < timestamp + 100 ) {
        old_angle = raw_elec_angle;

        uint16_t index = (uint16_t)( (float)cal_buf_len*raw_elec_angle / (2.0f*PI) );
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
            offset_samples[ppair_counter] = encoder.get_value();
            ppair_counter++;
        }
    }

    // encoder-magnet nonlinearity compensation lookup table should be calculated here
    for( int i = 0; i < ppair_counter; i++ ) {
        int16_t enc_angle = offset_samples[i];
        meas_elec_offset += (encoder.CPR / drive_info.common.ppairs) - (enc_angle % (encoder.CPR / drive_info.common.ppairs));
    }
    meas_elec_offset /= ppair_counter;
}

#endif
