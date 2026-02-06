#if defined(STM32G4) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#if defined(HAL_TIM_MODULE_ENABLED) && defined(HAL_ADC_MODULE_ENABLED) && defined(HAL_CORDIC_MODULE_ENABLED)

#include "foc.hpp"

#include "arm_math.h"
#include "main.h"
#include <numeric>

#include "voltbro/math/transform.hpp"


#ifdef DEBUG
static volatile float current_target_angle = 0;
static volatile encoder_data current_encoder_value = 0;
#endif

void FOC::set_windings_calibration(float target_angle) {
    #ifdef DEBUG
    current_target_angle = target_angle;
    #endif
    float s = arm_sin_f32(target_angle);
    float c = arm_cos_f32(target_angle);

    float DVA, DVB, DVC;

    //inverse dq0 transform on voltages
    abc(s, c, -drive_info.calibration_voltage, 0, &DVA, &DVB, &DVC);

    DQs[0] = 1000 + (int16_t)(1000.0f*DVA);
    DQs[1] = 1000 + (int16_t)(1000.0f*DVB);
    DQs[2] = 1000 + (int16_t)(1000.0f*DVC);

    set_pwm();
}


// NOTE: caller has to GUARANTEE that calculations_buffer is (CALIBRATION_BUFF_SIZE + 1) * sizeof(int) bytes long
void FOC::calibrate(CalibrationData& calibration_data, std::byte* additional_buffer) {
    auto zero_pwm = [this]() {
        DQs[0] = 0;
        DQs[1] = 0;
        DQs[2] = 0;
        set_pwm();
     };
    auto set_electric_angle = [this](float angle, uint32_t delay=20) {
        set_windings_calibration(angle);
        HAL_Delay(delay);
        for (int i = 0; i < 5; i++) {
            update_angle();
        }
    };

    // Sanity check: quarter mechanical turn forward (open-loop)
    {
        const float elec_step = 0.5f;
        const float end_angle = pi2 * (float)drive_info.common.ppairs * drive_info.common.gear_ratio / 4;
        float ang = 0.0f;
        while (ang < end_angle) {
            set_electric_angle(ang, 10);
            ang += elec_step;
        }
    }

#pragma region Electrical Offset
    const int ppairs = drive_info.common.ppairs;
    const int samples_count = ppairs * 2;
#ifdef MONITOR
    int offset_samples[28] = {0};
#else
    int offset_samples[samples_count] = {0};
#endif

    auto get_circular_error = [&](int meas, int expected, int cycle_len) {
        int diff = meas - expected;
        while (diff > (cycle_len / 2)) {
            diff -= cycle_len;
        }
        while (diff < (-cycle_len / 2)) {
            diff += cycle_len;
        }
        return diff;
    };
    auto return_to_zero = [this, &ppairs, &set_electric_angle, &get_circular_error]() {
        float angle = 0.0f;
        set_electric_angle(angle, 100);

        float step = ppairs * pi2 / 4000.0f;
        if (raw_elec_angle < (encoder.CPR / 2.0f)) {
            step = -step;
        }
        while(abs(get_circular_error(encoder.get_value(), 0, encoder.CPR)) > (encoder.CPR / 1000)) {
            angle += step;
            set_electric_angle(angle, 5);
        }
        set_electric_angle(0.0f, 100);

        return raw_elec_angle;
    };

    const int ppair_step_counts = 50;
    const int encoder_steps_per_pair = encoder.CPR / ppairs;
    const float ppair_step = pi2 / ppair_step_counts;
    const int overshoot_steps = 4;

    auto offset_calibration_pass = [&](int* offset_array, int offset) {
        // Undershoot at the start to avoid hysteresis then roll forward a bit
        for (int j = 1; j < overshoot_steps; j++) {
            set_electric_angle(0 - ppair_step * j);
        }
        for (int j = overshoot_steps; j >= 1; j--) {
            set_electric_angle(0 - ppair_step * j);
        }

        for (int i = 0; i < ppairs; i++) {
            const float local_zero_point = pi2 * i;
            set_electric_angle(local_zero_point);

            int expected_value = encoder_steps_per_pair * i;
            int current_value = int(encoder.get_value()) - offset;
            int current_offset = get_circular_error(current_value, expected_value, encoder.CPR);
            offset_array[i] = current_offset;

            for (int j = 0; j < ppair_step_counts; j++) {
                set_electric_angle(local_zero_point + ppair_step * (j + 1));
            }
        }

        // Overshoot at the end to avoid hysteresis then roll back a bit
        for (int j = 1; j < overshoot_steps; j++) {
            set_electric_angle(pi2 * ppairs + ppair_step * j);
        }
        for (int j = overshoot_steps; j >= 1; j--) {
            set_electric_angle(pi2 * ppairs + ppair_step * j);
        }

        for (int i = ppairs - 1; i >= 0; i--) {
            const float local_zero_point = pi2 * (i + 1);
            set_electric_angle(local_zero_point);

            int expected_value = encoder_steps_per_pair * (i + 1);
            int current_value = int(encoder.get_value()) - offset;
            int current_offset = get_circular_error(current_value, expected_value, encoder.CPR);
            offset_array[samples_count - i - 1] = current_offset;

            for (int j = 0; j < ppair_step_counts; j++) {
                set_electric_angle(local_zero_point - ppair_step * (j + 1));
            }
        }

        int offset_sum = 0;
        for (auto& sample : offset_samples) {
            offset_sum += sample;
        }
        return offset_sum / samples_count;
    };

    return_to_zero();
    int average_offset = offset_calibration_pass(offset_samples, 0);
    calibration_data.meas_elec_offset = average_offset;

#ifdef MONITOR
    int verification_samples[28] = {0};
#else
    int verification_samples[samples_count] = {0};
#endif
    return_to_zero();
    int average_error = offset_calibration_pass(verification_samples, average_offset);
    for (auto sample : verification_samples) {
        if (abs(sample) > (encoder_steps_per_pair / 10)) {
            zero_pwm();
            Error_Handler(); // Calibration failed
        }
    }
#pragma endregion

    return_to_zero();

#pragma region Curve Sampling
    std::array<int, CALIBRATION_BUFF_SIZE>* fwd_calibration_array = &calibration_data.calibration_array;
    std::array<int, CALIBRATION_BUFF_SIZE>* bwd_calibration_array = new (additional_buffer) std::array<int, CALIBRATION_BUFF_SIZE>{};

    const int SENTINEL_EMPTY = 0x7FFFFFFF;
    for (auto& array: {fwd_calibration_array, bwd_calibration_array}) {
        array->fill(SENTINEL_EMPTY);
    }

    const float electrical_span = (float)ppairs * pi2;
    const float electric_angle_delta = electrical_span / (float)calibration_data.calibration_array.size();
    float encoder_steps_in_electrical_radian = (float)encoder.CPR / (ppairs * pi2);
    float electric_angle = 0;

    // Undershoot at the start to avoid hysteresis then roll forward a bit
    for (int j = 1; j < overshoot_steps; j++) {
        set_electric_angle(0 - ppair_step * j);
    }
    for (int j = overshoot_steps; j >= 1; j--) {
        set_electric_angle(0 - ppair_step * j);
    }
    while (electric_angle < electrical_span) {
        set_electric_angle(electric_angle, 5);

        int expected_value = (int)roundf(electric_angle * encoder_steps_in_electrical_radian);
        int offset_encoder_value = static_cast<int>(encoder.get_value()) - average_offset;
        if (offset_encoder_value < 0) {
            offset_encoder_value += encoder.CPR;
        }
        size_t idx = static_cast<size_t>(offset_encoder_value >> 4);
        (*fwd_calibration_array)[idx] = get_circular_error(offset_encoder_value, expected_value, encoder.CPR);

        electric_angle += electric_angle_delta;
    }
#pragma endregion

    zero_pwm();
    asm volatile("nop");  // debug breakpoint
}

#endif
#endif
