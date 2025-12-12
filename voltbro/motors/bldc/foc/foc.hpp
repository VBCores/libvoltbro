#pragma once
#include <cstdint>
#if defined(STM32G4) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#if defined(HAL_TIM_MODULE_ENABLED) && defined(HAL_ADC_MODULE_ENABLED) && defined(HAL_CORDIC_MODULE_ENABLED)

#include <algorithm>
#include <array>
#include <cmath>
#include <string_view>
#include <utility>

#include "arm_math.h"
#include "stm32g4xx_ll_cordic.h"

#include "../bldc.h"
#include "voltbro/math/regulators/pid.hpp"
#include "voltbro/math/transform.hpp"

constexpr size_t CALIBRATION_BUFF_SIZE = 1024;
using __non_const_calib_array_t = std::array<int, CALIBRATION_BUFF_SIZE>;
using calibration_array_t = const __non_const_calib_array_t;

struct CalibrationData {
    static constexpr uint32_t TYPE_ID = 0x88ABCDEF;
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

struct FOCTarget {
    float torque = 0.0f;
    float angle = 0.0f;
    float velocity = 0.0f;
    float angle_kp = 0.0f;
    float velocity_kp = 0.0f;
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
template<class ENCODER_T>
class FOC: public BLDCController  {
protected:
    FOCTarget foc_target;
    ENCODER_T& encoder;
    PIDRegulator q_reg;
    PIDRegulator d_reg;
    PIDRegulator control_reg = PIDRegulator();
    const KalmanConfig kalman_config;
    float T;
    float raw_elec_angle = 0;
    float elec_angle = 0;
    calibration_array_t* lookup_table = nullptr;
    bool is_limited = false;

    void apply_kalman();
    virtual void update_angle();
    void update_electric_angle();

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
        PIDConfig&& q_config,
        PIDConfig&& d_config,
        const DriveLimits& drive_limits,
        const DriveInfo& drive_info,
        TIM_HandleTypeDef* htim,
        ENCODER_T& encoder,
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
        q_reg(std::move(q_config)),
        d_reg(std::move(d_config)),
        kalman_config(std::move(kalman_config))
        {}

    float get_electric_angle() {
        return elec_angle;
    }

    void set_foc_point(FOCTarget&& target) {
        point_type = SetPointType::UNIVERSAL;
        foc_target = std::move(target);
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
    const ENCODER_T& get_encoder() const {
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

#ifdef FOC_PROFILE
struct FOCProfile {
    volatile uint32_t sensors = 0;
    volatile uint32_t currents = 0;
    volatile uint32_t outer_loop = 0;
    volatile uint32_t pwm = 0;
    volatile uint32_t total = 0;
};
static volatile FOCProfile foc_profile;

struct SensorsProfile {
    volatile uint32_t inverter = 0;
    volatile uint32_t angle = 0;
    volatile uint32_t kalman = 0;
    volatile uint32_t total = 0;
};
static volatile SensorsProfile sensors_profile;
#endif

template <class ENCODER_T>
void FOC<ENCODER_T>::update_electric_angle() {
    encoder.update_value();

    encoder_data raw_value = encoder.get_value();
    int offset_value = (int)raw_value + encoder.electric_offset;  // + lookup_table[raw_value >> 5]
    static const float cpr_offset = (float)encoder.CPR / (2.0f * drive_info.common.ppairs);
    offset_value -= cpr_offset;
    if(offset_value > (encoder.CPR - 1)) {
        offset_value -= encoder.CPR;
    }
    else if( offset_value < 0 ) {
        offset_value += encoder.CPR;
    }

    raw_elec_angle = offset_value * (pi2 / (float)encoder.CPR);
}

template <class ENCODER_T>
void FOC<ENCODER_T>::update_angle() {
    update_electric_angle();
    const float rads_per_rev = pi2 / drive_info.common.gear_ratio;
    int revolutions = encoder.get_revolutions() % drive_info.common.gear_ratio;
    float base_angle = 0;
    if (revolutions < 0) {
        base_angle = (drive_info.common.gear_ratio + revolutions) * rads_per_rev;
    }
    else {
        base_angle = revolutions * rads_per_rev;
    }
    shaft_angle = base_angle + (raw_elec_angle / drive_info.common.gear_ratio);
}

template <class ENCODER_T>
void FOC<ENCODER_T>::apply_kalman() {
    static float prev_angle = -pi2 - 2;

    if (prev_angle < (-pi2 - 1)) {
        prev_angle = raw_elec_angle;
        return;
    }

    float travel = raw_elec_angle - prev_angle;
    if (travel < -PI) {
        travel += pi2;
    } else if (travel > PI) {
        travel -= pi2;
    }

#pragma region KALMAN_PAPER
    /*
     * Source: "A digital speed filter for motion control drives
     *          with a low resolution position encoder",
     * AUTOMATIKA, 44(2003),
     * A. Bellini, S. Bifaretti, S. Constantini
     */
    // TODO: get acceleration from inverter?
    static float Th_hat = 0.0f; // Theta hat, rad
    static float W_hat = 0.0f; // Omega hat, rad/s
    static float E_hat = 0.0f; // Epsilon hat, rad/s^2

    // (11)
    float nTh = Th_hat + W_hat * T + (E_hat + kalman_config.expected_a) * (T*T) / 2.0f;
    float nW = W_hat + (E_hat + kalman_config.expected_a) * T;
    float nE = E_hat;

    nTh = mfmod(nTh, pi2);
    if( nTh < 0.0f ){
        nTh += pi2;
    }

    // (19)
    Th_hat = nTh + kalman_config.g1 * travel;
    W_hat = nW + kalman_config.g2 * travel;
    E_hat = nE + kalman_config.g3 * travel;
#pragma endregion KALMAN_PAPER

    const float ab = pi2 / (float)drive_info.common.ppairs;
    elec_angle = (float)drive_info.common.ppairs * mfmod(nTh, ab);
    shaft_velocity = nW / drive_info.common.gear_ratio;

    prev_angle = nTh;
}

#if defined(DEBUG) || defined(MONITOR)
#define IS_GLOBAL_CONTROL_VARIABLES
static volatile float I_D = 0;
static volatile float I_Q = 0;
static float V_d, V_q;
static volatile float elec_angle_glob = 0;
static volatile float mech_angle_glob = 0;
static volatile float i_q_error, i_d_error;
static float d_response, q_response, i_q_set;
static volatile float shart_torque_glob = 0;
static volatile float shart_velocity_glob = 0;
static volatile float control_error_glob = 0;
static volatile float controller_response_glob = 0;
static volatile float value_foc_p = 0;
static volatile float value_foc_v = 0;
static volatile float value_foc_p_kp = 0;
static volatile float value_foc_v_kp = 0;
static volatile float value_foc_t = 0;
#endif

template <class ENCODER_T>
void FOC<ENCODER_T>::update_sensors() {
#ifdef FOC_PROFILE
    const uint32_t start_total = DWT->CYCCNT;
    uint32_t t_start = DWT->CYCCNT;
#endif
    inverter.update();
#ifdef FOC_PROFILE
    sensors_profile.inverter = DWT->CYCCNT - t_start;
    t_start = DWT->CYCCNT;
#endif
    update_angle();
#ifdef FOC_PROFILE
    sensors_profile.angle = DWT->CYCCNT - t_start;
    t_start = DWT->CYCCNT;
#endif
    apply_kalman();
#ifdef FOC_PROFILE
    sensors_profile.kalman = DWT->CYCCNT - t_start;
    sensors_profile.total = DWT->CYCCNT - start_total;
#endif
    #if defined(DEBUG) || defined(MONITOR)
    elec_angle_glob = elec_angle;
    mech_angle_glob = shaft_angle;
    #endif
}

template <class ENCODER_T>
void FOC<ENCODER_T>::update() {
#ifdef FOC_PROFILE
    const uint32_t start_total = DWT->CYCCNT;
    uint32_t t_start = DWT->CYCCNT;
#endif
    update_sensors();
#ifdef FOC_PROFILE
    foc_profile.sensors = DWT->CYCCNT - t_start;
#endif

    // calculate sin and cos of electrical angle with the help of CORDIC.
    // convert electrical angle from float to q31. electrical theta should be [-pi, pi]
    int32_t ElecTheta_q31 = (int32_t)((elec_angle / PI - 1.0f) * 2147483648.0f);
    // load angle value into CORDIC. Input value is in PIs!
    LL_CORDIC_WriteData(CORDIC, ElecTheta_q31);

    struct {
        int32_t cosOutput = (int32_t)LL_CORDIC_ReadData(CORDIC);  // Read cosine
        int32_t sinOutput = (int32_t)LL_CORDIC_ReadData(CORDIC);  // Read sine
    } elec_angles_q31;

    struct {
        float c;
        float s;
    } elec_angles;

    //arm_q31_to_float(&elec_angles_q31.cosOutput, &elec_angles.c, 2);

    // the values are negative to level out [-pi, pi] representation of electrical angle at the CORDIC input
    elec_angles.c = -(float32_t)elec_angles_q31.cosOutput / 2147483648.0f;  // convert to float from q31
    elec_angles.s = -(float32_t)elec_angles_q31.sinOutput / 2147483648.0f;  // convert to float from q31

    #ifndef IS_GLOBAL_CONTROL_VARIABLES
    float V_d, V_q;
    static float I_D = 0;
    static float I_Q = 0;
    #endif
    #ifdef FOC_PROFILE
    t_start = DWT->CYCCNT;
    #endif
    // LPF for motor current
    float tempD, tempQ;
    // dq0 transform on currents
    dq0(elec_angles.s, elec_angles.c, inverter.get_A(), inverter.get_B(), inverter.get_C(), &tempD, &tempQ);
    const float diff_D = I_D - tempD;
    const float diff_Q = I_Q - tempQ;

    constexpr float LPF_COEFFICIENT = 0.0925f;  // Low-pass filter coefficient
    I_D = I_D - (LPF_COEFFICIENT * diff_D);
    I_Q = I_Q - (LPF_COEFFICIENT * diff_Q);
    #ifdef FOC_PROFILE
    foc_profile.currents = DWT->CYCCNT - t_start;
    #endif

    const float gear_ratio_f = static_cast<float>(drive_info.common.gear_ratio);
    const float busV = inverter.get_busV();

    shaft_torque = -I_Q * drive_info.torque_const * gear_ratio_f;
    #ifdef IS_GLOBAL_CONTROL_VARIABLES
    shart_torque_glob = shaft_torque;
    shart_velocity_glob = shaft_velocity;
    #endif

    if (point_type == SetPointType::VOLTAGE) {
        V_d = 0;
        V_q = -target;
    }
    else {
        #ifndef IS_GLOBAL_CONTROL_VARIABLES
        float i_d_error, i_q_error, d_response, q_response, i_q_set;
        #endif

        i_d_error = -I_D;
        d_response = d_reg.regulation(i_d_error, T, busV);
        V_d = std::clamp(d_response, -busV, busV);

        i_q_set = 0.0f;
        #ifdef FOC_PROFILE
        t_start = DWT->CYCCNT;
        #endif
        if (point_type == SetPointType::UNIVERSAL) {
            #ifdef MONITOR
            value_foc_p = foc_target.angle;
            value_foc_v = foc_target.velocity;
            value_foc_p_kp = foc_target.angle_kp;
            value_foc_v_kp = foc_target.velocity_kp;
            value_foc_t = foc_target.torque;
            #endif
            i_q_set = -1.0f / drive_info.torque_const * (
                foc_target.angle_kp * (foc_target.angle - get_angle()) +
                foc_target.velocity_kp * (foc_target.velocity - shaft_velocity) +
                (foc_target.torque / gear_ratio_f)
            );
        }
        else if (point_type == SetPointType::TORQUE) {
            i_q_set = -target / drive_info.torque_const / gear_ratio_f;
        }
        else {
            float control_error = 0;
            if (point_type == SetPointType::POSITION) {
                control_error = target - shaft_angle;
            }
            else if (point_type == SetPointType::VELOCITY) {
                control_error = target - shaft_velocity;
            }
            float controller_response = control_reg.regulation(control_error, T, false);
            #ifdef IS_GLOBAL_CONTROL_VARIABLES
            control_error_glob = control_error;
            controller_response_glob = controller_response;
            #endif
            i_q_set = -controller_response;
        }

        const float abs_max_current_from_torque = (drive_info.max_torque / drive_info.torque_const / gear_ratio_f);
        if (fabs(i_q_set) > fabs(abs_max_current_from_torque)) {
            i_q_set = copysign(abs_max_current_from_torque, i_q_set);
        }
        if (
            drive_limits.current_limit > 0 &&
            (fabs(i_q_set) > fabs(drive_limits.current_limit))
        ) {
            i_q_set = copysign(drive_limits.current_limit, i_q_set);
        }
        // absolute limit on currents defined by the hardware safe operation region
        if (fabs(i_q_set) > 30.0f) {
            i_q_set = copysign(30.0f, i_q_set);
        }

        i_q_error = i_q_set - I_Q;
        q_response = q_reg.regulation(i_q_error, T, busV);
        V_q = q_response;
        #ifdef FOC_PROFILE
        foc_profile.outer_loop = DWT->CYCCNT - t_start;
        #endif
    }

    limit_norm(&V_d, &V_q, busV);

    #ifdef FOC_PROFILE
    t_start = DWT->CYCCNT;
    #endif
    float v_u = 0, v_v = 0, v_w = 0;
    float dtc_u = 0, dtc_v = 0, dtc_w = 0;

    // inverse dq0 transform on voltages
    abc(elec_angles.s, elec_angles.c, V_d, V_q, &v_u, &v_v, &v_w);
    // space vector modulation
    svm(busV, v_u, v_v, v_w, &dtc_u, &dtc_v, &dtc_w);

    DQs[0] = (uint16_t)(float(full_pwm + 1) * dtc_u);
    DQs[1] = (uint16_t)(float(full_pwm + 1) * dtc_v);
    DQs[2] = (uint16_t)(float(full_pwm + 1) * dtc_w);

    set_pwm();
    #ifdef FOC_PROFILE
    foc_profile.pwm = DWT->CYCCNT - t_start;
    #endif

#ifdef FOC_PROFILE
    foc_profile.total = DWT->CYCCNT - start_total;
#endif
}

#ifdef DEBUG
static volatile float current_target_angle = 0;
static volatile encoder_data current_encoder_value = 0;
#endif

template <class ENCODER_T>
void FOC<ENCODER_T>::set_windings_calibration(float target_angle) {
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

template <class ENCODER_T>
bool FOC<ENCODER_T>::check_if_inverted(float start_angle, float d_delta) {
    float current_angle = start_angle;

    float old_angle = raw_elec_angle;
    float diff = 0;
    size_t step_counter = 0;
    size_t decrease_counter = 0;
    size_t increase_counter = 0;
    while(step_counter < 400) {
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
#ifdef DEBUG
        current_encoder_value = encoder.get_value();
#endif
    }

    return decrease_counter >= increase_counter;
}

template <class ENCODER_T>
float FOC<ENCODER_T>::reset_to_zero(float start_angle, float d_delta) {
    float current_angle = start_angle;

    float old_angle = raw_elec_angle;
    while((old_angle - raw_elec_angle) < PI) {
        old_angle = raw_elec_angle;
        current_angle += d_delta;
        set_windings_calibration(current_angle);
        HAL_Delay(5);
        update_angle();
#ifdef DEBUG
        current_encoder_value = encoder.get_value();
#endif
    }

    return raw_elec_angle;
}

template <class ENCODER_T>
void FOC<ENCODER_T>::calibrate(CalibrationData& calibration_data) {
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
#ifdef DEBUG
        current_encoder_value = encoder.get_value();
#endif

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
#ifdef DEBUG
        current_encoder_value = encoder.get_value();
#endif
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
#ifdef DEBUG
        current_encoder_value = encoder.get_value();
#endif

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
#endif
