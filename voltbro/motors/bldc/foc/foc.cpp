#include "foc.h"
#if defined(HAL_TIM_MODULE_ENABLED) && defined(HAL_ADC_MODULE_ENABLED) && defined(HAL_CORDIC_MODULE_ENABLED)

#include "arm_math.h"

#include "cordic.h"
#include "stm32g4xx_ll_cordic.h"
#include "voltbro/math/transform.hpp"

#define MONITOR

#if defined(DEBUG) || defined(MONITOR)
volatile float raw_elec_angle_glob = 0;
volatile float shaft_angle_glob = 0;
#endif
void FOC::update_angle() {
    encoder.update_value();

    encoder_data raw_value = encoder.get_value();
    int offset_value = (int)raw_value + encoder.electric_offset;  // + lookup_table[raw_value >> 5]
    offset_value -= (float)encoder.CPR / (2.0f * drive_info.common.ppairs);
    if(offset_value > (encoder.CPR - 1)) {
        offset_value -= encoder.CPR;
    }
    else if( offset_value < 0 ) {
        offset_value += encoder.CPR;
    }

    raw_elec_angle = offset_value * (pi2 / (float)encoder.CPR);
    #if defined(DEBUG) || defined(MONITOR)
        raw_elec_angle_glob = raw_elec_angle;
    #endif

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
    #if defined(DEBUG) || defined(MONITOR)
        shaft_angle_glob = shaft_angle;
    #endif
}

void FOC::apply_kalman() {
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
    const float a = 0.0f; // expected acceleration, rad/s^2
    const float g1 =  0.003785056342917592f;
    const float g2 = 0.11891101743266574f;
    const float g3 = 1.5473769821028327f;
    static float Th_hat = 0.0f; // Theta hat, rad
    static float W_hat = 0.0f; // Omega hat, rad/s
    static float E_hat = 0.0f; // Epsilon hat, rad/s^2

    // (11)
    float nTh = Th_hat + W_hat*T + (E_hat + a)*(T*T)/2.0f;
    float nW = W_hat + (E_hat + a)*T;
    float nE = E_hat;

    nTh = mfmod(nTh, pi2);
    if( nTh < 0.0f ){
        nTh += pi2;
    }

    // (19)
    Th_hat = nTh + g1*travel;
    W_hat = nW + g2*travel;
    E_hat = nE + g3*travel;
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
float V_d, V_q;
volatile float elec_angle_glob = 0;
volatile float mech_angle_glob = 0;
static volatile float i_q_error, i_d_error;
static float d_response, q_response, i_q_set;
static volatile float shart_torque_glob = 0;
static volatile float shart_velocity_glob = 0;
#endif

void FOC::update_sensors() {
    inverter.update();
    update_angle();
    apply_kalman();
    #if defined(DEBUG) || defined(MONITOR)
        elec_angle_glob = elec_angle;
        mech_angle_glob = shaft_angle;
    #endif
}

void FOC::update() {
    update_sensors();

    // calculate sin and cos of electrical angle with the help of CORDIC.
    // convert electrical angle from float to q31. electrical theta should be [-pi, pi]
    int32_t ElecTheta_q31 = (int32_t)((elec_angle / PI - 1.0f) * 2147483648.0f);
    // load angle value into CORDIC. Input value is in PIs!
    LL_CORDIC_WriteData(CORDIC, ElecTheta_q31);

    int32_t cosOutput = (int32_t)LL_CORDIC_ReadData(CORDIC);  // Read cosine
    int32_t sinOutput = (int32_t)LL_CORDIC_ReadData(CORDIC);  // Read sine

    // the values are negative to level out [-pi, pi] representation of electrical angle at the CORDIC input
    float c = -(float32_t)cosOutput / 2147483648.0f;  // convert to float from q31
    float s = -(float32_t)sinOutput / 2147483648.0f;  // convert to float from q31

    #ifndef IS_GLOBAL_CONTROL_VARIABLES
    float V_d, V_q;
    static float I_D = 0;
    static float I_Q = 0;
    #endif
    // LPF for motor current
    float tempD, tempQ;
    // dq0 transform on currents
    dq0(s, c, inverter.get_A(), inverter.get_B(), inverter.get_C(), &tempD, &tempQ);
    const float diff_D = I_D - tempD;
    const float diff_Q = I_Q - tempQ;

    /*
    const float LPF_COEFFICIENT = 0.0925f;  // Low-pass filter coefficient
    I_D = I_D - (LPF_COEFFICIENT * diff_D);
    I_Q = I_Q - (LPF_COEFFICIENT * diff_Q);
    */
    I_D = tempD;  // use raw values for now
    I_Q = tempQ;

    shaft_torque = I_Q * drive_info.torque_const * (float)drive_info.common.gear_ratio;
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
<<<<<<< HEAD
        i_d_error = -I_D;
        d_response = d_reg.regulation(i_d_error, T);
        V_d = std::clamp(d_response, -inverter.get_busV(), inverter.get_busV());
=======
        /*
        i_d_error = -I_D;
        d_response = q_reg.regulation(i_d_error, T, true);
        V_d = std::clamp(d_response, -inverter.get_busV(), inverter.get_busV());
        */
        V_d = 0;
>>>>>>> 0a119b2 (wip)

        i_q_set = 0.0f;
        if (point_type == SetPointType::TORQUE) {
            i_q_set = -target / drive_info.torque_const / (float)drive_info.common.gear_ratio;
        }
        else {
            float control_error = 0;
            if (point_type == SetPointType::POSITION) {
                control_error = target - shaft_angle;
            }
            else if (point_type == SetPointType::VELOCITY) {
                control_error = target - shaft_velocity;
            }
            i_q_set = control_reg.regulation(control_error, T, false);
        }

        float abs_max_current_from_torque = (drive_info.max_torque / drive_info.torque_const / (float)drive_info.common.gear_ratio);
        if (abs(i_q_set) > abs_max_current_from_torque) {
            i_q_set = copysign(abs_max_current_from_torque, i_q_set);
        }
<<<<<<< HEAD
        if (
            drive_limits.current_limit > 0 &&
            (abs(i_q_set) > abs(drive_limits.current_limit))
        ) {
            i_q_set = copysign(drive_limits.current_limit, i_q_set);
        }
=======
>>>>>>> 0a119b2 (wip)
        // absolute limit on currents defined by the hardware safe operation region
        if (abs(i_q_set) > 30.0f) {
            i_q_set = copysign(30.0f, i_q_set);
        }

        i_q_error = i_q_set - I_Q;
<<<<<<< HEAD
        q_response = q_reg.regulation(i_q_error, T);
=======
        q_response = q_reg.regulation(i_q_error, T, false);
>>>>>>> 0a119b2 (wip)
        V_q = std::clamp(q_response, -inverter.get_busV(), inverter.get_busV());
    }

    limit_norm(&V_d, &V_q, inverter.get_busV());

    float v_u = 0, v_v = 0, v_w = 0;
    float dtc_u = 0, dtc_v = 0, dtc_w = 0;

    // inverse dq0 transform on voltages
    abc(s, c, V_d, V_q, &v_u, &v_v, &v_w);
    // space vector modulation
    svm(inverter.get_busV(), v_u, v_v, v_w, &dtc_u, &dtc_v, &dtc_w);

    DQs[0] = (uint16_t)(float(full_pwm + 1) * dtc_u);
    DQs[1] = (uint16_t)(float(full_pwm + 1) * dtc_v);
    DQs[2] = (uint16_t)(float(full_pwm + 1) * dtc_w);

    set_pwm();
}

#endif
