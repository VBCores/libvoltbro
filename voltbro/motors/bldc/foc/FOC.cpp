#include "FOC.h"
#if defined(HAL_TIM_MODULE_ENABLED) && defined(HAL_CORDIC_MODULE_ENABLED)

#include "arm_math.h"

#include "cordic.h"
#include "stm32g4xx_ll_cordic.h"
#include "voltbro/math/transform.h"

#if defined(DEBUG) || defined(MONITOR)
volatile float raw_elec_angle_glob = 0;
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
    const float g1 = 0.015f;
    const float g2 = 1.891f;
    const float g3 = 98.47f;
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
static float I_D = 0;
static float I_Q = 0;
float V_d, V_q;
volatile float elec_angle_glob = 0;
volatile float mech_angle_glob = 0;
#endif
void FOC::regulate(float _) {
    inverter.update();
    update_angle();
    apply_kalman();
    #if defined(DEBUG) || defined(MONITOR)
        elec_angle_glob = elec_angle;
        mech_angle_glob = shaft_angle;
    #endif

    switch (mode) {
        case FOCMode::PI_CURRENT: {
            float set_angle = 1.0f * PI;
            float s = arm_sin_f32(set_angle);
            float c = arm_cos_f32(set_angle);

            float V_d = -0.25f;
            float V_q = 0.0f;

            float DVA = 0;
            float DVB = 0;
            float DVC = 0;

            abc(s, c, V_d, V_q, &DVA, &DVB, &DVC);

            DQs[0] = 1000 + (int16_t)(1000.0f * DVA);
            DQs[1] = 1000 + (int16_t)(1000.0f * DVB);
            DQs[2] = 1000 + (int16_t)(1000.0f * DVC);
        }
            break;
        case FOCMode::NORMAL: {
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
            float tempD = I_D;
            float tempQ = I_Q;
            // dq0 transform on currents
            dq0(s, c, inverter.get_A(), inverter.get_B(), inverter.get_C(), &tempD, &tempQ);
            const float diff_D = I_D - tempD;
            const float diff_Q = I_Q - tempQ;
            I_D = I_D - (0.0925f * diff_D);  // TODO: remove magic numbers
            I_Q = I_Q - (0.0925f * diff_Q);  // TODO: remove magic numbers

            shaft_torque = I_Q * drive_info.torque_const * (float)drive_info.common.gear_ratio;

            if (control_config.point_type != SetPointType::VOLTAGE) {
                float i_d_set = 0.0f;
                float i_d_error = i_d_set - I_D;
                static float I_d_integral = 0.0f;
                I_d_integral += 0.1f * 0.0455f * i_d_error;  // TODO: remove magic numbers
                I_d_integral = fmaxf(fminf(I_d_integral, inverter.get_busV()), -inverter.get_busV());
                V_d = i_d_error + I_d_integral;

                float i_q_set = -1.0f / drive_info.torque_const;
                switch (control_config.point_type) {
                    case SetPointType::VELOCITY:
                        i_q_set *= control_config.main_regulator.regulation(
                            control_config.target - shaft_velocity,
                            T
                        );
                        break;
                    case SetPointType::POSITION:
                        i_q_set *=
                            control_config.aux_regulator.regulation(control_config.target - shaft_angle, T);
                        break;
                    case SetPointType::TORQUE:
                        i_q_set *= control_config.target / (float)drive_info.common.gear_ratio;
                        break;
                    default:
                        Error_Handler();
                }

                if (abs(i_q_set) > drive_info.max_torque) {
                    i_q_set = copysign(drive_info.max_torque, i_q_set);
                }
                // absolute limit on currents defined by the hardware safe operation region
                if (abs(i_q_set) > 30.0f) {
                    i_q_set = copysign(30.0f, i_q_set);
                }

                float i_q_error = i_q_set - I_Q;
                static float I_q_integral = 0.0f;
                I_q_integral += 0.1f * 0.0455f * i_q_error;  // TODO: remove magic numbers
                I_q_integral = fmaxf(fminf(I_q_integral, inverter.get_busV()), -inverter.get_busV());

                V_q = i_q_error + I_q_integral;
            }
            else {
                V_d = 0;
                V_q = -control_config.target;
            }
            limit_norm(&V_d, &V_q, inverter.get_busV());

            float v_u = 0, v_v = 0, v_w = 0;
            float dtc_u = 0, dtc_v = 0, dtc_w = 0;

            // inverse dq0 transform on voltages
            abc(s, c, V_d, V_q, &v_u, &v_v, &v_w);
            // space vector modulation
            svm(inverter.get_busV(), v_u, v_v, v_w, &dtc_u, &dtc_v, &dtc_w);

            DQs[0] = (uint16_t)(2000.0f * dtc_u);  // TODO: remove magic numbers
            DQs[1] = (uint16_t)(2000.0f * dtc_v);  // TODO: remove magic numbers
            DQs[2] = (uint16_t)(2000.0f * dtc_w);  // TODO: remove magic numbers
        }
            break;
    }

    set_pwm();
}

#endif