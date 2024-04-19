#include "FOC.h"
#ifdef HAL_TIM_MODULE_ENABLED

#include "arm_math.h"

#include "cordic.h"
#include "stm32g4xx_ll_cordic.h"
#include "voltbro/math/transform.h"

int lookup_table[128] = {-29,-27,-26,-25,-24,-23,-22,-21,-21,-20,-20,-19,-19,-18,-18,-18,-17,-17,-17,-17,-17,-17,-17,-16,-16,-16,-15,-15,-15,-15,-15,-15,-15,-15,-15,-15,-15,-15,-16,-16,-17,-17,-17,-18,-18,-19,-19,-19,-20,-20,-21,-21,-22,-22,-22,-22,-23,-23,-23,-23,-23,-24,-24,-24,-23,-23,-22,-22,-22,-22,-21,-21,-20,-20,-19,-18,-18,-17,-17,-16,-16,-15,-15,-15,-15,-15,-15,-15,-15,-15,-15,-16,-16,-16,-17,-17,-17,-17,-18,-19,-20,-21,-21,-23,-24,-25,-26,-27,-28,-29,-30,-31,-31,-32,-33,-34,-34,-35,-35,-35,-35,-36,-35,-35,-34,-33,-32,-30};

void FOC::update_angle() {
    encoder.update_value();

    encoder_data raw_value = encoder.get_value();
    int offset_value = (int)raw_value + lookup_table[raw_value >> 8] + encoder.electric_offset;
    offset_value -= (float)encoder.CPR / (2.0f * drive_info.common.ppairs);
    if(offset_value > (encoder.CPR - 1)) {
        offset_value -= encoder.CPR;
    }
    else if( offset_value < 0 ) {
        offset_value += encoder.CPR;
    }

    shaft_angle = offset_value * (pi2 / (float)encoder.CPR);
}

void FOC::apply_kalman() {
    static float prev_angle = -pi2 - 2;

    if (prev_angle < (-pi2 - 1)) {
        prev_angle = shaft_angle;
        return;
    }

    float travel = shaft_angle - prev_angle;
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
    const float g1 = 0.03f;
    const float g2 = 7.5f;
    const float g3 = 769.5f;
    static float Th_hat = 0.0f; // Theta hat, rad
    static float W_hat = 0.0f; // Omega hat, rad/s
    static float E_hat = 0.0f; // Epsilon hat, rad/s^2
    static float old_angle = 0.0f;

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
    shaft_velocity = nW;

    prev_angle = nTh;
}

#ifdef DEBUG
static float I_D = 0;
static float I_Q = 0;
#endif
void FOC::regulate(float _) {
    inverter.update();
    update_angle();
    apply_kalman();

    switch (mode) {
        case FOCMode::NORMAL: {
            // calculate sin and cos of electrical angle with the help of CORDIC.
            // convert electrical angle from float to q31. electrical theta should be [-pi, pi]
            int32_t ElecTheta_q31 = (int32_t)((elec_angle / PI - 1.0f) * 2147483648.0f);
            // load angle value into CORDIC. Input value is in PIs!
            LL_CORDIC_WriteData(CORDIC, ElecTheta_q31);

            static float I_d_integral = 0.0f;
            static float I_q_integral = 0.0f;

            int32_t cosOutput = (int32_t)LL_CORDIC_ReadData(CORDIC);  // Read cosine
            int32_t sinOutput = (int32_t)LL_CORDIC_ReadData(CORDIC);  // Read sine

            // the values are negative to level out [-pi, pi] representation of electrical angle at the CORDIC input
            float c = -(float32_t)cosOutput / 2147483648.0f;  // convert to float from q31
            float s = -(float32_t)sinOutput / 2147483648.0f;  // convert to float from q31

            // LPF for motor current
#ifndef DEBUG
            static float I_D = 0;
            static float I_Q = 0;
#endif
            float tempD = I_D;
            float tempQ = I_Q;
            // dq0 transform on currents
            dq0(s, c, inverter.get_A(), inverter.get_B(), inverter.get_C(), &tempD, &tempQ);
            I_D = I_D - (0.0925f * (I_D - tempD));
            I_Q = I_Q - (0.0925f * (I_Q - tempQ));

            shaft_torque = I_Q * drive_info.torque_const * (float)drive_info.common.gear_ratio;

            float i_d_set = 0.0f;
            float i_q_set = -1.0f / drive_info.torque_const;
            switch (control_config.point_type) {
                case SetPointType::VELOCITY:
                    i_q_set *= control_config.main_regulator.regulation(
                        control_config.velocity_target - shaft_velocity,
                        T
                    );
                    break;
                case SetPointType::POSITION:
                    i_q_set *= control_config.aux_regulator.regulation(
                        control_config.position_target - shaft_angle,
                        T
                    );
                    break;
                case SetPointType::TORQUE:
                    i_q_set *= control_config.torque_target / (float)drive_info.common.gear_ratio;
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

            float i_d_error = i_d_set - I_D;
            float i_q_error = i_q_set - I_Q;

            I_d_integral += 0.1f * 0.0455f * i_d_error;
            I_q_integral += 0.1f * 0.0455f * i_q_error;

            I_d_integral =
                fmaxf(fminf(I_d_integral, 1.0f * inverter.get_busV()), -1.0f * inverter.get_busV());
            I_q_integral =
                fmaxf(fminf(I_q_integral, 1.0f * inverter.get_busV()), -1.0f * inverter.get_busV());

            float V_d = 1.0f * i_d_error + I_d_integral;
            float V_q = 1.0f * i_q_error + I_q_integral;

            limit_norm(&V_d, &V_q, 1.0f * inverter.get_busV());

            float v_u = 0, v_v = 0, v_w = 0;
            float dtc_u = 0, dtc_v = 0, dtc_w = 0;

            // inverse dq0 transform on voltages
            abc(s, c, V_d, V_q, &v_u, &v_v, &v_w);
            // space vector modulation
            svm(inverter.get_busV(), v_u, v_v, v_w, &dtc_u, &dtc_v, &dtc_w);

            DQs[0] = (uint16_t)(2000.0f * dtc_u);
            DQs[1] = (uint16_t)(2000.0f * dtc_v);
            DQs[2] = (uint16_t)(2000.0f * dtc_w);
        }
            break;
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
    }

    set_pwm();
}

#endif