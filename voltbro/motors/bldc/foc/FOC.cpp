#include "FOC.h"
#ifdef HAL_TIM_MODULE_ENABLED

int lookup_table[1024] = {-16,-17,-18,-18,-18,-19,-19,-18,-17,-16,-16,-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1,-1,0,0,0,0,1,1,1,1,1,0,0,0,0,-1,-2,-2,-3,-3,-5,-6,-7,-8,-8,-10,-11,-12,-13,-14,-14,-15,-17,-18,-18,-19,-19,-20,-21,-21,-21,-21,-21,-22,-22,-21,-21,-21,-20,-20,-20,-19,-18,-18,-17,-17,-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-7,-6,-6,-5,-5,-4,-4,-4,-4,-4,-3,-4,-4,-5,-5,-5,-5,-6,-7,-7,-8,-8,-8,-9,-10,-10,-11,-11,-11,-12,-13,-13,-13,-13,-14,-15};

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

void FOC::regulate(float _) {
    update_angle();
    apply_kalman();

    /*
    // calculate sin and cos of electrical angle with the help of CORDIC.
    // convert electrical angle from float to q31. electrical theta should be [-pi, pi]
    int32_t ElecTheta_q31 = (int32_t)((Dev.ElecTheta/PI - 1.0f) * 2147483648.0f);
    // load angle value into CORDIC. Input value is in PIs!
    LL_CORDIC_WriteData(CORDIC, ElecTheta_q31);

    // read and process ADC readings from DMA linked buffer
    ProcessADC();
    Inv.busV = 12.0f*3.3f*ADC1_buf[0] / ( 16.0f*4096.0f ); // drive input voltage

    static float I_d_integral = 0.0f;
    static float I_q_integral = 0.0f;

    int32_t cosOutput = (int32_t)LL_CORDIC_ReadData(CORDIC); // Read cosine
    int32_t sinOutput = (int32_t)LL_CORDIC_ReadData(CORDIC); // Read sine

    // the values are negative to level out [-pi, pi] representation of electrical angle at the CORDIC input
    float c = -(float32_t)cosOutput / 2147483648.0f; // convert to float from q31
    float s = -(float32_t)sinOutput / 2147483648.0f; // convert to float from q31

    // LPF for motor current
    float tempD = Inv.I_D;
    float tempQ = Inv.I_Q;
    dq0(s, c, Inv.I_A, Inv.I_B, Inv.I_C, &tempD, &tempQ);    //dq0 transform on currents
    Inv.I_D = Inv.I_D - (0.0925f * (Inv.I_D - tempD));
    Inv.I_Q = Inv.I_Q - (0.0925f * (Inv.I_Q - tempQ));

    Dev.ShaftTorque = Inv.I_Q * TORQUE_CONST * (float)config->gear_ratio;

    float i_d_set = 0.0f;
    float i_q_set = (-1.0f/TORQUE_CONST)*( Dev.mech_gain*(Dev.mech_SetP - Dev.ShaftAngle) + Dev.vel_gain*(Dev.vel_SetP - Dev.ShaftVelocity) + (Dev.torq_SetP / (float)config->gear_ratio) ); // Kp[Amp/rad], Kd[Amp/rad*s]

    if( i_q_set > max_torque ) {
        i_q_set = max_torque;
    }
    else if(i_q_set < -max_torque) {
        i_q_set = -max_torque;
    }

    // absolute limit on currents defined by the hardware safe operation region
    if(i_q_set > 30.0f) {
        i_q_set = 30.0f;
    }
    else if(i_q_set < -30.0f) {
        i_q_set = -30.0f;
    }

    float i_d_error = i_d_set - Inv.I_D;
    float i_q_error = i_q_set - Inv.I_Q;

    I_d_integral += 0.1f*0.0455f*i_d_error;
    I_q_integral += 0.1f*0.0455f*i_q_error;

    I_d_integral = fmaxf(fminf(I_d_integral, 1.0f*Inv.busV), - 1.0f*Inv.busV);
    I_q_integral = fmaxf(fminf(I_q_integral, 1.0f*Inv.busV), - 1.0f*Inv.busV);

    float V_d = 1.0f*i_d_error + I_d_integral;// + v_d_ff;
    float V_q = 1.0f*i_q_error + I_q_integral;// + v_q_ff;

    limit_norm(&V_d, &V_q, 1.0f*Inv.busV);

    float v_u = 0, v_v = 0, v_w = 0;
    float dtc_u = 0, dtc_v = 0, dtc_w = 0;

    abc(s, c, V_d, V_q, &v_u, &v_v, &v_w); //inverse dq0 transform on voltages
    svm(Inv.busV, v_u, v_v, v_w, &dtc_u, &dtc_v, &dtc_w); //space vector modulation

    DQs[0] = (uint16_t)(2000.0f*dtc_u);
    DQs[1] = (uint16_t)(2000.0f*dtc_v);
    DQs[2] = (uint16_t)(2000.0f*dtc_w);
    */

    set_pwm();
}

#endif