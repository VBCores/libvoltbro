#include "FOC.h"
#ifdef HAL_TIM_MODULE_ENABLED

void FOC::callback() {
    encoder.update_value();
    shaft_angle = calculate_angle(drive_info.common, encoder);
    shaft_velocity = kalman_velocity(shaft_angle);
}

float FOC::kalman_velocity(float new_angle) {
    static float prev_angle = -pi2 - 2;

    if (prev_angle < (-pi2 - 1)) {
        prev_angle = new_angle;
        return 0;
    }

    float travel = new_angle - prev_angle;
    if (travel < -PI) {
        travel += pi2;
    } else if (travel > PI) {
        travel -= pi2;
    }

#pragma region KALMAN_PAPER
    const float a = 0.0f; // expected acceleration, rad/s^2
    const float g1 = 0.03f;
    const float g2 = 7.5f;
    const float g3 = 769.5f;
    static float Th_hat = 0.0f; // Theta hat, rad
    static float W_hat = 0.0f; // Omega hat, rad/s
    static float E_hat = 0.0f; // Epsilon hat, rad/s^2
    static float old_angle = 0.0f;

    float nTh = Th_hat + W_hat*T + (E_hat + a)*(T*T)/2.0f;
    float nW = W_hat + (E_hat + a)*T;
    float nE = E_hat;

    nTh = mfmod(nTh, 2.0f*PI);
    if( nTh < 0.0f ){
        nTh += 2.0f*PI;
    }

    Th_hat = nTh + g1*travel;
    W_hat = nW + g2*travel;
    E_hat = nE + g3*travel;
#pragma endregion KALMAN_PAPER

    prev_angle = nTh;
    return nW;
}

#endif