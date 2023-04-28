/*
 * bldc.c
 *
 *  Created on: Jan 24, 2023
 *      Author: Igor Beschastnov,
 */

#include "bldc.h"

#ifdef HAL_TIM_MODULE_ENABLED

#include <stdlib.h>

#include "math/math_ops.h"
#include "math/transform.h"

#define LOOKUP_SIZE 2048
// clang-format off
// TODO: Вообще говоря, этого здесь в принципе быть не должно
int16_t lookup[LOOKUP_SIZE] = {16, 16, 16, 16, 16, 16, 16, 15, 15, 14, 14, 14, 13, 13, 12, 11, 11, 10, 10, 9, 8, 8, 7, 6, 6, 5, 4, 4, 3, 2, 2, 1, 1, 0, -1, -1, -2, -2, -3, -4, -4, -5, -5, -6, -6, -7, -7, -8, -8, -9, -9, -10, -10, -11, -11, -12, -12, -13, -13, -13, -14, -14, -15, -15, -16, -16, -16, -17, -17, -18, -18, -18, -19, -19, -20, -20, -21, -21, -21, -22, -22, -23, -23, -23, -24, -24, -25, -25, -26, -26, -27, -27, -27, -28, -28, -29, -29, -30, -30, -31, -31, -31, -32, -32, -33, -33, -34, -34, -34, -35, -35, -36, -36, -37, -37, -37, -38, -38, -39, -39, -40, -40, -41, -41, -41, -42, -42, -43, -43, -44, -44, -45, -45, -45, -46, -46, -47, -47, -47, -48, -48, -49, -49, -50, -50, -50, -51, -51, -52, -52, -52, -53, -53, -54, -54, -54, -55, -55, -56, -56, -57, -57, -57, -58, -58, -59, -59, -60, -60, -60, -61, -61, -62, -62, -63, -63, -64, -64, -65, -65, -65, -66, -66, -67, -67, -68, -68, -69, -69, -70, -70, -71, -71, -71, -72, -72, -73, -73, -74, -74, -75, -75, -76, -76, -77, -77, -77, -78, -78, -79, -79, -79, -80, -80, -81, -81, -81, -82, -82, -83, -83, -83, -84, -84, -84, -85, -85, -85, -86, -86, -86, -87, -87, -87, -88, -88, -88, -89, -89, -89, -89, -90, -90, -90, -91, -91, -91, -91, -92, -92, -92, -92, -93, -93, -93, -93, -93, -94, -94, -94, -94, -94, -95, -95, -95, -95, -95, -95, -96, -96, -96, -96, -96, -96, -96, -96, -97, -97, -97, -97, -97, -97, -97, -97, -97, -97, -97, -97, -97, -97, -97, -98, -98, -98, -98, -98, -98, -98, -98, -98, -98, -98, -98, -98, -98, -97, -97, -97, -97, -97, -97, -97, -97, -97, -97, -97, -97, -96, -96, -96, -96, -96, -96, -96, -95, -95, -95, -95, -95, -95, -94, -94, -94, -94, -93, -93, -93, -93, -92, -92, -92, -92, -91, -91, -91, -90, -90, -89, -89, -89, -88, -88, -88, -87, -87, -86, -86, -85, -85, -85, -84, -84, -83, -83, -82, -82, -81, -81, -80, -80, -79, -79, -78, -78, -77, -77, -76, -76, -76, -75, -75, -74, -74, -73, -72, -72, -71, -71, -70, -70, -69, -69, -68, -68, -67, -67, -66, -66, -65, -65, -64, -64, -63, -63, -62, -62, -61, -61, -60, -59, -59, -58, -58, -57, -57, -56, -56, -55, -55, -54, -53, -53, -52, -52, -51, -51, -50, -49, -49, -48, -48, -47, -46, -46, -45, -45, -44, -44, -43, -42, -42, -41, -40, -40, -39, -39, -38, -37, -37, -36, -35, -35, -34, -34, -33, -32, -32, -31, -31, -30, -29, -29, -28, -28, -27, -27, -26, -25, -25, -24, -24, -23, -23, -22, -22, -21, -20, -20, -19, -19, -18, -18, -17, -17, -16, -16, -15, -15, -14, -14, -13, -13, -12, -11, -11, -11, -10, -10, -9, -9, -8, -8, -7, -7, -6, -6, -5, -5, -4, -4, -3, -3, -2, -2, -2, -1, -1, 0, 0, 1, 1, 2, 2, 2, 3, 3, 4, 4, 5, 5, 5, 6, 6, 7, 7, 7, 8, 8, 9, 9, 10, 10, 10, 11, 11, 11, 12, 12, 12, 13, 13, 13, 14, 14, 15, 15, 15, 15, 16, 16, 16, 17, 17, 17, 18, 18, 18, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 21, 22, 22, 22, 22, 23, 23, 23, 23, 23, 23, 24, 24, 24, 24, 24, 24, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 24, 24, 24, 24, 24, 24, 24, 23, 23, 23, 23, 23, 23, 23, 22, 22, 22, 22, 22, 21, 21, 21, 21, 21, 20, 20, 20, 20, 19, 19, 19, 19, 19, 18, 18, 18, 18, 17, 17, 17, 17, 16, 16, 16, 15, 15, 15, 15, 14, 14, 14, 13, 13, 13, 12, 12, 12, 11, 11, 10, 10, 10, 9, 9, 8, 8, 8, 7, 7, 6, 6, 6, 5, 5, 4, 4, 3, 3, 3, 2, 2, 1, 1, 1, 0, 0, -1, -1, -1, -2, -2, -3, -3, -3, -4, -4, -5, -5, -5, -6, -6, -7, -7, -8, -8, -8, -9, -9, -10, -10, -11, -11, -11, -12, -12, -13, -13, -14, -14, -14, -15, -15, -16, -16, -16, -17, -17, -18, -18, -18, -19, -19, -19, -20, -20, -20, -21, -21, -21, -21, -22, -22, -22, -23, -23, -23, -23, -24, -24, -24, -24, -25, -25, -25, -26, -26, -26, -26, -26, -27, -27, -27, -27, -27, -28, -28, -28, -28, -28, -28, -29, -29, -29, -29, -29, -29, -29, -29, -29, -29, -29, -29, -29, -29, -29, -29, -29, -29, -29, -29, -29, -29, -29, -29, -28, -28, -28, -28, -28, -28, -28, -28, -27, -27, -27, -27, -27, -27, -26, -26, -26, -26, -26, -25, -25, -25, -25, -24, -24, -24, -24, -23, -23, -23, -22, -22, -22, -21, -21, -21, -20, -20, -20, -19, -19, -18, -18, -18, -17, -17, -17, -16, -16, -16, -15, -15, -15, -14, -14, -13, -13, -12, -12, -12, -11, -11, -10, -10, -9, -9, -8, -8, -7, -7, -6, -6, -5, -5, -4, -4, -3, -3, -2, -2, -1, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 8, 9, 9, 10, 10, 10, 11, 11, 11, 12, 12, 12, 13, 13, 14, 14, 14, 14, 15, 15, 15, 16, 16, 16, 16, 17, 17, 17, 17, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 19, 19, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 20, 20, 20, 20, 20, 20, 20, 19, 19, 19, 19, 19, 19, 18, 18, 18, 18, 18, 17, 17, 17, 17, 17, 16, 16, 16, 16, 15, 15, 15, 15, 14, 14, 14, 13, 13, 13, 13, 12, 12, 12, 11, 11, 11, 11, 10, 10, 10, 9, 9, 9, 8, 8, 8, 8, 7, 7, 7, 6, 6, 6, 5, 5, 5, 4, 4, 4, 3, 3, 2, 2, 2, 1, 1, 1, 0, 0, -1, -1, -1, -2, -2, -3, -3, -4, -4, -4, -5, -5, -6, -6, -7, -7, -7, -8, -8, -9, -9, -10, -10, -11, -11, -12, -12, -12, -13, -13, -14, -15, -15, -16, -16, -17, -17, -18, -18, -19, -19, -20, -21, -21, -22, -22, -23, -23, -24, -25, -25, -26, -26, -27, -27, -28, -29, -29, -30, -30, -31, -32, -32, -33, -34, -34, -35, -35, -36, -37, -37, -38, -38, -39, -40, -40, -41, -42, -42, -43, -43, -44, -44, -45, -46, -46, -47, -47, -48, -49, -49, -50, -50, -51, -52, -52, -53, -53, -54, -55, -55, -56, -56, -57, -58, -58, -59, -59, -60, -61, -61, -62, -62, -63, -63, -64, -65, -65, -66, -66, -67, -67, -68, -68, -69, -69, -70, -70, -71, -72, -72, -73, -73, -74, -74, -75, -75, -76, -76, -77, -77, -78, -78, -79, -79, -80, -80, -81, -81, -82, -82, -83, -83, -84, -84, -85, -85, -86, -86, -87, -87, -88, -88, -89, -89, -90, -90, -91, -91, -92, -92, -93, -93, -93, -94, -94, -95, -95, -95, -96, -96, -96, -97, -97, -98, -98, -98, -99, -99, -99, -99, -100, -100, -100, -100, -101, -101, -101, -101, -102, -102, -102, -102, -102, -102, -102, -103, -103, -103, -103, -103, -103, -103, -103, -103, -103, -103, -103, -103, -103, -103, -103, -102, -102, -102, -102, -102, -102, -102, -102, -102, -102, -102, -102, -101, -101, -101, -101, -101, -101, -101, -101, -101, -101, -101, -101, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -99, -99, -99, -99, -99, -99, -99, -99, -99, -98, -98, -98, -98, -98, -98, -98, -97, -97, -97, -97, -97, -97, -97, -96, -96, -96, -96, -96, -95, -95, -95, -95, -94, -94, -94, -94, -93, -93, -93, -92, -92, -92, -91, -91, -91, -90, -90, -90, -89, -89, -89, -88, -88, -88, -87, -87, -86, -86, -86, -85, -85, -85, -84, -84, -84, -83, -83, -82, -82, -82, -81, -81, -81, -80, -80, -79, -79, -79, -78, -78, -78, -78, -77, -77, -77, -76, -76, -76, -76, -76, -75, -75, -75, -75, -75, -74, -74, -74, -74, -74, -74, -73, -73, -73, -73, -73, -73, -73, -73, -73, -73, -72, -72, -72, -72, -72, -72, -72, -72, -72, -72, -72, -72, -72, -72, -72, -72, -72, -72, -72, -72, -71, -71, -71, -71, -71, -70, -70, -70, -69, -69, -69, -68, -68, -67, -67, -66, -66, -65, -65, -64, -64, -63, -63, -62, -61, -61, -60, -60, -59, -59, -58, -57, -57, -56, -56, -55, -55, -54, -54, -53, -53, -52, -51, -51, -50, -50, -49, -49, -48, -48, -47, -47, -46, -46, -45, -45, -44, -44, -43, -43, -42, -42, -42, -41, -41, -40, -40, -39, -39, -38, -38, -38, -37, -37, -36, -36, -36, -35, -35, -34, -34, -34, -33, -33, -33, -32, -32, -32, -32, -31, -31, -31, -30, -30, -30, -30, -29, -29, -29, -28, -28, -28, -28, -27, -27, -27, -26, -26, -26, -25, -25, -25, -25, -24, -24, -24, -23, -23, -23, -22, -22, -22, -21, -21, -21, -20, -20, -20, -19, -19, -19, -18, -18, -18, -17, -17, -17, -16, -16, -16, -15, -15, -15, -15, -14, -14, -14, -14, -13, -13, -13, -13, -12, -12, -12, -12, -11, -11, -11, -11, -11, -10, -10, -10, -10, -10, -9, -9, -9, -9, -9, -9, -8, -8, -8, -8, -8, -7, -7, -7, -7, -7, -6, -6, -6, -6, -6, -6, -5, -5, -5, -5, -5, -5, -4, -4, -4, -4, -4, -3, -3, -3, -3, -2, -2, -2, -2, -1, -1, -1, -1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 17, 18, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 19, 19, 19, 19, 20, 20, 20, 20, 20, 20, 20, 20, 20, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18};
// clang-format on

volatile float calib_elec_angle = 0;

void stupid_control(
    DriverState* driver,
    InverterState* inverter,
    PIDConfig* pid,
    uint16_t* DQA,
    uint16_t* DQB,
    uint16_t* DQC
);
void six_step_control(
    DriverState* driver,
    GEncoder* encoder,
    InverterState* inverter,
    PIDConfig* pid,
    uint16_t* DQA,
    uint16_t* DQB,
    uint16_t* DQC
);

void ProcessADC(InverterState* inverter, const uint32_t ADC_buf[]) {
    // current sensors zero current output voltage is 3.3V/2 = 1.65V
    const float shunt_res = 0.001f;   // 0.001 mOhm shunt resistance
    const float op_amp_gain = 50.0f;  // current sensor gain

    inverter->I_A = ((3.3f * (float)ADC_buf[1] / 4096.0f) - 1.65f) /
                    (shunt_res * op_amp_gain);
    inverter->I_B = ((3.3f * (float)ADC_buf[2] / 4096.0f) - 1.65f) /
                    (shunt_res * op_amp_gain);
    inverter->I_C = ((3.3f * (float)ADC_buf[3] / 4096.0f) - 1.65f) /
                    (shunt_res * op_amp_gain);

    inverter->busV =
        12.0f * 3.3f * ADC_buf[0] / 4096.0f;  // drivers input voltage
}

#ifdef DEBUG
static float i_q_set = 0;
static float V_d;
static float V_q;

static uint16_t DQA = 0;
static uint16_t DQB = 0;
static uint16_t DQC = 0;

static float v_u = 0, v_v = 0, v_w = 0;
static float dtc_u = 0, dtc_v = 0, dtc_w = 0;

static float I_d_integral = 0.0f;
static float I_q_integral = 0.0f;

static float cf;
static float sf;

static float i_d_error;
static float i_q_error;
#endif

void foc_control(
    DriverState* driver,
    InverterState* inverter,
    uint16_t* dqa,
    uint16_t* dqb,
    uint16_t* dqc
) {
#ifndef DEBUG
    static float I_d_integral = 0.0f;
    static float I_q_integral = 0.0f;
    float c;
    float s;
#endif

    cf = arm_cos_f32(driver->ElecTheta);
    sf = arm_sin_f32(driver->ElecTheta);

    // DEFINITELY should smooth current!
    float tempD = inverter->I_D;
    float tempQ = inverter->I_Q;
    dq0(sf, cf, inverter->I_A, inverter->I_B, inverter->I_C, &tempD, &tempQ
    );  // dq0 transform on currents
    inverter->I_D = inverter->I_D - (0.0925f * (inverter->I_D - tempD));
    inverter->I_Q = inverter->I_Q - (0.0925f * (inverter->I_Q - tempQ));

    float i_d_set = 0.0f;

#ifndef DEBUG
    float i_q_set;
#endif
#ifdef SHAFT_INV
    i_q_set = (1.0f / TORQUE_CONST) *
              (driver->mech_gain * (driver->mech_SetP - driver->ShaftAngle) +
               driver->vel_gain * (driver->vel_SetP - driver->ShaftVelocity) +
               driver->torq_SetP);  // Kp[Amp/rad], Kd[Amp/rad*s]
#else
    i_q_set = (-1.0f / driver->torque_const) *
              (driver->mech_gain * (driver->mech_SetP - driver->ShaftAngle) +
               driver->vel_gain * (driver->vel_SetP - driver->ShaftVelocity) +
               driver->torq_SetP);  // Kp[Amp/rad], Kd[Amp/rad*s]
#endif

    // absolute limit on currents defined by the hardware safe operation region
    if (i_q_set > driver->max_torque) {
        i_q_set = driver->max_torque;
    } else if (i_q_set < -driver->max_torque) {
        i_q_set = -driver->max_torque;
    }

#ifndef DEBUG
    float i_d_error;
    float i_q_error;
#endif
    i_d_error = i_d_set - inverter->I_D;
    i_q_error = i_q_set - inverter->I_Q;

    const float p_val = 0.3f;
    const float i_val = 0.01f;
    const float vbus_val = 1.0f;

    I_d_integral += p_val * i_val * i_d_error;
    I_q_integral += p_val * i_val * i_q_error;

    I_d_integral = fmaxf(
        fminf(I_d_integral, vbus_val * inverter->busV),
        -vbus_val * inverter->busV
    );
    I_q_integral = fmaxf(
        fminf(I_q_integral, vbus_val * inverter->busV),
        -vbus_val * inverter->busV
    );

    V_d = p_val * i_d_error + I_d_integral;  // + v_d_ff;
    V_q = p_val * i_q_error + I_q_integral;  // + v_q_ff;

    limit_norm(&V_d, &V_q, vbus_val * inverter->busV);

#ifndef DEBUG
    float v_u = 0, v_v = 0, v_w = 0;
    float dtc_u = 0, dtc_v = 0, dtc_w = 0;
#else
    v_u = 0;
    v_v = 0;
    v_w = 0;
    dtc_u = 0;
    dtc_v = 0;
    dtc_w = 0;
#endif

    // inverse dq0 transform on voltages
    abc(sf, cf, V_d, V_q, &v_u, &v_v, &v_w);
    // space vector modulation
    svm(inverter->busV, v_u, v_v, v_w, &dtc_u, &dtc_v, &dtc_w);

    *dqa = (uint16_t)(2000.0f * dtc_u);
    *dqb = (uint16_t)(2000.0f * dtc_v);
    *dqc = (uint16_t)(2000.0f * dtc_w);
}

void motor_control(
    DriverState* driver,
    InverterState* inverter,
    PIDConfig* pid,
    const uint32_t ADC_buf[],
    GEncoder* encoder,
    TIM_HandleTypeDef* htim
) {
#ifdef DEBUG
    DQA = 0;
    DQB = 0;
    DQC = 0;
    V_d = 0;
    V_q = 0;
#else
    uint_16t DQA = 0;
    uint_16t DQB = 0;
    uint_16t DQC = 0;
    float V_d = 0;
    float V_q = 0;
#endif

#ifdef SIMPLE_ANGLES
    CalculateAnglesSimple(driver, encoder, pid);
#elifdef KALMAN_ANGLES
    CalculateAngles(driver, encoder);
#endif
    ProcessADC(inverter, ADC_buf);

    if (driver->State == FOC) {
        foc_control(driver, inverter, &DQA, &DQB, &DQC);
    } else if (driver->State == CURRENT_STEP) {
        /*
        float c = -1; // cos(PI) = -1
        float s = 0; // sine{PI} = 0;

        dq0(s, c, inverter->I_A, inverter->I_B, inverter->I_C, &inverter->I_D,
        &inverter->I_Q);    //dq0 transform on currents

        float i_d_set = 0.0f;
        float i_q_set = driver->current_SetP;//-0.5*betha;

        float i_d_error = i_d_set - inverter->I_D;
        float i_q_error = i_q_set - inverter->I_Q;

        I_d_integral += i_d_error;
        I_q_integral += i_q_error;

        limit_norm(&I_d_integral, &I_q_integral, 1000.0f);        // Limit
        integrators to prevent windup

        float V_d = I_Kp*i_d_error + I_Kd*I_d_integral;
        float V_q = I_Kp*i_q_error + I_Kd*I_q_integral;

        limit_norm(&V_d, &V_q, 1.0f); // Normalize voltage vector to lie within
        circle of radius v_bus abc(s, c, V_d, V_q, &DVA, &DVB, &DVC); //inverse
        dq0 transform on voltages

        uint16_t DQA = 1000 + (uint16_t)(1000.0f*DVA);
        uint16_t DQB = 1000 + (uint16_t)(1000.0f*DVB);
        uint16_t DQC = 1000 + (uint16_t)(1000.0f*DVC);
        */
    } else if (driver->State == CURRENT) {
        // float target_angle = 0.01f;  // actually, elec angle
        float target_angle = calib_elec_angle;
        sf = arm_sin_f32(target_angle);
        cf = arm_cos_f32(target_angle);

        // set the electrical angle to -pi, rotor should be pointing at +pi
        const float const_Vd = -0.5f;
        const float const_Vq = 0.0f;

        float DVA = 0;
        float DVB = 0;
        float DVC = 0;

        abc(sf, cf, const_Vd, const_Vq, &DVA, &DVB, &DVC
        );  // inverse dq0 transform on voltages

        DQA = 1000 + (int16_t)(1000.0f * DVA);
        DQB = 1000 + (int16_t)(1000.0f * DVB);
        DQC = 1000 + (int16_t)(1000.0f * DVC);
    } else if (driver->State == ROTATE) {
        sf = arm_sin_f32(calib_elec_angle);
        cf = arm_cos_f32(calib_elec_angle);

        const float const_Vd = -0.4f;
        const float const_Vq = 0.0f;

        float DVA, DVB, DVC;

        abc(sf, cf, const_Vd, const_Vq, &DVA, &DVB, &DVC
        );  // inverse dq0 transform on voltages

        DQA = 1000 + (int16_t)(1000.0f * DVA);
        DQB = 1000 + (int16_t)(1000.0f * DVB);
        DQC = 1000 + (int16_t)(1000.0f * DVC);
    } else if (driver->State == STUPID_CONTROL) {
        stupid_control(driver, inverter, pid, &DQA, &DQB, &DQC);
    } else if (driver->State == SIX_STEP_CONTROL) {
        six_step_control(driver, encoder, inverter, pid, &DQA, &DQB, &DQC);
    }
    else if (driver->State == CLBR) {

    }

    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, DQA);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, DQB);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, DQC);

    encoder->value = encoder->get_angle(encoder);
}

#ifdef DEBUG
static float nTh = 0.0f;     // n-th Theta, rad
static float Th_hat = 0.0f;  // Theta hat, rad
static float W_hat = 0.0f;   // Omega hat, rad/s
static float E_hat = 0.0f;   // Epsilon hat, rad/s^2
static float mech_theta;
static float angle_error;
static float old_angle = 0.0f;
#endif

void CalculateAngles(DriverState* driver, GEncoder* encoder) {
    const float a = 0.0f;     // expected acceleration, rad/s^2
    const float T = 0.0001f;  // encoder value sampling period, s

    const float g1 = 0.035f;
    const float g2 = 4.896f;
    const float g3 = 269.562f;

#ifndef DEBUG
    static float nTh = 0.0f;     // n-th Theta, rad
    static float Th_hat = 0.0f;  // Theta hat, rad
    static float W_hat = 0.0f;   // Omega hat, rad/s
    static float E_hat = 0.0f;   // Epsilon hat, rad/s^2
    static float old_angle = 0.0f;
#else
    nTh = 0.0f;  // n-th Theta, rad
#endif

    uint16_t raw_angle = encoder->value;

    // elec_offset shows difference between encoder zero-point and electrical
    // angle of Pi, to get the electrical angle value we need to subtract this
    // Pi = CPR/(2*ppairs) in electrical radians
    raw_angle += encoder->elec_offset;
    raw_angle -= encoder->CPR / (2 * driver->ppairs);

    // limit the angle within the [0; CPR] segment
    if (raw_angle > (encoder->CPR - 1)) {
        raw_angle -= encoder->CPR;
    } else if (raw_angle < 0) {
        raw_angle += encoder->CPR;
    }

    // rotor angular position calculation
#ifndef DEBUG
    float mech_theta;
#endif
    mech_theta = raw_angle * (2.0f * PI / (float)encoder->CPR);

    // all this is floating-point math.
    // (11)
    nTh = Th_hat + W_hat * T + (E_hat + a) * (T * T) / 2.0f;
    float nW = W_hat + (E_hat + a) * T;
    float nE = E_hat;

    // limit the angle within the [0; 2Pi ) segment
    nTh = mfmod(nTh, 2.0f * PI);
    if (nTh < 0) {
        nTh += 2.0f * PI;
    }

#ifndef DEBUG
    float angle_error;
#endif
    angle_error = mech_theta - nTh;

    // limit the angle prediction error within the [0; 2Pi ) segment
    if (angle_error > PI) {
        angle_error -= 2.0f * PI;
    } else if (angle_error < -PI) {
        angle_error += 2.0f * PI;
    }

    // (19)
    Th_hat = nTh + g1 * angle_error;
    W_hat = nW + g2 * angle_error;
    E_hat = nE + g3 * angle_error;

    // nTh - отфильтрованный raw_angle приведенный в радианы

    // electrical theta zero matches rotor angular position zero
    const float ab = 2.0f * PI / (float)driver->ppairs;
    driver->ElecTheta = (float)driver->ppairs * mfmod(nTh, ab);

    // has the rotor made a full turn?
    if (old_angle - nTh > PI) {
        driver->RotorTurns++;
    } else if (old_angle - nTh < -PI) {
        driver->RotorTurns--;
    }

    old_angle = nTh;

#ifndef SHAFT_INV
    driver->ShaftVelocity = nW / (float)driver->gear_ratio;
    driver->ShaftAngle = (nTh + (float)driver->RotorTurns * 2.0f * PI) /
                         (float)driver->gear_ratio;
#else
    Driver->ShaftVelocity = -nW / (float)GearRatio;
    Driver->ShaftAngle =
        -(nTh + (float)Driver->RotorTurns * 2.0f * PI) / (float)GearRatio;
#endif
}
#endif
