/*
 * six_step_control.h
 *
 *  Created on: May 2, 2023
 *      Author: Igor Beschastnov
 */

#ifndef VBLIB_BLDC_SIX_STEP_CONTROL_H_
#define VBLIB_BLDC_SIX_STEP_CONTROL_H_

typedef struct {
    float elec_speed;
    float elec_predicted;
    float Vd;
} SixStepState;

#define REPORT_STATE

#endif  /* VBLIB_BLDC_SIX_STEP_CONTROL_H_ */
