/*
 * report.h
 *
 *  Created on: Jan 24, 2023
 *      Author: Igor Beschastnov
 */

#ifndef VBLIB_BLDC_REPORT_H_
#define VBLIB_BLDC_REPORT_H_

typedef struct {
    float elec_speed;
    float elec_predicted;
    float Vd;
} BLDCReport;

#define REPORT_STATE

#endif  /* VBLIB_BLDC_REPORT_H_ */
