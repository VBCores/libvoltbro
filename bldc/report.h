/*
 * report.h
 *
 *  Created on: Jan 24, 2023
 *      Author: Igor Beschastnov
 */

#ifndef VBLIB_BLDC_REPORT_H_
#define VBLIB_BLDC_REPORT_H_

#ifdef __cplusplus
#include <cstdint>
extern "C" {
#else
#include <stdbool.h>
#include <stdint.h>
#endif

typedef struct {
    uint16_t PWM;
} BLDCReport;

BLDCReport* get_report();
void write_report(uint16_t PWM);

#endif /* VBLIB_BLDC_REPORT_H_ */
