#include "report.h"

static BLDCReport report;

BLDCReport* get_report() {
    return &report;
}

void write_report(uint16_t PWM) {
    report.PWM = PWM;
}