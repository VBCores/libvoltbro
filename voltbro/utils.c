#if defined(STM32G474xx) || defined(STM32_G)
#include "stm32g4xx.h"
#include <arm_math.h>
#else
#include <math.h>
#endif

#include "utils.h"

#define EPS 1e-10

uint32_t dac_value(double dac_voltage) {
    return floor(4095 * dac_voltage / 3.3);
}

bool is_close(float x, float y) {
    return fabsf(x - y) < EPS;
}
