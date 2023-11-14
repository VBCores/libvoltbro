#include <math.h>
#include <stdlib.h>

#if defined(STM32G474xx) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#else
#include "stm32f4xx_hal.h"
#endif

#include "utils.h"

void blink_notify_led(int blinks, GPIO_TypeDef* GPIOx, pin GPIO_Pin) {
    for (int i = 0; i < (blinks * 2); i++) {
        HAL_GPIO_TogglePin(GPIOx, GPIO_Pin);
        HAL_Delay(100);
    }
}

void blink_notify(int blinks) {
    for (int i = 0; i < (blinks * 2); i++) {
        HAL_GPIO_TogglePin(NotifyLED_GPIOx, NotifyLED_PIN);
        HAL_Delay(100);
    }
}

uint32_t dac_value(double dac_voltage) {
    return floor(4095 * dac_voltage / 3.3);
}
