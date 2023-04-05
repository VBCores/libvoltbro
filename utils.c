#include <math.h>
#include <stdlib.h>

#ifdef STM32_G
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

O1HeapInstance* heapInit(void** memoryArena, size_t heapSize) {
    void* mem = malloc(heapSize);
    if (mem == NULL) {
        return NULL;
    }
    size_t offset_mismatch = (size_t)mem % O1HEAP_ALIGNMENT;
    *memoryArena = mem + offset_mismatch;
    return o1heapInit(*memoryArena, heapSize);
}

uint32_t dac_value(double dac_voltage) {
    return floor(4095 * dac_voltage / 3.3);
}
