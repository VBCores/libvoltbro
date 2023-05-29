/*
 * cyphal.h
 *
 *  Created on: Nov 22, 2022
 *      Author: Igor Beschastnov
 */

#ifndef INC_CYPHAL_H_
#define INC_CYPHAL_H_
#if defined(HAL_CAN_MODULE_ENABLED) || defined(HAL_FDCAN_MODULE_ENABLED)

#if !defined(STM32G474xx) && !defined(STM32_G)
#include "stm32f4xx_hal.h"
#else
#include "stm32g4xx_hal.h"
#endif

#include <libcanard/canard.h>
#include <uavcan/node/Heartbeat_1_0.h>

#include "o1heap/o1heap.h"

#if defined(STM32G474xx) || defined(STM32_G)
extern FDCAN_HandleTypeDef hfdcan1;
#else
extern CAN_HandleTypeDef hcan1;
#endif

extern O1HeapInstance* o1heap;
extern CanardInstance
    canard;  // This is the core structure that keeps all the states and
             // allocated resources of the library instance
extern CanardTxQueue
    queue;  // Prioritized transmission queue that keeps CAN frames destined for
            // transmission via one CAN interface

#ifdef __cplusplus
extern "C" {
#endif

extern void Error_Handler();

uint32_t micros(void);

void init_cyphal(CanardNodeID node_id);

void send_heartbeat(uint8_t health, uint8_t mode);

void cyphal_push(
    const CanardMicrosecond timeout,
    const CanardTransferMetadata* metadata,
    const size_t size,
    const void* const buffer
);

void process_tx_queue();

#if defined(STM32G474xx) || defined(STM32_G)
void process_rx_frame(
    const FDCAN_RxHeaderTypeDef* rx_header,
    uint8_t rx_data[]
);
#else
void process_rx_frame(const CAN_RxHeaderTypeDef* rx_header, uint8_t rx_data[]);
#endif

void cyphal_subscribe(
    CanardPortID port_id,
    size_t extent,
    CanardRxSubscription* const subscription,
    void(*callback)(CanardRxTransfer*)
);


#ifdef __cplusplus
}
#endif
#endif
#endif /* INC_CYPHAL_H_ */
