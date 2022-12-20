#include <sys/cdefs.h>
/*
 * cyphal.h
 *
 *  Created on: Nov 22, 2022
 *      Author: Igor Beschastnov
 */

#ifndef INC_CYPHAL_H_
#define INC_CYPHAL_H_

#include "stm32g4xx_hal.h"

#include <libcanard/canard.h>
#include <uavcan/node/Heartbeat_1_0.h>
#include <voltbro/echo/echo_service_1_0.h>

extern FDCAN_HandleTypeDef hfdcan1;

extern CanardInstance canard;  // This is the core structure that keeps all the states and allocated resources of the library instance
extern CanardTxQueue queue;    // Prioritized transmission queue that keeps CAN frames destined for transmission via one CAN interface

#ifdef __cplusplus
extern "C" {
#endif

extern void *memAllocate(CanardInstance *const ins, const size_t amount);

extern void memFree(CanardInstance *const ins, void *const pointer);

_Noreturn extern void Error_Handler();

uint32_t micros(void);

void init_cyphal(CanardNodeID node_id);

void send_heartbeat();

extern void subscribe_all();

void cyphal_push(
    const CanardMicrosecond      timeout,
    const CanardTransferMetadata *metadata,
    const size_t                 size,
    const void *const            buffer
);

void process_tx_queue();

void process_rx_frame(const FDCAN_RxHeaderTypeDef *rx_header, uint8_t rx_data[]);

#ifdef __cplusplus
}
#endif
#endif /* INC_CYPHAL_H_ */
