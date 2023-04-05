/*
 * cyphal.c
 *
 *  Created on: Nov 22, 2022
 *      Author: Igor Beschastnov
 */

#ifdef STM32_G
#include "stm32g4xx_hal.h"
#else
#include "stm32f4xx_hal.h"
#endif

#ifdef HAL_CAN_MODULE_ENABLED
#include "cyphal.h"

#ifndef CYPHAL_DF
uint32_t TxMailbox;
#endif

CanardInstance canard;
CanardTxQueue queue;

static CanardTransferID heartbeatTransferId = 0;
static uint8_t hbeat_ser_buf[uavcan_node_Heartbeat_1_0_EXTENT_BYTES_];

uint32_t micros(void) {
    // TODO: ??? or remove
    return 0;
}

void* memAllocate(CanardInstance* const ins, const size_t amount) {
    (void)ins;
    return o1heapAllocate(o1heap, amount);
}

void memFree(CanardInstance* const ins, void* const pointer) {
    (void)ins;
    o1heapFree(o1heap, pointer);
}

void init_cyphal(CanardNodeID NodeId) {
    canard = canardInit(&memAllocate, &memFree);
    canard.node_id = NodeId;

    queue = canardTxInit(200, CANARD_MTU_CAN_CLASSIC);
}

void send_heartbeat() {
    uint32_t tick = HAL_GetTick();
    uavcan_node_Heartbeat_1_0 heartbeat = {
        .uptime = (uint32_t)(tick / 1000),
        .health = {uavcan_node_Health_1_0_NOMINAL},
        .mode = {uavcan_node_Mode_1_0_OPERATIONAL},
        .vendor_specific_status_code = canard.node_id,
    };
    // Serialize the heartbeat message
    size_t hbeat_ser_buf_size =
        uavcan_node_Heartbeat_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;
    if (uavcan_node_Heartbeat_1_0_serialize_(
            &heartbeat,
            hbeat_ser_buf,
            &hbeat_ser_buf_size
        ) < 0) {
        Error_Handler();
    }
    const CanardTransferMetadata heartbeat_transfer_metadata = {
        .priority = CanardPriorityNominal,
        .transfer_kind = CanardTransferKindMessage,
        .port_id = uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_,
        .remote_node_id = CANARD_NODE_ID_UNSET,
        .transfer_id = heartbeatTransferId,
    };
    if (canardTxPush(
            &queue,
            &canard,
            0,
            &heartbeat_transfer_metadata,
            hbeat_ser_buf_size,
            hbeat_ser_buf
        ) < 0) {
        Error_Handler();
    }
    heartbeatTransferId++;
}

#ifdef STM32_G
const uint32_t CanardFDCANLengthToDLC[65] = {
    // 0-8
    FDCAN_DLC_BYTES_0,
    FDCAN_DLC_BYTES_1,
    FDCAN_DLC_BYTES_2,
    FDCAN_DLC_BYTES_3,
    FDCAN_DLC_BYTES_4,
    FDCAN_DLC_BYTES_5,
    FDCAN_DLC_BYTES_6,
    FDCAN_DLC_BYTES_7,
    FDCAN_DLC_BYTES_8,
    // 9-12
    FDCAN_DLC_BYTES_12,
    FDCAN_DLC_BYTES_12,
    FDCAN_DLC_BYTES_12,
    FDCAN_DLC_BYTES_12,
    // 13-16
    FDCAN_DLC_BYTES_16,
    FDCAN_DLC_BYTES_16,
    FDCAN_DLC_BYTES_16,
    FDCAN_DLC_BYTES_16,
    // 17-20
    FDCAN_DLC_BYTES_20,
    FDCAN_DLC_BYTES_20,
    FDCAN_DLC_BYTES_20,
    FDCAN_DLC_BYTES_20,
    // 20-24
    FDCAN_DLC_BYTES_24,
    FDCAN_DLC_BYTES_24,
    FDCAN_DLC_BYTES_24,
    FDCAN_DLC_BYTES_24,
    // 24-32
    FDCAN_DLC_BYTES_32,
    FDCAN_DLC_BYTES_32,
    FDCAN_DLC_BYTES_32,
    FDCAN_DLC_BYTES_32,
    FDCAN_DLC_BYTES_32,
    FDCAN_DLC_BYTES_32,
    FDCAN_DLC_BYTES_32,
    FDCAN_DLC_BYTES_32,
    // 33-48
    FDCAN_DLC_BYTES_48,
    FDCAN_DLC_BYTES_48,
    FDCAN_DLC_BYTES_48,
    FDCAN_DLC_BYTES_48,
    FDCAN_DLC_BYTES_48,
    FDCAN_DLC_BYTES_48,
    FDCAN_DLC_BYTES_48,
    FDCAN_DLC_BYTES_48,
    FDCAN_DLC_BYTES_48,
    FDCAN_DLC_BYTES_48,
    FDCAN_DLC_BYTES_48,
    FDCAN_DLC_BYTES_48,
    FDCAN_DLC_BYTES_48,
    FDCAN_DLC_BYTES_48,
    FDCAN_DLC_BYTES_48,
    FDCAN_DLC_BYTES_48,
    // 49-64
    FDCAN_DLC_BYTES_64,
    FDCAN_DLC_BYTES_64,
    FDCAN_DLC_BYTES_64,
    FDCAN_DLC_BYTES_64,
    FDCAN_DLC_BYTES_64,
    FDCAN_DLC_BYTES_64,
    FDCAN_DLC_BYTES_64,
    FDCAN_DLC_BYTES_64,
    FDCAN_DLC_BYTES_64,
    FDCAN_DLC_BYTES_64,
    FDCAN_DLC_BYTES_64,
    FDCAN_DLC_BYTES_64,
    FDCAN_DLC_BYTES_64,
    FDCAN_DLC_BYTES_64,
    FDCAN_DLC_BYTES_64,
    FDCAN_DLC_BYTES_64,
};

static inline uint8_t CanardDLCToFDCANLength(uint32_t fdcan_dlc) {
    uint8_t dlc_index = (uint8_t)(fdcan_dlc / 65536);
    if (dlc_index <= 8) {
        return dlc_index;
    }
    if (dlc_index <= 12) {
        return 8 + 4 * (dlc_index - 8);
    }
    return 32 + 16 * (dlc_index - 13);
}
#endif

void process_tx_queue() {
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
        return;
    }

    // Look at top of the TX queue of individual CAN frames
    while (queue.size != 0) {
        const CanardTxQueueItem* ti = canardTxPeek(&queue);

        if ((0U == ti->tx_deadline_usec) ||
            (ti->tx_deadline_usec > micros()))  // Check the deadline.
        {
            /* Instantiate a frame for the media layer */
#ifdef STM32_G
            FDCAN_TxHeaderTypeDef TxHeader;

            TxHeader.Identifier = ti->frame.extended_can_id;
            TxHeader.IdType = FDCAN_EXTENDED_ID;
            TxHeader.TxFrameType = FDCAN_DATA_FRAME;
            TxHeader.DataLength =
                CanardFDCANLengthToDLC[ti->frame.payload_size];
            TxHeader.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
            TxHeader.BitRateSwitch = FDCAN_BRS_ON;
            TxHeader.FDFormat = FDCAN_FD_CAN;
            TxHeader.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
            TxHeader.MessageMarker = 0x0;

            uint8_t TxData[64];

            memcpy(TxData, (uint8_t*)ti->frame.payload, ti->frame.payload_size);

            // if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1))  //
            // https://forum.opencyphal.org/t/uavcan-can-tx-buffer-management-in-can-fd-controllers/1215
            if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) !=
                HAL_OK) {
                break;
            }
#else
            CAN_TxHeaderTypeDef TxHeader;
            TxHeader.IDE = CAN_ID_EXT;
            TxHeader.RTR = CAN_RTR_DATA;
            TxHeader.DLC = CanardCANLengthToDLC[ti->frame.payload_size];
            TxHeader.ExtId = ti->frame.extended_can_id;
            TxHeader.TransmitGlobalTime = DISABLE;

            uint8_t TxData[8];

            memcpy(TxData, (uint8_t*)ti->frame.payload, ti->frame.payload_size);

            if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) !=
                HAL_OK) {
                break;
            }
#endif
        }
        // After the frame is transmitted or if it has timed out while waiting,
        // pop it from the queue and deallocate:
        canard.memory_free(&canard, canardTxPop(&queue, ti));
    }
}

void cyphal_push(
    const CanardMicrosecond tx_deadline_usec,
    const CanardTransferMetadata* const metadata,
    const size_t payload_size,
    const void* const payload
) {
    int32_t push_state = canardTxPush(
        &queue,
        &canard,
        tx_deadline_usec,
        metadata,
        payload_size,
        payload
    );
    if (push_state < 0) {
        Error_Handler();
    }
}

#ifdef STM32_G
void process_rx_frame(const FDCAN_RxHeaderTypeDef* RxHeader, uint8_t RxData[]) {
#else
void process_rx_frame(const CAN_RxHeaderTypeDef* RxHeader, uint8_t RxData[]) {
#endif
    CanardFrame rxf;

#ifdef STM32_G
    rxf.extended_can_id = RxHeader->Identifier;
    rxf.payload_size = CanardDLCToFDCANLength(RxHeader->DataLength);
#else
    rxf.extended_can_id = RxHeader->ExtId;
    rxf.payload_size = CanardCANDLCToLength[RxHeader->DLC];
#endif
    rxf.payload = (void*)RxData;

    CanardRxTransfer transfer = {.payload = NULL};
    CanardRxSubscription* subscription = NULL;
    const int8_t accept_result = canardRxAccept(
        (CanardInstance* const)&canard,
        micros(),
        &rxf,
        0,
        &transfer,
        &subscription
    );
    if (accept_result != 1) {
        goto exit;
    }
    if (subscription == NULL) {
        goto exit;
    }
    void (*processor)(CanardRxTransfer*) = subscription->user_reference;
    if (processor == NULL) {
        goto exit;
    }
    processor(&transfer);
exit:
    if (transfer.payload != NULL) {
        canard.memory_free(&canard, transfer.payload);
    }
}

#endif