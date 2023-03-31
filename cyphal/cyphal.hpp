/*
 * cyphal.hpp
 *
 *  Created on: Nov 24, 2022
 *      Author: Igor Beschastnov
 *
 *  Весь код в vblib сишный, за исключением шаблонов, которые могут пригодится если вы используете C++ и хотите убрать кучу boilerplate кода
 */

#ifndef INC_CYPHAL_HPP_
#define INC_CYPHAL_HPP_
#ifdef HAL_CAN_MODULE_ENABLED

#include "cyphal.h"
#include <libcanard/canard.h>

// Shortcuts, always inlined

#define absolute_inline __attribute__((always_inline)) static inline

template <typename ObjType> using cyphal_serializer = int8_t (*)(const ObjType *const, uint8_t *const, size_t *const);
template <typename ObjType> using cyphal_deserializer = int8_t (*)(ObjType *const, const uint8_t*, size_t *const);

template <typename ObjType>
absolute_inline void send_cyphal(
    ObjType *obj,
    uint8_t buf[],
    CanardPortID port,
    CanardTransferID *transfer_id,
    CanardPriority priority,
    CanardTransferKind transfer_kind,
    CanardNodeID to_node_id,
    unsigned long buffer_size,
    cyphal_serializer<ObjType> serializer
) {
    size_t cyphal_buf_size = buffer_size;
    if (serializer(obj, buf, &cyphal_buf_size) < 0) {
        Error_Handler();
    }
    const CanardTransferMetadata cyphal_transfer_metadata = {
            .priority = priority,
            .transfer_kind = transfer_kind,
            .port_id = port,
            .remote_node_id = to_node_id,
            .transfer_id = *transfer_id,
    };
    cyphal_push(
            0,
            &cyphal_transfer_metadata,
            cyphal_buf_size,
            buf
    );
    (*transfer_id)++;
}

template <typename ObjType>
absolute_inline void send_cyphal_default_msg(
    ObjType *obj,
    uint8_t buf[],
    CanardPortID port,
    CanardTransferID *transfer_id,
    unsigned long buffer_size,
    cyphal_serializer<ObjType> serializer
) {
    send_cyphal<ObjType>(
            obj,
            buf,
            port,
            transfer_id,
            CanardPriorityNominal,
            CanardTransferKindMessage,
            CANARD_NODE_ID_UNSET,
            buffer_size,
            serializer
    );
}

template <typename ObjType>
absolute_inline void send_cyphal_default_msg_to(
    ObjType *obj,
    uint8_t buf[],
    CanardPortID port,
    CanardTransferID *transfer_id,
    CanardNodeID to_node_id,
    unsigned long buffer_size,
    cyphal_serializer<ObjType> serializer
) {
    send_cyphal<ObjType>(
            obj,
            buf,
            port,
            transfer_id,
            CanardPriorityNominal,
            CanardTransferKindMessage,
            to_node_id,
            buffer_size,
            serializer
    );
}

template <typename ObjType>
absolute_inline void send_cyphal_response(
    ObjType *obj,
    uint8_t buf[],
    CanardRxTransfer *transfer,
    CanardPortID port,
    unsigned long buffer_size,
    cyphal_serializer<ObjType> serializer
) {
    size_t cyphal_buf_size = buffer_size;
    if (serializer(obj, buf, &cyphal_buf_size) < 0) {
        Error_Handler();
    }
    const CanardTransferMetadata cyphal_transfer_metadata = {
            .priority = CanardPriorityNominal,
            .transfer_kind = CanardTransferKindResponse,
            .port_id = port,
            .remote_node_id = transfer->metadata.remote_node_id,
            .transfer_id = transfer->metadata.transfer_id,
    };
    cyphal_push(
            0,
            &cyphal_transfer_metadata,
            cyphal_buf_size,
            buf
    );
}

template <typename ObjType>
absolute_inline void cyphal_deserialize_transfer(
    ObjType *obj,
    CanardRxTransfer* transfer,
    size_t buf_size,
    cyphal_deserializer<ObjType> deserializer
) {
    size_t inout_buf_size = buf_size;
    if( deserializer(obj,(uint8_t *) transfer->payload, &inout_buf_size) < 0 ) {
        Error_Handler();
    }
}

#endif
#endif /* INC_CYPHAL_HPP_ */
