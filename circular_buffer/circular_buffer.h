/*
 * circular_buffer.h
 *
 *  Created on: Mar 1, 2023
 *      Author: Igor Beschastnov
 */

#ifndef VBLIB_INCREMENTAL_ENCODER_ENCODER_H_
#define VBLIB_INCREMENTAL_ENCODER_ENCODER_H_

#ifdef __cplusplus
#include <cstdint>
extern "C" {
#else
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#endif

typedef struct CircularBuffer {
    size_t capacity;
    size_t size;
    int32_t data[];
} CircularBuffer;

CircularBuffer* make_buffer(size_t capacity);
void add_to_buffer(CircularBuffer* buffer, int32_t value);
int32_t get_avg_buffer(CircularBuffer* buffer);

__attribute__((always_inline)) static inline int32_t
get_from_buffer(CircularBuffer* buffer, size_t index) {
    return buffer->data[index];
}
__attribute__((always_inline)) static inline void
set_to_buffer(CircularBuffer* buffer, size_t index, int32_t value) {
    buffer->data[index] = value;
}

#ifdef __cplusplus
}
#endif
#endif /* VBLIB_INCREMENTAL_ENCODER_ENCODER_H_ */
