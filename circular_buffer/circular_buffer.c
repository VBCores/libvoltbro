/*
 * circular_buffer.c
 *
 *  Created on: Mar 1, 2023
 *      Author: Igor Beschastnov
 */

#include "circular_buffer.h"

CircularBuffer* make_buffer(size_t capacity) {
    CircularBuffer* buffer =
        malloc(sizeof(CircularBuffer) + sizeof(int32_t) * capacity);
    if (buffer == NULL) {
        return NULL;
    }
    buffer->capacity = capacity;
    return buffer;
}

void add_to_buffer(CircularBuffer* buffer, int32_t value) {
    if (buffer->size >= buffer->capacity) {
        for (int i = 1; i < buffer->capacity; i++) {
            buffer->data[i - 1] = buffer->data[i];
        }
        buffer->data[buffer->capacity - 1] = value;
    } else {
        buffer->data[buffer->size] = value;
        buffer->size += 1;
    }
}

int32_t get_avg_buffer(CircularBuffer* buffer) {
    int32_t acc = 0;
    for (int i = 0; i < buffer->size; i++) {
        acc += buffer->data[i];
    }
    return (uint16_t)((float)acc / (float)buffer->size);
}