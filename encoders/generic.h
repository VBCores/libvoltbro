#pragma once

#ifndef VBLIB_ENCODERS_GENERIC_H_
#define VBLIB_ENCODERS_GENERIC_H_

#ifdef __cplusplus
#include <cstdint>
extern "C" {
#else
#include <stdbool.h>
#include <stdint.h>
#endif

typedef struct GEncoder {
    bool inverted;
    bool last_error;
    bool is_electrical;
    const uint16_t CPR;
    uint16_t value;
    uint16_t elec_offset;
    // encoder interface
    uint16_t (*get_angle)(struct GEncoder*);
    uint64_t revolutions;  // number of ELECTRICAL turns
} GEncoder;

#ifdef __cplusplus
}
#endif

#endif /* VBLIB_ENCODERS_GENERIC_H_ */