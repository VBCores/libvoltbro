#pragma once
#if defined(STM32G474xx) || defined(STM32_G)

#include <cstdint>

#include "utils.h"
#include "dsp/low_pass_filter.hpp"

class GenericEncoder {
protected:
    bool last_error = false;

    /* WARNING! Explicitly specify alignment for guaranteed atomic reads and writes. Explanation:
     * https://developer.arm.com/documentation/dui0375/g/C-and-C---Implementation-Details/Basic-data-types-in-ARM-C-and-C-- or https://stackoverflow.com/a/52785864
     * short version: all reads/writes to var are atomic if it is "self"-aligned (1/2/4 byte)
     * (Please, copy this comment to all variables that can be accessed concurrently - as a warning and a reminder) */
    arm_atomic(int) revolutions;
    arm_atomic(encoder_data) value = 0;
public:
    const bool is_electrical = false;
    const encoder_data electric_offset = 0;
    const bool is_inverted = false;
    const encoder_data CPR;

    GenericEncoder(encoder_data CPR, bool is_inverted = false): CPR(CPR), is_inverted(is_inverted) {};

    virtual void update_value() = 0;
    virtual inline encoder_data get_value() const = 0;

    inline void incr_revolutions() {
        revolutions++;
    }
    inline void decr_revolutions() {
        revolutions--;
    }
    inline int get_revolutions() const{
        return revolutions;
    }

};

#endif
