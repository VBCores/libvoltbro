#pragma once

#include <cstdint>

#include "utils.h"

class GenericEncoder {
protected:
    bool last_error = false;

    int16_t revolutions;
    encoder_data value = 0;
public:
    const bool is_electrical = false;
    const encoder_data electric_offset = 0;
    const bool is_inverted = false;
    const encoder_data CPR;

    GenericEncoder(encoder_data CPR, bool is_inverted = false): CPR(CPR), is_inverted(is_inverted) {};

    virtual void update_value() = 0;

    inline void incr_revolutions() {
        revolutions++;
    }
    inline void decr_revolutions() {
        revolutions--;
    }
    virtual inline encoder_data get_value() const {
        return value;
    }
    inline uint16_t get_revolutions() {
        return revolutions;
    }

};
