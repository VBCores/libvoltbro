#pragma once

#include "utils.h"

template <class T>
class ReservedObject {
private:
    unsigned char buffer[sizeof(T)];
    T* obj;
public:
    template <class... Args>
    void create(Args&&... args) {
        obj = new (buffer) T(args...);
    }

    T* pointer() {
        return obj;
    }
    T* operator->() {
        return obj;
    }

    ~ReservedObject() {
        obj->~T();
    }
};

constexpr const micros MICROS_S = 1000000;
constexpr const micros MICROS_0_1S = MICROS_S / 10;
constexpr const micros MICROS_0_01S = MICROS_S / 100;
constexpr const micros MICROS_0_001S = MICROS_S / 1000;
