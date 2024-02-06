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

constexpr micros MICROS_S = 1'000'000;
constexpr micros MICROS_0_1S = 100'000;
constexpr micros MICROS_0_01S = 10'000;
constexpr micros MICROS_0_001S = 1000;