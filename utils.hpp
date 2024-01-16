#pragma once

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
