#pragma once

template <typename T> 
class Array{
    T* container;
    uint16_t capacity;

    public:
    Array(uint8_t _capacity = 20) {
        container = new T[capacity];
        capacity = _capacity;
    }

    Array (uint8_t _capacity, T* array) {
        capacity = _capacity;
        container = array;
    }

    ~Array() {
        delete container;
    }

    T& operator[] (uint16_t pos) {
        return container[pos];
    } 

    void realloc(uint16_t _capacity) {
        T* temp = container;
        container = new T[_capacity];

        for (int i  = 0; i < Min(_capacity, capacity); ++i)
            container[i] = temp[i];

        delete temp;
    }

    uint16_t size() {
        return capacity();
    }

    void each(void (*func)(T)) {
        for (T*p = container; p != container + capacity; ++p) {
            func(*p);
        }
    }
};