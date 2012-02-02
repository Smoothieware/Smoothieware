#ifndef CIRCBUFFER_H
#define CIRCBUFFER_H

template <class T>
class CircBuffer {
public:
    CircBuffer(int length) {
        write = 0;
        read = 0;
        size = length + 1;
        buf = (T *)malloc(size * sizeof(T));
    };

    bool isFull() {
        return ((write + 1) % size == read);
    };

    bool isEmpty() {
        return (read == write);
    };

    void queue(T k) {
        if (isFull()) {
            read++;
            read %= size;
        }
        buf[write++] = k;
        write %= size;
    }

    uint16_t available() {
        return (write >= read) ? write - read : size - read + write;
    };

    bool dequeue(T * c) {
        bool empty = isEmpty();
        if (!empty) {
            *c = buf[read++];
            read %= size;
        }
        return(!empty);
    };

private:
    volatile uint16_t write;
    volatile uint16_t read;
    uint16_t size;
    T * buf;
};

#endif
