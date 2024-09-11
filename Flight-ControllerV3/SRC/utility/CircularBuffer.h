/*
 * CircularBuffer.h
 *
 *  Created on: Jul 12, 2024
 *      Author: Ikenna
 */

#ifndef UTILITY_CIRCULARBUFFER_H_
#define UTILITY_CIRCULARBUFFER_H_

#include <stdint.h>
#include <stddef.h>

class CircularBuffer {
public:
    CircularBuffer();
    ~CircularBuffer();

    void put(uint8_t item);
    uint8_t get();
    uint8_t getHead(uint8_t* data, uint8_t len);
    void reset();
    bool isEmpty() const;
    bool isFull() const;
    size_t capacity() const;
    size_t size() const;

private:
    static const size_t sm_BufferSize = 100;
    volatile uint8_t m_buffer[sm_BufferSize];
    volatile size_t m_head;
    volatile size_t m_tail;
    volatile bool m_full;
};

#endif /* UTILITY_CIRCULARBUFFER_H_ */
