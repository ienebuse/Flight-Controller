/*
 * CircularBuffer.cpp
 *
 *  Created on: Jul 12, 2024
 *      Author: Ikenna
 */

#include <CircularBuffer.h>
#include <typedefs.h>
#include <string.h>

const size_t CircularBuffer::sm_BufferSize;

CircularBuffer::CircularBuffer() : m_head(0), m_tail(0), m_full(false) {
}

CircularBuffer::~CircularBuffer() {
}

void CircularBuffer::put(uint8_t item) {
    m_buffer[m_head] = item;
    if (m_full) {
        m_tail = (m_tail + 1) % sm_BufferSize;
    }
    m_head = (m_head + 1) % sm_BufferSize;
    m_full = m_head == m_tail;
}

uint8_t CircularBuffer::get() {
    uint8_t item = m_buffer[m_tail];
    m_full = false;
    m_tail = (m_tail + 1) % sm_BufferSize;
    return item;
}

uint8_t CircularBuffer::getHead(uint8_t* data, uint8_t len) {
	uint8_t retLen = 0;
	if(isEmpty()) {
		return retLen;
	}


	// we need to save the state so it is not updated with new data ?????
	uint8_t head = m_head;
	uint8_t tail = m_tail;

	if(head > tail) {
		retLen = MIN(head - tail, len);
		memcpy(data, (void*)&m_buffer[tail], retLen);
	}
	else {

	}
}

void CircularBuffer::reset() {
    m_head = m_tail;
    m_full = false;
}

bool CircularBuffer::isEmpty() const {
    return (!m_full && (m_head == m_tail));
}

bool CircularBuffer::isFull() const {
    return m_full;
}

size_t CircularBuffer::capacity() const {
    return sm_BufferSize;
}

size_t CircularBuffer::size() const {
    size_t size = sm_BufferSize;

    if (!m_full) {
        if (m_head >= m_tail) {
            size = m_head - m_tail;
        } else {
            size = sm_BufferSize + m_head - m_tail;
        }
    }

    return size;
}

