//
//  Simple fixed size ring buffer.
//  Manage objects by value.
//  Thread safe for single Producer and single Consumer.
//  By Dennis Lang http://home.comcast.net/~lang.dennis/code/ring/ring.html
//  Slightly modified for naming

#pragma once

template <class T, size_t RingSize>
class TSRingBuffer
{
public:
    TSRingBuffer()
        : m_size(RingSize), m_buffer(new T[RingSize]), m_rIndex(0), m_wIndex(0)
    { }

    ~TSRingBuffer()
    {
        delete [] m_buffer;
    };

    size_t next(size_t n) const
    {
        return (n + 1) % m_size;
    }

    bool empty() const
    {
        return (m_rIndex == m_wIndex);
    }

    bool full() const
    {
        return (next(m_wIndex) == m_rIndex);
    }

    bool put(const T &value)
    {
        if (full())
            return false;
        m_buffer[m_wIndex] = value;
        m_wIndex = next(m_wIndex);
        return true;
    }

    bool get(T &value)
    {
        if (empty())
            return false;
        value = m_buffer[m_rIndex];
        m_rIndex = next(m_rIndex);
        return true;
    }

private:
    size_t          m_size;
    T              *m_buffer;

    // volatile is only used to keep compiler from placing values in registers.
    // volatile does NOT make the index thread safe.
    volatile size_t m_rIndex;
    volatile size_t m_wIndex;
};
