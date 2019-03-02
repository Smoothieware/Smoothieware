#pragma once

class Block;

class BlockQueue {

    // friend classes
    friend class Planner;
    friend class Conveyor;

public:
    BlockQueue();
    BlockQueue(unsigned int length);

    ~BlockQueue();

    /*
     * direct accessors
     */
    Block& head();
    Block& tail();

    void push_front(Block&) __attribute__ ((warning("Not thread-safe if pop_back() is used in ISR context!"))); // instead, prepare(head_ref()); produce_head();
    Block& pop_back(void) __attribute__ ((warning("Not thread-safe if head_ref() is used to prepare new items, or push_front() is used in ISR context!"))); // instead, consume(tail_ref()); consume_tail();

    /*
     * pointer accessors
     */
    Block* head_ref();
    Block* tail_ref();

    void  produce_head(void);
    void  consume_tail(void);

    /*
     * queue status
     */
    bool is_empty(void) const;
    bool is_full(void) const;

    /*
     * resize
     *
     * returns true on success, or false if queue is not empty or not enough memory available
     */
    bool resize(unsigned int);

    /*
     * provide
     * Block*      - new buffer pointer
     * int length - number of items in buffer (NOT size in bytes!)
     *
     * cause BlockQueue to use a specific memory location instead of allocating its own
     *
     * returns true on success, or false if queue is not empty
     */
    //bool provide(Block*, unsigned int length);

protected:
    /*
     * these functions are protected as they should only be used internally
     * or in extremely specific circumstances
     */
    Block& item(unsigned int);
    Block* item_ref(unsigned int);

    unsigned int next(unsigned int) const;
    unsigned int prev(unsigned int) const;

    /*
     * buffer variables
     */
    unsigned int length;

    volatile unsigned int head_i;
    volatile unsigned int tail_i;
    volatile unsigned int isr_tail_i;

private:
    Block* ring;
};
