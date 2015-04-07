#ifndef _HEAPRING_H
#define _HEAPRING_H

template<class kind> class HeapRing {

    // smoothie-specific friend classes
    friend class Planner;
    friend class Conveyor;
    friend class Block;

public:
    HeapRing();
    HeapRing(unsigned int length);

    ~HeapRing();

    /*
     * direct accessors
     */
    kind& head();
    kind& tail();

    void push_front(kind&) __attribute__ ((warning("Not thread-safe if pop_back() is used in ISR context!"))); // instead, prepare(head_ref()); produce_head();
    kind& pop_back(void) __attribute__ ((warning("Not thread-safe if head_ref() is used to prepare new items, or push_front() is used in ISR context!"))); // instead, consume(tail_ref()); consume_tail();

    /*
     * pointer accessors
     */
    kind* head_ref();
    kind* tail_ref();

    void  produce_head(void);
    void  consume_tail(void);

    /*
     * queue status
     */
    bool is_empty(void);
    bool is_full(void);

    /*
     * resize
     *
     * returns true on success, or false if queue is not empty or not enough memory available
     */
    bool resize(unsigned int);

    /*
     * provide
     * kind*      - new buffer pointer
     * int length - number of items in buffer (NOT size in bytes!)
     *
     * cause HeapRing to use a specific memory location instead of allocating its own
     *
     * returns true on success, or false if queue is not empty
     */
    bool provide(kind*, unsigned int length);

protected:
    /*
     * these functions are protected as they should only be used internally
     * or in extremely specific circumstances
     */
    kind& item(unsigned int);
    kind* item_ref(unsigned int);

    unsigned int next(unsigned int);
    unsigned int prev(unsigned int);

    /*
     * buffer variables
     */
    unsigned int length;

    volatile unsigned int head_i;
    volatile unsigned int tail_i;

private:
    kind* ring;
};

#endif /* _HEAPRING_H */
