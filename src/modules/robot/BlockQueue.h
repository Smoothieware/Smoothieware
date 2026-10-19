#pragma once

#include <stdint.h>

class Block;

class BlockQueue {

    // the planner owns the head side and walks the ring when it replans; everything the Conveyor needs is public below
    friend class Planner;

public:
    BlockQueue();

    ~BlockQueue();

    /*
     * direct accessors
     *
     * All the one-liners in this class are defined here in the header so the compiler inlines them: they are used by
     * the step interrupt, where a function call per access would be pure overhead.
     */
    Block& head() { return *item_ref(head_i); }
    Block& tail() { return *item_ref(tail_i); }

    /*
     * pointer accessors
     */
    Block* head_ref() { return item_ref(head_i); }
    Block* tail_ref() { return item_ref(tail_i); }

    void  produce_head(void);
    void  consume_tail(void);

    /*
     * queue status
     */
    bool is_empty(void) const { return head_i == tail_i; }
    bool is_full(void) const;

    /*
     * Step interrupt side (the isr_tail_i cursor, see the long comment in Conveyor.cpp). The interrupt only ever moves
     * isr_tail_i, and never frees anything: finished blocks are left for the idle side below to clean up.
     */
    bool   isr_has_block() const { return isr_tail_i != head_i; }       // something published and not yet finished
    Block* isr_block()           { return item_ref(isr_tail_i); }       // the block to run now
    void   isr_finish_block()    { isr_tail_i = next(isr_tail_i); }     // done with it, hand it to the idle side
    void   isr_finish_all()      { while (isr_has_block()) isr_finish_block(); } // flush: everything queued counts as done

    /*
     * Idle (main loop) side: blocks the interrupt has finished, still to be cleared and returned to the free pool.
     * Call has_finished_block(), then clear() tail_ref() and consume_tail().
     */
    bool has_finished_block() const { return tail_i != isr_tail_i; }

    /*
     * resize
     *
     * length is the number of block slots, arena_bytes the bytes the blocks live in (0 = the default: room for
     * length - 1 blocks in which every motor moves, see Block::bytes_dense(), plus one largest record). The arena and
     * the offset ring come from the main heap. arena_bytes must be at least 3 * Block::bytes_needed().
     * Returns true on success, or false if queue is not empty, the size is not allowed or not enough memory available
     */
    bool resize(unsigned int length, unsigned int arena_bytes = 0);

    unsigned int get_arena_bytes() const { return arena_bytes; }
    unsigned int get_length() const { return length; }
    unsigned int used_bytes() const; // bytes between the oldest queued block and the end of the one being filled

    /*
     * Walking the ring from oldest to newest, for diagnostics (index is opaque, only pass back what you were given)
     */
    unsigned int first_index() const { return tail_i; }
    bool         is_end_index(unsigned int i) const { return i == head_i; }
    unsigned int next_index(unsigned int i) const { return next(i); }
    Block*       block_at(unsigned int i) { return item_ref(i); }

protected:
    /*
     * these functions are protected as they should only be used internally
     * or in extremely specific circumstances
     */
    Block& item(unsigned int i) { return *item_ref(i); }

    // the block record for ring slot i (the arena holds the blocks, offsets[] where each one starts, in 8 byte units)
    Block* item_ref(unsigned int i) { return reinterpret_cast<Block *>(arena + ((unsigned int)offsets[i] << 3)); }

    // ring arithmetic, wrapping at length
    unsigned int next(unsigned int i) const { return (length == 0 || i + 1 >= length) ? 0 : i + 1; }
    unsigned int prev(unsigned int i) const { return length == 0 ? 0 : (i == 0 ? length - 1 : i - 1); }

    /*
     * buffer variables
     */
    unsigned int length;

    volatile unsigned int head_i;
    volatile unsigned int tail_i;
    volatile unsigned int isr_tail_i;

private:
    // Blocks are packed one after the other in the arena, the ring only holds where each one starts (in units of 8
    // bytes). The block at head_i is already placed, so the planner can fill it; head_end is where the next one goes.
    bool can_place_after(unsigned int end, unsigned int *start) const;
    void place_head(unsigned int start);

    uint8_t *arena;
    unsigned int arena_bytes;
    uint16_t *offsets;
    unsigned int head_end; // byte offset just after the block at head_i
};
