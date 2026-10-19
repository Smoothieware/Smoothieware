#pragma once

#include <stdint.h>

class Block;

class BlockQueue {

    // friend classes
    friend class Planner;
    friend class Conveyor;

public:
    BlockQueue();

    ~BlockQueue();

    /*
     * direct accessors
     */
    Block& head();
    Block& tail();

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
     * length is the number of block slots, arena_bytes the bytes the blocks live in (0 = the default: room for
     * length - 1 blocks in which every motor moves, see Block::bytes_dense(), plus one largest record). The arena and
     * the offset ring come from the main heap. arena_bytes must be at least 3 * Block::bytes_needed().
     * Returns true on success, or false if queue is not empty, the size is not allowed or not enough memory available
     */
    bool resize(unsigned int length, unsigned int arena_bytes = 0);

    unsigned int get_arena_bytes() const { return arena_bytes; }
    unsigned int get_length() const { return length; }
    unsigned int used_bytes() const; // bytes between the oldest queued block and the end of the one being filled

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
    // Blocks are packed one after the other in the arena, the ring only holds where each one starts (in units of 8
    // bytes). The block at head_i is already placed, so the planner can fill it; head_end is where the next one goes.
    bool can_place_after(unsigned int end, unsigned int *start) const;
    void place_head(unsigned int start);

    uint8_t *arena;
    unsigned int arena_bytes;
    uint16_t *offsets;
    unsigned int head_end; // byte offset just after the block at head_i
};
