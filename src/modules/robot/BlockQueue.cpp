#include "BlockQueue.h"
#include "Block.h"

#include <cstdlib>
#include <new>
#include "cmsis.h"
#include "platform_memory.h"

/*
 * constructors
 */

BlockQueue::BlockQueue()
{
    head_i = tail_i = length = 0;
    isr_tail_i = tail_i;
    arena = nullptr;
    arena_bytes = 0;
    offsets = nullptr;
    head_end = 0;
}

/*
 * destructor
 */

BlockQueue::~BlockQueue()
{
    head_i = tail_i = length = 0;
    isr_tail_i = tail_i;
    if(arena != nullptr)
        free(arena);
    if(offsets != nullptr)
        free(offsets);
    arena = nullptr;
    offsets = nullptr;
    arena_bytes = 0;
}

/*
 * arena placement
 */

// Can a block of Block::bytes_needed() bytes be placed after the byte offset `end`, without running into the oldest
// queued block? On success *start is where it goes (0 if the arena wraps). Called with the queue state stable.
bool BlockQueue::can_place_after(unsigned int end, unsigned int *start) const
{
    const unsigned int need = Block::bytes_needed();
    // the oldest queued block; if nothing else is queued this is the block being filled, which must not be overwritten either
    const unsigned int oldest = (unsigned int)offsets[tail_i] << 3;
    if (end > oldest) {
        // free space is the rest of the arena, then (after a wrap) the part before the oldest block
        if (end + need <= arena_bytes) { *start = end; return true; }
        *start = 0;
        return need <= oldest;
    }
    // already wrapped: free space is between end and the oldest block
    *start = end;
    return end + need <= oldest;
}

void BlockQueue::place_head(unsigned int start)
{
    offsets[head_i] = (uint16_t)(start >> 3);
    Block *b = new (arena + start) Block();
    b->bind();
    head_end = start + Block::bytes_needed(); // the whole record is reserved; produce_head() keeps only what is used
}

void BlockQueue::produce_head()
{
    while (is_full());
    // the block being published keeps only its used bytes (header + axis records of the moving motors)
    head_end = ((unsigned int)offsets[head_i] << 3) + (unsigned int)head_ref()->bytes_used();
    unsigned int start = 0;
    can_place_after(head_end, &start);
    head_i = next(head_i);
    place_head(start);
}

void BlockQueue::consume_tail()
{
    if (!is_empty())
        tail_i = next(tail_i);
}

/*
 * queue status accessors
 */

bool BlockQueue::is_full() const
{
    // full when there is no slot, or no room in the arena, for the block after the one being filled
    if (next(head_i) == tail_i)
        return true;
    unsigned int start;
    return !can_place_after(head_end, &start);
}

unsigned int BlockQueue::used_bytes() const
{
    if (arena == nullptr) return 0;
    const unsigned int oldest = (unsigned int)offsets[tail_i] << 3;
    return head_end >= oldest ? head_end - oldest : head_end + (arena_bytes - oldest);
}

/*
 * resize
 */

bool BlockQueue::resize(unsigned int length, unsigned int bytes)
{
    if (is_empty())
    {
        if (length == 0)
        {
            __disable_irq();

            if (is_empty()) // check again in case something was pushed
            {
                head_i = tail_i = this->length = 0;

                __enable_irq();

                if (arena != nullptr)
                    free(arena);
                if (offsets != nullptr)
                    free(offsets);
                arena = nullptr;
                offsets = nullptr;
                arena_bytes = 0;

                return true;
            }

            __enable_irq();

            return false;
        }

        // Default: room for length - 1 queued blocks that use every motor without a compensation (the ring can hold no
        // more), plus the largest record for the block being filled. The old queue kept a separate tick_info array for
        // each of its blocks (see Block::bytes_dense()); this takes less in total, and all of it from the main heap.
        if (bytes == 0)
            bytes = Block::bytes_dense() * length + (Block::bytes_needed() - Block::bytes_dense());
        // Three largest records: a new head goes after the block just published or, after a wrap, before it. With the
        // published block starting at s, either s >= bytes_needed() (room before it) or s < bytes_needed() and then
        // bytes - s - bytes_needed() >= bytes_needed() (room after it), so even an empty queue can always take the next
        // block. With less, the head could be stuck forever between the end of the arena and a block that is already done.
        if (bytes < Block::bytes_needed() * 3)
            return false;
        // offsets are 16 bit counts of 8 bytes
        if (bytes > (65535u << 3))
            return false;

        // Note: we don't use realloc so we can fall back to the existing queue if allocation fails
        uint8_t *newarena = static_cast<uint8_t *>(malloc(bytes));
        uint16_t *newoffsets = static_cast<uint16_t *>(malloc(sizeof(uint16_t) * length));

        if (newarena != nullptr && newoffsets != nullptr)
        {
            uint8_t *oldarena = arena;
            uint16_t *oldoffsets = offsets;

            __disable_irq();

            if (is_empty()) // check again in case something was pushed while malloc did its thing
            {
                arena = newarena;
                offsets = newoffsets;
                arena_bytes = bytes;
                this->length = length;
                head_i = tail_i = isr_tail_i = 0;
                place_head(0);

                __enable_irq();

                if (oldarena != nullptr)
                    free(oldarena);
                if (oldoffsets != nullptr)
                    free(oldoffsets);

                return true;
            }

            __enable_irq();
        }

        if (newarena != nullptr) free(newarena);
        if (newoffsets != nullptr) free(newoffsets);
    }

    return false;
}
