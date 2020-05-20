#include "BlockQueue.h"
#include "Block.h"

#include <cstdlib>
#include "cmsis.h"
#include "platform_memory.h"

/*
 * constructors
 */

BlockQueue::BlockQueue()
{
    head_i = tail_i = length = 0;
    isr_tail_i = tail_i;
    ring = nullptr;
}

BlockQueue::BlockQueue(unsigned int length)
{
    head_i = tail_i = 0;
    isr_tail_i = tail_i;
    void *v= AHB0.alloc(sizeof(Block) * length);
    ring = new(v) Block[length];
    // TODO: handle allocation failure
    this->length = length;
}

/*
 * destructor
 */

BlockQueue::~BlockQueue()
{
    head_i = tail_i = length = 0;
    isr_tail_i = tail_i;
    if(ring != nullptr)
        AHB0.dealloc(ring); // delete [] ring;
    ring = nullptr;
}

/*
 * index accessors (protected)
 */

unsigned int BlockQueue::next(unsigned int item) const
{
    if (length == 0)
        return 0;

    if (++item >= length)
        return 0;

    return item;
}

unsigned int BlockQueue::prev(unsigned int item) const
{
    if (length == 0)
        return 0;

    if (item == 0)
        return (length - 1);
    else
        return (item - 1);
}

/*
 * reference accessors
 */

Block& BlockQueue::head()
{
    return ring[head_i];
}

Block& BlockQueue::tail()
{
    return ring[tail_i];
}

Block& BlockQueue::item(unsigned int i)
{
    return ring[i];
}

void BlockQueue::push_front(Block& item)
{
    ring[head_i] = item;
    head_i = next(head_i);
}

Block& BlockQueue::pop_back()
{
    Block& r = ring[tail_i];
    tail_i = next(tail_i);
    return r;
}

/*
 * pointer accessors
 */

Block* BlockQueue::head_ref()
{
    return &ring[head_i];
}

Block* BlockQueue::tail_ref()
{
    return &ring[tail_i];
}

Block* BlockQueue::item_ref(unsigned int i)
{
    return &ring[i];
}

void BlockQueue::produce_head()
{
    while (is_full());
    head_i = next(head_i);
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
    //__disable_irq();
    bool r = (next(head_i) == tail_i);
    //__enable_irq();

    return r;
}

bool BlockQueue::is_empty() const
{
    //__disable_irq();
    bool r = (head_i == tail_i);
    //__enable_irq();

    return r;
}

/*
 * resize
 */

bool BlockQueue::resize(unsigned int length)
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

                if (ring != nullptr)
                    AHB0.dealloc(ring); // delete [] ring;
                ring = nullptr;

                return true;
            }

            __enable_irq();

            return false;
        }

        // Note: we don't use realloc so we can fall back to the existing ring if allocation fails
        void *v= AHB0.alloc(sizeof(Block) * length);
        Block* newring = new(v) Block[length];

        if (newring != nullptr)
        {
            Block* oldring = ring;

            __disable_irq();

            if (is_empty()) // check again in case something was pushed while malloc did its thing
            {
                ring = newring;
                this->length = length;
                head_i = tail_i = 0;

                __enable_irq();

                if (oldring != nullptr)
                    AHB0.dealloc(oldring); // delete [] oldring;

                return true;
            }

            __enable_irq();

            AHB0.dealloc(newring); // delete [] newring;
        }
    }

    return false;
}

// bool BlockQueue::provide(Block* buffer, unsigned int length)
// {
//     __disable_irq();

//     if (is_empty())
//     {
//         Block* oldring = ring;

//         if ((buffer != nullptr) && (length > 0))
//         {
//             ring = buffer;
//             this->length = length;
//             head_i = tail_i = 0;

//             __enable_irq();

//             if (oldring != nullptr)
//                 delete [] oldring;
//             return true;
//         }
//     }

//     __enable_irq();

//     return false;
// }
