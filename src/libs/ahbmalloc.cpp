#include "ahbmalloc.h"

#define BANK_SIZE 16384
#define AVERAGE_BLOCK_SIZE 256

extern uint8_t* Image$$RW_IRAM2$$Base;
extern uint8_t* Image$$RW_IRAM3$$Base;

typedef struct {
    uint16_t size;
    uint16_t offset;
} FREEBLOCK;

uint8_t* bank_address[AHB_NUM_BANKS] = { Image$$RW_IRAM2$$Base, Image$$RW_IRAM3$$Base };

FREEBLOCK free_block_map[AHB_NUM_BANKS][BANK_SIZE / AVERAGE_BLOCK_SIZE] __attribute__ ((section (".bss")));

#define freeblock free_block_map[bank]

void* ahbmalloc(size_t size, BANK bank)
{
    // initialise size of first free block
    if (freeblock[0].size == 0)
        freeblock[0].size = BANK_SIZE;

    for (int i = 0; freeblock[i].size > 0; i++) {
        if (freeblock[i].size >= size) {
            uint16_t offset = freeblock[i].offset;
            uint16_t rem = freeblock[i].size - size;
            freeblock[i].offset += size;
            freeblock[i].size = rem;
            if (rem == 0) {
                // TODO: move free block ranges down if we end up with a hole eg by completely using a free fragment
            }
            return &bank_address[bank][offset];
        }
    }
    return NULL;
}

void ahbfree(void* ptr, size_t size)
{
    uint8_t *p = ((uint8_t *) ptr);
    BANK bank = AHB_NUM_BANKS;
    int i;
    for (i = 0; i < AHB_NUM_BANKS; i++)
    {
        if ((p - bank_address[i]) < BANK_SIZE) {
            bank = (BANK) i;
        }
    }
    if (bank == AHB_NUM_BANKS)
        return;

    int before = 0, after = 0;
    int last = 0;
    uint16_t offset = (p - bank_address[i]);

    for (i = 0; freeblock[i].size > 0; i++)
    {
        last = i;
        // this block is before our address
        if (offset >= freeblock[i].offset)
        {
            // if our block is before our 'before' block
            if (offset < freeblock[before].offset)
                before = i;
            // if this block is closer than our previously selected 'before' block
            else if ((offset - freeblock[i].offset) < (offset - freeblock[before].offset))
                before = i;
        }
        else // this block is after our address
        {
            // if our block is after our 'after' block
            if (offset > freeblock[after].offset)
                after = i;
            // if our block is closer than our previously selected 'after' block
            else if ((freeblock[i].offset - offset) < (freeblock[after].offset - offset))
                after = i;
        }
    }
    if ((freeblock[before].offset + freeblock[before].size) == offset) {
        // we have a free block just before us, simply extend it
        freeblock[before].size += size;
        return;
    }
    if ((freeblock[after].offset - size) == offset) {
        // we have a free block just after us, move it backwards
        freeblock[after].offset -= size;
        freeblock[after].size += size;
        return;
    }
    // no free blocks before or after us, create new free block on the stack
    last++;
    if (last < (BANK_SIZE / AVERAGE_BLOCK_SIZE)) {
        freeblock[last].offset = offset;
        freeblock[last].size = size;
        return;
    }
    // we can't extend any free blocks and have no room for more free blocks
    // so unfortunately our only option is to leak this memory
    return;
}

//# excellent telecom
