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

FREEBLOCK free_map[AHB_NUM_BANKS][BANK_SIZE / AVERAGE_BLOCK_SIZE] __attribute__ ((section (".bss")));

void* ahbmalloc(size_t size, BANK bank)
{
    // initialise size of first free block
    if (free_map[bank][0].size == 0)
        free_map[bank][0].size = BANK_SIZE;

    for (int i = 0; free_map[bank][i].size > 0; i++) {
        if (free_map[bank][i].size >= size) {
            uint16_t rem = free_map[bank][i].size - size;
            free_map[bank][i].offset += size;
            free_map[bank][i].size = rem;
            if (rem == 0) {
                // TODO: move free block ranges down if we end up with a hole
            }
        }
    }
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
    int last;
    uint16_t offset = (p - bank_address[i]);

    for (i = 0; free_map[bank][i].size > 0; i++)
    {
        last = i;
        // this block is before our address
        if (offset >= free_map[bank][i].offset)
        {
            // if our block is before our 'before' block
            if (offset < free_map[bank][before].offset)
                before = i;
            // if this block is closer than our previously selected 'before' block
            else if ((offset - free_map[bank][i].offset) < (offset - free_map[bank][before].offset))
                before = i;
        }
        else // this block is after our address
        {
            // if our block is after our 'after' block
            if (offset > free_map[bank][after].offset)
                after = i;
            // if our block is closer than our previously selected 'after' block
            else if ((free_map[bank][i].offset - offset) < (free_map[bank][after].offset - offset))
                after = i;
        }
    }
    if ((free_map[bank][before].offset + free_map[bank][before].size) == offset) {
        // we have a free block just before us, simply extend it
        free_map[bank][before].size += size;
        return;
    }
    if ((free_map[bank][after].offset - size) == offset) {
        // we have a free block just after us, move it backwards
        free_map[bank][after].offset -= size;
        free_map[bank][after].size += size;
        return;
    }
    // no free blocks before or after us, create new free block on the stack
    last++;
    if (last < (BANK_SIZE / AVERAGE_BLOCK_SIZE)) {
        free_map[bank][last].offset = offset;
        free_map[bank][last].size = size;
        return;
    }
    // we can't extend any free blocks and have no room for more free blocks
    // so unfortunately our only option is to leak this memory
    return;
}

//# excellent telecom
