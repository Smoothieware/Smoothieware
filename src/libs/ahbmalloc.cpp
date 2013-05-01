#include "ahbmalloc.h"

#include <string.h> // for memcpy

#include <stdint.h>
// #include <stdio.h>
#define printf(...)

#define BANK_SIZE 16384
#define AVERAGE_BLOCK_SIZE 256
#define BLOCKS  (BANK_SIZE / AVERAGE_BLOCK_SIZE)

extern uint8_t Image$$RW_IRAM2$$Base;
extern uint8_t Image$$RW_IRAM3$$Base;
// #define Image$$RW_IRAM2$$Base ((uint8_t *) 0x2007C000)
// #define Image$$RW_IRAM3$$Base ((uint8_t *) 0x20080000)

typedef struct {
    uint16_t size;
    uint16_t offset;
} FREEBLOCK;

uint8_t* bank_address[AHB_NUM_BANKS] = { &Image$$RW_IRAM2$$Base, &Image$$RW_IRAM3$$Base };

FREEBLOCK free_block_map[AHB_NUM_BANKS][BLOCKS] __attribute__ ((section (".bss")));

#define freeblock free_block_map[bank]

void* ahbmalloc(size_t size, BANK bank)
{
    // initialise size of first free block
    if ((freeblock[0].size == 0) && (freeblock[0].offset == 0))
        freeblock[0].size = BANK_SIZE;

    for (int i = 0; freeblock[i].size > 0; i++) {
        if (freeblock[i].size >= size) {
            uint16_t offset = freeblock[i].offset;
            uint16_t rem = freeblock[i].size - size;
            freeblock[i].offset += size;
            freeblock[i].size = rem;
            if (rem == 0) {
                // move free block ranges down if we end up with a hole eg by completely using a free fragment
                memcpy(&freeblock[i], &freeblock[i + 1], (BLOCKS - i - 1) * sizeof(FREEBLOCK));
                bzero(&freeblock[BLOCKS - 1], sizeof(FREEBLOCK));
            }
            return bank_address[bank] + offset;
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
        if ((p >= bank_address[i]) && ((p - bank_address[i]) < BANK_SIZE)) {
            bank = (BANK) i;
        }
    }
    if (bank == AHB_NUM_BANKS)
        return;

    int before = -1, after = -1;
    int last = 0;

	// printf("pointer is %p, bank address is %p\n", p, bank_address[bank]);

    uint16_t offset = (p - bank_address[bank]);

	printf("\tfinding before and after blocks\n");

    for (i = 0; freeblock[i].size > 0; i++)
    {
        last = i;
        // this block is before our address (or at the same point, which should never happen unless user double frees)
        if (offset >= freeblock[i].offset)
        {
        	if (before >= 0)
        	{
            	// if our block is before our 'before' block
            	if (offset < freeblock[before].offset)
                	before = i;
            	// if this block is closer than our previously selected 'before' block
            	else if ((offset - freeblock[i].offset) < (offset - freeblock[before].offset))
                	before = i;
            }
            else if ((freeblock[i].offset + freeblock[i].size) == offset)
            	before = i;
        }
        else // this block is after our address
        {
        	if (after >= 0) {
            	// if our block is after our 'after' block
            	if (offset > freeblock[after].offset)
                	after = i;
            	// if our block is closer than our previously selected 'after' block
            	else if ((freeblock[i].offset - offset) < (freeblock[after].offset - offset))
                	after = i;
           	}
           	else if ((offset + size) == freeblock[i].offset)
           		after = i;
        }
    }

    printf("\tFound before:%d, after:%d\n", before, after);

	printf("\t        offset is 0x%04X, size is %d\n", offset, size);
	printf("\tbefore: offset is 0x%04X, size is %d\n", freeblock[before].offset, freeblock[before].size);
	printf("\tafter:  offset is 0x%04X, size is %d\n", freeblock[after].offset, freeblock[after].size);

    // detect free blocks both before and after, this allows us to make three sections into one and free the list of one block descriptor
    if (
    	(before >= 0) &&
    	(after >= 0) &&
        ((freeblock[before].offset + freeblock[before].size) >= offset) &&
        ((freeblock[after].offset - size) <= offset)
    )
    {
    	printf("\t*** Combining before and after blocks\n");
		// Note: before and after blocks are not required to be adjacent in the list
        freeblock[before].size += size + freeblock[after].size;
        if ((after + 1) < BLOCKS)
            memmove(&freeblock[after], &freeblock[after + 1], (BLOCKS - after - 1) * sizeof(FREEBLOCK));
        bzero(&freeblock[BLOCKS - 1], sizeof(FREEBLOCK));
		printf("\tbefore: offset is 0x%04X, size is %d\n", freeblock[before].offset, freeblock[before].size);
		printf("\tafter:  offset is 0x%04X, size is %d\n", freeblock[after].offset, freeblock[after].size);
		return;
    }
    if (
    	(before >= 0) &&
    	((freeblock[before].offset + freeblock[before].size) >= offset)
    	)
    {
    	printf("\t*** Extending before block\n");
        // we have a free block just before us, simply extend it
        freeblock[before].size = (offset + size) - freeblock[before].offset;
		printf("\tbefore: offset is 0x%04X, size is %d\n", freeblock[before].offset, freeblock[before].size);
        return;
    }
    if (
    	(after >= 0) &&
    	((freeblock[after].offset - size) <= offset)
    	)
    {
    	printf("\t*** Moving after block\n");
        // we have a free block just after us, move it backwards while keeping the endpoint the same
        freeblock[after].size = (freeblock[after].offset + freeblock[after].size) - offset;
        freeblock[after].offset = offset;
		printf("\tafter:  offset is 0x%04X, size is %d\n", freeblock[after].offset, freeblock[after].size);
        return;
    }
    // no free blocks before or after us, create new free block on the stack
    last++;
    if (last < BLOCKS) {
	    printf("\tmake new entry at %d\n", last);
        freeblock[last].offset = offset;
        freeblock[last].size = size;
        return;
    }
    // we can't extend any free blocks and have no room for more free blocks
    // so unfortunately our only option is to leak this memory
	printf("\t*** No adjacent free blocks and no room on the free block list, %d bytes of ram leaked!\n", size);
    return;
}
