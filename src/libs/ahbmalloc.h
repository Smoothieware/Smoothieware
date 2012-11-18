#ifndef _AHBMALLOC_H
#define _AHBMALLOC_H

#include <stdint.h>
#include <stdlib.h>

typedef enum {
    AHB_BANK_0,
    AHB_BANK_1,
    AHB_NUM_BANKS
} BANK;

void* ahbmalloc(size_t size, BANK bank);
void ahbfree(void* ptr);

#endif /* _AHBMALLOC_H */
