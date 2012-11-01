#ifndef _DISK_H
#define _DISK_H

#include <stdint.h>

class MSD_Disk {
public:
    /*
     * read a block on a storage chip
     *
     * @param data pointer where will be stored read data
     * @param block block number
     * @returns 0 if successful
     */
    virtual int disk_read(char * data, uint32_t block) = 0;

    /*
     * write a block on a storage chip
     *
     * @param data data to write
     * @param block block number
     * @returns 0 if successful
     */
    virtual int disk_write(const char * data, uint32_t block) = 0;

    /*
     * Disk initilization
     */
    virtual int disk_initialize() = 0;

    /*
     * Return the number of blocks
     *
     * @returns number of blocks
     */
    virtual uint32_t disk_sectors() = 0;

    /*
     * Return memory size
     *
     * @returns memory size
     */
    virtual uint64_t disk_size() = 0;

    virtual uint32_t disk_blocksize() = 0;

    /*
     * To check the status of the storage chip
     *
     * @returns status: 0: OK, 1: disk not initialized, 2: no medium in the drive, 4: write protected
     */
    virtual int disk_status() = 0;

    /*
     * check if this disk can do DMA operations
     */
    virtual bool disk_canDMA() = 0;
};

#endif /* _DISK_H */
