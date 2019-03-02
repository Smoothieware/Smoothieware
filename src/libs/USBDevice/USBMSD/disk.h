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
    virtual int disk_read(char * data, uint32_t block) { return 0; };

    /*
     * write a block on a storage chip
     *
     * @param data data to write
     * @param block block number
     * @returns 0 if successful
     */
    virtual int disk_write(const char * data, uint32_t block) { return 0; };

    /*
     * Disk initilization
     */
    virtual int disk_initialize() { return 0; };

    /*
     * Return the number of blocks
     *
     * @returns number of blocks
     */
    virtual uint32_t disk_sectors() { return 0; };

    /*
     * Return memory size
     *
     * @returns memory size
     */
    virtual uint64_t disk_size() { return 0; };

    virtual uint32_t disk_blocksize() { return 0; };

    /*
     * To check the status of the storage chip
     *
     * @returns status: 0: OK, 1: disk not initialized, 2: no medium in the drive, 4: write protected
     */
    virtual int disk_status() { return 0; };

    /*
     * check if this disk can do DMA operations
     */
    virtual bool disk_canDMA() { return 0; };

    virtual int disk_sync() { return 0; };

    virtual bool busy() = 0;
};

#endif /* _DISK_H */
