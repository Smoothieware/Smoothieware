#ifndef _SDFAT_H
#define _SDFAT_H

#include "disk.h"
#include "FATFileSystem.h"

class SDFAT : public mbed::FATFileSystem {
public:
    SDFAT(const char *n, MSD_Disk *disk);

    virtual int disk_initialize();
    virtual int disk_status();
    virtual int disk_read(char *buffer, int sector);
    virtual int disk_write(const char *buffer, int sector);
    virtual int disk_sync();
    virtual int disk_sectors();

    int remount();

protected:
    MSD_Disk *d;
};

#endif /* _SDFAT_H */
