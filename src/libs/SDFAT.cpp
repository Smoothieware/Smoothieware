#include "SDFAT.h"

SDFAT::SDFAT(const char *n, MSD_Disk *disk) : mbed::FATFileSystem(n)
{
    d = disk;
}

int SDFAT::disk_initialize()
{
    return d->disk_initialize();
}

int SDFAT::disk_status()
{
    return d->disk_status();
}

int SDFAT::disk_read(char *buffer, int sector)
{
    return d->disk_read(buffer, sector);
}

int SDFAT::disk_write(const char *buffer, int sector)
{
    return d->disk_write(buffer, sector);
}

int SDFAT::disk_sync()
{
    return d->disk_sync();
}

int SDFAT::disk_sectors()
{
    return d->disk_sectors();
}
int SDFAT::remount() {
    f_mount(_fsid, NULL);
    f_mount(_fsid, &_fs);
    
	return 0;
}