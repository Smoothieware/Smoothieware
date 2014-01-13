#ifndef __SFTPD_H__
#define __SFTPD_H__

/*
 * Implement RFC913  Simple File Transfer
 */


#include <stdio.h>
extern "C" {
#include "psock.h"
}

class Sftpd
{
public:
    Sftpd();
    virtual ~Sftpd();

    void appcall(void);
    void init(void);

private:
    FILE *fd;
    enum STATES { STATE_NORMAL, STATE_CONNECTED, STATE_GET_LENGTH, STATE_DOWNLOAD, STATE_CLOSE };
    STATES state;
    int acked();
    int handle_command();
    int handle_download();
    int senddata();

    struct psock sin;
    char buf[80];
    const char *outbuf;
    unsigned int filesize;
    char *filename;
};

#endif /* __sftpd_H__ */
