#pragma GCC diagnostic ignored "-Wunused-but-set-variable"

#include "sftpd.h"
#include "string.h"
#include "stdlib.h"

extern "C" {
#include "uip.h"
}

#define ISO_nl 0x0a
#define ISO_cr 0x0d
#define ISO_sp 0x20

#define DEBUG_PRINTF(...)

Sftpd::Sftpd()
{
    fd = NULL;
    state = STATE_NORMAL;
    outbuf = NULL;
    filename= NULL;
}

Sftpd::~Sftpd()
{
    if (fd != NULL) {
        fclose(fd);
    }
}

int Sftpd::senddata()
{
    if (outbuf != NULL) {
        DEBUG_PRINTF("sftp: senddata %s\n", outbuf);
        strcpy((char *)uip_appdata, outbuf);
        uip_send(uip_appdata, strlen(outbuf));
    }
    return 0;
}

int Sftpd::handle_command()
{
    PSOCK_BEGIN(&sin);

    do {
        PSOCK_READTO(&sin, ISO_nl);
        buf[PSOCK_DATALEN(&sin) - 1] = 0;
        int len = PSOCK_DATALEN(&sin) - 1;
        DEBUG_PRINTF("sftp: got command: %s, %d\n", buf, len);

        if (state == STATE_CONNECTED) {
            if (strncmp(buf, "USER", 4) == 0) {
                outbuf = "!user logged in\n";

            } else if (strncmp(buf, "KILL", 4) == 0) {
                if (len < 6) {
                    outbuf = "- incomplete KILL command\n";
                } else {
                    char *fn = &buf[5];
                    int s = remove(fn);
                    if (s == 0) outbuf = "+ deleted\n";
                    else outbuf = "- delete failed\n";
                }

            } else if (strncmp(buf, "DONE", 4) == 0) {
                outbuf = "+ exit\n";
                state = STATE_CLOSE;

            } else if (strncmp(buf, "STOR", 4) == 0) {
                if (len < 11) {
                    outbuf = "- incomplete STOR command\n";
                } else {
                    char *fn = &buf[9];
                    if(this->filename != NULL) free(this->filename);
                    this->filename= strdup(fn); // REMOVE when bug fixed
                    // get { NEW|OLD|APP }
                    if (strncmp(&buf[5], "OLD", 3) == 0) {
                        DEBUG_PRINTF("sftp: Opening file: %s\n", fn);
                        fd = fopen(fn, "w");
                        if (fd != NULL) {
                            outbuf = "+ new file\n";
                            state = STATE_GET_LENGTH;
                        } else {
                            outbuf = "- failed\n";
                        }
                    } else if (strncmp(&buf[5], "APP", 3) == 0) {
                        fd = fopen(fn, "a");
                        if (fd != NULL) {
                            outbuf = "+ append file\n";
                            state = STATE_GET_LENGTH;
                        } else {
                            outbuf = "- failed\n";
                        }
                    } else {
                        outbuf = "- Only OLD|APP supported\n";
                    }
                }

            } else {
                outbuf = "- Unknown command\n";
            }

        } else if (state == STATE_GET_LENGTH) {
            if (len < 6 || strncmp(buf, "SIZE", 4) != 0) {
                fclose(fd);
                fd = NULL;
                outbuf = "- Expected size\n";
                state = STATE_CONNECTED;

            } else {
                filesize = atoi(&buf[5]);
                if (filesize > 0) {
                    outbuf = "+ ok, waiting for file\n";
                    state = STATE_DOWNLOAD;
                } else {
                    fclose(fd);
                    fd = NULL;
                    outbuf = "- bad filesize\n";
                    state = STATE_CONNECTED;
                }
            }

        } else {
            DEBUG_PRINTF("WTF state: %d\n", state);
        }

    } while(state == STATE_CONNECTED || state == STATE_GET_LENGTH);

    PSOCK_END(&sin);
}

int Sftpd::handle_download()
{
    // Note this is not using PSOCK and it consumes all read data
    char *readptr = (char *)uip_appdata;
    unsigned int readlen = uip_datalen();
    DEBUG_PRINTF("sftp: starting download, expecting %d bytes, read %d\n", filesize, readlen);

    if (filesize > 0 && readlen > 0) {
        if (readlen > filesize) readlen = filesize;
        if (fwrite(readptr, 1, readlen, fd) != readlen) {
            DEBUG_PRINTF("sftp: Error writing file\n");
            fclose(fd);
            fd = NULL;
            outbuf = "- Error saving file\n";
            state = STATE_CONNECTED;
            return 0;
        }
        filesize -= readlen;
        DEBUG_PRINTF("sftp: saved %d bytes %d left\n", readlen, filesize);
    }
    if (filesize == 0) {
        DEBUG_PRINTF("sftp: download complete\n");
        fclose(fd);
        fd = NULL;
        outbuf = "+ Saved file\n";
        state = STATE_CONNECTED;
        return 0;
    }
    return 1;
}

int Sftpd::acked()
{
    outbuf= NULL;
    return 0;
}


void Sftpd::appcall(void)
{
    if (uip_connected()) {
        // TODO check for other connections
        PSOCK_INIT(&sin, buf, sizeof(buf));
        state = STATE_CONNECTED;
        outbuf = "+Smoothie SFTP Service\n";
    }

    if (state == STATE_CLOSE) {
        DEBUG_PRINTF("sftp: state close\n");
        state = STATE_NORMAL;
        uip_close();
        return;
    }

    if (uip_closed() || uip_aborted() || uip_timedout()) {
        DEBUG_PRINTF("sftp: closed\n");
        if (fd != NULL)
            fclose(fd);
        fd = NULL;
        state = STATE_NORMAL;
        return;
    }

    if (uip_acked()) {
        DEBUG_PRINTF("sftp: acked\n");
        this->acked();
    }

    if (uip_newdata()) {
        DEBUG_PRINTF("sftp: newdata\n");
        if (state == STATE_DOWNLOAD) {
            if(handle_download() == 0) {
                // we need to reset the input PSOCK again before using it after using the raw input buffer
                PSOCK_INIT(&sin, buf, sizeof(buf));
            }
        } else {
            handle_command();
        }
    }

    if (uip_rexmit() || uip_newdata() || uip_acked() || uip_connected() || uip_poll()) {
        this->senddata();
    }

}

void Sftpd::init(void)
{

}


