/**
 * Basic FTP server (see RFC959).
 * Author: David Robertson <david@robertson.yt>
 *
 * For an excellent overview of the FTP protocol see https://cr.yp.to/ftp.html
 */

#ifndef __Ftpd_H__
#define __Ftpd_H__

#include <stdio.h>
#include "DirHandle.h"
extern "C" {
#include "psock.h"
}

#define FTP_CONTROL_PORT      21
#define FTP_PASSIVE_DATA_PORT 10000

class Ftpd {
public:
    Ftpd();
    virtual ~Ftpd();
    void appcall(void);
    bool handles_port(int);
    
    
private:
    enum TASK { LIST, RETR, STOR };
    struct control_conn_state {
        struct psock p;     // Protosocket
        char ib[128];       // Input buffer
        char *args;         // Pointer to location inside ib where command arguments begin, NULL if no args
        char pwd[128];      // Working directory
        bool binary;        // Binary/ASCII mode flag
        bool passive;       // Active/Passive mode flag
        char *rename_from;  // Filename stored by RNFR command while awaiting RNTO
        char *filename;     // Used to store filename parsed from args
        TASK task;          // Which type of command should the data thread expect
        bool done;          // Flag for when the data connection is done
        bool error;         // Flag for if the data connection encountered an error
    };
    struct data_conn_state {
        char ob[256];                       // Output buffer
        int  oblen;                         // number of bytes in ob to send 
        struct control_conn_state *control; // The associated control connection
        FILE *fd;                           // File handle
        DIR  *dir;                          // Directory handle
    };
    
    struct control_conn_state *lastc;       
    
    char *parse_path(char *, char *);
    void control_appcall(void);
    int  handle_control_connection(struct control_conn_state *);
    void make_ip_str(char *);
    void data_appcall(void);
    void list_connected(struct data_conn_state *);
    void list_acked(struct data_conn_state *);
    void retr_connected(struct data_conn_state *);
    void retr_acked(struct data_conn_state *);
    void stor_connected(struct data_conn_state *);
    void stor_newdata(struct data_conn_state *);
};

#endif /* __Ftpd_H__ */