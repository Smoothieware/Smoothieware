/**
 * Basic FTP server (see RFC959).
 * Author: David Robertson <david@robertson.yt>
 *
 * For an excellent overview of the FTP protocol see https://cr.yp.to/ftp.html
 */

#pragma GCC diagnostic ignored "-Wunused-but-set-variable"

#include "ftpd.h"
extern "C" {
#include "uip.h"
}
#include "FATFileSystem.h"
#include "string.h"
#include "stdlib.h"

//#define DEBUG_PRINTF printf
#define DEBUG_PRINTF(...)


////// Public Interface

Ftpd::Ftpd() {  // Constructor
    uip_listen(HTONS(FTP_CONTROL_PORT));
    uip_listen(HTONS(FTP_PASSIVE_DATA_PORT));
}
Ftpd::~Ftpd() { // Destructor
    uip_unlisten(HTONS(FTP_CONTROL_PORT));
    uip_unlisten(HTONS(FTP_PASSIVE_DATA_PORT));
}
void Ftpd::appcall(void) {
    if (uip_conn->lport == HTONS(FTP_CONTROL_PORT)) {
        control_appcall();
    } else {
        data_appcall();
    }
}
bool Ftpd::handles_port(int port) {
    return port == FTP_CONTROL_PORT || port == FTP_PASSIVE_DATA_PORT;
}


////// Private Utilities

char *Ftpd::parse_path(char *pwd, char* input) {
    // Construct the absolute path to the file. Caller must free returned string.
    // TODO: deal with . and .. in paths
    char *result;
    if (input == NULL || strlen(input) == 0) { // No input, return a copy of pwd
        result = (char *) malloc(strlen(pwd) + 1);
        strcpy(result, pwd);
    } else if (input[0] == '/') {              // Got an absolute path, return a copy of input
        result = (char *) malloc(strlen(input) + 1);
        strcpy(result, input);
    } else {                                   // Got a relative path, use concatenation
        result = (char *) malloc(strlen(pwd) + 1 + strlen(input) + 1);
        strcpy(result, pwd);
        if (pwd[strlen(pwd)-1] != '/') {
            strcat(result, "/");
        }
        strcat(result, input);
    }
    return result;
}


////// Control Channel

void Ftpd::control_appcall(void) {
    struct control_conn_state *s;

    if(uip_connected()) { // New connection
        DEBUG_PRINTF("FTP: [control] new connection\n");
        s = (struct control_conn_state *) calloc(1, sizeof(struct control_conn_state));
        strcpy(s->pwd, "/");
        s->binary  = false;
        s->passive = false;
        s->done    = false;
        s->error   = false;
        s->rename_from = NULL;
        
        uip_conn->appstate = s;
        PSOCK_INIT(&s->p, s->ib, sizeof(s->ib));
        
        lastc = s;
    }

    s = (control_conn_state *) uip_conn->appstate;
    
    if (uip_closed() || uip_aborted() || uip_timedout()) {
        DEBUG_PRINTF("FTP: [control] connection closed\n");
        free(s);
        s = NULL;
        uip_conn->appstate = NULL;
    } else {
        handle_control_connection(s);
    }
}
int Ftpd::handle_control_connection(struct control_conn_state *s) {
    char *cursor; // may not be preserved across PSOCK calls
    char tmp[20]; // may not be preserved across PSOCK calls
    PSOCK_BEGIN(&s->p);
    PSOCK_SEND_STR(&s->p, "220 Smoothie FTP Service\r\n");
    while (true) {
        PSOCK_READTO(&s->p, '\n');
        s->ib[PSOCK_DATALEN(&s->p) - 2] = '\0'; // null terminate, stripping off CRLF
        s->args = strchr(s->ib, ' ');
        if (s->args != NULL) { s->args++; }
        s->done = false; 
        
        // Authentication
        if (strncmp(s->ib, "USER", 4) == 0) {
            PSOCK_SEND_STR(&s->p, "331 OK.\r\n");
        } else if (strncmp(s->ib, "PASS", 4) == 0) {
            // TODO: implement password auth
            PSOCK_SEND_STR(&s->p, "230 OK.\r\n");
            //PSOCK_SEND_STR(&s->p, "530 Incorrect password.\r\n");
            
        // Connection / Misc
        } else if (strncmp(s->ib, "SYST", 4) == 0) { 
            PSOCK_SEND_STR(&s->p, "215 UNIX Type: L8\r\n");
        } else if (strncmp(s->ib, "NOOP", 4) == 0) {
            PSOCK_SEND_STR(&s->p, "200 OK.\r\n");
        } else if (strncmp(s->ib, "QUIT", 4) == 0) {
            PSOCK_SEND_STR(&s->p, "221 Goodbye.\r\n");
            break;
        } else if (strncmp(s->ib, "TYPE", 4) == 0) {
            if (s->args) {
                s->binary = (s->args[0] == 'I');
                if (s->binary) {
                    PSOCK_SEND_STR(&s->p, "200 Binary Mode.\r\n");
                } else {
                    PSOCK_SEND_STR(&s->p, "200 ASCII Mode.\r\n");
                }
            } else {
                PSOCK_SEND_STR(&s->p, "500 Error\r\n");
            }
        } else if (strncmp(s->ib, "PORT", 4) == 0) {
            // Active mode: connect to client on supplied port to send data
            PSOCK_SEND_STR(&s->p, "520 Not implemented\r\n");
        } else if (strncmp(s->ib, "PASV", 4) == 0) {
            // Passive mode: listen for data connection from client on FTP_PASSIVE_DATA_PORT
            s->passive = true;
            lastc = s;
            PSOCK_SEND_STR(&s->p, "227 Entering Passive Mode (");
            
            // IP address    
            make_ip_str(tmp);
            PSOCK_SEND_STR(&s->p, tmp);  
            
            // Port (split into high/low byte)
            sprintf(tmp, "%d,%d)\r\n", FTP_PASSIVE_DATA_PORT>>8&0xFF, FTP_PASSIVE_DATA_PORT&0xFF);
            PSOCK_SEND_STR(&s->p, tmp);
        
        
        
        // Working Directory
        } else if (strncmp(s->ib, "PWD", 3) == 0) {
            PSOCK_SEND_STR(&s->p, "257 \"");
            PSOCK_SEND_STR(&s->p, s->pwd);
            PSOCK_SEND_STR(&s->p, "\"\r\n"); 
        } else if (strncmp(s->ib, "CWD", 3) == 0) {
            // TODO: check it actually exists first, and 550 if not
            if (s->args[0] == '/') {
                // Absolute path
                strncpy(s->pwd, s->args, sizeof(s->pwd)-1);
                s->pwd[sizeof(s->pwd)-1] = '\0';
            } else {
                // Relative path
                if (s->pwd[strlen(s->pwd)-1] != '/') {
                    strcat(s->pwd, "/");
                }
                strncat(s->pwd, s->args, sizeof(s->pwd)-strlen(s->pwd)-1);
                s->pwd[sizeof(s->pwd)-1] = '\0';
            }
            PSOCK_SEND_STR(&s->p, "250 OK\r\n"); 
        } else if (strncmp(s->ib, "CDUP", 4) == 0) {
            if (strlen(s->pwd) == 1) {
                // at the root, can't CDUP any more
                PSOCK_SEND_STR(&s->p, "550 Failed\r\n");
            } else {
                cursor = strrchr(s->pwd, '/');
                if (cursor != NULL) {
                    if (cursor == s->pwd) {
                        cursor++; // don't delete first slash if we try to CDUP to the root 
                    }
                    *(cursor) = '\0'; // terminate the string here
                }
                PSOCK_SEND_STR(&s->p, "200 OK.\r\n");
            }
            

            
        // File / Directory Management
        } else if (strncmp(s->ib, "DELE", 4) == 0) { // DELEte a file
            s->filename = parse_path(s->pwd, s->args);
            if (remove(s->filename) == 0) {
                PSOCK_SEND_STR(&s->p, "250 Deleted\r\n");
            } else {
                PSOCK_SEND_STR(&s->p, "550 Failed\r\n");
            }
            free(s->filename);
            s->filename = NULL;
                            
        } else if (strncmp(s->ib, "RNFR", 4) == 0) { // ReName FRom
            if (s->rename_from != NULL) {
                free(s->rename_from);
                s->rename_from = NULL;
            }
            s->rename_from = parse_path(s->pwd, s->args);
            PSOCK_SEND_STR(&s->p, "350 Ready\r\n");
        } else if (strncmp(s->ib, "RNTO", 4) == 0) { // ReName TO
            if (s->rename_from != NULL) {
                s->filename = parse_path(s->pwd, s->args);
                
                if (rename(s->rename_from, s->filename) == 0) {
                    PSOCK_SEND_STR(&s->p, "250 Renamed\r\n");
                } else {
                    PSOCK_SEND_STR(&s->p, "550 Failed\r\n");
                }
                
                free(s->filename);
                s->filename = NULL;
                free(s->rename_from);
                s->rename_from = NULL;
            } else {
                PSOCK_SEND_STR(&s->p, "550 Failed\r\n");
            }
        } else if (strncmp(s->ib, "MKD", 3) == 0) { // MaKe Directory
            s->filename = parse_path(s->pwd, s->args);
            if (mkdir(s->filename, 0) == 0) {
                PSOCK_SEND_STR(&s->p, "257 Created\r\n");
            } else {
                PSOCK_SEND_STR(&s->p, "550 Failed\r\n");
            }
            free(s->filename);
            s->filename = NULL;
            
        } else if (strncmp(s->ib, "RMD", 3) == 0) { // ReMove Directory
            s->filename = parse_path(s->pwd, s->args);
            if (remove(s->filename) == 0) {
                PSOCK_SEND_STR(&s->p, "250 Deleted\r\n");
            } else {
                PSOCK_SEND_STR(&s->p, "550 Failed\r\n");
            }
            free(s->filename);
            s->filename = NULL;
            
        
        // Upload / Download / Directory Listing
        } else if (strncmp(s->ib, "LIST", 4)==0 || strncmp(s->ib, "RETR", 4)==0 || strncmp(s->ib, "STOR", 4)==0) {
            DEBUG_PRINTF("FTP: [control] got LIST/RETR/STOR command\n");
            if        (strncmp(s->ib, "LIST", 4)==0) {
                s->task = LIST;
            } else if (strncmp(s->ib, "RETR", 4)==0) {
                s->task = RETR;
            } else if (strncmp(s->ib, "STOR", 4)==0) {
                s->task = STOR;
            } else {
                PSOCK_SEND_STR(&s->p, "500 Internal Server Error");
                continue;
            } 
            
            s->filename = parse_path(s->pwd, s->args);
                
            lastc = s;
            PSOCK_SEND_STR(&s->p, "150 Opening data connection\r\n");
            DEBUG_PRINTF("FTP: [control] 150 waiting for data thread to finish\n");
            PSOCK_WAIT_UNTIL(&s->p, s->done);
            
            if (s->error) {
                DEBUG_PRINTF("FTP: [control] 451 data thread completed with error \n");
                PSOCK_SEND_STR(&s->p, "451 Error\r\n");
            } else {
                DEBUG_PRINTF("FTP: [control] 226 data thread completed successfully\n");
                PSOCK_SEND_STR(&s->p, "226 Transfer complete.\r\n");
            }
            
            free(s->filename);
            s->filename = NULL;
            
                
        } else {
            PSOCK_SEND_STR(&s->p, "500 Unrecognised command\r\n");
        }
    } 
  
    PSOCK_CLOSE(&s->p);
    PSOCK_END(&s->p);
}

void Ftpd::make_ip_str(char *o) {
    uip_ipaddr_t hostaddr;
    uip_gethostaddr(&hostaddr);
    sprintf(o, "%u,%u,%u,%u,", uip_ipaddr1(hostaddr), uip_ipaddr2(hostaddr), uip_ipaddr3(hostaddr), uip_ipaddr4(hostaddr));    
}






////// Data Channel

void Ftpd::data_appcall(void) {
    struct data_conn_state *s;
    s = (struct data_conn_state *) uip_conn->appstate;
    
    if(uip_connected()) {
        // New connection
        s = (struct data_conn_state *) calloc(1, sizeof(struct data_conn_state));        
        uip_conn->appstate = s;
        s->oblen = 0;
        
        s->control = lastc; // TODO: find a more robust way to get the associated control connection.
        
        s->control->error = false;
        
        if (s->control->task == LIST) {
            list_connected(s);
        } else if (s->control->task == RETR) {
            retr_connected(s);
        } else if (s->control->task == STOR) {
            stor_connected(s);
        }
    }
    
    if(uip_acked()) {
        s->oblen = 0;
        s->ob[0] = '\0';
        
        if (s->control->task == LIST) {
            list_acked(s);
        } else if (s->control->task == RETR) {
            retr_acked(s);
        }
    }
    
    if(uip_newdata()) {
        if (s->control->task == STOR) {
            stor_newdata(s);
        }
    }
    
    if(uip_rexmit() || uip_newdata() || uip_acked() || uip_connected() || uip_poll()) {
        if (s->oblen > 0) {
            uip_send(s->ob, s->oblen);
        }
    }
    
    if(uip_closed() || uip_aborted() || uip_timedout()) {
        if (s != NULL) {
            DEBUG_PRINTF("FTP: [data] cleaning up\n");
            s->control->done = true;
            if (s->fd        != NULL) { fclose(s->fd);      }
            if (s->dir       != NULL) { closedir(s->dir);   }
            free(s);
            s = NULL;
        }
        uip_conn->appstate = NULL;
        return;
    }
}


void Ftpd::list_connected(struct data_conn_state *s) {
    DEBUG_PRINTF("FTP: [data] LIST connected\n");
    s->dir = opendir(s->control->filename);
    if (s->dir == NULL) {
        s->control->error = true;
        uip_close();
        return;
    }
    list_acked(s);
}
void Ftpd::list_acked(struct data_conn_state *s) {
    DEBUG_PRINTF("FTP: [data] LIST acked\n");
    // Easily Parsed LIST Format: https://cr.yp.to/ftp/list/eplf.html
    // TODO: last modified timestamp
    struct dirent *dirent = readdir(s->dir);
    if (dirent == NULL) {
        uip_close();
        return;
    }
    if (dirent->d_isdir) { // Directory
        sprintf(s->ob, "+/,\t%s\r\n", dirent->d_name);
    } else {               // File  
        int size = 0;
        char *filename2 = parse_path(s->control->filename, dirent->d_name);
        s->fd = fopen(filename2, "r");
        free(filename2);
        if (s->fd){
            fseek(s->fd, 0, SEEK_END);
            size = ftell(s->fd);
            fclose(s->fd);
            s->fd = NULL;
        }
        sprintf(s->ob, "+r,s%d,\t%s\r\n", size, dirent->d_name);
    }
    s->oblen = strlen(s->ob);
}


void Ftpd::retr_connected(struct data_conn_state *s) {
    DEBUG_PRINTF("FTP: [data] RETR connected\n");
    if (s->control->binary) {
        s->fd = fopen(s->control->filename, "rb");
    } else {
        s->fd = fopen(s->control->filename, "r");
    }
    if (s->fd == NULL) {
        s->control->error = true;
        uip_close();
        return;
    }
    retr_acked(s);
}
void Ftpd::retr_acked(struct data_conn_state *s) {
    DEBUG_PRINTF("FTP: [data] RETR acked\n");
    int count = fread(&s->ob, 1, sizeof(s->ob), s->fd);
    s->oblen = count;
    if (count == 0) {
        uip_close();
    }
}


void Ftpd::stor_connected(struct data_conn_state *s) {
    DEBUG_PRINTF("FTP: [data] STOR connected\n");
    if (s->control->binary) {
        s->fd = fopen(s->control->filename, "wb");
    } else {
        s->fd = fopen(s->control->filename, "w");
    }
    if (s->fd == NULL) {
        DEBUG_PRINTF("FTP: [data] fopen failed\n");
        s->control->error = true;
        uip_close();
    } 
}
void Ftpd::stor_newdata(struct data_conn_state *s) {
    DEBUG_PRINTF("FTP: [data] STOR newdata\n");
    fwrite(uip_appdata, 1, uip_datalen(), s->fd);
    // HACK ALERT... to work around the fwrite/filesystem bug where writing large amounts of data 
    // corrupts the file we workaround by closing the file, then reopening for append until we are done
    fclose(s->fd);
    if (s->control->binary) {
        s->fd = fopen(s->control->filename, "ab");
    } else {
        s->fd = fopen(s->control->filename, "a");
    }
}