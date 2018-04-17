#pragma GCC diagnostic ignored "-Wredundant-decls"
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
#pragma GCC diagnostic ignored "-Wcast-align"
#pragma GCC diagnostic ignored "-Wcast-qual"
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"

/**
 * \addtogroup apps
 * @{
 */

/**
 * \defgroup httpd Web server
 * @{
 * The uIP web server is a very simplistic implementation of an HTTP
 * server. It can serve web pages and files from a read-only ROM
 * filesystem, and provides a very small scripting language.

 */

/**
 * \file
 *         Web server
 * \author
 *         Adam Dunkels <adam@sics.se>
 */


/*
 * Copyright (c) 2004, Adam Dunkels.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the uIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 * $Id: httpd.c,v 1.2 2006/06/11 21:46:38 adam Exp $
 */

#include <stdio.h>

#include "uip.h"
#include "httpd.h"
#include "httpd-fs.h"
#include "http-strings.h"

#include <string.h>
#include "stdio.h"
#include "stdlib.h"

#include "CommandQueue.h"
#include "CallbackStream.h"

#include "c-fifo.h"

#define STATE_WAITING 0
#define STATE_HEADERS 1
#define STATE_BODY    2
#define STATE_OUTPUT  3
#define STATE_UPLOAD  4

#define GET     1
#define POST    2
#define OPTIONS 3

#define ISO_nl      0x0a
#define ISO_space   0x20
#define ISO_bang    0x21
#define ISO_percent 0x25
#define ISO_period  0x2e
#define ISO_slash   0x2f
#define ISO_colon   0x3a

#define DEBUG_PRINTF printf
//#define DEBUG_PRINTF(...)


// this callback gets the results of a command, line by line. need to check if
// we need to stall the upstream sender return 0 if stalled 1 if ok to keep
// providing more -1 if the connection has closed or is not in output state.
// need to see which connection to send to based on state and add result to
// that fifo for each connection. NOTE this will not get called if the
// connection has been closed and the stream will get deleted when the last
// command has been executed
static int command_result(const char *str, void *state)
{
    struct httpd_state *s = (struct httpd_state *)state;
    if (s == NULL) {
        // connection was closed so discard, this should never happen
        DEBUG_PRINTF("ERROR: command result for closed state %d\n", (int)state);
        return -1;
    }

    if (str == NULL) {
        DEBUG_PRINTF("End of command (%p)\n", state);
        fifo_push(s->fifo, NULL);

    } else {
        if (fifo_size(s->fifo) < 10) {
            DEBUG_PRINTF("Got command result (%p): %s", state, str);
            fifo_push(s->fifo, strdup(str));
            return 1;
        } else {
            DEBUG_PRINTF("command result fifo is full (%p)\n", state);
            return 0;
        }
    }
    return 1;
}

static void create_callback_stream(struct httpd_state *s)
{
    // need to create a callback stream here, but do one per connection pass
    // the state to the callback, also create the fifo for the command results
    s->fifo = new_fifo();
    s->pstream = new_callback_stream(command_result, s);
}

// Used to save files to SDCARD during upload
static FILE *fd;
static char *output_filename = NULL;
static int file_cnt = 0;
static int open_file(const char *fn)
{
    if (output_filename != NULL) free(output_filename);
    output_filename = malloc(strlen(fn) + 5);
    strcpy(output_filename, "/sd/");
    strcat(output_filename, fn);
    fd = fopen(output_filename, "w");
    if (fd == NULL) {
        free(output_filename);
        output_filename = NULL;
        return 0;
    }
    return 1;
}

static int close_file()
{
    free(output_filename);
    output_filename = NULL;
    fclose(fd);
    return 1;
}

static int save_file(uint8_t *buf, unsigned int len)
{
    if (fwrite(buf, 1, len, fd) == len) {
        file_cnt += len;
        return 1;

    } else {
        close_file();
        return 0;
    }
}

static int fs_open(struct httpd_state *s)
{
    if (strncmp(s->filename, "/sd/", 4) == 0) {
        DEBUG_PRINTF("Opening file %s\n", s->filename);
        s->fd = fopen(s->filename, "r");
        if (s->fd == NULL) {
            DEBUG_PRINTF("Failed to open: %s\n", s->filename);
            return 0;
        }
        return 1;

    } else {
        s->fd = NULL;
        return httpd_fs_open(s->filename, &s->file);
    }
}

/*---------------------------------------------------------------------------*/
static PT_THREAD(send_command_response(struct httpd_state *s))
{
    PSOCK_BEGIN(&s->sout);

    do {
        PSOCK_WAIT_UNTIL( &s->sout, fifo_size(s->fifo) > 0 );
        s->strbuf = fifo_pop(s->fifo);
        if (s->strbuf != NULL) {
            // send it
            DEBUG_PRINTF("Sending response: %s", s->strbuf);
            // TODO send as much as we can in one packet
            PSOCK_SEND_STR(&s->sout, s->strbuf);
            // free the strdup
            free(s->strbuf);
        }else if(--s->command_count <= 0) {
            // when all commands have completed exit
            break;
        }
    } while (1);

    PSOCK_END(&s->sout);
}

/*---------------------------------------------------------------------------*/
static unsigned short generate_part_of_file(void *state)
{
    struct httpd_state *s = (struct httpd_state *)state;

    if (s->file.len > uip_mss()) {
        s->len = uip_mss();
    } else {
        s->len = s->file.len;
    }
    memcpy(uip_appdata, s->file.data, s->len);

    return s->len;
}
/*---------------------------------------------------------------------------*/
static unsigned short generate_part_of_sd_file(void *state)
{
    struct httpd_state *s = (struct httpd_state *)state;

    int len = fread(uip_appdata, 1, uip_mss(), s->fd);
    if (len <= 0) {
        // we need to send something
        strcpy(uip_appdata, "\r\n");
        len = 2;
        s->len = 0;
    } else {
        s->len = len;
    }
    return len;
}
/*---------------------------------------------------------------------------*/
static
PT_THREAD(send_file(struct httpd_state *s))
{
    PSOCK_BEGIN(&s->sout);

    do {
        PSOCK_GENERATOR_SEND(&s->sout, generate_part_of_file, s);
        s->file.len -= s->len;
        s->file.data += s->len;
    } while (s->file.len > 0);

    PSOCK_END(&s->sout);
}

/*---------------------------------------------------------------------------*/
static PT_THREAD(send_sd_file(struct httpd_state *s))
{
    PSOCK_BEGIN(&s->sout);

    do {
        PSOCK_GENERATOR_SEND(&s->sout, generate_part_of_sd_file, s);
    } while (s->len > 0);

    fclose(s->fd);
    s->fd = NULL;

    PSOCK_END(&s->sout);
}

/*---------------------------------------------------------------------------*/
static PT_THREAD(send_headers_3(struct httpd_state *s, const char *statushdr, char send_content_type))
{
    char *ptr;

    PSOCK_BEGIN(&s->sout);

    PSOCK_SEND_STR(&s->sout, statushdr);
    PSOCK_SEND_STR(&s->sout, http_header_all);

    if (send_content_type) {
        ptr = strrchr(s->filename, ISO_period);
        if (ptr == NULL) {
            PSOCK_SEND_STR(&s->sout, http_content_type_plain); // http_content_type_binary);
        } else if (strncmp(http_html, ptr, 5) == 0) {
            PSOCK_SEND_STR(&s->sout, http_content_type_html);
        } else if (strncmp(http_css, ptr, 4) == 0) {
            PSOCK_SEND_STR(&s->sout, http_content_type_css);
        } else if (strncmp(http_png, ptr, 4) == 0) {
            PSOCK_SEND_STR(&s->sout, http_content_type_png);
        } else if (strncmp(http_gif, ptr, 4) == 0) {
            PSOCK_SEND_STR(&s->sout, http_content_type_gif);
        } else if (strncmp(http_jpg, ptr, 4) == 0) {
            PSOCK_SEND_STR(&s->sout, http_content_type_jpg);
        } else {
            PSOCK_SEND_STR(&s->sout, http_content_type_plain);
        }
    }
    PSOCK_END(&s->sout);
}
static PT_THREAD(send_headers(struct httpd_state *s, const char *statushdr))
{
    return send_headers_3(s, statushdr, 1);
}
/*---------------------------------------------------------------------------*/
static
PT_THREAD(handle_output(struct httpd_state *s))
{
    PT_BEGIN(&s->outputpt);

    if (s->method == OPTIONS) {
        PT_WAIT_THREAD(&s->outputpt, send_headers(s, http_header_preflight));
        PSOCK_SEND_STR(&s->sout, "OK\r\n");
    }
    else if (s->method == POST) {
        if (strcmp(s->filename, "/command") == 0) {
            DEBUG_PRINTF("Executed command post\n");
            PT_WAIT_THREAD(&s->outputpt, send_headers(s, http_header_200));
            // send response as we get it
            PT_WAIT_THREAD(&s->outputpt, send_command_response(s));

        } else if (strcmp(s->filename, "/command_silent") == 0) {
            DEBUG_PRINTF("Executed silent command post\n");
            PT_WAIT_THREAD(&s->outputpt, send_headers(s, http_header_200));

        } else if (strcmp(s->filename, "/upload") == 0) {
            DEBUG_PRINTF("upload output: %d\n", s->uploadok);
            if (s->uploadok == 0) {
                PT_WAIT_THREAD(&s->outputpt, send_headers(s, http_header_503));
                PSOCK_SEND_STR(&s->sout, "FAILED\r\n");
            } else {
                PT_WAIT_THREAD(&s->outputpt, send_headers(s, http_header_200));
                PSOCK_SEND_STR(&s->sout, "OK\r\n");
            }

        } else {
            DEBUG_PRINTF("Unknown POST: %s\n", s->filename);
            httpd_fs_open(http_404_html, &s->file);
            strcpy(s->filename, http_404_html);
            PT_WAIT_THREAD(&s->outputpt, send_headers(s, http_header_404));
            PT_WAIT_THREAD(&s->outputpt, send_file(s));
        }

    } else {
        // Presume method GET
        if (!fs_open(s)) { // Note this has the side effect of opening the file
            DEBUG_PRINTF("404 file not found\n");
            httpd_fs_open(http_404_html, &s->file);
            strcpy(s->filename, http_404_html);
            PT_WAIT_THREAD(&s->outputpt, send_headers(s, http_header_404));
            PT_WAIT_THREAD(&s->outputpt, send_file(s));

        } else if (s->cache_page) {
            if (s->fd != NULL) {
                // if it was an sd file then we need to close it
                fclose(s->fd);
                s->fd = NULL;
            }
            // tell it it has not changed
            DEBUG_PRINTF("304 Not Modified\n");
            PT_WAIT_THREAD(&s->outputpt, send_headers_3(s, http_header_304, 0));

        } else {
            DEBUG_PRINTF("sending file %s\n", s->filename);
            PT_WAIT_THREAD(&s->outputpt, send_headers(s, http_header_200));
            if (s->fd != NULL) {
                // send from sd card
                PT_WAIT_THREAD(&s->outputpt, send_sd_file(s));

            } else {
                // send from FLASH
                PT_WAIT_THREAD(&s->outputpt, send_file(s));
            }
        }
    }

    PSOCK_CLOSE(&s->sout);
    PT_END(&s->outputpt);
}

/*---------------------------------------------------------------------------*/
// this forces us to yield every other call as we read all data everytime
static char has_newdata(struct httpd_state *s)
{
    if (s->upload_state == 1) {
        /* All data in uip_appdata buffer already consumed. */
        s->upload_state = 0;
        return 0;
    } else if (uip_newdata()) {
        /* There is new data that has not been consumed. */
        return 1;
    } else {
        /* There is no new data. */
        return 0;
    }
}

/*
 * handle the uploaded data, as there may be part of that buffer still in the last packet buffer
 * write that first from the buf/len parameters
 */
static PT_THREAD(handle_uploaded_data(struct httpd_state *s, uint8_t *buf, int len))
{
    PT_BEGIN(&s->inputpt);

    DEBUG_PRINTF("Uploading file: %s, %d\n", s->upload_name, s->content_length);

    // The body is the raw data to be stored to the file
    if (!open_file(s->upload_name)) {
        DEBUG_PRINTF("failed to open file\n");
        s->uploadok = 0;
        PT_EXIT(&s->inputpt);
    }

    DEBUG_PRINTF("opened file: %s\n", s->upload_name);

    if (len > 0) {
        // write the first part of the buffer
        if (!save_file(buf, len)) {
            DEBUG_PRINTF("initial write failed\n");
            s->uploadok = 0;
            PT_EXIT(&s->inputpt);
        }
        s->content_length -= len;
    }

    s->upload_state = 1; // first time through we need to yield to get new data

    // save the entire input buffer
    while (s->content_length > 0) {
        PT_WAIT_UNTIL(&s->inputpt, has_newdata(s));
        s->upload_state = 1;

        u8_t *readptr = (u8_t *)uip_appdata;
        int readlen = uip_datalen();
        //DEBUG_PRINTF("read %d bytes of data\n", readlen);

        if (readlen > 0) {
            if (!save_file(readptr, readlen)) {
                DEBUG_PRINTF("write failed\n");
                s->uploadok = 0;
                PT_EXIT(&s->inputpt);
            }
            s->content_length -= readlen;
        }
    }

    close_file();
    s->uploadok = 1;
    DEBUG_PRINTF("finished upload\n");

    PT_END(&s->inputpt);
}
/*---------------------------------------------------------------------------*/
static
PT_THREAD(handle_input(struct httpd_state *s))
{
    PSOCK_BEGIN(&s->sin);

    PSOCK_READTO(&s->sin, ISO_space);

    if (strncmp(s->inputbuf, http_get, 3) == 0) {
        s->method = GET;
    } else if (strncmp(s->inputbuf, http_post, 4) == 0) {
        s->method = POST;
    } else if (strncmp(s->inputbuf, http_options, 7) == 0) {
        s->method = OPTIONS;
    } else {
        DEBUG_PRINTF("Unexpected method: %s\n", s->inputbuf);
        PSOCK_CLOSE_EXIT(&s->sin);
    }

    DEBUG_PRINTF("Method: %s\n", s->method == POST ? "POST" : (s->method == GET ? "GET" : "OPTIONS"));

    PSOCK_READTO(&s->sin, ISO_space);

    if (s->inputbuf[0] != ISO_slash) {
        PSOCK_CLOSE_EXIT(&s->sin);
    }

    if (s->inputbuf[1] == ISO_space) {
        strncpy(s->filename, http_index_html, sizeof(s->filename));
    } else {
        s->inputbuf[PSOCK_DATALEN(&s->sin) - 1] = 0;
        strncpy(s->filename, &s->inputbuf[0], sizeof(s->filename));
    }

    DEBUG_PRINTF("filename: %s\n", s->filename);

    /*  httpd_log_file(uip_conn->ripaddr, s->filename);*/

    s->state = STATE_HEADERS;
    s->content_length = 0;
    s->cache_page = 0;
    while (1) {
        if (s->state == STATE_HEADERS) {
            // read the headers of the request
            PSOCK_READTO(&s->sin, ISO_nl);
            s->inputbuf[PSOCK_DATALEN(&s->sin) - 1] = 0;
            if (s->inputbuf[0] == '\r') {
                DEBUG_PRINTF("end of headers\n");
                if (s->method == OPTIONS) {
                   s->state = STATE_OUTPUT;
                   break;
                } else if (s->method == GET) {
                    s->state = STATE_OUTPUT;
                    break;
                } else if (s->method == POST) {
                    if (strcmp(s->filename, "/upload") == 0) {
                        s->state = STATE_UPLOAD;
                    } else {
                        s->state = STATE_BODY;
                    }
                }
            } else {
                DEBUG_PRINTF("reading header: %s\n", s->inputbuf);
                // handle headers here
                if (strncmp(s->inputbuf, http_content_length, sizeof(http_content_length) - 1) == 0) {
                    s->inputbuf[PSOCK_DATALEN(&s->sin) - 2] = 0;
                    s->content_length = atoi(&s->inputbuf[sizeof(http_content_length) - 1]);
                    DEBUG_PRINTF("Content length= %s, %d\n", &s->inputbuf[sizeof(http_content_length) - 1], s->content_length);

                } else if (strncmp(s->inputbuf, "X-Filename: ", 11) == 0) {
                    s->inputbuf[PSOCK_DATALEN(&s->sin) - 2] = 0;
                    strncpy(s->upload_name, &s->inputbuf[12], sizeof(s->upload_name) - 1);
                    DEBUG_PRINTF("Upload name= %s\n", s->upload_name);

                } else if (strncmp(s->inputbuf, http_cache_control, sizeof(http_cache_control) - 1) == 0) {
                    s->inputbuf[PSOCK_DATALEN(&s->sin) - 2] = 0;
                    s->cache_page = strncmp(http_no_cache, &s->inputbuf[sizeof(http_cache_control) - 1], sizeof(http_no_cache) - 1) != 0;
                    DEBUG_PRINTF("cache page= %d\n", s->cache_page);
                }
            }

        } else if (s->state == STATE_BODY) {
            if (s->method == POST && strcmp(s->filename, "/command") == 0) {
                // create a callback stream and fifo for the results as it is a command
                create_callback_stream(s);

            } else if (s->method == POST && strcmp(s->filename, "/command_silent") == 0) {
                // stick the command  on the command queue specifying null output stream
                s->pstream = NULL;

            } else { // unknown POST
                DEBUG_PRINTF("Unknown Post URL: %s\n", s->filename);
                s->state = STATE_OUTPUT;
                break;
            }
            s->command_count= 0;
            // read the Body of the request, each line is a command
            if (s->content_length > 0) {
                DEBUG_PRINTF("start reading body %d...\n", s->content_length);
                while (s->content_length > 2) {
                    PSOCK_READTO(&s->sin, ISO_nl);
                    s->inputbuf[PSOCK_DATALEN(&s->sin) - 1] = 0;
                    s->content_length -= PSOCK_DATALEN(&s->sin);
                    // stick the command  on the command queue, with this connections stream output
                    DEBUG_PRINTF("Adding command: %s, left: %d\n", s->inputbuf, s->content_length);
                    network_add_command(s->inputbuf, s->pstream);
                    s->command_count++; // count number of command lines we submit
                }
                DEBUG_PRINTF("Read body done\n");
                s->state = STATE_OUTPUT;

            } else {
                s->state = STATE_OUTPUT;
            }
            break;

        } else if (s->state == STATE_UPLOAD) {
            PSOCK_WAIT_THREAD(&s->sin, handle_uploaded_data(s, PSOCK_GET_START_OF_REST_OF_BUFFER(&s->sin), PSOCK_GET_LENGTH_OF_REST_OF_BUFFER(&s->sin)));
            PSOCK_MARK_BUFFER_READ(&s->sin);
            s->state = STATE_OUTPUT;
            break;

        } else {
            DEBUG_PRINTF("WTF State: %d", s->state);
            break;
        }
    }

    PSOCK_END(&s->sin);
}
/*---------------------------------------------------------------------------*/
static void
handle_connection(struct httpd_state *s)
{
    if (s->state != STATE_OUTPUT) {
        handle_input(s);
    }
    if (s->state == STATE_OUTPUT) {
        handle_output(s);
    }
}
/*---------------------------------------------------------------------------*/
void
httpd_appcall(void)
{
    struct httpd_state *s = (struct httpd_state *)(uip_conn->appstate);

    if (uip_connected()) {
        s = malloc(sizeof(struct httpd_state));
        if (s == NULL) {
            DEBUG_PRINTF("Connection: Out of memory\n");
            uip_abort();
            return;
        }
        uip_conn->appstate = s;
        DEBUG_PRINTF("Connection: %d.%d.%d.%d:%d\n",
                     uip_ipaddr1(uip_conn->ripaddr), uip_ipaddr2(uip_conn->ripaddr),
                     uip_ipaddr3(uip_conn->ripaddr), uip_ipaddr4(uip_conn->ripaddr),
                     HTONS(uip_conn->rport));

        PSOCK_INIT(&s->sin, s->inputbuf, sizeof(s->inputbuf) - 1);
        PSOCK_INIT(&s->sout, s->inputbuf, sizeof(s->inputbuf) - 1);
        PT_INIT(&s->outputpt);
        PT_INIT(&s->inputpt);
        s->state = STATE_WAITING;
        /*    timer_set(&s->timer, CLOCK_SECOND * 100);*/
        s->timer = 0;
        s->fd = NULL;
        s->strbuf = NULL;
        s->fifo = NULL;
        s->pstream = NULL;
    }

    if (s == NULL) {
        DEBUG_PRINTF("ERROR no state context: %d\n", uip_flags);
        uip_abort();
        return;
    }

    // check for timeout on connection here so we can cleanup if we abort
    if (uip_poll()) {
        ++s->timer;
        if (s->timer >= 20 * 2) { // we have a 0.5 second poll and we want 20 second timeout
            DEBUG_PRINTF("Timer expired, aborting\n");
            uip_abort();
        }
    } else {
        s->timer = 0;
    }

    if (uip_closed() || uip_aborted() || uip_timedout()) {
        DEBUG_PRINTF("Closing connection: %d\n", HTONS(uip_conn->rport));
        if (s->fd != NULL) fclose(fd); // clean up
        if (s->strbuf != NULL) free(s->strbuf);
        if (s->pstream != NULL) {
            // free these if they were allocated
            delete_fifo(s->fifo);
            delete_callback_stream(s->pstream); // this will mark it as closed and will get deleted when no longer needed
        }
        free(s) ;
        uip_conn->appstate = NULL;

    } else {
        handle_connection(s);
    }
}

/*---------------------------------------------------------------------------*/
/**
 * \brief      Initialize the web server
 *
 *             This function initializes the web server and should be
 *             called at system boot-up.
 */
void httpd_init(void)
{
    uip_listen(HTONS(80));
}
/*---------------------------------------------------------------------------*/
/** @} */
