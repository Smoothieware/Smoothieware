/*
 * Copyright (c) 2001-2005, Adam Dunkels.
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
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * This file is part of the uIP TCP/IP stack.
 *
 * $Id: httpd.h,v 1.2 2006/06/11 21:46:38 adam Exp $
 *
 */

#ifndef __HTTPD_H__
#define __HTTPD_H__

#include "psock.h"
#include "httpd-fs.h"
#include "stdio.h"

struct httpd_state {
  unsigned char timer;
  struct psock sin, sout;
  struct pt outputpt, inputpt;
  char inputbuf[100];
  char filename[60];
  char upload_name[60];
  char method;
  char state;
  struct httpd_fs_file file;
  FILE *fd;
  uint16_t len;
  char *strbuf;
  int content_length;
  uint16_t count;
  uint8_t uploadok;
  uint8_t upload_state;
  uint8_t cache_page;
  void *pstream;
  void *fifo;
  uint16_t command_count;
};

#ifdef __cplusplus
extern "C" {
#endif

void httpd_init(void);
void httpd_appcall(void);

void httpd_log(char *msg);
void httpd_log_file(u16_t *requester, char *file);

#ifdef __cplusplus
}
#endif

#endif /* __HTTPD_H__ */
