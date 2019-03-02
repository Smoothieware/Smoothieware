/**
 * \file
 * Interface for the Contiki shell.
 * \author Adam Dunkels <adam@dunkels.com>
 *
 * Some of the functions declared in this file must be implemented as
 * a shell back-end in the architecture specific files of a Contiki
 * port.
 */


/*
 * Copyright (c) 2003, Adam Dunkels.
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
 * This file is part of the Contiki desktop OS.
 *
 * $Id: shell.h,v 1.1 2006/06/07 09:43:54 adam Exp $
 *
 */
#ifndef __SHELL_H__
#define __SHELL_H__

class Telnetd;
class StreamOutput;

class Shell
{
public:
    Shell(Telnetd *telnet);
    ~Shell();

    /**
     * Start the shell back-end.
     *
     * Called by the front-end when a new shell is started.
     */
    void start(void);

    /**
     * Process a shell command.
     *
     * This function will be called by the shell GUI / telnet server whan
     * a command has been entered that should be processed by the shell
     * back-end.
     *
     * \param command The command to be processed.
     */
    void input(char *command);

    int output(const char *str);
    void close();

    /**
     * Print a prompt to the shell window.
     *
     * This function can be used by the shell back-end to print out a
     * prompt to the shell window.
     *
     * \param prompt The prompt to be printed.
     *
     */
    void prompt(const char *prompt);

    int queue_size();
    int can_output();
    static int command_result(const char *str, void *ti);
    StreamOutput *getStream() { return pstream; }
    void setConsole();

private:
    bool parse(register char *str, const struct ptentry *t);
    Telnetd *telnet; // telnet instance we are connected to
    StreamOutput *pstream;
    bool isConsole;
};

#endif /* __SHELL_H__ */
