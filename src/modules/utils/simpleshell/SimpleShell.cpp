/*
    This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
    Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
    Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
    You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/


#include "SimpleShell.h"
#include "libs/Kernel.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "libs/SerialMessage.h"
#include "libs/StreamOutput.h"
#include "modules/robot/Conveyor.h"
#include "DirHandle.h"
#include "mri.h"
#include "version.h"
#include "PublicDataRequest.h"
#include "AppendFileStream.h"
#include "FileStream.h"
#include "checksumm.h"
#include "PublicData.h"
#include "Gcode.h"
//#include "StepTicker.h"

#include "modules/tools/temperaturecontrol/TemperatureControlPublicAccess.h"
#include "modules/robot/RobotPublicAccess.h"
#include "NetworkPublicAccess.h"
#include "platform_memory.h"
#include "SwitchPublicAccess.h"
#include "SDFAT.h"
#include "Thermistor.h"
#include "md5.h"

#include "system_LPC17xx.h"
#include "LPC17xx.h"

#include "mbed.h" // for wait_ms()

extern unsigned int g_maximumHeapAddress;

#include <malloc.h>
#include <mri.h>
#include <stdio.h>
#include <stdint.h>

extern "C" uint32_t  __end__;
extern "C" uint32_t  __malloc_free_list;
extern "C" uint32_t  _sbrk(int size);

// command lookup table
const SimpleShell::ptentry_t SimpleShell::commands_table[] = {
    {"ls",       SimpleShell::ls_command},
    {"cd",       SimpleShell::cd_command},
    {"pwd",      SimpleShell::pwd_command},
    {"cat",      SimpleShell::cat_command},
    {"rm",       SimpleShell::rm_command},
    {"mv",       SimpleShell::mv_command},
    {"upload",   SimpleShell::upload_command},
    {"reset",    SimpleShell::reset_command},
    {"dfu",      SimpleShell::dfu_command},
    {"break",    SimpleShell::break_command},
    {"help",     SimpleShell::help_command},
    {"?",        SimpleShell::help_command},
    {"version",  SimpleShell::version_command},
    {"mem",      SimpleShell::mem_command},
    {"get",      SimpleShell::get_command},
    {"set_temp", SimpleShell::set_temp_command},
    {"switch",   SimpleShell::switch_command},
    {"net",      SimpleShell::net_command},
    {"load",     SimpleShell::load_command},
    {"save",     SimpleShell::save_command},
    {"remount",  SimpleShell::remount_command},
    {"calc_thermistor", SimpleShell::calc_thermistor_command},
    {"thermistors", SimpleShell::print_thermistors_command},
    {"md5sum",   SimpleShell::md5sum_command},

    // unknown command
    {NULL, NULL}
};

int SimpleShell::reset_delay_secs = 0;

// Adam Greens heap walk from http://mbed.org/forum/mbed/topic/2701/?page=4#comment-22556
static uint32_t heapWalk(StreamOutput *stream, bool verbose)
{
    uint32_t chunkNumber = 1;
    // The __end__ linker symbol points to the beginning of the heap.
    uint32_t chunkCurr = (uint32_t)&__end__;
    // __malloc_free_list is the head pointer to newlib-nano's link list of free chunks.
    uint32_t freeCurr = __malloc_free_list;
    // Calling _sbrk() with 0 reserves no more memory but it returns the current top of heap.
    uint32_t heapEnd = _sbrk(0);
    // accumulate totals
    uint32_t freeSize = 0;
    uint32_t usedSize = 0;

    stream->printf("Used Heap Size: %lu\n", heapEnd - chunkCurr);

    // Walk through the chunks until we hit the end of the heap.
    while (chunkCurr < heapEnd) {
        // Assume the chunk is in use.  Will update later.
        int      isChunkFree = 0;
        // The first 32-bit word in a chunk is the size of the allocation.  newlib-nano over allocates by 8 bytes.
        // 4 bytes for this 32-bit chunk size and another 4 bytes to allow for 8 byte-alignment of returned pointer.
        uint32_t chunkSize = *(uint32_t *)chunkCurr;
        // The start of the next chunk is right after the end of this one.
        uint32_t chunkNext = chunkCurr + chunkSize;

        // The free list is sorted by address.
        // Check to see if we have found the next free chunk in the heap.
        if (chunkCurr == freeCurr) {
            // Chunk is free so flag it as such.
            isChunkFree = 1;
            // The second 32-bit word in a free chunk is a pointer to the next free chunk (again sorted by address).
            freeCurr = *(uint32_t *)(freeCurr + 4);
        }

        // Skip past the 32-bit size field in the chunk header.
        chunkCurr += 4;
        // 8-byte align the data pointer.
        chunkCurr = (chunkCurr + 7) & ~7;
        // newlib-nano over allocates by 8 bytes, 4 bytes for the 32-bit chunk size and another 4 bytes to allow for 8
        // byte-alignment of the returned pointer.
        chunkSize -= 8;
        if (verbose)
            stream->printf("  Chunk: %lu  Address: 0x%08lX  Size: %lu  %s\n", chunkNumber, chunkCurr, chunkSize, isChunkFree ? "CHUNK FREE" : "");

        if (isChunkFree) freeSize += chunkSize;
        else usedSize += chunkSize;

        chunkCurr = chunkNext;
        chunkNumber++;
    }
    stream->printf("Allocated: %lu, Free: %lu\r\n", usedSize, freeSize);
    return freeSize;
}


void SimpleShell::on_module_loaded()
{
    this->register_for_event(ON_CONSOLE_LINE_RECEIVED);
    this->register_for_event(ON_GCODE_RECEIVED);
    this->register_for_event(ON_SECOND_TICK);

    reset_delay_secs = 0;
}

void SimpleShell::on_second_tick(void *)
{
    // we are timing out for the reset
    if (reset_delay_secs > 0) {
        if (--reset_delay_secs == 0) {
            system_reset(false);
        }
    }
}

void SimpleShell::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);
    string args = get_arguments(gcode->get_command());

    if (gcode->has_m) {
        if (gcode->m == 20) { // list sd card
            gcode->stream->printf("Begin file list\r\n");
            ls_command("/sd", gcode->stream);
            gcode->stream->printf("End file list\r\n");

        } else if (gcode->m == 30) { // remove file
            rm_command("/sd/" + args, gcode->stream);

        } else if(gcode->m == 501) { // load config override
            if(args.empty()) {
                load_command("/sd/config-override", gcode->stream);
            } else {
                load_command("/sd/config-override." + args, gcode->stream);
            }

        } else if(gcode->m == 504) { // save to specific config override file
            if(args.empty()) {
                save_command("/sd/config-override", gcode->stream);
            } else {
                save_command("/sd/config-override." + args, gcode->stream);
            }
        }
    }
}

bool SimpleShell::parse_command(const char *cmd, string args, StreamOutput *stream)
{
    for (const ptentry_t *p = commands_table; p->command != NULL; ++p) {
        if (strncasecmp(cmd, p->command, strlen(p->command)) == 0) {
            p->func(args, stream);
            return true;
        }
    }

    return false;
}

// When a new line is received, check if it is a command, and if it is, act upon it
void SimpleShell::on_console_line_received( void *argument )
{
    SerialMessage new_message = *static_cast<SerialMessage *>(argument);

    // ignore comments and blank lines and if this is a G code then also ignore it
    char first_char = new_message.message[0];
    if(strchr(";( \n\rGMTN", first_char) != NULL) return;

    string possible_command = new_message.message;

    //new_message.stream->printf("Received %s\r\n", possible_command.c_str());
    string cmd = shift_parameter(possible_command);

    // find command and execute it
    parse_command(cmd.c_str(), possible_command, new_message.stream);
}

// Act upon an ls command
// Convert the first parameter into an absolute path, then list the files in that path
void SimpleShell::ls_command( string parameters, StreamOutput *stream )
{
    string path, opts;
    while(!parameters.empty()) {
        string s = shift_parameter( parameters );
        if(s.front() == '-') {
            opts.append(s);
        } else {
            path = s;
            if(!parameters.empty()) {
                path.append(" ");
                path.append(parameters);
            }
            break;
        }
    }

    path = absolute_from_relative(path);

    DIR *d;
    struct dirent *p;
    d = opendir(path.c_str());
    if (d != NULL) {
        while ((p = readdir(d)) != NULL) {
            stream->printf("%s", lc(string(p->d_name)).c_str());
            if(p->d_isdir) {
                stream->printf("/");
            } else if(opts.find("-s", 0, 2) != string::npos) {
                stream->printf(" %d", p->d_fsize);
            }
            stream->printf("\r\n");
        }
        closedir(d);
    } else {
        stream->printf("Could not open directory %s\r\n", path.c_str());
    }
}

extern SDFAT mounter;

void SimpleShell::remount_command( string parameters, StreamOutput *stream )
{
    mounter.remount();
    stream->printf("remounted\r\n");
}

// Delete a file
void SimpleShell::rm_command( string parameters, StreamOutput *stream )
{
    const char *fn = absolute_from_relative(shift_parameter( parameters )).c_str();
    int s = remove(fn);
    if (s != 0) stream->printf("Could not delete %s \r\n", fn);
}

// Rename a file
void SimpleShell::mv_command( string parameters, StreamOutput *stream )
{
    string from = absolute_from_relative(shift_parameter( parameters ));
    string to = absolute_from_relative(shift_parameter(parameters));
    int s = rename(from.c_str(), to.c_str());
    if (s != 0) stream->printf("Could not rename %s to %s\r\n", from.c_str(), to.c_str());
    else stream->printf("renamed %s to %s\r\n", from.c_str(), to.c_str());
}

// Change current absolute path to provided path
void SimpleShell::cd_command( string parameters, StreamOutput *stream )
{
    string folder = absolute_from_relative( parameters );

    DIR *d;
    d = opendir(folder.c_str());
    if (d == NULL) {
        stream->printf("Could not open directory %s \r\n", folder.c_str() );
    } else {
        THEKERNEL->current_path = folder;
        closedir(d);
    }
}

// Responds with the present working directory
void SimpleShell::pwd_command( string parameters, StreamOutput *stream )
{
    stream->printf("%s\r\n", THEKERNEL->current_path.c_str());
}

// Output the contents of a file, first parameter is the filename, second is the limit ( in number of lines to output )
void SimpleShell::cat_command( string parameters, StreamOutput *stream )
{
    // Get parameters ( filename and line limit )
    string filename          = absolute_from_relative(shift_parameter( parameters ));
    string limit_paramater   = shift_parameter( parameters );
    int limit = -1;
    if ( limit_paramater != "" ) {
        char *e = NULL;
        limit = strtol(limit_paramater.c_str(), &e, 10);
        if (e <= limit_paramater.c_str())
            limit = -1;
    }

    // Open file
    FILE *lp = fopen(filename.c_str(), "r");
    if (lp == NULL) {
        stream->printf("File not found: %s\r\n", filename.c_str());
        return;
    }
    string buffer;
    int c;
    int newlines = 0;
    int linecnt = 0;
    // Print each line of the file
    while ((c = fgetc (lp)) != EOF) {
        buffer.append((char *)&c, 1);
        if ( char(c) == '\n' || ++linecnt > 80) {
            newlines++;
            stream->puts(buffer.c_str());
            buffer.clear();
            if(linecnt > 80) linecnt = 0;
        }
        if ( newlines == limit ) {
            break;
        }
    };
    fclose(lp);
}

void SimpleShell::upload_command( string parameters, StreamOutput *stream )
{
    // this needs to be a hack. it needs to read direct from serial and not allow on_main_loop run until done
    // NOTE this will block all operation until the upload is complete, so do not do while printing
    if(!THEKERNEL->conveyor->is_queue_empty()) {
        stream->printf("upload not allowed while printing or busy\n");
        return;
    }

    // open file to upload to
    string upload_filename = absolute_from_relative( parameters );
    FILE *fd = fopen(upload_filename.c_str(), "w");
    if(fd != NULL) {
        stream->printf("uploading to file: %s, send control-D or control-Z to finish\r\n", upload_filename.c_str());
    } else {
        stream->printf("failed to open file: %s.\r\n", upload_filename.c_str());
        return;
    }

    int cnt = 0;
    bool uploading = true;
    while(uploading) {
        if(!stream->ready()) {
            // we need to kick things or they die
            THEKERNEL->call_event(ON_IDLE);
            continue;
        }

        char c = stream->_getc();
        if( c == 4 || c == 26) { // ctrl-D or ctrl-Z
            uploading = false;
            // close file
            fclose(fd);
            stream->printf("uploaded %d bytes\n", cnt);
            return;

        } else {
            // write character to file
            cnt++;
            if(fputc(c, fd) != c) {
                // error writing to file
                stream->printf("error writing to file. ignoring all characters until EOF\r\n");
                fclose(fd);
                fd = NULL;
                uploading= false;

            } else {
                if ((cnt%400) == 0) {
                    // HACK ALERT to get around fwrite corruption close and re open for append
                    fclose(fd);
                    fd = fopen(upload_filename.c_str(), "a");
                }
            }
        }
    }
    // we got an error so ignore everything until EOF
    char c;
    do {
        if(stream->ready()) {
            c= stream->_getc();
        }else{
            THEKERNEL->call_event(ON_IDLE);
            c= 0;
        }
    } while(c != 4 && c != 26);
}

// loads the specified config-override file
void SimpleShell::load_command( string parameters, StreamOutput *stream )
{
    // Get parameters ( filename )
    string filename = absolute_from_relative(parameters);
    if(filename == "/") {
        filename = THEKERNEL->config_override_filename();
    }

    FILE *fp = fopen(filename.c_str(), "r");
    if(fp != NULL) {
        char buf[132];
        stream->printf("Loading config override file: %s...\n", filename.c_str());
        while(fgets(buf, sizeof buf, fp) != NULL) {
            stream->printf("  %s", buf);
            if(buf[0] == ';') continue; // skip the comments
            struct SerialMessage message = {&(StreamOutput::NullStream), buf};
            THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message);
        }
        stream->printf("config override file executed\n");
        fclose(fp);

    } else {
        stream->printf("File not found: %s\n", filename.c_str());
    }
}

// saves the specified config-override file
void SimpleShell::save_command( string parameters, StreamOutput *stream )
{
    // Get parameters ( filename )
    string filename = absolute_from_relative(parameters);
    if(filename == "/") {
        filename = THEKERNEL->config_override_filename();
    }

    THEKERNEL->conveyor->wait_for_empty_queue(); //just to be safe as it can take a while to run

    //remove(filename.c_str()); // seems to cause a hang every now and then
    {
        FileStream fs(filename.c_str());
        fs.printf("; DO NOT EDIT THIS FILE\n");
        // this also will truncate the existing file instead of deleting it
    }

    // stream that appends to file
    AppendFileStream *gs = new AppendFileStream(filename.c_str());
    // if(!gs->is_open()) {
    //     stream->printf("Unable to open File %s for write\n", filename.c_str());
    //     return;
    // }

    __disable_irq();
    // issue a M500 which will store values in the file stream
    Gcode *gcode = new Gcode("M500", gs);
    THEKERNEL->call_event(ON_GCODE_RECEIVED, gcode );
    delete gs;
    delete gcode;
    __enable_irq();

    stream->printf("Settings Stored to %s\r\n", filename.c_str());
}

// show free memory
void SimpleShell::mem_command( string parameters, StreamOutput *stream)
{
    bool verbose = shift_parameter( parameters ).find_first_of("Vv") != string::npos ;
    unsigned long heap = (unsigned long)_sbrk(0);
    unsigned long m = g_maximumHeapAddress - heap;
    stream->printf("Unused Heap: %lu bytes\r\n", m);

    uint32_t f = heapWalk(stream, verbose);
    stream->printf("Total Free RAM: %lu bytes\r\n", m + f);

    stream->printf("Free AHB0: %lu, AHB1: %lu\r\n", AHB0.free(), AHB1.free());
    if (verbose) {
        AHB0.debug(stream);
        AHB1.debug(stream);
    }
}

static uint32_t getDeviceType()
{
#define IAP_LOCATION 0x1FFF1FF1
    uint32_t command[1];
    uint32_t result[5];
    typedef void (*IAP)(uint32_t *, uint32_t *);
    IAP iap = (IAP) IAP_LOCATION;

    __disable_irq();

    command[0] = 54;
    iap(command, result);

    __enable_irq();

    return result[1];
}

// get network config
void SimpleShell::net_command( string parameters, StreamOutput *stream)
{
    void *returned_data;
    bool ok = PublicData::get_value( network_checksum, get_ipconfig_checksum, &returned_data );
    if(ok) {
        char *str = (char *)returned_data;
        stream->printf("%s\r\n", str);
        free(str);

    } else {
        stream->printf("No network detected\n");
    }
}

// print out build version
void SimpleShell::version_command( string parameters, StreamOutput *stream)
{
    Version vers;
    uint32_t dev = getDeviceType();
    const char *mcu = (dev & 0x00100000) ? "LPC1769" : "LPC1768";
    stream->printf("Build version: %s, Build date: %s, MCU: %s, System Clock: %ldMHz\r\n", vers.get_build(), vers.get_build_date(), mcu, SystemCoreClock / 1000000);
}

// Reset the system
void SimpleShell::reset_command( string parameters, StreamOutput *stream)
{
    stream->printf("Smoothie out. Peace. Rebooting in 5 seconds...\r\n");
    reset_delay_secs = 5; // reboot in 5 seconds
}

// go into dfu boot mode
void SimpleShell::dfu_command( string parameters, StreamOutput *stream)
{
    stream->printf("Entering boot mode...\r\n");
    system_reset(true);
}

// Break out into the MRI debugging system
void SimpleShell::break_command( string parameters, StreamOutput *stream)
{
    stream->printf("Entering MRI debug mode...\r\n");
    __debugbreak();
}

// used to test out the get public data events
void SimpleShell::get_command( string parameters, StreamOutput *stream)
{
    string what = shift_parameter( parameters );
    void *returned_data;

    if (what == "temp") {
        string type = shift_parameter( parameters );
        bool ok = PublicData::get_value( temperature_control_checksum, get_checksum(type), current_temperature_checksum, &returned_data );

        if (ok) {
            struct pad_temperature temp =  *static_cast<struct pad_temperature *>(returned_data);
            stream->printf("%s temp: %f/%f @%d\r\n", type.c_str(), temp.current_temperature, temp.target_temperature, temp.pwm);
        } else {
            stream->printf("%s is not a known temperature device\r\n", type.c_str());
        }

    } else if (what == "pos") {
        bool ok = PublicData::get_value( robot_checksum, current_position_checksum, &returned_data );

        if (ok) {
            float *pos = static_cast<float *>(returned_data);
            stream->printf("Position X: %f, Y: %f, Z: %f\r\n", pos[0], pos[1], pos[2]);

        } else {
            stream->printf("get pos command failed\r\n");
        }
    }
}

// used to test out the get public data events
void SimpleShell::set_temp_command( string parameters, StreamOutput *stream)
{
    string type = shift_parameter( parameters );
    string temp = shift_parameter( parameters );
    float t = temp.empty() ? 0.0 : strtof(temp.c_str(), NULL);
    bool ok = PublicData::set_value( temperature_control_checksum, get_checksum(type), &t );

    if (ok) {
        stream->printf("%s temp set to: %3.1f\r\n", type.c_str(), t);
    } else {
        stream->printf("%s is not a known temperature device\r\n", type.c_str());
    }
}

void SimpleShell::print_thermistors_command( string parameters, StreamOutput *stream)
{
    Thermistor::print_predefined_thermistors(stream);
}

void SimpleShell::calc_thermistor_command( string parameters, StreamOutput *stream)
{
    string s = shift_parameter( parameters );
    int saveto= -1;
    // see if we have -sn as first argument
    if(s.find("-s", 0, 2) != string::npos) {
        // save the results to thermistor n
        saveto= strtol(s.substr(2).c_str(), nullptr, 10);
    }else{
        parameters= s;
    }

    std::vector<float> trl= parse_number_list(parameters.c_str());
    if(trl.size() == 6) {
        // calculate the coefficients
        float c1, c2, c3;
        std::tie(c1, c2, c3) = Thermistor::calculate_steinhart_hart_coefficients(trl[0], trl[1], trl[2], trl[3], trl[4], trl[5]);
        stream->printf("Steinhart Hart coefficients:  I%1.18f J%1.18f K%1.18f\n", c1, c2, c3);
        if(saveto == -1) {
            stream->printf("  Paste the above in the M305 S0 command, then save with M500\n");
        }else{
            char buf[80];
            int n = snprintf(buf, sizeof(buf), "M305 S%d I%1.18f J%1.18f K%1.18f", saveto, c1, c2, c3);
            string g(buf, n);
            Gcode gcode(g, &(StreamOutput::NullStream));
            THEKERNEL->call_event(ON_GCODE_RECEIVED, &gcode );
            stream->printf("  Setting Thermistor %d to those settings, save with M500\n", saveto);
        }

    }else{
        // give help
        stream->printf("Usage: calc_thermistor T1,R1,T2,R2,T3,R3\n");
    }
}

// used to test out the get public data events for switch
void SimpleShell::switch_command( string parameters, StreamOutput *stream)
{
    string type = shift_parameter( parameters );
    string value = shift_parameter( parameters );
    bool ok = false;
    if(value == "on" || value == "off") {
        bool b = value == "on";
        ok = PublicData::set_value( switch_checksum, get_checksum(type), state_checksum, &b );
    } else {
        float v = strtof(value.c_str(), NULL);
        ok = PublicData::set_value( switch_checksum, get_checksum(type), value_checksum, &v );
    }
    if (ok) {
        stream->printf("switch %s set to: %s\r\n", type.c_str(), value.c_str());
    } else {
        stream->printf("%s is not a known switch device\r\n", type.c_str());
    }
}

void SimpleShell::md5sum_command( string parameters, StreamOutput *stream )
{
    string filename = absolute_from_relative(parameters);

    // Open file
    FILE *lp = fopen(filename.c_str(), "r");
    if (lp == NULL) {
        stream->printf("File not found: %s\r\n", filename.c_str());
        return;
    }
    MD5 md5;
    uint8_t buf[64];
    do {
        size_t n= fread(buf, 1, sizeof buf, lp);
        if(n > 0) md5.update(buf, n);
    } while(!feof(lp));

    stream->printf("%s %s\n", md5.finalize().hexdigest().c_str(), filename.c_str());
    fclose(lp);
}



void SimpleShell::help_command( string parameters, StreamOutput *stream )
{
    stream->printf("Commands:\r\n");
    stream->printf("version\r\n");
    stream->printf("mem [-v]\r\n");
    stream->printf("ls [-s] [folder]\r\n");
    stream->printf("cd folder\r\n");
    stream->printf("pwd\r\n");
    stream->printf("cat file [limit]\r\n");
    stream->printf("rm file\r\n");
    stream->printf("mv file newfile\r\n");
    stream->printf("remount\r\n");
    stream->printf("play file [-v]\r\n");
    stream->printf("progress - shows progress of current play\r\n");
    stream->printf("abort - abort currently playing file\r\n");
    stream->printf("reset - reset smoothie\r\n");
    stream->printf("dfu - enter dfu boot loader\r\n");
    stream->printf("break - break into debugger\r\n");
    stream->printf("config-get [<configuration_source>] <configuration_setting>\r\n");
    stream->printf("config-set [<configuration_source>] <configuration_setting> <value>\r\n");
    stream->printf("get temp [bed|hotend]\r\n");
    stream->printf("set_temp bed|hotend 185\r\n");
    stream->printf("get pos\r\n");
    stream->printf("net\r\n");
    stream->printf("load [file] - loads a configuration override file from soecified name or config-override\r\n");
    stream->printf("save [file] - saves a configuration override file as specified filename or as config-override\r\n");
    stream->printf("upload filename - saves a stream of text to the named file\r\n");
    stream->printf("calc_thermistor [-s0] T1,R1,T2,R2,T3,R3 - calculate the Steinhart Hart coefficients for a thermistor\r\n");
    stream->printf("thermistors - print out the predefined thermistors\r\n");
    stream->printf("md5sum file - prints md5 sum of the given file\r\n");
}

