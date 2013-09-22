/*
    This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
    Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
    Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
    You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/


#include "libs/Kernel.h"
#include "SimpleShell.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "libs/SerialMessage.h"
#include "libs/StreamOutput.h"
#include "modules/robot/Conveyor.h"
#include "DirHandle.h"
#include "mri.h"
#include "version.h"
#include "PublicDataRequest.h"

#include "modules/tools/temperaturecontrol/TemperatureControlPublicAccess.h"
#include "modules/robot/RobotPublicAccess.h"

//extern "C" caddr_t _sbrk(int incr);
extern unsigned int g_maximumHeapAddress;

#include <malloc.h>
#include <mri.h>
#include <stdio.h>
#include <stdint.h>

extern "C" uint32_t  __end__;
extern "C" uint32_t  __malloc_free_list;
extern "C" uint32_t  _sbrk(int size);


// Adam Greens heap walk from http://mbed.org/forum/mbed/topic/2701/?page=4#comment-22556
static void heapWalk(StreamOutput* stream, bool verbose)
{
    uint32_t chunkNumber = 1;
    // The __end__ linker symbol points to the beginning of the heap.
    uint32_t chunkCurr = (uint32_t)&__end__;
    // __malloc_free_list is the head pointer to newlib-nano's link list of free chunks.
    uint32_t freeCurr = __malloc_free_list;
    // Calling _sbrk() with 0 reserves no more memory but it returns the current top of heap.
    uint32_t heapEnd = _sbrk(0);
    // accumulate totals
    uint32_t freeSize= 0;
    uint32_t usedSize= 0;

    stream->printf("Used Heap Size: %lu\n", heapEnd - chunkCurr);

    // Walk through the chunks until we hit the end of the heap.
    while (chunkCurr < heapEnd)
    {
        // Assume the chunk is in use.  Will update later.
        int      isChunkFree = 0;
        // The first 32-bit word in a chunk is the size of the allocation.  newlib-nano over allocates by 8 bytes.
        // 4 bytes for this 32-bit chunk size and another 4 bytes to allow for 8 byte-alignment of returned pointer.
        uint32_t chunkSize = *(uint32_t*)chunkCurr;
        // The start of the next chunk is right after the end of this one.
        uint32_t chunkNext = chunkCurr + chunkSize;

        // The free list is sorted by address.
        // Check to see if we have found the next free chunk in the heap.
        if (chunkCurr == freeCurr)
        {
            // Chunk is free so flag it as such.
            isChunkFree = 1;
            // The second 32-bit word in a free chunk is a pointer to the next free chunk (again sorted by address).
            freeCurr = *(uint32_t*)(freeCurr + 4);
        }

        // Skip past the 32-bit size field in the chunk header.
        chunkCurr += 4;
        // 8-byte align the data pointer.
        chunkCurr = (chunkCurr + 7) & ~7;
        // newlib-nano over allocates by 8 bytes, 4 bytes for the 32-bit chunk size and another 4 bytes to allow for 8
        // byte-alignment of the returned pointer.
        chunkSize -= 8;
        if(verbose)
            stream->printf("  Chunk: %lu  Address: 0x%08lX  Size: %lu  %s\n", chunkNumber, chunkCurr, chunkSize, isChunkFree ? "CHUNK FREE" : "");

        if(isChunkFree) freeSize += chunkSize;
        else usedSize += chunkSize;

        chunkCurr = chunkNext;
        chunkNumber++;
    }
    stream->printf("Allocated: %lu, Free: %lu\r\n", usedSize, freeSize);
}


void SimpleShell::on_module_loaded(){
    this->current_path = "/";
    this->register_for_event(ON_CONSOLE_LINE_RECEIVED);
    this->reset_delay_secs= 0;

    this->register_for_event(ON_SECOND_TICK);
    this->register_for_event(ON_GCODE_RECEIVED);
}

void SimpleShell::on_second_tick(void*) {
    // we are timing out for the reset
    if (this->reset_delay_secs > 0) {
        if(--this->reset_delay_secs == 0){
            system_reset(false);
        }
    }
}

void SimpleShell::on_gcode_received(void *argument) {
    Gcode *gcode = static_cast<Gcode*>(argument);

    if (gcode->has_m) {
        if (gcode->m == 20) { // list sd card
            gcode->mark_as_taken();
            gcode->stream->printf("Begin file list\r\n");
            ls_command("/sd", gcode->stream);
            gcode->stream->printf("End file list\r\n");
        }
    }
}

// When a new line is received, check if it is a command, and if it is, act upon it
void SimpleShell::on_console_line_received( void* argument ){
    SerialMessage new_message = *static_cast<SerialMessage*>(argument);
    string possible_command = new_message.message;

    //new_message.stream->printf("Received %s\r\n", possible_command.c_str());

    // We don't compare to a string but to a checksum of that string, this saves some space in flash memory
    unsigned short check_sum = get_checksum( possible_command.substr(0,possible_command.find_first_of(" \r\n")) );  // todo: put this method somewhere more convenient

    // Act depending on command
    if (check_sum == ls_command_checksum)
        this->ls_command(  get_arguments(possible_command), new_message.stream );
    else if (check_sum == cd_command_checksum)
        this->cd_command(  get_arguments(possible_command), new_message.stream );
    else if (check_sum == pwd_command_checksum)
        this->pwd_command( get_arguments(possible_command), new_message.stream );
    else if (check_sum == cat_command_checksum)
        this->cat_command( get_arguments(possible_command), new_message.stream );
    else if (check_sum == break_command_checksum)
        this->break_command(get_arguments(possible_command),new_message.stream );
    else if (check_sum == reset_command_checksum)
        this->reset_command(get_arguments(possible_command),new_message.stream );
    else if (check_sum == dfu_command_checksum)
        this->dfu_command(get_arguments(possible_command),new_message.stream );
    else if (check_sum == help_command_checksum)
        this->help_command(get_arguments(possible_command),new_message.stream );
    else if (check_sum == version_command_checksum)
        this->version_command(get_arguments(possible_command),new_message.stream );
    else if (check_sum == get_command_checksum)
        this->get_command(get_arguments(possible_command),new_message.stream );
    else if (check_sum == set_temp_command_checksum)
        this->set_temp_command(get_arguments(possible_command),new_message.stream );
    else if (check_sum == mem_command_checksum)
        this->mem_command(get_arguments(possible_command),new_message.stream );

}

// Convert a path indication ( absolute or relative ) into a path ( absolute )
string SimpleShell::absolute_from_relative( string path ){
    if( path[0] == '/' ){ return path; }
    if( path[0] == '.' ){ return this->current_path; }
    return this->current_path + path;
}

// Act upon an ls command
// Convert the first parameter into an absolute path, then list the files in that path
void SimpleShell::ls_command( string parameters, StreamOutput* stream ){
    string folder = this->absolute_from_relative( parameters );
    DIR* d;
    struct dirent* p;
    d = opendir(folder.c_str());
    if(d != NULL) {
        while((p = readdir(d)) != NULL) { stream->printf("%s\r\n", lc(string(p->d_name)).c_str()); }
        closedir(d);
    } else {
        stream->printf("Could not open directory %s \r\n", folder.c_str());
    }
}

// Change current absolute path to provided path
void SimpleShell::cd_command( string parameters, StreamOutput* stream ){
    string folder = this->absolute_from_relative( parameters );
    if( folder[folder.length()-1] != '/' ){ folder += "/"; }
    DIR *d;
    d = opendir(folder.c_str());
    if(d == NULL) {
        stream->printf("Could not open directory %s \r\n", folder.c_str() );
    }else{
        this->current_path = folder;
        closedir(d);
    }
}

// Responds with the present working directory
void SimpleShell::pwd_command( string parameters, StreamOutput* stream ){
    stream->printf("%s\r\n", this->current_path.c_str());
}

// Output the contents of a file, first parameter is the filename, second is the limit ( in number of lines to output )
void SimpleShell::cat_command( string parameters, StreamOutput* stream ){

    // Get parameters ( filename and line limit )
    string filename          = this->absolute_from_relative(shift_parameter( parameters ));
    string limit_paramater   = shift_parameter( parameters );
    int limit = -1;
    if( limit_paramater != "" )
    {
        char* e = NULL;
        limit = strtol(limit_paramater.c_str(), &e, 10);
        if (e <= limit_paramater.c_str())
            limit = -1;
    }

    // Open file
    FILE *lp = fopen(filename.c_str(), "r");
    if(lp == NULL) {
        stream->printf("File not found: %s\r\n", filename.c_str());
        return;
    }
    string buffer;
    int c;
    int newlines = 0;

    // Print each line of the file
    while ((c = fgetc (lp)) != EOF){
        buffer.append((char *)&c, 1);
        if( char(c) == '\n' ){
            newlines++;
            stream->puts(buffer.c_str());
            buffer.clear();
        }
        if( newlines == limit ){ break; }
    };
    fclose(lp);

}

// show free memory
void SimpleShell::mem_command( string parameters, StreamOutput* stream){
    bool verbose= shift_parameter( parameters ).find_first_of("Vv") != string::npos ;
    unsigned long heap= (unsigned long)_sbrk(0);
    unsigned long m= g_maximumHeapAddress - heap;
    stream->printf("Unused Heap: %lu bytes\r\n", m);

    heapWalk(stream, verbose);
}

// print out build version
void SimpleShell::version_command( string parameters, StreamOutput* stream){
    Version vers;
    stream->printf("Build version: %s, Build date: %s,  System Clock: %ldMHz\r\n", vers.get_build(), vers.get_build_date(), SystemCoreClock / 1000000);
}

// Reset the system
void SimpleShell::reset_command( string parameters, StreamOutput* stream){
    stream->printf("Smoothie out. Peace. Rebooting in 5 seconds...\r\n");
    this->reset_delay_secs= 5; // reboot in 5 seconds
}

// go into dfu boot mode
void SimpleShell::dfu_command( string parameters, StreamOutput* stream){
    stream->printf("Entering boot mode...\r\n");
    system_reset(true);
}

// Break out into the MRI debugging system
void SimpleShell::break_command( string parameters, StreamOutput* stream){
    stream->printf("Entering MRI debug mode...\r\n");
    __debugbreak();
}

// used to test out the get public data events
void SimpleShell::get_command( string parameters, StreamOutput* stream){
    int what= get_checksum(shift_parameter( parameters ));
    void *returned_data;

    if(what == get_temp_command_checksum) {
        string type= shift_parameter( parameters );
        bool ok= this->kernel->public_data->get_value( temperature_control_checksum, get_checksum(type), current_temperature_checksum, &returned_data );

        if(ok) {
            struct pad_temperature temp=  *static_cast<struct pad_temperature*>(returned_data);
            stream->printf("%s temp: %f/%f @%d\r\n", type.c_str(), temp.current_temperature, temp.target_temperature, temp.pwm);
        }else{
            stream->printf("%s is not a known temperature device\r\n", type.c_str());
        }

    }else if(what == get_pos_command_checksum) {
        bool ok= this->kernel->public_data->get_value( robot_checksum, current_position_checksum, &returned_data );

        if(ok) {
            double *pos= static_cast<double *>(returned_data);
            stream->printf("Position X: %f, Y: %f, Z: %f\r\n", pos[0], pos[1], pos[2]);

        }else{
            stream->printf("get pos command failed\r\n");
        }
    }
}

// used to test out the get public data events
void SimpleShell::set_temp_command( string parameters, StreamOutput* stream){
    string type= shift_parameter( parameters );
    string temp= shift_parameter( parameters );
    double t= temp.empty() ? 0.0 : strtod(temp.c_str(), NULL);
    bool ok= this->kernel->public_data->set_value( temperature_control_checksum, get_checksum(type), &t );

    if(ok) {
        stream->printf("%s temp set to: %3.1f\r\n", type.c_str(), t);
    }else{
        stream->printf("%s is not a known temperature device\r\n", type.c_str());
    }
}

void SimpleShell::help_command( string parameters, StreamOutput* stream ){
    stream->printf("Commands:\r\n");
    stream->printf("version\r\n");
    stream->printf("mem [-v]\r\n");
    stream->printf("ls [folder]\r\n");
    stream->printf("cd folder\r\n");
    stream->printf("pwd\r\n");
    stream->printf("cat file [limit]\r\n");
    stream->printf("play file [-q]\r\n");
    stream->printf("progress - shows progress of current play\r\n");
    stream->printf("abort - abort currently playing file\r\n");
    stream->printf("reset - reset smoothie\r\n");
    stream->printf("dfu - enter dfu boot loader\r\n");
    stream->printf("break - break into debugger\r\n");
    stream->printf("config-get [<configuration_source>] <configuration_setting>\r\n");
    stream->printf("config-set [<configuration_source>] <configuration_setting> <value>\r\n");
    stream->printf("config-load [<file_name>]\r\n");
    stream->printf("get temp [bed|hotend]\r\n");
    stream->printf("set_temp bed|hotend 185\r\n");
    stream->printf("get pos\r\n");
}

