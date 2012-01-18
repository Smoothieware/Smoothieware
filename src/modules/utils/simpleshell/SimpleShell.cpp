#include "mbed.h"
#include "libs/Kernel.h"
#include "SimpleShell.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"


void SimpleShell::on_module_loaded(){
    this->current_path = "/";
    this->register_for_event(ON_CONSOLE_LINE_RECEIVED);
}

// When a new line is received, check if it is a command, and if it is, act upon it
void SimpleShell::on_console_line_received( void* argument ){
    SerialMessage new_message = *static_cast<SerialMessage*>(argument);
    string possible_command = new_message.message;

    // We don't compare to a string but to a checksum of that string, this saves some space in flash memory
    unsigned short check_sum = get_checksum( possible_command.substr(0,possible_command.find_first_of(" \r\n")) );  // todo: put this method somewhere more convenient

    // Act depending on command
    switch( check_sum ){
        case ls_command_checksum      : this->ls_command(  get_arguments(possible_command), new_message.stream ); break;
        case cd_command_checksum      : this->cd_command(  get_arguments(possible_command), new_message.stream ); break;
        case cat_command_checksum     : this->cat_command( get_arguments(possible_command), new_message.stream ); break;
        case play_command_checksum    : this->play_command(get_arguments(possible_command), new_message.stream ); break; 
    }
}

// Convert a path indication ( absolute or relative ) into a path ( absolute )
string SimpleShell::absolute_from_relative( string path ){
    if( path[0] == '/' ){ return path; }
    if( path[0] == '.' ){ return this->current_path; } 
    return this->current_path + path;
}

// Act upon an ls command
// Convert the first parameter into an absolute path, then list the files in that path
void SimpleShell::ls_command( string parameters, Stream* stream ){
    string folder = this->absolute_from_relative( parameters );
    DIR* d;
    struct dirent* p;
    d = opendir(folder.c_str());
    if(d != NULL) {
        while((p = readdir(d)) != NULL) { stream->printf("%s\r\n", lc(string(p->d_name)).c_str()); }
    } else {
        stream->printf("Could not open directory %s \r\n", folder.c_str());
    }
}

// Change current absolute path to provided path
void SimpleShell::cd_command( string parameters, Stream* stream ){
    string folder = this->absolute_from_relative( parameters );
    if( folder[folder.length()-1] != '/' ){ folder += "/"; }
    DIR *d;
    struct dirent *p;
    d = opendir(folder.c_str());
    if(d == NULL) { 
        stream->printf("Could not open directory %s \r\n", folder.c_str() ); 
    }else{
        this->current_path = folder;
    }
}

// Output the contents of a file, first parameter is the filename, second is the limit ( in number of lines to output )
void SimpleShell::cat_command( string parameters, Stream* stream ){
    
    // Get parameters ( filename and line limit ) 
    string filename          = this->absolute_from_relative(shift_parameter( parameters ));
    string limit_paramater   = shift_parameter( parameters );
    int limit = -1;
    if( limit_paramater != "" ){ limit = int(atof(limit_paramater.c_str())); }
   
    // Open file 
    FILE *lp = fopen(filename.c_str(), "r");
    string buffer;
    int c;
    int newlines = 0; 
    
    // Print each line of the file
    while ((c = fgetc (lp)) != EOF){
        if( char(c) == '\n' ){  newlines++; }
        stream->putc(c); 
        if( newlines == limit ){ break; }
    }; 
    fclose(lp);

}

// Play a gcode file by considering each line as if it was received on the serial console
void SimpleShell::play_command( string parameters, Stream* stream ){

    // Get filename
    string filename          = this->absolute_from_relative(shift_parameter( parameters ));
 
    // Open file 
    FILE *lp = fopen(filename.c_str(), "r");
    string buffer;
    int c;
    
    // Print each line of the file
    while ((c = fgetc (lp)) != EOF){
        if (c == '\n'){
            stream->printf("%s\n", buffer.c_str());
            struct SerialMessage message; 
            message.message = buffer;
            message.stream = stream;
            this->kernel->call_event(ON_CONSOLE_LINE_RECEIVED, &message); 
            buffer.clear();
        }else{
            buffer += c;
        }
    }; 
    fclose(lp);

}



