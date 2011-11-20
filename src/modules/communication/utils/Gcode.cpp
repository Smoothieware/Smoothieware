#include <string>
using std::string;
#include "Gcode.h"
#include "mbed.h"

Gcode::Gcode(){}

// Whether or not a Gcode has a letter
bool Gcode::has_letter( char letter ){
    return ( this->command.find( letter ) != string::npos );
}

// Retrieve the value for a given letter
double Gcode::get_value( char letter ){
    size_t start = this->command.find(letter);
    size_t end =   this->command.find_first_not_of("1234567890.-", start+1);
    if( end == string::npos ){ end = this->command.length()+1; }
    string extracted = this->command.substr( start+1, end-start-1 );
    double value = atof(extracted.c_str());
    return value; 
}
