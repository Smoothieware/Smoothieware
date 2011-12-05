#include <string>
using std::string;
#include "Gcode.h"
#include "mbed.h"

Gcode::Gcode(){}

// Whether or not a Gcode has a letter
bool Gcode::has_letter( char letter ){
    //return ( this->command->find( letter ) != string::npos );
    for (size_t i=0; i < this->command.length(); i++){
        if( this->command.at(i) == letter ){
            return true;
        }
    }
    return false;
}

// Retrieve the value for a given letter
//double Gcode::get_value_old( char letter ){
//    size_t start = this->command.find(letter);
//    size_t end =   this->command.find_first_not_of("1234567890.-", start+1);
//    if( end == string::npos ){ end = this->command.length()+1; }
//    string extracted = this->command.substr( start+1, end-start-1 );
//    double value = atof(extracted.c_str());
//    return value; 
//}


// Retrieve the value for a given letter
// We don't use the high-level methods of std::string because they call malloc and it's very bad to do that inside of interrupts
double Gcode::get_value( char letter ){
    __disable_irq();
    for (size_t i=0; i <= this->command.length()-1; i++){
         if( letter == this->command.at(i) ){
            size_t beginning = i+1;
            char buffer[20];
            for(size_t j=beginning; j <= this->command.length(); j++){
                char c;
                if( j == this->command.length() ){ c = ';'; }else{ c = this->command.at(j); }
                if( c != '.' && c != '-' && ( c < '0' || c > '9' ) ){
                    buffer[j-beginning] = '\0';
                    __enable_irq();
                    return atof(buffer); 
                }else{
                    buffer[j-beginning] = c;
                }
            }
         }
    }
    __enable_irq();
    return 0; 
}
