#include "libs/utils.h"
#include "mbed.h"
#include <string>
using std::string;



uint16_t get_checksum(string to_check){
   // From: http://en.wikipedia.org/wiki/Fletcher%27s_checksum 
   uint16_t sum1 = 0;
   uint16_t sum2 = 0;
   for( int index = 0; index < to_check.length(); ++index ){
      sum1 = (sum1 + to_check[index]) % 255;
      sum2 = (sum2 + sum1) % 255;
   }
   return (sum2 << 8) | sum1;
}

// Convert to lowercase
string lc(string str){
    for (int i=0;i<strlen(str.c_str());i++)
        if (str[i] >= 0x41 && str[i] <= 0x5A)
        str[i] = str[i] + 0x20;
    return str;
}

// Get the first parameter, and remove it from the original string
string shift_parameter( string &parameters ){
    size_t beginning = parameters.find_first_of(" ");
    if( beginning == string::npos ){ string temp = parameters; parameters = ""; return temp; } 
    string temp = parameters.substr( 0, beginning );
    parameters = parameters.substr(beginning+1, parameters.size());
    return temp;
}

// Separate command from arguments
string get_arguments( string possible_command ){
    size_t beginning = possible_command.find_first_of(" ");
    if( beginning == string::npos ){ return ""; } 
    return possible_command.substr( beginning+1, possible_command.size() - beginning);
}



