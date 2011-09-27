#ifndef utils_h
#define utils_h

#include <string>
using std::string;
#include "mbed.h"

string lc(string str);

uint16_t get_checksum(string to_check);

string shift_parameter( string &parameters );

string get_arguments( string possible_command );






#endif
