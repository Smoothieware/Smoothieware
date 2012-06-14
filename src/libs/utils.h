#ifndef utils_h
#define utils_h

#include <stdint.h>
using namespace std;
#include <string>
using std::string;

string lc(string str);

string remove_non_number( string str );

uint16_t get_checksum(string to_check);

string shift_parameter( string &parameters );

string get_arguments( string possible_command );

void system_reset( void );




#endif
