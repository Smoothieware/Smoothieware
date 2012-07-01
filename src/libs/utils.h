#ifndef utils_h
#define utils_h

#include <stdint.h>
using namespace std;
#include <string>
#include <vector>
using std::string;

string lc(string str);

string remove_non_number( string str );

uint16_t get_checksum(string to_check);

vector<uint16_t> get_checksums(string key);

string shift_parameter( string &parameters );

string get_arguments( string possible_command );

bool file_exists( string file_name );

void system_reset( void );




#endif
