#ifndef utils_h
#define utils_h

#include <stdint.h>
using namespace std;
#include <string>
#include <vector>
using std::string;

// because apparently MAX_INT doesn't come with arm-none-eabi-gcc
#define MAX_INT 0x7FFFFFFF

string lc(string str);

string remove_non_number( string str );

uint16_t get_checksum(string to_check);

void get_checksums(uint16_t check_sums[], string key);

string shift_parameter( string &parameters );

string get_arguments( string possible_command );

bool file_exists( string file_name );

void system_reset( void );

int find_first_of(const char*, const int  );
int find_first_of(const char*, const char*);


#endif
