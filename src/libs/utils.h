#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>
#include <string>
#include <vector>
using std::string;
using std::vector;

string lc(const string& str);

bool is_alpha( int );
bool is_digit( int );
bool is_numeric( int );
bool is_alphanum( int );
bool is_whitespace( int );

vector<string> split(const char *str, char c = ',');
vector<float> parse_number_list(const char *str);

string remove_non_number( string str );

uint16_t get_checksum(const string& to_check);
uint16_t get_checksum(const char* to_check);

void get_checksums(uint16_t check_sums[], const string& key);

string shift_parameter( string &parameters );

string get_arguments( string possible_command );

bool file_exists( const string file_name );

void system_reset( bool dfu= false );

string absolute_from_relative( string path );


#endif
