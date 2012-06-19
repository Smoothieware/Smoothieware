/*  
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>. 
*/

#include "libs/Kernel.h"
#include "libs/utils.h"
#include "system_LPC17xx.h"
using namespace std;
#include <string>
using std::string;
#include <cstring>


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

vector<uint16_t> get_checksums(string key){
    vector<uint16_t> check_sums;
    size_t begin_key = 0;
    while( begin_key < key.size() ){
        size_t end_key =  key.find_first_of(" .", begin_key);
        string key_node = key.substr(begin_key, end_key - begin_key);
        check_sums.push_back(get_checksum(key_node));
        begin_key = end_key + 1;
    }
    return check_sums;
}

// Convert to lowercase
string lc(string str){
    for (int i=0;i<strlen(str.c_str());i++)
        if (str[i] >= 0x41 && str[i] <= 0x5A)
        str[i] = str[i] + 0x20;
    return str;
}

// Remove non-number characters
string remove_non_number( string str ){
    string number_mask = "0123456789-.";
    size_t found=str.find_first_not_of(number_mask);
    while (found!=string::npos){
        //str[found]='*';
        str.replace(found,1,""); 
        found=str.find_first_not_of(number_mask);
    }
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

// Prepares and executes a watchdog reset
void system_reset( void ){
    LPC_WDT->WDCLKSEL = 0x1;                // Set CLK src to PCLK
    uint32_t clk = SystemCoreClock / 16;    // WD has a fixed /4 prescaler, PCLK default is /4
    LPC_WDT->WDTC = 1 * (float)clk;         // Reset in 1 second
    LPC_WDT->WDMOD = 0x3;                   // Enabled and Reset
    LPC_WDT->WDFEED = 0xAA;                 // Kick the dog!
    LPC_WDT->WDFEED = 0x55;
}

