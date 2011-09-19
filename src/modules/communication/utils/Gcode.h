#ifndef GCODE_H
#define GCODE_H
#include <string>
using std::string;
// Object to represent a Gcode comman

class Gcode {
    public:
        Gcode();
        bool has_letter( char letter );
        double get_value ( char letter );

        string command;
};
#endif
