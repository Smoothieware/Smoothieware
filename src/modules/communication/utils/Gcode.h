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
        double millimeters_of_travel;
        bool call_on_gcode_execute_event_immediatly;
        bool on_gcode_execute_event_called;
};
#endif
