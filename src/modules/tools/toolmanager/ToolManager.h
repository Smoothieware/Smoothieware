/*
      this file is part of smoothie (http://smoothieware.org/). the motion control part is heavily based on grbl (https://github.com/simen/grbl).
      smoothie is free software: you can redistribute it and/or modify it under the terms of the gnu general public license as published by the free software foundation, either version 3 of the license, or (at your option) any later version.
      smoothie is distributed in the hope that it will be useful, but without any warranty; without even the implied warranty of merchantability or fitness for a particular purpose. see the gnu general public license for more details.
      you should have received a copy of the gnu general public license along with smoothie. if not, see <http://www.gnu.org/licenses/>.
*/

#ifndef TOOLMANAGER_H
#define TOOLMANAGER_H

using namespace std;
#include <vector>

class Tool;

class ToolManager : public Module
{
public:
    ToolManager();

    void on_module_loaded();
    void on_gcode_received(void *);
    void on_config_reload(void *);
    void on_get_public_data(void *argument);
    void on_set_public_data(void *argument);
    void add_tool(Tool *tool_to_add);

private:
    vector<Tool *> tools;

    int active_tool;
    uint16_t current_tool_name;
    bool return_error_on_unhandled_gcode;
};



#endif
