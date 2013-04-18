/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ACTION_H
#define ACTION_H

using namespace std;
#include <string>
#include <vector>

class Block;
class Planner;
class Conveyor;
class Module;

class Action;
class ActionData;

class Action {
    public:
        Action();

        ActionData* first_data;
        ActionData* gc_data;

        // this is an efficiency shortcut.
        Block* block_data;

        Action* clean();

        Action* add_data(ActionData*);
        Action* remove_data(ActionData*);

        // TODO: find a sensible way to ditch this and have actions access global conveyor
        Conveyor* conveyor;
};

class ActionData {
public:
    ActionData(){};
    virtual ~ActionData(){};

    Module* owner;
    Action* action;

    virtual void finish(void) {
        action->remove_data(this);
    };

    ActionData* next;
};

#endif
