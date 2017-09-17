#pragma once

#include "Robot.h"

class AutoPushPop
{
public:
    AutoPushPop(){ THEROBOT->push_state(); }
    ~AutoPushPop(){ THEROBOT->pop_state(); }
};
