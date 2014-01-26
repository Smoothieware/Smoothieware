#ifndef _HOMER_H
#define _HOMER_H

#include "Module.h"

class Homer : public Module
{
public:
    Homer();

    void on_module_loaded(void);

    void on_gcode_received(void*);
};

#endif /* _HOMER_H */
