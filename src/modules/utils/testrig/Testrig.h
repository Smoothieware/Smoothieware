#ifndef _TESTRIG_H
#define _TESTRIG_H

#include "libs/Kernel.h"
#include "libs/Pin.h"


class Testrig : public Module {
  public:
    Testrig();

    void on_module_loaded(void);
    void on_config_reload(void *);
    void on_gcode_received(void *argument);

  private:

};

#endif /* _TESTRIG_H */
