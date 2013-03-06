#ifndef CURRENTCONTROL_H
#define CURRENTCONTROL_H

#include "libs/Kernel.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "libs/Pin.h"
#include "libs/Digipot.h"

#define alpha_current_checksum                  CHECKSUM("alpha_current")
#define beta_current_checksum                   CHECKSUM("beta_current")
#define gamma_current_checksum                  CHECKSUM("gamma_current")
#define delta_current_checksum                  CHECKSUM("delta_current")
#define currentcontrol_module_enable_checksum   CHECKSUM("currentcontrol_module_enable")

class CurrentControl : public Module {
    public:
        CurrentControl();
        virtual ~CurrentControl() {};

        void on_module_loaded();
        void on_gcode_received(void *);

        double alpha_current;
        double beta_current;
        double gamma_current;
        double delta_current;

        Digipot* digipot;
};





#endif
