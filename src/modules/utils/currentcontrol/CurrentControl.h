#ifndef CURRENTCONTROL_H
#define CURRENTCONTROL_H

#include "libs/Kernel.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "libs/Pin.h"

#define alpha_current_checksum   22381 
#define beta_current_checksum    60163 
#define gamma_current_checksum   12906 
#define delta_current_checksum   30321
#define currentcontrol_module_enable_checksum 38842

class CurrentControl : public Module {
    public:
        CurrentControl();
       
        void on_module_loaded();
       
        double alpha_current;
        double beta_current;
        double gamma_current;
        double delta_current; 
};





#endif
