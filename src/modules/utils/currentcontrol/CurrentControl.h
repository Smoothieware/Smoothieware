#ifndef CURRENTCONTROL_H
#define CURRENTCONTROL_H

#include "libs/Kernel.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "libs/Pin.h"
#include "DigipotBase.h"

#define alpha_current_checksum                  CHECKSUM("alpha_current")
#define beta_current_checksum                   CHECKSUM("beta_current")
#define gamma_current_checksum                  CHECKSUM("gamma_current")
#define delta_current_checksum                  CHECKSUM("delta_current")
#define epsilon_current_checksum                CHECKSUM("epsilon_current")
#define zeta_current_checksum                   CHECKSUM("zeta_current")
#define eta_current_checksum                    CHECKSUM("eta_current")
#define theta_current_checksum                  CHECKSUM("theta_current")
#define currentcontrol_module_enable_checksum   CHECKSUM("currentcontrol_module_enable")
#define digipotchip_checksum                    CHECKSUM("digipotchip")
#define digipot_max_current                     CHECKSUM("digipot_max_current")
#define digipot_factor                          CHECKSUM("digipot_factor")

#define mcp4451_checksum                        CHECKSUM("mcp4451")
#define ad5206_checksum                         CHECKSUM("ad5206")

class CurrentControl : public Module {
    public:
        CurrentControl();
        virtual ~CurrentControl() {};

        void on_module_loaded();
        void on_gcode_received(void *);

    private:
        float alpha_current;
        float beta_current;
        float gamma_current;
        float delta_current;
        float epsilon_current;
        float zeta_current;
        float eta_current;
        float theta_current;
        float original_delta_current;

        DigipotBase* digipot;

};





#endif
