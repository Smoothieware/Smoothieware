#include "libs/Kernel.h"
#include "CurrentControl.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "ConfigValue.h"
#include "libs/StreamOutput.h"

#include "Gcode.h"
#include "Config.h"
#include "checksumm.h"

// add new digipot chips here
#include "mcp4451.h"
#include "ad5206.h"

#include <string>
using namespace std;

CurrentControl::CurrentControl(){
    digipot= NULL;
}

void CurrentControl::on_module_loaded(){
    if( !THEKERNEL->config->value( currentcontrol_module_enable_checksum )->by_default(false)->as_bool() ){
        // as this module is not needed free up the resource
        delete this;
        return;
    }

    // allocate digipot, if already allocated delete it first
    delete digipot;

    // see which chip to use
    int chip_checksum = get_checksum(THEKERNEL->config->value(digipotchip_checksum)->by_default("mcp4451")->as_string());
    if(chip_checksum == mcp4451_checksum) {
        digipot = new MCP4451();
    }else if(chip_checksum == ad5206_checksum) {
        digipot = new AD5206();
    }else { // need a default so use smoothie
        digipot = new MCP4451();
    }

    // Get configuration
    this->alpha_current =           THEKERNEL->config->value(alpha_current_checksum  )->by_default(0.8f)->as_number();
    this->beta_current  =           THEKERNEL->config->value(beta_current_checksum   )->by_default(0.8f)->as_number();
    this->gamma_current =           THEKERNEL->config->value(gamma_current_checksum  )->by_default(0.8f)->as_number();
    this->delta_current =           THEKERNEL->config->value(delta_current_checksum  )->by_default(0.8f)->as_number();
    this->epsilon_current =         THEKERNEL->config->value(epsilon_current_checksum)->by_default(-1)->as_number();
    this->zeta_current  =           THEKERNEL->config->value(zeta_current_checksum   )->by_default(-1)->as_number();
    this->eta_current =             THEKERNEL->config->value(eta_current_checksum    )->by_default(-1)->as_number();
    this->theta_current =           THEKERNEL->config->value(theta_current_checksum  )->by_default(-1)->as_number();

    digipot->set_max_current(       THEKERNEL->config->value(digipot_max_current     )->by_default(2.0f)->as_number());
    digipot->set_factor(            THEKERNEL->config->value(digipot_factor          )->by_default(113.33f)->as_number());

    this->digipot->set_current(0, this->alpha_current);
    this->digipot->set_current(1, this->beta_current );
    this->digipot->set_current(2, this->gamma_current);
    this->digipot->set_current(3, this->delta_current);
    if(this->epsilon_current >= 0){
        this->digipot->set_current(4, this->epsilon_current);
        this->digipot->set_current(5, this->zeta_current );
        this->digipot->set_current(6, this->eta_current);
        this->digipot->set_current(7, this->theta_current);
    }

    this->original_delta_current= this->delta_current; // remember this to determine if we want to save on M500

    this->register_for_event(ON_GCODE_RECEIVED);
}


void CurrentControl::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode*>(argument);
    char alpha[8] = { 'X', 'Y', 'Z', 'E', 'A', 'B', 'C', 'D' };
    if (gcode->has_m)
    {
        if (gcode->m == 907)
        {
            int i;
            for (i = 0; i < 8; i++)
            {
                if (gcode->has_letter(alpha[i])){
                    float c= gcode->get_value(alpha[i]);
                    this->digipot->set_current(i, c);
                    switch(i) {
                        case 0: this->alpha_current= c; break;
                        case 1: this->beta_current= c; break;
                        case 2: this->gamma_current= c; break;
                        case 3: this->delta_current= c; break;
                        case 4: this->epsilon_current= c; break;
                        case 5: this->zeta_current= c; break;
                        case 6: this->eta_current= c; break;
                        case 7: this->theta_current= c; break;
                    }
                }
                gcode->stream->printf("%c:%3.1fA%c", alpha[i], this->digipot->get_current(i), (i == 7)?'\n':' ');
            }

        }else if(gcode->m == 500 || gcode->m == 503) {
            if(this->delta_current != this->original_delta_current) { // if not the same as loaded by config then save it
                gcode->stream->printf(";Extruder current:\nM907 E%1.5f\n", this->delta_current);
            }
        }
    }
}
