#include "libs/Kernel.h"
#include "CurrentControl.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"

#include "Gcode.h"

// add new digipot chips here
#include "mcp4451.h"
#include "ad5206.h"

#include <string>
using namespace std;

CurrentControl::CurrentControl(){
    digipot= NULL;
}

void CurrentControl::on_module_loaded(){
    if( !this->kernel->config->value( currentcontrol_module_enable_checksum )->by_default(false)->as_bool() ){
        // as this module is not needed free up the resource
        delete this;
        return;
    }

    // allocate digipot, if already allocated delete it first
    delete digipot;

    // see which chip to use
    int chip_checksum = get_checksum(this->kernel->config->value(digipotchip_checksum)->by_default("mcp4451")->as_string());
    if(chip_checksum == mcp4451_checksum) {
        digipot = new MCP4451();
    }else if(chip_checksum == ad5206_checksum) {
        digipot = new AD5206();
    }else { // need a default so use smoothie
        digipot = new MCP4451();
    }

    // Get configuration
    this->alpha_current =           this->kernel->config->value(alpha_current_checksum  )->by_default(0.8)->as_number();
    this->beta_current  =           this->kernel->config->value(beta_current_checksum   )->by_default(0.8)->as_number();
    this->gamma_current =           this->kernel->config->value(gamma_current_checksum  )->by_default(0.8)->as_number();
    this->delta_current =           this->kernel->config->value(delta_current_checksum  )->by_default(0.8)->as_number();
    this->epsilon_current =         this->kernel->config->value(epsilon_current_checksum)->by_default(-1)->as_number();
    this->zeta_current  =           this->kernel->config->value(zeta_current_checksum   )->by_default(-1)->as_number();
    this->eta_current =             this->kernel->config->value(eta_current_checksum    )->by_default(-1)->as_number();
    this->theta_current =           this->kernel->config->value(theta_current_checksum  )->by_default(-1)->as_number();

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
                if (gcode->has_letter(alpha[i]))
                    this->digipot->set_current(i, gcode->get_value(alpha[i]));
                gcode->stream->printf("%c:%3.1fA%c", alpha[i], this->digipot->get_current(i), (i == 7)?'\n':' ');
            }
        }
    }
}
