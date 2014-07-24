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

CurrentControl::CurrentControl()
{
    digipot = NULL;
}

void CurrentControl::on_module_loaded()
{
    if( !THEKERNEL->config->value( currentcontrol_module_enable_checksum )->by_default(false)->as_bool() ) {
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
    } else if(chip_checksum == ad5206_checksum) {
        digipot = new AD5206();
    } else { // need a default so use smoothie
        digipot = new MCP4451();
    }

    // Get configuration
    this->digipot->set_current(0, THEKERNEL->config->value(alpha_current_checksum  )->by_default(0.8f)->as_number());
    this->digipot->set_current(1, THEKERNEL->config->value(beta_current_checksum   )->by_default(0.8f)->as_number());
    this->digipot->set_current(2, THEKERNEL->config->value(gamma_current_checksum  )->by_default(0.8f)->as_number());
    this->digipot->set_current(3, THEKERNEL->config->value(delta_current_checksum  )->by_default(0.8f)->as_number());
    this->digipot->set_current(4, THEKERNEL->config->value(epsilon_current_checksum)->by_default(-1)->as_number());
    this->digipot->set_current(5, THEKERNEL->config->value(zeta_current_checksum   )->by_default(-1)->as_number());
    this->digipot->set_current(6, THEKERNEL->config->value(eta_current_checksum    )->by_default(-1)->as_number());
    this->digipot->set_current(7, THEKERNEL->config->value(theta_current_checksum  )->by_default(-1)->as_number());

    digipot->set_max_current( THEKERNEL->config->value(digipot_max_current )->by_default(2.0f)->as_number());
    digipot->set_factor( THEKERNEL->config->value(digipot_factor )->by_default(113.33f)->as_number());

    this->register_for_event(ON_GCODE_RECEIVED);
}


void CurrentControl::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode*>(argument);
    char alpha[8] = { 'X', 'Y', 'Z', 'E', 'A', 'B', 'C', 'D' };
    if (gcode->has_m) {
        if (gcode->m == 907) {
            for (int i = 0; i < 8; i++) {
                if (gcode->has_letter(alpha[i])) {
                    float c = gcode->get_value(alpha[i]);
                    this->digipot->set_current(i, c);
                }
            }

        } else if(gcode->m == 500 || gcode->m == 503) {
            gcode->stream->printf(";Motor currents:\nM907 ");
            for (int i = 0; i < 8; i++) {
                float c = this->digipot->get_current(i);
                if(c >= 0)
                    gcode->stream->printf("%c%1.5f ", alpha[i], c);
            }
            gcode->stream->printf("\n");
        }
    }
}
