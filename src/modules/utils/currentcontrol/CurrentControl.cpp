#include "libs/Kernel.h"
#include "CurrentControl.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include <string>
using namespace std;

CurrentControl::CurrentControl(){}

void CurrentControl::on_module_loaded(){
    if( !this->kernel->config->value( currentcontrol_module_enable_checksum )->by_default(false)->as_bool() ){ return; } 

    // Get configuration
    this->alpha_current =           this->kernel->config->value(alpha_current_checksum  )->by_default(0.8)->as_number(); 
    this->beta_current  =           this->kernel->config->value(beta_current_checksum   )->by_default(0.8)->as_number(); 
    this->gamma_current =           this->kernel->config->value(gamma_current_checksum  )->by_default(0.8)->as_number(); 
    this->delta_current =           this->kernel->config->value(delta_current_checksum  )->by_default(0.8)->as_number();

    this->kernel->digipot->set_current(0, this->alpha_current);
    this->kernel->digipot->set_current(1, this->beta_current );
    this->kernel->digipot->set_current(2, this->gamma_current);
    this->kernel->digipot->set_current(3, this->delta_current);

}


