#include "Testrig.h"

#include "Config.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "Gcode.h"
#include "StreamOutputPool.h"

Testrig::Testrig() {}

void Testrig::on_module_loaded(){
    on_config_reload(this);
    this->register_for_event(ON_GCODE_RECEIVED);
}

void Testrig::on_config_reload(void *argument){
}

void Testrig::on_gcode_received(void *argument){
    Gcode *gcode = static_cast<Gcode *>(argument);
    
    // For each possible M-code we are interrested in 
    if(gcode->has_m) {
        if(gcode->m == 691) {
            // Just acknowledging the module's existence for testing purposes 
            THEKERNEL->streams->printf("Testrig module exists\r\n");
        }
        if(gcode->m == 692) {
            // Start the testing procedure 
        }
    }
}

