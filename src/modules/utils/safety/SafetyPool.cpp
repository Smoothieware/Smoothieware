#include "SafetyPool.h"

#include "libs/Kernel.h"
#include "Config.h"
#include "ConfigValue.h"

#include "ColdExtrusionPrevention.h"

void SafetyPool::load_prevention_controllers() {
    vector<uint16_t> modules;
    THEKERNEL->config->get_module_list(&modules, coldextrusionprevention_checksum);

    for(auto cs: modules) {
        if( THEKERNEL->config->value(coldextrusionprevention_checksum, cs, enable_checksum )->as_bool() ) {
            ColdExtrusionPrevention *ctrl = new ColdExtrusionPrevention(cs);
            THEKERNEL->add_module(ctrl);
        }
    }
}
