#include "SecurityPool.h"

#include "libs/Kernel.h"
#include "Config.h"
#include "ConfigValue.h"

#include "ColdExtrusionPrevention.h"

void SecurityPool::load_prevention_controllers() {
    vector<uint16_t> modules;
    THEKERNEL->config->get_module_list(&modules, coldextrusionprevention_checksum);

    for(auto cs: modules) {
        if( THEKERNEL->config->value(coldextrusionprevention_checksum, cs, enable_checksum )->as_bool() ) {
            ColdExtrusionPrevention *ctrl = new ColdExtrusionPrevention(cs);
            controllers.push_back(cs);
            THEKERNEL->add_module(ctrl);
        }
    }
}