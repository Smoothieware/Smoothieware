//
//  set_output_pin.cpp
//
//  Created by Ron Aldrich on 6/20/14.
//

#include "set_output_pin.h"

#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <vector>

#include "checksumm.h"
#include "Pin.h"
#include "Kernel.h"
#include "ConfigValue.h"
#include "Config.h"

static uint16_t default_checksum = CHECKSUM("set_output_pin");

void set_output_pin::set_output_pins()
{
    std::vector <uint16_t> thePins;
    
    THEKERNEL->config->get_checksums(&thePins, default_checksum);
 
    for( auto i : thePins )
    {
        Pin thePin;
        thePin.from_string(THEKERNEL->config->value(default_checksum, i)->as_string())->as_output()->set(0);
    }
}
