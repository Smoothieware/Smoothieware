//
//  Defaults.cpp
//
//  Created by Ron Aldrich on 6/20/14.
//

#include "Defaults.h"

#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <vector>

#include "checksumm.h"
#include "Pin.h"
#include "Kernel.h"
#include "ConfigValue.h"
#include "Config.h"

static uint16_t default_checksum = CHECKSUM("default");

void Defaults::set_defaults()
{
    std::vector <uint16_t> thePins;
    
    THEKERNEL->config->get_checksums(&thePins, default_checksum);
    
    for (std::vector<uint16_t>::iterator i = thePins.begin(); i != thePins.end(); i++)
    {
        Pin thePin;
        thePin.from_string(THEKERNEL->config->value(default_checksum,
                                                    *i)->as_string())->as_output()->set(0);
        
        
    }
}
