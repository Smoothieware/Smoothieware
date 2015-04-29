#ifndef SECURITY_POOL_H_
#define SECURITY_POOL_H_

#include <vector>
#include "checksumm.h"

#define coldextrusionprevention_checksum CHECKSUM("coldextrusionprevention")
#define enable_checksum                  CHECKSUM("enable")

class SecurityPool {
    public:
        void load_prevention_controllers();
        const std::vector<uint16_t>& get_controllers() const { return controllers; }

    private:
        std::vector<uint16_t> controllers;
};

#endif

