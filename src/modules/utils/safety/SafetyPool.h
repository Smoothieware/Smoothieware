#ifndef SAFETY_POOL_H_
#define SAFETY_POOL_H_

#include <vector>
#include "checksumm.h"

#define coldextrusionprevention_checksum CHECKSUM("coldextrusionprevention")
#define enable_checksum                  CHECKSUM("enable")

class SafetyPool {
    public:
        void load_prevention_controllers();

    private:
};

#endif

