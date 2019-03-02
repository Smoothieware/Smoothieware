#include "version.h"
const char *Version::get_build(void) const {
    #ifdef DISABLEMSD
        return __GITVERSIONSTRING__ "NOMSD";
    #else
        return __GITVERSIONSTRING__;
    #endif
}
const char *Version::get_build_date(void) const {
    return __DATE__ " " __TIME__;
}
