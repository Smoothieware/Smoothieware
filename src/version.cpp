#include "version.h"
const char *Version::get_build(void) const {
    return "edge-8028708";
}
const char *Version::get_build_date(void) const {
    return __DATE__ " " __TIME__;
}
