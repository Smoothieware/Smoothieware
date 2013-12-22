#include "version.h"
const char *Version::get_build(void) const {
    return __GITVERSIONSTRING__;
}
const char *Version::get_build_date(void) const {
    return __DATE__ " " __TIME__;
}
