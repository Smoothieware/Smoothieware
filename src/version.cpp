#include "version.h"
const char *Version::get_build(void) const {
    return "feature/add-version-command-582559c";
}
const char *Version::get_build_date(void) const {
    return __DATE__ " " __TIME__;
}
