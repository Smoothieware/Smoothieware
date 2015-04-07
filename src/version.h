#ifndef _VERSION__H
#define _VERSION__H
class Version {
    public:
        const char *get_build(void) const;
        const char *get_build_date(void) const;
};
#endif
