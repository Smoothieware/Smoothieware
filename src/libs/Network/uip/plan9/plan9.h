#ifndef __PLAN9_H__
#define __PLAN9_H__

/*
 * 9P network filesystem protocol
 *
 * by Daniel Mendler <mail@daniel-mendler.de>
 *
 * Resources:
 *
 *   - Documentation: http://9p.cat-v.org/
 *   - List of implementations: http://9p.cat-v.org/implementations
 *   - Specification: http://ericvh.github.io/9p-rfc/
 *   - Linux documentation: https://www.kernel.org/doc/Documentation/filesystems/9p.txt
 *
 * How to use it:
 *
 *   1. Add "network.plan9.enable true" to the config
 *   2. Mount under Linux with "mount -t 9p $ip /mnt/smoothie
 */

#include <map>
#include <string>
#include <stdint.h>

class Plan9
{
public:
    Plan9();
    ~Plan9();

    static void init();
    static void appcall();

    struct Entry {
        uint8_t     type;
        uint32_t    vers;
        int         refcount;
        std::string path;

        Entry() {}
        Entry(uint8_t t, const std::string& p)
            : type(t), vers(0), refcount(0), path(p) {}
    };

private:
    void handler();

    Entry* get_entry(uint8_t, const std::string&);
    Entry* get_entry(uint32_t) const;
    void add_fid(uint32_t, Entry*);
    void remove_fid(uint32_t);

    std::map<uint32_t,    Entry*> fids;
    std::map<std::string, Entry>  entries;
};

#endif
