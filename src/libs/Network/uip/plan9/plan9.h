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
#include <queue>
#include <string>
#include <stdint.h>

extern "C" {
#include "psock.h"
}

class Plan9
{
public:
    Plan9();
    ~Plan9();

    static void init();
    static void appcall();

    struct EntryData {
        uint8_t     type;
        int         refcount;

        EntryData() {}
        EntryData(uint8_t t)
            : type(t), refcount(0) {}
    };

    typedef std::map<std::string, EntryData> EntryMap;
    typedef EntryMap::value_type*            Entry;
    typedef std::map<uint32_t, Entry>        FidMap;
    union Message;

private:
    int receive();
    int send();
    bool process(Message*, Message*);

    Entry add_entry(uint32_t, uint8_t, const std::string&);
    Entry get_entry(uint32_t);
    bool add_fid(uint32_t, Entry);
    void remove_fid(uint32_t);

    static const uint32_t INITIAL_MSIZE = 300;
    EntryMap             entries;
    FidMap               fids;
    psock                sin, sout;
    char                 bufin[INITIAL_MSIZE], bufout[INITIAL_MSIZE];
    std::queue<Message*> queue;
    uint32_t             msize, queue_bytes;
};

#endif
