/*
 * 9P network filesystem protocol implementation
 *
 * by Daniel Mendler <mail@daniel-mendler.de>
 */

#include "plan9.h"
#include "DirHandle.h"
#include "FATFileSystem.h"
#include "StreamOutputPool.h"
#include "Kernel.h"
#include "utils.h"
#include <string.h>
#include "uip.h"

//#define DEBUG_PRINTF(...) THEKERNEL->streams->printf("9p " __VA_ARGS__)
#define DEBUG_PRINTF(...)

#define FAIL(text) \
    DEBUG_PRINTF("error %s %d\n", text, __LINE__);      \
    return send_error(text)

#define CHECK(cond, text) if (!(cond)) { FAIL(text); }
#define IOUNIT            (uip_mss() - sizeof (Message::Twrite))
#define PACKEDSTRUCT      struct __attribute__ ((packed))

namespace {

// See error mapping http://lxr.free-electrons.com/source/net/9p/error.c
const char ENOENT_TEXT[]      = "No such file or directory",
           EIO_TEXT[]         = "Input/output error",
           FID_UNKNOWN_TEXT[] = "fid unknown or out of range",
           FID_IN_USE_TEXT[]  = "fid already in use",
           EBADMSG_TEXT[]     = "Bad message",
           EEXIST_TEXT[]      = "File exists",
           EFAULT_TEXT[]      = "Bad address",
           ENOSYS_TEXT[]      = "Function not implemented",
           ENOTEMPTY_TEXT[]   = "Directory not empty";

enum {
    // 9P message types
    Tversion = 100,
    Tauth    = 102,
    Tattach  = 104,
    Terror   = 106,
    Rerror,
    Tflush   = 108,
    Twalk    = 110,
    Topen    = 112,
    Tcreate  = 114,
    Tread    = 116,
    Twrite   = 118,
    Tclunk   = 120,
    Tremove  = 122,
    Tstat    = 124,
    Twstat   = 126,

    // Qid type
    QTDIR     = 0x80, // directories
    QTAPPEND  = 0x40, // append only files
    QTEXCL    = 0x20, // exclusive use files
    QTMOUNT   = 0x10, // mounted channel
    QTAUTH    = 0x08, // authentication file
    QTTMP     = 0x04, // non-backed-up file
    QTLINK    = 0x02, // symbolic link
    QTFILE    = 0x00, // plain file

    // mode
    OREAD     = 0,	// open for read
    OWRITE    = 1,	// write
    ORDWR     = 2,	// read and write
    OEXEC     = 3,      // execute, == read but check execute permission
    OTRUNC    = 0x10,	// or'ed in (except for exec), truncate file first
    ORCLOSE   = 0x40, 	// or'ed in, remove on close

    // permission bits
    DMDIR    = 0x80000000, // directories
    DMAPPEND = 0x40000000, // append only files
    DMEXCL   = 0x20000000, // exclusive use files
    DMMOUNT  = 0x10000000, // mounted channel
    DMAUTH   = 0x08000000, // authentication file
    DMTMP    = 0x04000000, // non-backed-up file

    MAXWELEM = 16,
};

PACKEDSTRUCT Header {
    uint32_t size;
    uint8_t  type;
    uint16_t tag;
};

PACKEDSTRUCT Qid {
    uint8_t  type;
    uint32_t vers;
    uint64_t path;

    Qid() {}
    Qid(Plan9::Entry* entry)
        : type(entry->type), vers(entry->vers), path(uint32_t(entry)) {}
};

PACKEDSTRUCT Stat {
    uint16_t size;
    uint16_t type;
    uint32_t dev;
    Qid      qid;
    uint32_t mode;
    uint32_t atime;
    uint32_t mtime;
    uint64_t length;
};

// Important: 9P assumes little endian byte ordering!
union __attribute__ ((packed)) Message {
    PACKEDSTRUCT {
        uint32_t size;
        uint8_t  type;
        uint16_t tag;
        uint32_t fid;
    };

    // size[4] Tversion tag[2] msize[4] version[s]
    // size[4] Rversion tag[2] msize[4] version[s]
    PACKEDSTRUCT {
        Header   _header;
        uint32_t msize;
    } Tversion, Rversion;

    // // size[4] Tauth tag[2] afid[4] uname[s] aname[s]
    // PACKEDSTRUCT {
    //     Header   _header;
    //     uint32_t afid;
    // } Tauth;

    // // size[4] Rauth tag[2] aqid[13]
    // PACKEDSTRUCT {
    //     Header   _header;
    //     Qid      aqid;
    // } Rauth;

    // size[4] Rerror tag[2] ename[s]
    // size[4] Tclunk tag[2] fid[4]
    // size[4] Rclunk tag[2]
    // size[4] Tremove tag[2] fid[4]
    // size[4] Rremove tag[2]
    // size[4] Tstat tag[2] fid[4]

    // size[4] Tattach tag[2] fid[4] afid[4] uname[s] aname[s]
    PACKEDSTRUCT {
        Header   _header;
        uint32_t fid;
        uint32_t afid;
    } Tattach;

    // size[4] Rattach tag[2] qid[13]
    PACKEDSTRUCT {
        Header   _header;
        Qid      qid;
    } Rattach;

    // size[4] Tflush tag[2] oldtag[2]
    // size[4] Rflush tag[2]
    PACKEDSTRUCT {
        Header   _header;
        uint16_t oldtag;
    } Tflush;

    // size[4] Twalk tag[2] fid[4] newfid[4] nwname[2] nwname*(wname[s])
    PACKEDSTRUCT {
        Header   _header;
        uint32_t fid;
        uint32_t newfid;
        uint16_t nwname;
        char     wname[0];
    } Twalk;

    // size[4] Rwalk tag[2] nwqid[2] nwqid*(wqid[13])
    PACKEDSTRUCT {
        Header   _header;
        uint16_t nwqid;
        Qid      wqid[0];
    } Rwalk;

    // size[4] Topen tag[2] fid[4] mode[1]
    PACKEDSTRUCT {
        Header   _header;
        uint32_t fid;
        uint8_t  mode;
    } Topen;

    // size[4] Ropen tag[2] qid[13] iounit[4]
    // size[4] Rcreate tag[2] qid[13] iounit[4]
    PACKEDSTRUCT {
        Header   _header;
        Qid      qid;
        uint32_t iounit;
    } Ropen, Rcreate;

    // size[4] Tcreate tag[2] fid[4] name[s] perm[4] mode[1]
    PACKEDSTRUCT {
        Header   _header;
        uint32_t fid;
        uint16_t name_size;
        char     name[0];
    } Tcreate;

    // size[4] Tread tag[2] fid[4] offset[8] count[4]
    PACKEDSTRUCT {
        Header   _header;
        uint32_t fid;
        uint64_t offset;
        uint32_t count;
    } Tread;

    // size[4] Rread tag[2] count[4] data[count]
    // size[4] Rwrite tag[2] count[4]
    PACKEDSTRUCT {
        Header   _header;
        uint32_t count;
    } Rread, Rwrite;

    // size[4] Twrite tag[2] fid[4] offset[8] count[4] data[count]
    PACKEDSTRUCT {
        Header   _header;
        uint32_t fid;
        uint64_t offset;
        uint32_t count;
    } Twrite;

    // size[4] Rstat tag[2] stat[n]
    PACKEDSTRUCT {
        Header   _header;
        uint16_t stat_size;
        Stat     stat;
    } Rstat;

    // // size[4] Twstat tag[2] fid[4] stat[n]
    // // size[4] Rwstat tag[2]
    PACKEDSTRUCT {
        Header   _header;
        uint32_t fid;
        uint16_t stat_size;
        Stat     stat;
    } Twstat;
};

inline char* putstr(char* p, char* end, const char* s)
{
    auto n = strlen(s);
    if (!p || p + 2 + n > end)
        return nullptr;
    *p++ = n & 0xFF;
    *p++ = (n >> 8) & 0xFF;
    memcpy(p, s, n);
    return p + n;
}

inline long flen(const std::string& path)
{
    FILE* fp = fopen(path.c_str(), "r");
    if (!fp)
        return 0;
    fseek(fp, 0, SEEK_END);
    long len = ftell(fp);
    fclose(fp);
    return len < 0 ? 0 : len;
}

size_t putstat(char* buf, char* end, Plan9::Entry* entry)
{
    if (buf + sizeof (Stat) > end)
        return 0;

    char* p = buf;
    Stat* stat = reinterpret_cast<Stat*>(p);
    stat->type = 0;
    stat->dev = 0;
    stat->qid = entry;
    stat->mode = entry->type == QTDIR ? (DMDIR | 0755) : 0644;
    stat->atime = stat->mtime = 1423420000;
    stat->length = (stat->mode & DMDIR) ? 0 : flen(entry->path);
    p += sizeof (Stat);

    p = putstr(p, end, entry->path == "/" ? "/" : entry->path.substr(entry->path.rfind('/') + 1).c_str());
    p = putstr(p, end, "smoothie");
    p = putstr(p, end, "smoothie");
    p = putstr(p, end, "smoothie");
    if (!p) return 0;

    stat->size = p - buf - 2;
    return p - buf;
}

inline void start_response(Message* msg, uint16_t size)
{
    msg->size = size;
    ++msg->type;
}

void send_error(const char* text)
{
    char* buf = static_cast<char*>(uip_appdata);
    Message* msg = static_cast<Message*>(uip_appdata);
    msg->type = Rerror;
    msg->size = putstr(buf + sizeof (Header), buf + uip_mss(), text) - buf;
    uip_send(msg, msg->size);
}

} // anonymous namespace

Plan9::Plan9()  {}
Plan9::~Plan9() {}

std::string join_path(const std::string& a, const std::string& b)
{
    return a.back() != '/' ? absolute_from_relative(a + "/" + b) :
            absolute_from_relative(a + b);
}

Plan9::Entry* Plan9::get_entry(uint8_t type, const std::string& path)
{
    std::string abspath = absolute_from_relative(path);
    auto i = entries.find(abspath);
    if (i != entries.end())
        return &(i->second);
    return &(entries[abspath] = Entry(type, abspath));
}

Plan9::Entry* Plan9::get_entry(uint32_t fid) const
{
    auto i = fids.find(fid);
    if (i == fids.end())
        return nullptr;
    return i->second;
}

void Plan9::add_fid(uint32_t fid, Entry* entry)
{
    fids[fid] = entry;
    ++entry->refcount;
}

void Plan9::remove_fid(uint32_t fid)
{
    --fids[fid]->refcount;
    fids.erase(fid);
}

void Plan9::init()
{
    uip_listen(HTONS(564));
}

void Plan9::appcall()
{
    Plan9* instance = static_cast<Plan9*>(uip_conn->appstate);

    if (uip_connected() && !instance) {
        instance = new Plan9;
        DEBUG_PRINTF("new instance: %p\n", instance);
        uip_conn->appstate = instance;
    }

    if (uip_closed() || uip_aborted() || uip_timedout()) {
        DEBUG_PRINTF("closed: %p\n", instance);
        if(instance) {
            delete instance;
            uip_conn->appstate = nullptr;
        }
        return;
    }

    if (!instance) {
        DEBUG_PRINTF("null instance\n");
        uip_abort();
        return;
    }

    if (uip_newdata())
        instance->handler();
}

void Plan9::handler()
{
    Entry* entry;
    char* buf = static_cast<char*>(uip_appdata);
    char* end = buf + uip_datalen();

    Message* msg = reinterpret_cast<Message*>(buf);
    CHECK(buf + 2 <= end, EBADMSG_TEXT);
    CHECK(buf + msg->size <= end, EBADMSG_TEXT);

    DEBUG_PRINTF("datalen=%d\n", uip_datalen());

    switch (msg->type) {
    case Tversion:
        DEBUG_PRINTF("Tversion\n");
        start_response(msg, sizeof (msg->Rversion));
        if (uip_mss() < msg->Rversion.msize)
            msg->Rversion.msize = uip_mss();
        msg->size = putstr(buf + msg->size, buf + uip_mss(), "9P2000") - buf;
        break;

    case Tattach:
        DEBUG_PRINTF("Tattach\n");
        CHECK(!get_entry(msg->fid), FID_IN_USE_TEXT);
        entry = get_entry(QTDIR, "/");
        add_fid(msg->fid, entry);
        start_response(msg, sizeof (msg->Rattach));
        msg->Rattach.qid = entry;
        break;

    case Tflush:
        DEBUG_PRINTF("Tflush\n");
        CHECK(msg->size == sizeof (msg->Tflush), EBADMSG_TEXT);
        start_response(msg, sizeof (Header));
        // do nothing
        break;

    case Twalk:
        DEBUG_PRINTF("Twalk fid=%lu newfid=%lu\n", msg->Twalk.fid, msg->Twalk.newfid);
        CHECK(entry = get_entry(msg->Twalk.fid), FID_UNKNOWN_TEXT);
        CHECK(!get_entry(msg->Twalk.newfid), FID_IN_USE_TEXT);

        if (msg->Twalk.nwname == 0) {
            start_response(msg, sizeof (msg->Rwalk));
            msg->Rwalk.nwqid = 0;
            add_fid(msg->Twalk.newfid, entry);
        } else {
            std::string path = entry->path;
            const char* wname = msg->Twalk.wname;
            uint16_t num_entries = 0;
            Entry* entries[MAXWELEM];

            CHECK(msg->Twalk.nwname <= MAXWELEM, EBADMSG_TEXT);

            for (uint16_t i = 0; i < msg->Twalk.nwname; ++i) {
                CHECK(wname + 2 <= end, EBADMSG_TEXT);
                uint16_t len = *wname++;
                len |= *wname++ << 8;
                CHECK(wname + len <= end, EBADMSG_TEXT);
                path = join_path(path, std::string(wname, len));
                wname += len;

                DEBUG_PRINTF("Twalk path=%s\n", path.c_str());

                DIR* dir = opendir(path.c_str());
                if (dir) {
                    closedir(dir);
                    entries[num_entries++] = get_entry(QTDIR, path);
                } else {
                    FILE* fp = fopen(path.c_str(), "r");
                    if (fp) {
                        fclose(fp);
                        entries[num_entries++] = get_entry(QTFILE, path);
                    }
                    i = msg->Twalk.nwname;
                }
            }

            CHECK(num_entries > 0, ENOENT_TEXT);
            add_fid(msg->Twalk.newfid, entries[num_entries - 1]);

            start_response(msg, sizeof (msg->Rwalk));
            msg->Rwalk.nwqid = num_entries;
            Qid* wqid = msg->Rwalk.wqid;
            for (uint16_t i = 0; i < num_entries; ++i) {
                *wqid++ = entries[i];
                msg->size += sizeof (Qid);
            }
        }
        break;
    case Tstat:
        CHECK(msg->size == sizeof (Header) + 4, EBADMSG_TEXT);
        CHECK(entry = get_entry(msg->fid), FID_UNKNOWN_TEXT);

        DEBUG_PRINTF("Tstat fid=%lu %s\n", msg->fid, entry->path.c_str());

        start_response(msg, sizeof (msg->Rstat));
        CHECK((msg->Rstat.stat_size = putstat(reinterpret_cast<char*>(&(msg->Rstat.stat)), buf + uip_mss(), entry)) > 0, EFAULT_TEXT);
        msg->size = sizeof (Header) + 2 + msg->Rstat.stat_size;
        break;

    case Tclunk:
        DEBUG_PRINTF("Tclunk fid=%lu\n", msg->fid);
        CHECK(get_entry(msg->fid), FID_UNKNOWN_TEXT);
        CHECK(msg->size == sizeof (Header) + 4, EBADMSG_TEXT);
        remove_fid(msg->fid);
        start_response(msg, sizeof (Header));
        break;

    case Topen:
        CHECK(msg->size == sizeof (msg->Topen), EBADMSG_TEXT);
        CHECK(entry = get_entry(msg->fid), FID_UNKNOWN_TEXT);
        DEBUG_PRINTF("Topen fid=%lu %s\n", msg->fid, entry->path.c_str());

        if (entry->type != QTDIR && (msg->Topen.mode & OTRUNC)) {
            FILE* fp = fopen(entry->path.c_str(), "w");
            CHECK(fp, EIO_TEXT);
            fclose(fp);
        }

        start_response(msg, sizeof (msg->Ropen));
        msg->Ropen.qid = entry;
        msg->Ropen.iounit = IOUNIT;
        break;

    case Tread:
        DEBUG_PRINTF("Tread fid=%lu\n", msg->fid);
        CHECK(msg->size == sizeof (msg->Tread), EBADMSG_TEXT);
        CHECK(msg->Tread.count <= IOUNIT, EBADMSG_TEXT);
        CHECK(entry = get_entry(msg->fid), FID_UNKNOWN_TEXT);

        if (entry->type == QTDIR) {
            DIR* dir = opendir(entry->path.c_str());
            CHECK(dir, EIO_TEXT);

            auto offset = msg->Tread.offset;
            auto count = msg->Tread.count;

            start_response(msg, sizeof (msg->Rread));
            char* data = buf + sizeof (msg->Rread);
            struct dirent* ent;
            while ((ent = readdir(dir)) && count > 0) {
                auto path = join_path(entry->path, ent->d_name);
                DEBUG_PRINTF("Tread path %s\n", path.c_str());

                Entry* child = get_entry(ent->d_isdir ? QTDIR : QTFILE, path);
                char buf[sizeof (Stat) + 128];
                size_t stat_size = putstat(buf, buf + sizeof (buf), child);
                CHECK(stat_size > 0, EFAULT_TEXT);

                if (offset >= stat_size) {
                    offset -= stat_size;
                } else {
                    uint16_t size = stat_size - offset;
                    if (size > count)
                        break;
                    memcpy(data + offset, buf, size);
                    data += size;
                    msg->Rread.count += size;
                    msg->size += size;
                    offset = 0;
                    count -= size;
                }
            }
            closedir(dir);
        } else {
            FILE* fp = fopen(entry->path.c_str(), "r");
            CHECK(fp, EIO_TEXT);
            if (fseek(fp, msg->Tread.offset, SEEK_SET)) {
                fclose(fp);
                FAIL(EIO_TEXT);
            }
            uint32_t count = msg->Tread.count;
            start_response(msg, sizeof (msg->Rread));
            msg->Rread.count = fread(buf + msg->size, 1, count, fp);
            auto ok = msg->Rread.count == count || !ferror(fp);
            fclose(fp);
            CHECK(ok, EIO_TEXT);
            msg->size += msg->Rread.count;
        }
        break;

    case Tcreate:
        {
            CHECK(msg->size == sizeof (msg->Tcreate) + msg->Tcreate.name_size + 4 + 1, EBADMSG_TEXT);
            CHECK(msg->Tcreate.name + msg->Tcreate.name_size + 4 <= end, EBADMSG_TEXT);
            CHECK(entry = get_entry(msg->fid), FID_UNKNOWN_TEXT);

            auto path = join_path(entry->path, std::string(msg->Tcreate.name, msg->Tcreate.name_size));
            uint32_t perm;
            memcpy(&perm, msg->Tcreate.name + msg->Tcreate.name_size, 4);

            DEBUG_PRINTF("Tcreate fid=%lu path=%s\n", msg->fid, path.c_str());
            CHECK(!(perm & ~(DMDIR | 0777)), ENOSYS_TEXT);

            if (perm & DMDIR) {
                CHECK(!mkdir(path.c_str(), 0755), EEXIST_TEXT);
            } else {
                FILE* fp = fopen(path.c_str(), "w");
                CHECK(fp, EIO_TEXT);
                fclose(fp);
            }
            ++entry->vers;
            --entry->refcount;
            entry = get_entry((perm & DMDIR) ? QTDIR : QTFILE, path);
            fids[msg->fid] = entry;
            start_response(msg, sizeof (msg->Rcreate));
            msg->Rcreate.qid = entry;
            msg->Rcreate.iounit = IOUNIT;
        }
        break;

    case Twrite:
        {
            DEBUG_PRINTF("Twrite fid=%lu\n", msg->fid);
            CHECK(msg->size == sizeof (msg->Twrite) + msg->Twrite.count, EBADMSG_TEXT);
            CHECK(msg->Twrite.count <= IOUNIT, EBADMSG_TEXT);
            CHECK(entry = get_entry(msg->fid), FID_UNKNOWN_TEXT);

            FILE* fp = fopen(entry->path.c_str(), "r+");
            CHECK(fp, EIO_TEXT);
            if (fseek(fp, msg->Twrite.offset, SEEK_SET)) {
                fclose(fp);
                FAIL(EIO_TEXT);
            }

            uint32_t count = fwrite(buf + sizeof (msg->Twrite), 1, msg->Twrite.count, fp);
            auto ok = count == msg->Twrite.count || !ferror(fp);
            fclose(fp);
            CHECK(ok, EIO_TEXT);

            start_response(msg, sizeof (msg->Rwrite));
            msg->Rwrite.count = count;
            ++entry->vers;
        }
        break;

    case Tremove:
        {
            DEBUG_PRINTF("Tremove fid=%lu\n", msg->fid);
            CHECK(msg->size == sizeof (Header) + 4, EBADMSG_TEXT);
            CHECK(entry = get_entry(msg->fid), FID_UNKNOWN_TEXT);
            Entry e = *entry;
            remove_fid(msg->fid);
            if (e.refcount == 0)
                entries.erase(e.path);
            CHECK(!remove(e.path.c_str()), e.type == QTDIR ? ENOTEMPTY_TEXT : EIO_TEXT);
            start_response(msg, sizeof (Header));
        }
        break;

    case Twstat:
        {
            DEBUG_PRINTF("Twstat fid=%lu\n", msg->fid);
            CHECK(entry = get_entry(msg->fid), FID_UNKNOWN_TEXT);
            char* name = buf + sizeof (msg->Twstat);
            uint16_t len = *name++;
            len |= *name++ << 8;
            CHECK(name + len <= end, EBADMSG_TEXT);
            start_response(msg, sizeof (Header));
            if (len > 0 && entry->path != "/") {
                std::string newpath = join_path(entry->path.substr(0, entry->path.rfind('/')), std::string(name, len));
                if (newpath != entry->path) {
                    CHECK(!rename(entry->path.c_str(), newpath.c_str()), EIO_TEXT);
                    Entry* newentry = get_entry(entry->type, newpath);
                    remove_fid(msg->fid);
                    if (entry->refcount == 0)
                        entries.erase(entry->path);
                    add_fid(msg->fid, newentry);
                }
            }
        }
        break;

    // not implemented
    // case Tauth:
    //     start_response(msg);
    //     msg->Rauth.aqid.type = 0;
    //     msg->Rauth.aqid.vers = 0;
    //     msg->Rauth.aqid.path = 1;
    //     break;

    default:
        DEBUG_PRINTF("Unknown message %u\n", msg->type);
        return send_error(ENOSYS_TEXT);
    }

    uip_send(msg, msg->size);
}
