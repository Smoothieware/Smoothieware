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
#include "uip.h"

//#define DEBUG_PRINTF(...) printf("9p " __VA_ARGS__)
#define DEBUG_PRINTF(...)

#define ERROR(...)       do { error(bufout, msize, __LINE__, ##__VA_ARGS__); return 0; } while (0)
#define CHECK(cond, ...) do { if (!(cond)) ERROR(__VA_ARGS__); } while (0)
#define IOUNIT           (msize - sizeof (Message::Twrite))
#define PACKEDSTRUCT     struct __attribute__ ((packed))
#define RESPONSE(t)      response->size = sizeof (response->t); response->type = request->type+1; response->tag = request->tag

namespace {

// See error mapping http://lxr.free-electrons.com/source/net/9p/error.c
const char ENOENT[]      = "No such file or directory",
           EIO[]         = "Input/output error",
           FID_UNKNOWN[] = "fid unknown or out of range",
           FID_IN_USE[]  = "fid already in use",
           EBADMSG[]     = "Bad message",
           EEXIST[]      = "File exists",
           EFAULT[]      = "Bad address",
           ENOSYS[]      = "Function not implemented",
           ENOTEMPTY[]   = "Directory not empty",
           ENFILE[]      = "Too many open files";

enum {
    // 9P message types
    Tversion   = 100,
    Tauth      = 102,
    Tattach    = 104,
    Rerror     = 107,
    Tflush     = 108,
    Twalk      = 110,
    Topen      = 112,
    Tcreate    = 114,
    Tread      = 116,
    Twrite     = 118,
    Tclunk     = 120,
    Tremove    = 122,
    Tstat      = 124,
    Twstat     = 126,

    // Qid type
    QTDIR       = 0x80, // directories
    QTAPPEND    = 0x40, // append only files
    QTEXCL      = 0x20, // exclusive use files
    QTMOUNT     = 0x10, // mounted channel
    QTAUTH      = 0x08, // authentication file
    QTTMP       = 0x04, // non-backed-up file
    QTLINK      = 0x02, // symbolic link
    QTFILE      = 0x00, // plain file

    // mode
    OREAD       = 0,	// open for read
    OWRITE      = 1,	// write
    ORDWR       = 2,	// read and write
    OEXEC       = 3,      // execute, == read but check execute permission
    OTRUNC      = 0x10,	// or'ed in (except for exec), truncate file first
    ORCLOSE     = 0x40, 	// or'ed in, remove on close

    // permission bits
    DMDIR       = 0x80000000, // directories
    DMAPPEND    = 0x40000000, // append only files
    DMEXCL      = 0x20000000, // exclusive use files
    DMMOUNT     = 0x10000000, // mounted channel
    DMAUTH      = 0x08000000, // authentication file
    DMTMP       = 0x04000000, // non-backed-up file

    MAXWELEM    = 16,
    MAXENTRIES  = 32,
    MAXFIDS     = 32,
    MAXREQUESTS = 4,
};

// TODO: Maybe this should be moved to utils?
class File {
    FILE* fp;
public:
    File(const std::string& path, const char* mode)
        : fp(fopen(path.c_str(), mode)) {}

    ~File()
    {
        if (fp)
            fclose(fp);
    }

    operator FILE*() { return fp; }
};

// TODO: Maybe this should be moved to utils?
class Dir {
    DIR* dir;
public:
    Dir(const std::string& path)
        : dir(opendir(path.c_str())) {}

    ~Dir()
    {
        if (dir)
            closedir(dir);
    }

    operator DIR*() { return dir; }
};

PACKEDSTRUCT Header {
    uint32_t size;
    uint8_t  type;
    uint16_t tag;
};

// TODO: move to utils?
uint64_t fletcher64(const std::string& s)
{
    uint32_t lo = 0, hi = 0;
    for (const char* p = s.c_str(); p < s.c_str() + s.size();) {
        uint32_t v = 0;
        for (int i = 0; i < 4 && p < s.c_str() + s.size(); ++i)
            v = (v << 8) | *p++;
        lo += v;
        hi += lo;
    }
    return (uint64_t(hi) << 32) | lo;
}

PACKEDSTRUCT Qid {
    uint8_t  type;
    uint32_t vers; // we don't use the version field
    uint64_t path;

    Qid() {}
    Qid(uint8_t t, const std::string& p)
            : type(t), vers(0), path(fletcher64(p)) {}
    Qid(Plan9::Entry e)
            : type(e->second.type), vers(0), path(fletcher64(e->first)) {}
};

PACKEDSTRUCT Stat {
    char     buf[0];
    uint16_t size;
    uint16_t type;
    uint32_t dev;
    Qid      qid;
    uint32_t mode;
    uint32_t atime;
    uint32_t mtime;
    uint64_t length;
};

} // anonymous namespace

// Important: 9P assumes little endian byte ordering!
union __attribute__ ((packed)) Plan9::Message {
    char buf[0];

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
    // size[4] Tremove tag[2] fid[4]
    // size[4] Tstat tag[2] fid[4]
    // size[4] Tattach tag[2] fid[4] afid[4] uname[s] aname[s]

    // size[4] Rclunk tag[2]
    // size[4] Rflush tag[2]
    // size[4] Rremove tag[2]
    // size[4] Rwstat tag[2]
    Header Rclunk, Rflush, Rremove, Rwstat;

    // size[4] Rattach tag[2] qid[13]
    PACKEDSTRUCT {
        Header   _header;
        Qid      qid;
    } Rattach;

    // size[4] Tflush tag[2] oldtag[2]
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

    // size[4] Twstat tag[2] fid[4] stat[n]
    PACKEDSTRUCT {
        Header   _header;
        uint32_t fid;
        uint16_t stat_size;
        Stat     stat;
    } Twstat;
};

namespace {

char* putstr(char* p, char* end, const char* s)
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
    File fp(path, "r");
    if (!fp)
        return 0;
    fseek(fp, 0, SEEK_END);
    return max(0l, ftell(fp));
}

size_t putstat(Stat* stat, char* end, uint8_t type, const std::string& path)
{
    char* p = stat->buf + sizeof (Stat);
    if (p > end)
        return 0;

    stat->type = 0;
    stat->dev = 0;
    stat->qid = Qid(type, path);
    stat->mode = type == QTDIR ? (DMDIR | 0755) : 0644;
    stat->atime = stat->mtime = 1423420000;
    stat->length = (stat->mode & DMDIR) ? 0 : flen(path);

    p = putstr(p, end, path == "/" ? "/" : path.substr(path.rfind('/') + 1).c_str());
    p = putstr(p, end, "smoothie");
    p = putstr(p, end, "smoothie");
    p = putstr(p, end, "smoothie");
    if (!p) return 0;

    stat->size = p - stat->buf - 2;
    return p - stat->buf;
}

inline void error(char* bufout, uint32_t msize, int line) {}
void error(char* bufout, uint32_t msize, int line, const char* text)
{
    DEBUG_PRINTF("error %s, line %d\n", text, __LINE__);
    Plan9::Message* response = reinterpret_cast<Plan9::Message*>(bufout);
    response->type = Rerror;
    response->size = sizeof (Header);
    response->size = putstr(response->buf + sizeof (Header), response->buf + msize, text) - response->buf;
}

// TODO: Move to utils
inline std::string absolute_path(const std::string& path)
{
    // TODO: remove /../ and /./ from paths
    return path;
}

// TODO: Move to utils
std::string join_path(const std::string& a, const std::string& b)
{
    return a.back() != '/' ? absolute_path(a + "/" + b) :
            absolute_path(a + b);
}

} // anonymous namespace

Plan9::Plan9()
: msize(INITIAL_MSIZE), queue_bytes(0)
{
    PSOCK_INIT(&sin, bufin + 4, sizeof(bufin) - 4);
    PSOCK_INIT(&sout, bufout + 4, sizeof(bufout) - 4);
}

Plan9::~Plan9()
{
    PSOCK_CLOSE(&sin);
    PSOCK_CLOSE(&sout);
}

Plan9::Entry Plan9::add_entry(uint32_t fid, uint8_t type, const std::string& path)
{
    CHECK(fids.find(fid) == fids.end(), FID_IN_USE);
    CHECK(fids.size() < MAXFIDS, ENFILE);
    std::string abspath = absolute_path(path);
    auto i = entries.find(abspath);
    if (i == entries.end()) {
        CHECK(entries.size() < MAXENTRIES, ENFILE);
        entries[abspath] = EntryData(type);
        i = entries.find(abspath);
    }
    Entry entry = &(*i);
    CHECK(add_fid(fid, entry));
    return entry;
}

Plan9::Entry Plan9::get_entry(uint32_t fid)
{
    auto i = fids.find(fid);
    CHECK(i != fids.end());
    return i->second;
}

bool Plan9::add_fid(uint32_t fid, Entry entry)
{
    CHECK(fids.find(fid) == fids.end(), FID_IN_USE);
    CHECK(fids.size() < MAXFIDS, ENFILE);
    fids[fid] = entry;
    ++entry->second.refcount;
    return true;
}

void Plan9::remove_fid(uint32_t fid)
{
    auto i = fids.find(fid);
    if (i != fids.end()) {
        fids.erase(i);
        --i->second->second.refcount;
        if (i->second->second.refcount == 0)
            entries.erase(i->second->first);
    }
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

    instance->receive();
    instance->send();
}

int Plan9::receive()
{
    Message *request = reinterpret_cast<Message*>(bufin);

    PSOCK_BEGIN(&sin);
    (void)PT_YIELD_FLAG; // avoid warning unused variable

    for (;;) {
        DEBUG_PRINTF("receive thread fids=%d entries=%d queue_size=%d queue_bytes=%lu\n", fids.size(), entries.size(), queue.size(), queue_bytes);

        PSOCK_READBUF_LEN(&sin, 4);
        memcpy(request, request->buf + 4, 4); // copy size to buffer beginning

        DEBUG_PRINTF("receive size=%lu\n", request->size);
        if (request->size > msize) {
            DEBUG_PRINTF("Bad message received %lu\n", request->size);
            PSOCK_CLOSE_EXIT(&sin);
        } else {
            PSOCK_READBUF_LEN(&sin, request->size - 4);
            DEBUG_PRINTF("receive size=%lu type=%u tag=%d\n", request->size, request->type, request->tag);
        }

        PSOCK_WAIT_UNTIL(&sin, queue_bytes < MAXREQUESTS * INITIAL_MSIZE);

        Message* copy = reinterpret_cast<Message*>(new char[request->size]);
        memcpy(copy, request, request->size);
        queue.push(copy);
        queue_bytes += copy->size;

        // store message
        DEBUG_PRINTF("store size=%lu type=%u tag=%d queue_size=%d queue_bytes=%lu\n", request->size, request->type, request->tag, queue.size(), queue_bytes);
    }

    PSOCK_END(&sin);
}

int Plan9::send()
{
    Message* response = reinterpret_cast<Message*>(bufout);

    PSOCK_BEGIN(&sout);
    (void)PT_YIELD_FLAG; // avoid warning unused variable

    for (;;) {
        DEBUG_PRINTF("send thread fids=%d entries=%d queue_size=%d queue_bytes=%lu\n", fids.size(), entries.size(), queue.size(), queue_bytes);

        PSOCK_WAIT_UNTIL(&sout, !queue.empty());

        {
            Message* request = queue.front();
            queue.pop();
            queue_bytes -= request->size;
            process(request, response);
            char* buf = request->buf;
            delete[] buf;
        }


        DEBUG_PRINTF("send size=%lu type=%u tag=%d\n", response->size, response->type, response->tag);
        PSOCK_SEND(&sout, response->buf, response->size);
    }

    PSOCK_END(&sout);
}


bool Plan9::process(Message* request, Message* response)
{
    Entry entry;

    switch (request->type) {
    case Tversion:
        DEBUG_PRINTF("Tversion\n");
        RESPONSE(Rversion);
        msize = response->Rversion.msize = min(INITIAL_MSIZE, request->Tversion.msize);
        response->size = putstr(response->buf + response->size, response->buf + msize, "9P2000") - response->buf;
        break;

    case Tattach:
        DEBUG_PRINTF("Tattach\n");
        CHECK(entry = add_entry(request->fid, QTDIR, "/"));
        RESPONSE(Rattach);
        response->Rattach.qid = entry;
        break;

    case Tflush:
        DEBUG_PRINTF("Tflush\n");
        CHECK(request->size == sizeof (request->Tflush), EBADMSG);
        RESPONSE(Rflush);
        // do nothing
        break;

    case Twalk:
        DEBUG_PRINTF("Twalk fid=%lu newfid=%lu nwname=%u\n", request->Twalk.fid, request->Twalk.newfid, request->Twalk.nwname);
        CHECK(entry = get_entry(request->fid));
        CHECK(request->Twalk.nwname <= MAXWELEM, EBADMSG);

        RESPONSE(Rwalk);
        response->Rwalk.nwqid = 0;

        if (request->Twalk.nwname == 0) {
            CHECK(add_fid(request->Twalk.newfid, entry));
        } else {
            std::string path = entry->first;
            const char* wname = request->Twalk.wname;
            size_t last_path_size = 0;
            Qid* wqid = response->Rwalk.wqid;

            for (uint16_t i = 0; i < request->Twalk.nwname; ++i) {
                CHECK(wname + 2 <= request->buf + request->size, EBADMSG);
                uint16_t len = *wname++;
                len |= *wname++ << 8;
                CHECK(wname + len <= request->buf + request->size, EBADMSG);
                path = join_path(path, std::string(wname, len));
                wname += len;

                DEBUG_PRINTF("Twalk path=%s\n", path.c_str());

                if (Dir(path)) {
                    *wqid++ = Qid(QTDIR, path);
                    ++response->Rwalk.nwqid;
                    last_path_size = path.size();
                } else {
                    if (File(path, "r")) {
                        *wqid++ = Qid(QTFILE, path);
                        ++response->Rwalk.nwqid;
                        last_path_size = path.size();
                    }
                    i = request->Twalk.nwname;
                }
            }

            CHECK(response->Rwalk.nwqid > 0, ENOENT);
            response->size += sizeof (Qid) * response->Rwalk.nwqid;
            CHECK(add_entry(request->Twalk.newfid, wqid[-1].type, path.substr(0, last_path_size)));
        }
        break;

    case Tstat:
        CHECK(request->size == sizeof (Header) + 4, EBADMSG);
        CHECK(entry = get_entry(request->fid));

        DEBUG_PRINTF("Tstat fid=%lu %s\n", request->fid, entry->first.c_str());

        RESPONSE(Rstat);
        CHECK((response->Rstat.stat_size = putstat(&response->Rstat.stat, response->buf + msize, entry->second.type, entry->first)) > 0, EFAULT);
        response->size = sizeof (Header) + 2 + response->Rstat.stat_size;
        break;

    case Tclunk:
        DEBUG_PRINTF("Tclunk fid=%lu\n", request->fid);
        CHECK(request->size == sizeof (Header) + 4, EBADMSG);
        remove_fid(request->fid);
        RESPONSE(Rclunk);
        break;

    case Topen:
        CHECK(request->size == sizeof (request->Topen), EBADMSG);
        CHECK(entry = get_entry(request->fid));
        DEBUG_PRINTF("Topen fid=%lu %s\n", request->fid, entry->first.c_str());

        if (entry->second.type != QTDIR && (request->Topen.mode & OTRUNC))
            CHECK(File(entry->first, "w"), EIO);

        RESPONSE(Ropen);
        response->Ropen.qid = entry;
        response->Ropen.iounit = IOUNIT;
        break;

    case Tread:
        DEBUG_PRINTF("Tread fid=%lu\n", request->fid);
        CHECK(request->size == sizeof (request->Tread) && request->Tread.count <= IOUNIT, EBADMSG);
        CHECK(entry = get_entry(request->fid));
        RESPONSE(Rread);

        if (entry->second.type == QTDIR) {
            Dir dir(entry->first);
            CHECK(dir, EIO);

            char* data = response->buf + sizeof (response->Rread);
            struct dirent* d;
            while ((d = readdir(dir)) && request->Tread.count > 0) {
                auto path = join_path(entry->first, d->d_name);
                DEBUG_PRINTF("Tread path %s\n", path.c_str());

                char stat_buf[sizeof (Stat) + 128];
                size_t stat_size = putstat(reinterpret_cast<Stat*>(stat_buf), stat_buf + sizeof (stat_buf), d->d_isdir ? QTDIR : QTFILE, path);
                CHECK(stat_size > 0, EFAULT);

                if (request->Tread.offset >= stat_size) {
                    request->Tread.offset -= stat_size;
                } else {
                    CHECK(request->Tread.offset == 0, EBADMSG);
                    if (stat_size > request->Tread.count) {
                        request->Tread.count = 0;
                    } else {
                        memcpy(data, stat_buf, stat_size);
                        data += stat_size;
                        response->Rread.count += stat_size;
                        response->size += stat_size;
                        request->Tread.count -= stat_size;
                    }
                }
            }
        } else {
            File fp(entry->first, "r");
            CHECK(fp, EIO);
            CHECK(!fseek(fp, request->Tread.offset, SEEK_SET), EIO);
            response->Rread.count = fread(response->buf + response->size, 1, request->Tread.count, fp);
            CHECK(response->Rread.count == request->Tread.count || !ferror(fp), EIO);
            response->size += response->Rread.count;
        }
        break;

    case Tcreate:
        {
            CHECK(request->size == sizeof (request->Tcreate) + request->Tcreate.name_size + 4 + 1 &&
                  request->Tcreate.name + request->Tcreate.name_size + 4 <= request->buf + request->size, EBADMSG);
            CHECK(entry = get_entry(request->fid));

            auto path = join_path(entry->first, std::string(request->Tcreate.name, request->Tcreate.name_size));
            uint32_t perm;
            memcpy(&perm, request->Tcreate.name + request->Tcreate.name_size, 4);

            DEBUG_PRINTF("Tcreate fid=%lu path=%s\n", request->fid, path.c_str());
            CHECK(!(perm & ~(DMDIR | 0777)), ENOSYS);

            if (perm & DMDIR)
                CHECK(!mkdir(path.c_str(), 0755), EEXIST);
            else
                CHECK(File(path, "w"), EIO);
            remove_fid(request->fid);
            CHECK(entry = add_entry(request->fid, (perm & DMDIR) ? QTDIR : QTFILE, path));
            RESPONSE(Rcreate);
            response->Rcreate.qid = entry;
            response->Rcreate.iounit = IOUNIT;
        }
        break;

    case Twrite:
        {
            DEBUG_PRINTF("Twrite fid=%lu\n", request->fid);
            CHECK(request->size == sizeof (request->Twrite) + request->Twrite.count &&
                  request->Twrite.count <= IOUNIT, EBADMSG);
            CHECK(entry = get_entry(request->fid));

            File fp(entry->first, "r+");
            CHECK(fp, EIO);
            CHECK(!fseek(fp, request->Twrite.offset, SEEK_SET), EIO);

            RESPONSE(Rwrite);
            response->Rwrite.count = fwrite(request->buf + sizeof (request->Twrite), 1, request->Twrite.count, fp);
            CHECK(response->Rwrite.count == request->Twrite.count || !ferror(fp), EIO);
        }
        break;

    case Tremove:
        {
            DEBUG_PRINTF("Tremove fid=%lu\n", request->fid);
            CHECK(request->size == sizeof (Header) + 4, EBADMSG);
            CHECK(entry = get_entry(request->fid));
            auto e = *entry;
            remove_fid(request->fid);
            CHECK(!remove(e.first.c_str()), e.second.type == QTDIR ? ENOTEMPTY : EIO);
            RESPONSE(Rremove);
        }
        break;

    case Twstat:
        {
            DEBUG_PRINTF("Twstat fid=%lu\n", request->fid);
            CHECK(entry = get_entry(request->fid));
            char* name = request->buf + sizeof (request->Twstat);
            uint16_t len = *name++;
            len |= *name++ << 8;
            CHECK(name + len <= request->buf + request->size, EBADMSG);
            RESPONSE(Rwstat);
            if (len > 0 && entry->first != "/") {
                std::string newpath = join_path(entry->first.substr(0, entry->first.rfind('/')), std::string(name, len));
                if (newpath != entry->first) {
                    CHECK(!rename(entry->first.c_str(), newpath.c_str()), EIO);
                    uint8_t type = entry->second.type;
                    remove_fid(request->fid);
                    CHECK(add_entry(request->fid, type, newpath));
                }
            }
        }
        break;

    // not implemented
    // case Tauth:
    //     response(msg);
    //     msg->Rauth.aqid.type = 0;
    //     msg->Rauth.aqid.vers = 0;
    //     msg->Rauth.aqid.path = 1;
    //     break;

    default:
        DEBUG_PRINTF("Unknown message %u\n", request->type);
        ERROR(ENOSYS);
    }
    return true;
}
