// wiikit runtime — IOS, the Starlet's operating system, in high-level emulation.
//
// The SDK talks to IOS through the IPC registers at 0xCD000000 exactly as on
// hardware, so its IOS client (IOS_Open, IOS_Ioctlv... and the DVD, ISFS,
// ES, STM libraries above it) runs recompiled. Here is the other side, as in
// Dolphin: a request is the physical address of a 0x20-byte block
//   +0 command (1 open, 2 close, 3 read, 4 write, 5 seek, 6 ioctl, 7 ioctlv)
//   +4 result  +8 fd  +0xC.. arguments
// sent with PPCMSG and X1; IOS acknowledges it (Y2), carries it out, and
// replies (Y1, ARMMSG = the block) after writing 8 at +0, the result at +4
// and the original command at +8. Each of Y1/Y2 raises the Hollywood IPC
// interrupt when its enable (IY1/IY2) is set. A request may stay pending
// (STM's event hook) and be answered later.
//
// Devices so far: /dev/di (the disc, from the extracted tree), /dev/fs and
// file paths (NAND, on a host folder), /dev/es (title identity, ticket and
// TMD views), /dev/stm/immediate and /dev/stm/eventhook. Anything else fails
// to open, and every call a device does not know is logged.
#include "disc.h"
#include "rt.h"
#include <deque>
#include <filesystem>
#include <map>
#include <memory>
#include <vector>

namespace fs = std::filesystem;

namespace {

constexpr uint32_t IRQ_IPC = 0x40000000;             // Hollywood PPC IRQ: IPC from Starlet
constexpr int32_t PENDING = INT32_MIN;
constexpr int32_t IPC_EINVAL = -4, IPC_ENOENT = -6, FS_EEXIST = -105, FS_ENOENT = -106,
                  ES_EINVAL = -1017;

// ---- the registers ----------------------------------------------------------------------------
struct Ctrl {
    bool x1 = false, x2 = false, y1 = false, y2 = false, iy1 = false, iy2 = false;
    uint32_t ppc() const { return iy2 << 5 | iy1 << 4 | x2 << 3 | y1 << 2 | y2 << 1 | (uint32_t)x1; }
} ctrl;
uint32_t ppc_msg = 0, arm_msg = 0, irq_flags = 0, irq_mask = IRQ_IPC;
std::deque<uint32_t> requests, replies;

void update_irq() {
    if ((ctrl.y1 && ctrl.iy1) || (ctrl.y2 && ctrl.iy2)) irq_flags |= IRQ_IPC;
    os_raise();
}

// ---- guest memory helpers ---------------------------------------------------------------------
struct Vec { uint32_t addr, size; };
uint32_t rd32(uint32_t a) { return ld32(a); }
void wr32(uint32_t a, uint32_t v) { st32(a, v); }
void wr64(uint32_t a, uint64_t v) { st64(a, v); }
void fill(uint32_t a, const void* src, size_t n) { std::memcpy(host(a), src, n); }
void zero(uint32_t a, size_t n) { std::memset(host(a), 0, n); }

// ---- devices ----------------------------------------------------------------------------------
struct Device {
    std::string name;
    virtual ~Device() = default;
    virtual int32_t open(uint32_t /*mode*/) { return 0; }
    virtual void close() {}
    virtual int32_t read(uint32_t, uint32_t) { return IPC_EINVAL; }
    virtual int32_t write(uint32_t, uint32_t) { return IPC_EINVAL; }
    virtual int32_t seek(int32_t, uint32_t) { return IPC_EINVAL; }
    virtual int32_t ioctl(uint32_t req, uint32_t in, uint32_t in_len, uint32_t out, uint32_t out_len, uint32_t addr) {
        rt_log("ios: %s ioctl %X (in %X/%u, out %X/%u) unknown", name.c_str(), req, in, in_len, out, out_len);
        (void)addr;
        return IPC_EINVAL;
    }
    virtual int32_t ioctlv(uint32_t req, std::vector<Vec>& in, std::vector<Vec>& io, uint32_t addr) {
        rt_log("ios: %s ioctlv %X (%zu in, %zu out) unknown", name.c_str(), req, in.size(), io.size());
        (void)addr;
        return IPC_EINVAL;
    }
};

std::string g_nand;                                   // host folder holding the NAND
std::vector<uint8_t> g_ticket, g_tmd;
uint64_t g_title = 0;

void reply(uint32_t addr, int32_t result);

// /dev/di: the disc drive. Results: 1 = success, 2 = drive error, whose code
// RequestError then returns. The SDK's __DVDCheckDevice relies on the errors:
// a raw read past the end of the disc must fail with 0x052100 (LBA out of
// range) and ReportKey, a DVD-video command, with 0x053100; a drive that
// obeys either is an "unauthorized device".
struct DI : Device {
    uint32_t error = 0;
    int32_t fail(uint32_t code) { error = code; return 2; }
    int32_t ioctl(uint32_t req, uint32_t in, uint32_t, uint32_t out, uint32_t out_len, uint32_t) override {
        uint32_t cmd = rd32(in) >> 24;
        switch (cmd) {
        case 0x8D: {                                  // UnencryptedRead: the disc header area only
            uint32_t len = rd32(in + 4);
            uint64_t off = (uint64_t)rd32(in + 8) << 2;
            if (off + len > 0x50000) return fail(0x00052100);
            zero(out, out_len);
            if (off < 0x100) fill(out, disc_boot().data() + off, std::min<size_t>(0x100 - off, len));
            return 1;
        }
        case 0xA4: return fail(0x00053100);           // ReportKey
        case 0xE0:                                    // RequestError
            wr32(out, error);
            error = 0;
            return 1;
        case 0x12:                                    // Inquiry
            zero(out, out_len);
            wr32(out, 0x00000002); wr32(out + 4, 0x20060526);
            return 1;
        case 0x70:                                    // ReadDiskID
            fill(out, disc_boot().data(), 0x20);
            return 1;
        case 0x71: {                                  // Read: length, offset >> 2
            uint32_t len = rd32(in + 4);
            uint64_t off = (uint64_t)rd32(in + 8) << 2;
            if (len > out_len) len = out_len;
            disc_read(off, host(out), len);
            return 1;
        }
        case 0x7A: case 0x95: case 0x96:              // cover, status, control registers
            wr32(out, 0);
            return 1;
        case 0x88:                                    // GetCoverStatus: disc in, cover closed
            wr32(out, 2);
            return 1;
        case 0x86: case 0x8E: case 0xAB: case 0xE3: case 0xE4: case 0xDB:
            return 1;                                 // cover irq, DVD video, seek, stop, audio, status
        }
        rt_log("ios: /dev/di command %02X unknown", cmd);
        return 2;
    }
    int32_t ioctlv(uint32_t req, std::vector<Vec>& in, std::vector<Vec>& io, uint32_t addr) override {
        if (req == 0x8B && !io.empty()) {             // OpenPartition: the partition is already open
            fill(io[0].addr, g_tmd.data(), std::min<size_t>(g_tmd.size(), io[0].size));
            if (io.size() > 1) wr32(io[1].addr, 0);
            return 1;
        }
        return Device::ioctlv(req, in, io, addr);
    }
};

// /dev/fs and the files it opens
std::string host_path(const std::string& guest) { return g_nand + guest; }

struct NandFile : Device {
    FILE* f = nullptr;
    uint32_t mode = 0;
    int32_t open(uint32_t m) override {
        mode = m;
        std::string p = host_path(name);
        if (!fs::is_regular_file(p)) return FS_ENOENT;
        f = std::fopen(p.c_str(), (m & 2) ? "r+b" : "rb");
        return f ? 0 : FS_ENOENT;
    }
    void close() override { if (f) std::fclose(f); f = nullptr; }
    int32_t read(uint32_t buf, uint32_t len) override {
        return (int32_t)std::fread(host(buf), 1, len, f);
    }
    int32_t write(uint32_t buf, uint32_t len) override {
        int32_t n = (int32_t)std::fwrite(host(buf), 1, len, f);
        std::fflush(f);
        return n;
    }
    int32_t seek(int32_t off, uint32_t whence) override {
        if (std::fseek(f, off, whence == 0 ? SEEK_SET : whence == 1 ? SEEK_CUR : SEEK_END)) return -101;
        return (int32_t)std::ftell(f);
    }
    int32_t ioctl(uint32_t req, uint32_t in, uint32_t in_len, uint32_t out, uint32_t out_len, uint32_t a) override {
        if (req == 0x0B) {                            // GetFileStats: size, position
            long pos = std::ftell(f);
            std::fseek(f, 0, SEEK_END);
            long size = std::ftell(f);
            std::fseek(f, pos, SEEK_SET);
            wr32(out, (uint32_t)size);
            wr32(out + 4, (uint32_t)pos);
            return 0;
        }
        return Device::ioctl(req, in, in_len, out, out_len, a);
    }
};

struct FS : Device {
    // the attribute block of CreateDir/CreateFile/SetAttr: owner, group, path at +6
    static std::string path_at(uint32_t a) { return guest_cstr(a, 64); }
    int32_t ioctl(uint32_t req, uint32_t in, uint32_t in_len, uint32_t out, uint32_t out_len, uint32_t a) override {
        switch (req) {
        case 0x01: return 0;                          // Format
        case 0x02:                                    // GetStats
            zero(out, out_len);
            wr32(out, 0x4000); wr32(out + 4, 0x5DEC); wr32(out + 8, 0x1DD4); wr32(out + 0x14, 0x146B);
            return 0;
        case 0x03: case 0x09: {                       // CreateDir, CreateFile
            std::string p = host_path(guest_cstr(in + 6, 64));
            if (fs::exists(p)) return FS_EEXIST;
            if (!fs::exists(fs::path(p).parent_path())) return FS_ENOENT;
            if (req == 0x03) fs::create_directory(p);
            else std::fclose(std::fopen(p.c_str(), "wb"));
            return 0;
        }
        case 0x05: return 0;                          // SetAttr
        case 0x06: {                                  // GetAttr: in = path; out = the block
            std::string g = path_at(in), p = host_path(g);
            if (!fs::exists(p)) return FS_ENOENT;
            zero(out, out_len);
            wr32(out, 0x1000); st16(out + 4, 1);
            fill(out + 6, g.c_str(), std::min<size_t>(g.size(), 63));
            st8(out + 0x46, 3); st8(out + 0x47, 3); st8(out + 0x48, 1);
            return 0;
        }
        case 0x07: {                                  // Delete
            std::string p = host_path(path_at(in));
            if (!fs::exists(p)) return FS_ENOENT;
            std::error_code ec;
            fs::remove_all(p, ec);
            return 0;
        }
        case 0x08: {                                  // Rename: two 64-byte paths
            std::string from = host_path(path_at(in)), to = host_path(path_at(in + 64));
            if (!fs::exists(from)) return FS_ENOENT;
            std::error_code ec;
            fs::remove_all(to, ec);
            fs::rename(from, to, ec);
            return ec ? -101 : 0;
        }
        case 0x0D: return 0;                          // Shutdown
        }
        return Device::ioctl(req, in, in_len, out, out_len, a);
    }
    int32_t ioctlv(uint32_t req, std::vector<Vec>& in, std::vector<Vec>& io, uint32_t a) override {
        if (req == 0x04 && !in.empty()) {             // ReadDir: count, or names
            std::string p = host_path(guest_cstr(in[0].addr, 64));
            if (!fs::is_directory(p)) return FS_ENOENT;
            std::vector<std::string> names;
            for (auto& e : fs::directory_iterator(p)) names.push_back(e.path().filename().string());
            if (in.size() == 1) {                     // count only
                wr32(io[0].addr, (uint32_t)names.size());
                return 0;
            }
            uint32_t max = rd32(in[1].addr), n = 0, o = io[0].addr;
            for (auto& nm : names) {
                if (n == max) break;
                fill(o, nm.c_str(), nm.size() + 1);
                o += (uint32_t)nm.size() + 1;
                ++n;
            }
            wr32(io[1].addr, n);
            return 0;
        }
        if (req == 0x0C && !in.empty()) {             // GetUsage: blocks, inodes
            wr32(io[0].addr, 0);
            wr32(io[1].addr, 1);
            return 0;
        }
        return Device::ioctlv(req, in, io, a);
    }
};

// /dev/es: the title's identity, its ticket and TMD
struct ES : Device {
    int32_t ioctlv(uint32_t req, std::vector<Vec>& in, std::vector<Vec>& io, uint32_t a) override {
        switch (req) {
        case 0x07:                                    // GetDeviceID
            wr32(io[0].addr, 0x0403AC68);
            return 0;
        case 0x0E: case 0x0C:                         // Get(Owned)TitleCount
            wr32(io[0].addr, 1);
            return 0;
        case 0x0F: case 0x0D:                         // Get(Owned)Titles
            wr64(io[0].addr, g_title);
            return 0;
        case 0x12:                                    // GetTicketViewCount
            wr32(io[0].addr, 1);
            return 0;
        case 0x13: case 0x1B:                         // GetTicketViews, DIGetTicketView
            zero(io[0].addr, 0xD8);
            fill(io[0].addr + 4, g_ticket.data() + 0x1D0, 0xD4);
            return 0;
        case 0x1D: {                                  // GetTitleDir
            char buf[32];
            std::snprintf(buf, sizeof buf, "/title/%08x/%08x/data", (uint32_t)(ld64(in[0].addr) >> 32),
                          (uint32_t)ld64(in[0].addr));
            fill(io[0].addr, buf, std::strlen(buf) + 1);
            return 0;
        }
        case 0x20:                                    // GetTitleID
            wr64(io[0].addr, g_title);
            return 0;
        case 0x16:                                    // GetConsumption: no limits
            if (io.size() > 1) wr32(io[1].addr, 0);
            return 0;
        case 0x2E:                                    // GetBoot2Version
            wr32(io[0].addr, 4);
            return 0;
        case 0x34: case 0x39:                         // Get(Stored/DI)TMDSize
            wr32(io[0].addr, (uint32_t)g_tmd.size());
            return 0;
        case 0x35: case 0x3A:                         // Get(Stored/DI)TMD
            fill(io[0].addr, g_tmd.data(), std::min<size_t>(g_tmd.size(), io[0].size));
            return 0;
        case 0x08: case 0x25:                         // LaunchTitle
            rt_log("ios: the game launches another title: leaving");
            std::fflush(stdout);
            std::_Exit(0);
        }
        Device::ioctlv(req, in, io, a);
        return ES_EINVAL;
    }
};

// /dev/stm: power and reset events
uint32_t g_eventhook = 0;                             // the pending event-hook request
struct STMImmediate : Device {
    int32_t ioctl(uint32_t req, uint32_t, uint32_t, uint32_t out, uint32_t out_len, uint32_t) override {
        if (req == 0x3002 && g_eventhook) {           // release the event hook
            wr32(g_eventhook + 4, 0);
            reply(g_eventhook, 0);
            g_eventhook = 0;
        }
        if (out && out_len) zero(out, out_len);
        return 0;                                     // VI dimming, LEDs, idle mode...: accepted
    }
};
struct STMEventHook : Device {
    int32_t ioctl(uint32_t req, uint32_t, uint32_t, uint32_t, uint32_t, uint32_t addr) override {
        if (req != 0x1000) return IPC_EINVAL;
        g_eventhook = addr;                           // answered on reset/power, or on release
        return PENDING;
    }
};

std::unique_ptr<Device> make_device(const std::string& path) {
    std::unique_ptr<Device> d;
    if (path == "/dev/di") d = std::make_unique<DI>();
    else if (path == "/dev/fs") d = std::make_unique<FS>();
    else if (path == "/dev/es") d = std::make_unique<ES>();
    else if (path == "/dev/stm/immediate") d = std::make_unique<STMImmediate>();
    else if (path == "/dev/stm/eventhook") d = std::make_unique<STMEventHook>();
    else if (!path.empty() && path[0] == '/' && path.rfind("/dev/", 0) != 0) d = std::make_unique<NandFile>();
    if (d) d->name = path;
    return d;
}

std::unique_ptr<Device> g_fds[32];

// ---- requests ---------------------------------------------------------------------------------
void reply(uint32_t addr, int32_t result) {
    uint32_t cmd = rd32(addr);
    wr32(addr, 8);
    wr32(addr + 4, (uint32_t)result);
    wr32(addr + 8, cmd);
    replies.push_back(addr);
}

int32_t execute(uint32_t addr) {
    uint32_t cmd = rd32(addr), fd = rd32(addr + 8);
    uint32_t a0 = rd32(addr + 0x0C), a1 = rd32(addr + 0x10), a2 = rd32(addr + 0x14), a3 = rd32(addr + 0x18),
             a4 = rd32(addr + 0x1C);
    if (cmd == 1) {                                   // open(path, mode)
        std::string path = guest_cstr(virt(a0), 64);
        auto d = make_device(path);
        if (!d) {
            rt_log("ios: open %s: no such device", path.c_str());
            return IPC_ENOENT;
        }
        int32_t r = d->open(a1);
        if (r < 0) return r;
        for (int i = 0; i < 32; ++i)
            if (!g_fds[i]) {
                g_fds[i] = std::move(d);
                if (g_mmio_log) rt_log("ios: open %s -> fd %d", path.c_str(), i);
                return i;
            }
        return -5;                                    // too many open
    }
    if (fd >= 32 || !g_fds[fd]) return IPC_EINVAL;
    Device& d = *g_fds[fd];
    switch (cmd) {
    case 2: d.close(); g_fds[fd].reset(); return 0;
    case 3: return d.read(virt(a0), a1);
    case 4: return d.write(virt(a0), a1);
    case 5: return d.seek((int32_t)a0, a1);
    case 6: return d.ioctl(a0, a1 ? virt(a1) : 0, a2, a3 ? virt(a3) : 0, a4, addr);
    case 7: {
        std::vector<Vec> in, io;
        uint32_t vec = virt(a3);
        for (uint32_t i = 0; i < a1 + a2; ++i) {
            uint32_t p = rd32(vec + 8 * i), n = rd32(vec + 8 * i + 4);
            (i < a1 ? in : io).push_back({p ? virt(p) : 0, n});
        }
        return d.ioctlv(a0, in, io, addr);
    }
    }
    rt_log("ios: command %u unknown", cmd);
    return IPC_EINVAL;
}

bool ready() { return !ctrl.y1 && !ctrl.y2 && !(irq_flags & IRQ_IPC); }

void update() {
    if (!requests.empty() && ready()) {
        uint32_t addr = virt(requests.front());
        requests.pop_front();
        ctrl.y2 = true;                               // acknowledge
        ctrl.x1 = false;
        update_irq();
        int32_t r = execute(addr);
        if (r != PENDING) reply(addr, r);
        return;
    }
    if (!replies.empty() && ready()) {
        arm_msg = replies.front() & 0x7FFFFFFFu;      // physical
        replies.pop_front();
        ctrl.y1 = true;
        update_irq();
    }
}

std::vector<uint8_t> slurp(const std::string& path) {
    std::vector<uint8_t> d;
    FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) return d;
    std::fseek(f, 0, SEEK_END);
    d.resize((size_t)std::ftell(f));
    std::fseek(f, 0, SEEK_SET);
    if (!d.empty() && std::fread(d.data(), 1, d.size(), f) != d.size()) d.clear();
    std::fclose(f);
    return d;
}

}  // namespace

void ios_ipc_write(uint32_t reg, uint32_t v) {
    switch (reg) {
    case 0: ppc_msg = v; break;
    case 1:
        ctrl.x1 = v & 1;
        ctrl.x2 = v >> 3 & 1;
        if (v >> 2 & 1) ctrl.y1 = false;
        if (v >> 1 & 1) ctrl.y2 = false;
        ctrl.iy1 = v >> 4 & 1;
        ctrl.iy2 = v >> 5 & 1;
        // the interrupt fires when Y1/Y2 is written with its enable set, even to clear it
        if (((v >> 2 & 1) && ctrl.iy1) || ((v >> 1 & 1) && ctrl.iy2)) irq_flags |= IRQ_IPC;
        if (ctrl.x1) requests.push_back(ppc_msg);
        update();
        break;
    case 2: arm_msg = v; break;
    }
    os_raise();
}

uint32_t ios_ipc_read(uint32_t reg) {
    switch (reg) {
    case 0: return ppc_msg;
    case 1: return ctrl.ppc();
    case 2: return arm_msg;
    }
    return 0;
}

void ios_irq_flag_clear(uint32_t v) {
    irq_flags &= ~v;
    update();
}
void ios_irq_mask_write(uint32_t v) { irq_mask = v; }
uint32_t ios_irq_flags() { return irq_flags; }
uint32_t ios_irq_mask() { return irq_mask; }

void ios_init(const char* extract_dir, const char* nand_root) {
    std::string root = extract_dir;
    g_ticket = slurp(root + "/ticket.bin");
    g_tmd = slurp(root + "/tmd.bin");
    if (g_ticket.size() < 0x2A4 || g_tmd.size() < 0x1E4)
        rt_die("%s needs ticket.bin and tmd.bin (python -m wiikit.disc GAME --extract)", extract_dir);
    for (int i = 0; i < 8; ++i) g_title = g_title << 8 | g_ticket[0x1DC + i];
    g_nand = nand_root;
    char data[64];
    std::snprintf(data, sizeof data, "/title/%08x/%08x/data", (uint32_t)(g_title >> 32), (uint32_t)g_title);
    for (const char* d : {"/shared2/sys", "/tmp", "/sys"}) fs::create_directories(g_nand + d);
    fs::create_directories(g_nand + data);
}
