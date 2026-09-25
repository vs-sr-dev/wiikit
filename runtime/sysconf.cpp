// wiikit runtime — SYSCONF, the console's settings, in the NAND at /shared2/sys.
//
// The SDK's SC library reads this file at boot (SCInit) and the game asks it
// for the aspect ratio, the language, the sound mode, the sensor bar's place.
// Without it the SDK falls back to an empty configuration: 4:3, where a
// widescreen game letterboxes its picture. The port writes one on the first
// run, set as a PC wants it (16:9, English, stereo); --aspect and --language
// change it as the Wii's settings menu would, and the change stays.
//
// The file is 0x4000 bytes: "SCv0", a big-endian u16 count of items, count+1
// u16 offsets (the last one is where the items end), the items, zeros, and
// "SCed" in the last four bytes. An item is a byte (type << 5 | name length
// - 1), the name, then its value: arrays carry their length - 1 first (u16
// for type 1, u8 for type 2); types 3 and 7 are one byte, 4 two, 5 four, 6
// eight. The layout is the one Dolphin writes.
#include "rt.h"
#include <cstring>
#include <filesystem>
#include <vector>

namespace fs = std::filesystem;

namespace {

enum : uint8_t { BIG_ARRAY = 1, SMALL_ARRAY, BYTE, SHORT, LONG, LONGLONG, BOOL };
constexpr size_t SIZE = 0x4000;

struct Item { uint8_t type; std::string name; std::vector<uint8_t> data; };

size_t fixed_size(uint8_t type) {
    switch (type) {
    case BYTE: case BOOL: return 1;
    case SHORT: return 2;
    case LONG: return 4;
    case LONGLONG: return 8;
    }
    return 0;
}

bool parse(const std::vector<uint8_t>& f, std::vector<Item>& items) {
    if (f.size() != SIZE || std::memcmp(f.data(), "SCv0", 4) != 0) return false;
    size_t n = (size_t)(f[4] << 8 | f[5]);
    for (size_t i = 0; i < n; ++i) {
        size_t p = (size_t)(f[6 + 2 * i] << 8 | f[7 + 2 * i]);
        if (p + 1 >= SIZE) return false;
        Item it;
        it.type = f[p] >> 5;
        size_t len = (size_t)(f[p] & 31) + 1;
        p += 1;
        if (p + len >= SIZE) return false;
        it.name.assign((const char*)&f[p], len);
        p += len;
        size_t size = fixed_size(it.type);
        if (it.type == BIG_ARRAY) { size = (size_t)(f[p] << 8 | f[p + 1]) + 1; p += 2; }
        else if (it.type == SMALL_ARRAY) { size = (size_t)f[p] + 1; p += 1; }
        if (!size || p + size > SIZE - 4) return false;
        it.data.assign(f.begin() + (ptrdiff_t)p, f.begin() + (ptrdiff_t)(p + size));
        items.push_back(std::move(it));
    }
    return true;
}

std::vector<uint8_t> build(const std::vector<Item>& items) {
    std::vector<uint8_t> f(SIZE);
    std::memcpy(f.data(), "SCv0", 4);
    f[4] = (uint8_t)(items.size() >> 8);
    f[5] = (uint8_t)items.size();
    size_t p = 6 + 2 * (items.size() + 1);
    for (size_t i = 0; i <= items.size(); ++i) {
        f[6 + 2 * i] = (uint8_t)(p >> 8);
        f[7 + 2 * i] = (uint8_t)p;
        if (i == items.size()) break;
        const Item& it = items[i];
        if (p + 3 + it.name.size() + it.data.size() > SIZE - 4) rt_die("sysconf: the items do not fit");
        f[p++] = (uint8_t)(it.type << 5 | (it.name.size() - 1));
        std::memcpy(&f[p], it.name.data(), it.name.size());
        p += it.name.size();
        if (it.type == BIG_ARRAY) { f[p++] = (uint8_t)((it.data.size() - 1) >> 8); f[p++] = (uint8_t)(it.data.size() - 1); }
        else if (it.type == SMALL_ARRAY) f[p++] = (uint8_t)(it.data.size() - 1);
        std::memcpy(&f[p], it.data.data(), it.data.size());
        p += it.data.size();
    }
    std::memcpy(&f[SIZE - 4], "SCed", 4);
    return f;
}

std::vector<uint8_t> read_file(const std::string& path) {
    std::vector<uint8_t> d;
    if (FILE* f = std::fopen(path.c_str(), "rb")) {
        d.resize(SIZE + 1);                                 // a larger file is no SYSCONF
        d.resize(std::fread(d.data(), 1, d.size(), f));
        std::fclose(f);
    }
    return d;
}

// What a console fresh from its setup has, for the items this SDK reads
std::vector<Item> defaults() {
    std::vector<Item> v;
    auto add = [&](uint8_t type, const char* name, std::vector<uint8_t> data) { v.push_back({type, name, std::move(data)}); };
    add(BIG_ARRAY, "BT.DINF", std::vector<uint8_t>(0x461));  // no Remotes paired
    add(LONG, "BT.SENS", {0, 0, 0, 3});                      // pointer sensitivity
    add(BYTE, "BT.BAR", {1});                                // the sensor bar above the screen
    add(BYTE, "BT.SPKV", {0x58});                            // Remote speaker volume
    add(BYTE, "BT.MOT", {1});                                // rumble on
    add(BYTE, "IPL.LNG", {1});                               // English
    add(BYTE, "IPL.AR", {1});                                // 16:9
    add(BYTE, "IPL.SND", {1});                               // stereo
    add(BYTE, "IPL.PGS", {0});                               // no progressive scan
    add(BYTE, "IPL.E60", {1});
    add(BYTE, "IPL.SSV", {1});                               // screen saver
    add(BYTE, "IPL.DH", {0});                                // display offset
    add(LONG, "IPL.CB", {0, 0, 0, 0});                       // RTC counter bias
    add(SMALL_ARRAY, "IPL.IDL", {0, 1});                     // no WiiConnect24 standby
    return v;
}

void set_byte(std::vector<Item>& items, const char* name, uint8_t value) {
    for (Item& it : items)
        if (it.name == name) { it.data.assign(1, value); return; }
    items.push_back({BYTE, name, {value}});
}

uint8_t get_byte(const std::vector<Item>& items, const char* name, uint8_t fallback) {
    for (const Item& it : items)
        if (it.name == name && it.data.size() == 1) return it.data[0];
    return fallback;
}

}  // namespace

bool sysconf_prepare(const char* nand_root, const SysconfOptions& o) {
    std::string dir = std::string(nand_root) + "/shared2/sys", path = dir + "/SYSCONF";
    fs::create_directories(dir);
    std::vector<Item> items;
    std::vector<uint8_t> old = read_file(path);
    bool fresh = old.empty();
    if (!fresh && !parse(old, items)) {
        rt_log("sysconf: %s is not a SYSCONF: writing a new one", path.c_str());
        fresh = true;
    }
    if (fresh) items = defaults();
    if (o.aspect >= 0) set_byte(items, "IPL.AR", (uint8_t)o.aspect);
    if (o.language >= 0) set_byte(items, "IPL.LNG", (uint8_t)o.language);
    std::vector<uint8_t> f = build(items);
    if ((fresh || o.aspect >= 0 || o.language >= 0) && f != old) {
        FILE* out = std::fopen(path.c_str(), "wb");
        if (!out || std::fwrite(f.data(), 1, f.size(), out) != f.size()) rt_die("sysconf: cannot write %s", path.c_str());
        std::fclose(out);
    }
    bool wide = get_byte(items, "IPL.AR", 0) == 1;
    rt_log("sysconf: %s, language %u", wide ? "16:9" : "4:3", get_byte(items, "IPL.LNG", 1));
    return wide;
}
