// wiiboot — run a recompiled Wii disc game.
//
//     wiiboot EXTRACT_DIR [--nand DIR] [--fonts DIR] [--symbols symbols.tsv] [--mmio-log] [--watch SECONDS]
//
// EXTRACT_DIR comes from `python -m wiikit.disc GAME --extract`. The NAND
// (saves, SYSCONF) is a host folder, EXTRACT_DIR/../nand by default. The
// boot ROM's fonts (font_western.bin, font_japanese.bin: Dolphin's Sys/GC
// has free ones) are looked for in EXTRACT_DIR/../fonts.
#include "boot.h"
#include "mem.h"
#include "rt.h"
#include <cstdlib>
#include <cstring>
#include <string>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: wiiboot EXTRACT_DIR [--nand DIR] [--fonts DIR] [--symbols symbols.tsv] "
                             "[--mmio-log] [--watch SECONDS]\n");
        return 2;
    }
    int watch = 0;
    std::string root = argv[1], nand = root + "/../nand", fonts = root + "/../fonts";
    for (int i = 2; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--nand") && i + 1 < argc) nand = argv[++i];
        else if (!std::strcmp(argv[i], "--fonts") && i + 1 < argc) fonts = argv[++i];
        else if (!std::strcmp(argv[i], "--symbols") && i + 1 < argc) rt_load_symbols(argv[++i]);
        else if (!std::strcmp(argv[i], "--mmio-log")) g_mmio_log = true;
        else if (!std::strcmp(argv[i], "--watch") && i + 1 < argc) watch = std::atoi(argv[++i]);
    }
    std::setvbuf(stdout, nullptr, _IOLBF, 1 << 16);
    if (!mem_init()) rt_die("cannot reserve the guest address space");
    uint32_t entry = boot_disc(root.c_str());
    hw_init();
    if (!hw_load_fonts(fonts.c_str())) rt_log("wiiboot: no boot ROM fonts in %s", fonts.c_str());
    ios_init(root.c_str(), nand.c_str());
    os_install();
    wpad_install();
    if (watch) os_watch(watch);
    rt_log("wiiboot: %s, entry %08X", root.c_str(), entry);
    os_run_main(entry);
}
