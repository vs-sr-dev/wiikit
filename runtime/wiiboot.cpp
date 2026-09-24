// wiiboot — run a recompiled Wii disc game.
//
//     wiiboot EXTRACT_DIR [--nand DIR] [--fonts DIR] [--symbols symbols.tsv] [--mmio-log] [--watch SECONDS]
//                         [--no-video] [--no-audio] [--scale N] [--frames-ahead N] [--dump DIR]
//                         [--dump-every N] [--quit-after SECONDS]
//
// EXTRACT_DIR comes from `python -m wiikit.disc GAME --extract`. The NAND
// (saves, SYSCONF) is a host folder, EXTRACT_DIR/../nand by default. The
// boot ROM's fonts (font_western.bin, font_japanese.bin) and the DSP ROM's
// resampling table (dsp_coef.bin) are looked for in EXTRACT_DIR/../fonts:
// Dolphin's Sys/GC has free ones.
//
// The game runs on its own threads; the main thread runs the window and the
// renderer (video.cpp), or with --no-video only waits.
#include "boot.h"
#include "mem.h"
#include "rt.h"
#include "video.h"
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: wiiboot EXTRACT_DIR [--nand DIR] [--fonts DIR] [--symbols symbols.tsv] "
                             "[--mmio-log] [--watch SECONDS]\n"
                             "                            [--no-video] [--no-audio] [--scale N] [--frames-ahead N] "
                             "[--dump DIR] [--dump-every N] [--quit-after SECONDS]\n");
        return 2;
    }
    int watch = 0;
    bool audio = true;
    VideoOptions vo;
    std::string root = argv[1], nand = root + "/../nand", fonts = root + "/../fonts";
    for (int i = 2; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--nand") && i + 1 < argc) nand = argv[++i];
        else if (!std::strcmp(argv[i], "--fonts") && i + 1 < argc) fonts = argv[++i];
        else if (!std::strcmp(argv[i], "--symbols") && i + 1 < argc) rt_load_symbols(argv[++i]);
        else if (!std::strcmp(argv[i], "--mmio-log")) g_mmio_log = true;
        else if (!std::strcmp(argv[i], "--watch") && i + 1 < argc) watch = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--no-video")) vo.enabled = false;
        else if (!std::strcmp(argv[i], "--no-audio")) audio = false;
        else if (!std::strcmp(argv[i], "--scale") && i + 1 < argc) vo.scale = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--frames-ahead") && i + 1 < argc) vo.frames_ahead = std::max(1, std::atoi(argv[++i]));
        else if (!std::strcmp(argv[i], "--dump") && i + 1 < argc) vo.dump_dir = argv[++i];
        else if (!std::strcmp(argv[i], "--dump-every") && i + 1 < argc) vo.dump_every = std::max(1, std::atoi(argv[++i]));
        else if (!std::strcmp(argv[i], "--quit-after") && i + 1 < argc) vo.quit_after = std::atof(argv[++i]);
        else { std::fprintf(stderr, "wiiboot: unknown option %s\n", argv[i]); return 2; }
    }
    std::setvbuf(stdout, nullptr, _IOLBF, 1 << 16);
    if (!mem_init()) rt_die("cannot reserve the guest address space");
    uint32_t entry = boot_disc(root.c_str());
    video_configure(vo);
    gx_init();
    hw_init();
    if (!hw_load_fonts(fonts.c_str())) rt_log("wiiboot: no boot ROM fonts in %s", fonts.c_str());
    if (!ax_load_coefs(fonts.c_str())) rt_log("wiiboot: no dsp_coef.bin in %s: linear resampling", fonts.c_str());
    audio_init(audio);
    ios_init(root.c_str(), nand.c_str());
    os_install();
    wpad_install();
    rt_game_install();
    if (watch) os_watch(watch);
    if (std::getenv("WIIKIT_PROFILE")) os_profile();
    char id[7] = {}, name[65] = {};                           // the disc header: game id and name
    std::memcpy(id, host(0x80000000), 6);
    if (FILE* f = std::fopen((root + "/sys/boot.bin").c_str(), "rb")) {
        if (std::fseek(f, 0x20, SEEK_SET) == 0 && std::fread(name, 1, 64, f)) {}
        std::fclose(f);
    }
    std::string title = std::string(name[0] ? name : "wiiboot") + " [" + id + "]";
    rt_log("wiiboot: %s, entry %08X", title.c_str(), entry);
    os_start_main(entry);
    if (vo.enabled) video_run(title.c_str());                 // does not return
    if (vo.quit_after > 0) {
        std::this_thread::sleep_for(std::chrono::duration<double>(vo.quit_after));
        gx_report();
        std::fflush(stdout);
        std::_Exit(0);
    }
    for (;;) std::this_thread::sleep_for(std::chrono::hours(1));
}
