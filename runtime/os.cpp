// wiikit runtime — the OS layer: guest threads, interrupts, time.
//
// Guest threads. The SDK's scheduler (run queues, SelectThread, mutexes,
// message queues, alarms) runs recompiled and unchanged. Only its context
// switch is replaced: OSLoadContext would restore another thread's registers
// and rfi into it; here every guest thread lives on a host thread of its own
// and OSLoadContext hands a baton to the host thread that carries the target
// context, then waits for the baton to come back. One guest thread runs at a
// time, as on the Wii's single core. A host thread parked inside
// OSLoadContext resumes there when its context is loaded again; the SDK's
// SelectThread then returns, which is where the real thread would have
// resumed (from OSSaveContext, with r3 = 1, then straight to SelectThread's
// return; its callers ignore the value).
//
// Interrupts. Devices raise their lines through os_raise(); recompiled code
// polls at loop back-edges and when it sets MSR[EE]. Delivery does what
// OSExceptionVector does: r3-r5, CR, LR, CTR, XER, SRR0/1 go into the current
// OSContext, which is marked as an exception context, and the handler from
// the table at 0x80003000 runs with r3 = exception number, r4 = context, on a
// copy of the registers and on the interrupted thread's stack. The handler
// ends in OSLoadContext(context), which here just returns to the delivery
// point, unless the handler rescheduled first: then this host thread parks
// inside SelectThread like any other, still inside the delivery.
#include "rt.h"
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <map>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <timeapi.h>
#ifndef STACK_SIZE_PARAM_IS_A_STACK_SIZE
#define STACK_SIZE_PARAM_IS_A_STACK_SIZE 0x00010000
#endif
#else
#include <pthread.h>
#endif

std::atomic<uint32_t> g_ppc_pending{0};

namespace {

constexpr uint32_t MSR_EE = 0x8000, MSR_RI = 0x2;
constexpr uint32_t CTX_CR = 0x80, CTX_LR = 0x84, CTX_CTR = 0x88, CTX_XER = 0x8C, CTX_FPR = 0x90,
                   CTX_FPSCR = 0x194, CTX_SRR0 = 0x198, CTX_SRR1 = 0x19C, CTX_STATE = 0x1A2,
                   CTX_GQR = 0x1A4, CTX_PSF = 0x1C8;
constexpr uint16_t CTX_STATE_EXC = 2;
constexpr uint32_t LOW_CURRENT_CONTEXT = 0x800000D4, LOW_CURRENT_THREAD = 0x800000E4;
constexpr uint32_t EXC_TABLE = 0x80003000;
constexpr int EXC_EXTERNAL = 4, EXC_DECREMENTER = 8;

struct ThreadExit {};

struct HostThread {
    PPCContext c{};
    uint32_t ctx = 0;                    // the guest OSContext this host thread carries
    std::condition_variable cv;
    bool zombie = false;                 // its guest thread was re-created: unwind and end
    std::vector<uint32_t> delivering;    // contexts interrupted on this host thread
    PPCContext* cur = &c;                // the registers in use: c, or a handler's copy
#ifdef _WIN32
    HANDLE handle = nullptr;             // for the sampling profiler
#endif
};

std::mutex g_mx;                         // the baton and the thread tables
HostThread* g_running = nullptr;
std::map<uint32_t, HostThread*> g_by_ctx;
std::map<uint32_t, bool> g_fresh;        // contexts made by OSCreateThread, not yet started
thread_local HostThread* t_self = nullptr;

PPCFunc orig_OSSaveContext, orig_OSCreateThread;

// ---- the baton ----------------------------------------------------------------------------
void wait_baton(std::unique_lock<std::mutex>& lk) {
    while (g_running != t_self && !t_self->zombie) t_self->cv.wait(lk);
    if (t_self->zombie) throw ThreadExit{};
}

void pass_to(HostThread* to) {
    std::unique_lock<std::mutex> lk(g_mx);
    g_running = to;
    to->cv.notify_one();
    wait_baton(lk);
}

// ---- host threads ---------------------------------------------------------------------------
void load_regs(PPCContext& c, uint32_t ctx) {
    for (int i = 0; i < 32; ++i) {
        c.r[i] = ld32(ctx + 4 * i);
        c.f[i] = as_f64(ld64(ctx + CTX_FPR + 8 * i));
        c.ps1[i] = as_f64(ld64(ctx + CTX_PSF + 8 * i));
    }
    mtcrf(c, 0xFF, ld32(ctx + CTX_CR));
    c.lr = ld32(ctx + CTX_LR);
    c.ctr = ld32(ctx + CTX_CTR);
    mtxer(c, ld32(ctx + CTX_XER));
    c.fpscr = ld32(ctx + CTX_FPSCR);
    c.msr = ld32(ctx + CTX_SRR1);
    for (int i = 0; i < 8; ++i) c.gqr[i] = ld32(ctx + CTX_GQR + 4 * i);
}

void thread_body(HostThread* h, uint32_t entry) {
    t_self = h;
    t_ppc = &h->c;
    try {
        {
            std::unique_lock<std::mutex> lk(g_mx);
            wait_baton(lk);
        }
        PPCContext& c = h->c;
        if (!entry) entry = ld32(h->ctx + CTX_SRR0);
        uint32_t exit_fn = c.lr;             // OSCreateThread points LR at OSExitThread
        ppc_call_indirect(c, entry);
        if (!h->ctx) rt_die("the main thread returned from %08X", entry);
        ppc_call_indirect(c, exit_fn);
        rt_die("thread %08X: OSExitThread returned", h->ctx);
    } catch (ThreadExit&) {
    }
}

struct Start { HostThread* h; uint32_t entry; };

#ifdef _WIN32
DWORD WINAPI host_thread(LPVOID p) {
    Start s = *static_cast<Start*>(p);
    delete static_cast<Start*>(p);
    thread_body(s.h, s.entry);
    return 0;
}
void spawn(HostThread* h, uint32_t entry) {
    // recompiled code nests C++ calls as deeply as the guest nests its own
    HANDLE t = CreateThread(nullptr, 64u << 20, host_thread, new Start{h, entry},
                            STACK_SIZE_PARAM_IS_A_STACK_SIZE, nullptr);
    if (!t) rt_die("cannot create a host thread");
    h->handle = t;                       // kept: the profiler samples it
}
#else
void* host_thread(void* p) {
    Start s = *static_cast<Start*>(p);
    delete static_cast<Start*>(p);
    thread_body(s.h, s.entry);
    return nullptr;
}
void spawn(HostThread* h, uint32_t entry) {
    pthread_attr_t a;
    pthread_attr_init(&a);
    pthread_attr_setstacksize(&a, 64u << 20);
    pthread_t t;
    if (pthread_create(&t, &a, host_thread, new Start{h, entry})) rt_die("cannot create a host thread");
    pthread_detach(t);
}
#endif

// this host thread carries the guest thread whose context is `ctx`
void adopt(uint32_t ctx) {
    std::lock_guard<std::mutex> lk(g_mx);
    if (t_self->ctx == ctx) return;
    if (t_self->ctx) rt_die("host thread of context %08X found running context %08X", t_self->ctx, ctx);
    t_self->ctx = ctx;
    g_by_ctx[ctx] = t_self;
}

// ---- the hooks -------------------------------------------------------------------------------
void hle_OSLoadContext(PPCContext& c) {
    uint32_t x = c.r[3];
    HostThread* self = t_self;
    uint16_t st = ld16(x + CTX_STATE);
    if (st & CTX_STATE_EXC) st16(x + CTX_STATE, st & ~CTX_STATE_EXC);
    if (!self->delivering.empty() && self->delivering.back() == x) return;  // end of an interrupt
    if (x == self->ctx) return;                                             // this thread goes on
    HostThread* to;
    {
        std::lock_guard<std::mutex> lk(g_mx);
        auto fr = g_fresh.find(x);
        if (fr != g_fresh.end()) {
            g_fresh.erase(fr);
            auto old = g_by_ctx.find(x);
            if (old != g_by_ctx.end()) {           // the OSThread was reused: retire its host thread
                old->second->zombie = true;
                old->second->cv.notify_one();
            }
            to = new HostThread;
            to->ctx = x;
            load_regs(to->c, x);
            g_by_ctx[x] = to;
            spawn(to, 0);
        } else {
            auto it = g_by_ctx.find(x);
            if (it == g_by_ctx.end()) rt_die("OSLoadContext(%08X): no thread carries this context", x);
            to = it->second;
        }
    }
    pass_to(to);
}

void hle_OSSaveContext(PPCContext& c) {
    if (t_self->ctx != c.r[3]) adopt(c.r[3]);
    orig_OSSaveContext(c);
}

void hle_OSCreateThread(PPCContext& c) {
    uint32_t thread = c.r[3];                  // OSThread begins with its OSContext
    orig_OSCreateThread(c);
    if (c.r[3]) {
        std::lock_guard<std::mutex> lk(g_mx);
        g_fresh[thread] = true;
    }
}

void hle_write_console(PPCContext& c) {        // (handle, buffer, &count, ref)
    uint32_t n = ld32(c.r[5]);
    std::fwrite(host(c.r[4]), 1, n, stdout);
    std::fflush(stdout);
    c.r[3] = 0;
}

void hle_PPCHalt(PPCContext&) { rt_die("PPCHalt: the game stopped the CPU"); }

void hle_OSFatal(PPCContext& c) {              // (fg color, bg color, message)
    rt_die("OSFatal: %s", guest_cstr(c.r[5], 1024).c_str());
}

void hle_leave(PPCContext&) {
    std::fflush(stdout);
    rt_log("wiikit: the game asked to leave (return to menu, restart or shut down)");
    std::_Exit(0);
}

// ---- time ------------------------------------------------------------------------------------
// The time base runs at a quarter of the 243 MHz bus clock: 60.75 MHz.
using Clock = std::chrono::steady_clock;
Clock::time_point g_t0;
std::atomic<int64_t> g_tb_offset{0};
std::mutex g_clock_mx;                           // the decrementer's state
bool g_dec_armed = false;
uint64_t g_dec_deadline = 0;                     // time base value at which DEC turns negative

bool dec_due() {
    std::lock_guard<std::mutex> lk(g_clock_mx);
    if (!g_dec_armed || os_tb_now() < g_dec_deadline) return false;
    g_dec_armed = false;
    return true;
}

// The clock thread sleeps until the next event, or until kicked (a new
// decrementer value). Condition-variable timeouts are only as fine as the
// system tick on some hosts (15.6 ms on Windows through winpthreads), far
// too coarse for OSSleepTicks. On Windows a high-resolution waitable timer
// (Windows 10 1803 and later) wakes it within a fraction of a millisecond;
// elsewhere, and on older Windows, it sleeps in whole milliseconds. It
// yields only through the last 200 microseconds.
std::atomic<bool> g_kick{false};
#ifdef _WIN32
#ifndef CREATE_WAITABLE_TIMER_HIGH_RESOLUTION
#define CREATE_WAITABLE_TIMER_HIGH_RESOLUTION 0x00000002
#endif
HANDLE g_kick_event = CreateEventW(nullptr, FALSE, FALSE, nullptr);
HANDLE g_clock_timer = CreateWaitableTimerExW(nullptr, nullptr, CREATE_WAITABLE_TIMER_HIGH_RESOLUTION,
                                              TIMER_ALL_ACCESS);
void clock_kick() { g_kick = true; SetEvent(g_kick_event); }
void clock_block(std::chrono::nanoseconds d) {
    if (!g_clock_timer) {
        WaitForSingleObject(g_kick_event, (DWORD)(d.count() / 1000000));
        return;
    }
    LARGE_INTEGER due;
    due.QuadPart = -(LONGLONG)(d.count() / 100);   // relative, in 100 ns units
    SetWaitableTimer(g_clock_timer, &due, 0, nullptr, nullptr, FALSE);
    HANDLE h[2] = {g_kick_event, g_clock_timer};
    WaitForMultipleObjects(2, h, FALSE, INFINITE);
}
const auto clock_margin = std::chrono::microseconds(g_clock_timer ? 200 : 1500);
#else
std::mutex g_kick_mx;
std::condition_variable g_kick_cv;
void clock_kick() { g_kick = true; g_kick_cv.notify_one(); }
void clock_block(std::chrono::nanoseconds d) {
    std::unique_lock<std::mutex> lk(g_kick_mx);
    g_kick_cv.wait_for(lk, d);
}
const auto clock_margin = std::chrono::microseconds(200);
#endif
void clock_sleep(Clock::time_point wake) {
    using namespace std::chrono;
    for (;;) {
        auto left = wake - Clock::now();
        if (left <= nanoseconds(0) || g_kick.exchange(false)) return;
        if (left > clock_margin + microseconds(100)) clock_block(duration_cast<nanoseconds>(left - clock_margin));
        else std::this_thread::yield();
    }
}

// The idle loop's host thread waits here for the interrupt line (ppc_idle).
std::atomic<bool> g_idle_waiting{false};
#ifdef _WIN32
HANDLE g_idle_event = CreateEventW(nullptr, FALSE, FALSE, nullptr);
void idle_wake() { SetEvent(g_idle_event); }
void idle_wait() { WaitForSingleObject(g_idle_event, 1); }
#else
std::mutex g_idle_mx;
std::condition_variable g_idle_cv;
void idle_wake() { g_idle_cv.notify_one(); }
void idle_wait() {
    std::unique_lock<std::mutex> lk(g_idle_mx);
    g_idle_cv.wait_for(lk, std::chrono::milliseconds(1));
}
#endif

// The clock keeps host time: the guest may rewrite the time base (mttb), so
// only the decrementer's distance is measured in guest ticks.
void clock_main() {
    const auto vi_period = std::chrono::nanoseconds(1001000000000ll / 60000);  // NTSC, 59.94 Hz
    auto next_vi = Clock::now() + vi_period;
    for (;;) {
        auto now = Clock::now();
        if (now >= next_vi) {
            hw_vi_retrace();
            next_vi += vi_period;
            if (next_vi <= now) next_vi = now + vi_period;
        }
        auto wake = std::min(next_vi, hw_tick());
        {
            std::lock_guard<std::mutex> lk(g_clock_mx);
            if (g_dec_armed) {
                int64_t ticks = (int64_t)(g_dec_deadline - os_tb_now());
                if (ticks <= 0) os_raise();
                else wake = std::min(wake, now + std::chrono::nanoseconds(ticks * 4000 / 243));
            }
        }
        clock_sleep(wake);
    }
}

// ---- interrupt delivery -----------------------------------------------------------------------
void deliver(PPCContext& c, int exc) {
    HostThread* self = t_self;
    uint32_t ctx = ld32(LOW_CURRENT_CONTEXT);
    if (!self->ctx && ctx && ctx == ld32(LOW_CURRENT_THREAD)) adopt(ctx);
    st32(ctx + 0x0C, c.r[3]);
    st32(ctx + 0x10, c.r[4]);
    st32(ctx + 0x14, c.r[5]);
    st16(ctx + CTX_STATE, ld16(ctx + CTX_STATE) | CTX_STATE_EXC);
    st32(ctx + CTX_CR, mfcr(c));
    st32(ctx + CTX_LR, c.lr);
    st32(ctx + CTX_CTR, c.ctr);
    st32(ctx + CTX_XER, mfxer(c));
    st32(ctx + CTX_SRR0, 0);                   // no guest PC: the host resumes the code
    st32(ctx + CTX_SRR1, c.msr | MSR_RI);
    PPCContext e = c;
    e.r[3] = exc;
    e.r[4] = ctx;
    e.r[5] = c.msr | MSR_RI;
    e.msr = c.msr & ~MSR_EE;
    uint32_t handler = ld32(EXC_TABLE + 4 * exc);
    self->delivering.push_back(ctx);
    PPCContext* outer = t_ppc;
    t_ppc = self->cur = &e;
    ppc_call_indirect(e, handler);
    t_ppc = self->cur = outer;
    self->delivering.pop_back();
}

#ifdef _WIN32
LONG WINAPI on_crash(EXCEPTION_POINTERS* ep) {
    const EXCEPTION_RECORD* r = ep->ExceptionRecord;
    if (r->ExceptionCode == EXCEPTION_ACCESS_VIOLATION && r->NumberParameters >= 2) {
        uintptr_t a = (uintptr_t)r->ExceptionInformation[1], base = (uintptr_t)g_mem;
        if (a >= base && a - base < (uint64_t(1) << 32))
            rt_die("guest %s at %08X, outside guest memory",
                   r->ExceptionInformation[0] ? "write" : "read", (uint32_t)(a - base));
        rt_die("host access violation at %p", (void*)a);
    }
    if (r->ExceptionCode == EXCEPTION_STACK_OVERFLOW) rt_die("host stack overflow");
    return EXCEPTION_CONTINUE_SEARCH;
}
#endif

}  // namespace

// ---- services ----------------------------------------------------------------------------------
uint64_t os_tb_now() {
    int64_t ns = std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - g_t0).count();
    return (uint64_t)(ns / 4000 * 243 + ns % 4000 * 243 / 4000) + (uint64_t)g_tb_offset.load();
}

uint64_t ppc_timebase() { return os_tb_now(); }

void ppc_mttb(int upper, uint32_t v) {
    uint64_t tb = os_tb_now();
    uint64_t want = upper ? ((uint64_t)v << 32 | (tb & 0xFFFFFFFFu)) : ((tb & ~0xFFFFFFFFull) | v);
    std::lock_guard<std::mutex> lk(g_clock_mx);
    g_tb_offset += (int64_t)(want - tb);
    g_dec_deadline += want - tb;               // DEC counts on its own: its deadline moves along
    clock_kick();
}

// DEC counts down at the time base rate; the exception is taken when its
// top bit goes from 0 to 1 (Dolphin's rule): writing a negative value over
// a non-negative one fires at once, over a negative one never.
void ppc_mtdec(uint32_t v) {
    std::lock_guard<std::mutex> lk(g_clock_mx);
    uint64_t now = os_tb_now();
    int32_t old = (int32_t)(uint32_t)(g_dec_deadline - 1 - now);
    if (v & 0x80000000u) {
        g_dec_armed = old >= 0;
        g_dec_deadline = now;
    } else {
        g_dec_armed = true;
        g_dec_deadline = now + v + 1;          // DEC reaches -1 after v + 1 ticks
    }
    clock_kick();
    if (g_dec_armed && now >= g_dec_deadline) os_raise();
}

uint32_t ppc_mfdec() {
    std::lock_guard<std::mutex> lk(g_clock_mx);
    return (uint32_t)(g_dec_deadline - 1 - os_tb_now());
}

void os_raise() {
    g_ppc_pending.store(1, std::memory_order_seq_cst);
    if (g_idle_waiting.load(std::memory_order_seq_cst)) idle_wake();
}

// Nothing but an interrupt can end the loop: wait for one (with a 1 ms
// bound, in case a line is raised some other way), then deliver it.
void ppc_idle(PPCContext& c) {
    if (!g_ppc_pending.load(std::memory_order_relaxed) && (c.msr & MSR_EE)) {
        g_idle_waiting.store(true, std::memory_order_seq_cst);
        if (!g_ppc_pending.load(std::memory_order_seq_cst)) idle_wait();
        g_idle_waiting.store(false, std::memory_order_relaxed);
    }
    PPC_POLL(c);
}

void ppc_poll(PPCContext& c) {
    while (c.msr & MSR_EE) {
        g_ppc_pending.store(0);
        int exc = hw_external_pending() ? EXC_EXTERNAL : dec_due() ? EXC_DECREMENTER : -1;
        if (exc < 0) return;
        g_ppc_pending.store(1);                // look again after this one
        deliver(c, exc);
    }
}

void ppc_syscall(PPCContext&, uint32_t) {}     // the SDK's only sc flash-invalidates the icache
void ppc_trap(PPCContext& c, uint32_t addr) {
    t_ppc = &c;
    rt_die("trap at %s", rt_name(addr).c_str());
}
void ppc_unimplemented(PPCContext& c, uint32_t addr, const char* what) {
    t_ppc = &c;
    rt_die("%s at %s", what, rt_name(addr).c_str());
}

void os_install() {
    g_t0 = Clock::now();
#ifdef _WIN32
    timeBeginPeriod(1);
    AddVectoredExceptionHandler(1, on_crash);
#endif
    ppc_hook("OSLoadContext", hle_OSLoadContext);
    orig_OSSaveContext = ppc_hook("OSSaveContext", hle_OSSaveContext);
    orig_OSCreateThread = ppc_hook("OSCreateThread", hle_OSCreateThread);
    if (!orig_OSSaveContext || !orig_OSCreateThread) rt_die("the OS hooks are missing from the build");
    ppc_hook("__write_console", hle_write_console);
    ppc_hook("PPCHalt", hle_PPCHalt);
    ppc_hook("OSFatal", hle_OSFatal);
    for (const char* n : {"OSReturnToMenu", "OSRestart", "OSShutdownSystem", "__OSReboot"})
        ppc_hook(n, hle_leave);
    ppc_hook("RealMode", [](PPCContext&) {});  // BAT set-up: the address space is flat already
    std::thread(clock_main).detach();
}

// --watch: every few seconds, where the running guest thread is
void watch_main(int seconds) {
    for (;;) {
        std::this_thread::sleep_for(std::chrono::seconds(seconds));
        std::lock_guard<std::mutex> lk(g_mx);
        HostThread* h = g_running;
        if (!h) continue;
        const PPCContext& c = *h->cur;
        std::fprintf(stderr, "watch: thread %08X, msr %08X, pending %u, %zu interrupt(s) deep, %zu threads\n",
                     h->ctx, c.msr, g_ppc_pending.load(), h->delivering.size(), g_by_ctx.size());
        rt_backtrace(c, stderr, 16);
        {
            std::lock_guard<std::mutex> ck(g_clock_mx);
            std::fprintf(stderr, "  decrementer %s, due in %lld ticks; external pending %d\n",
                         g_dec_armed ? "armed" : "idle", (long long)(g_dec_deadline - os_tb_now()),
                         (int)hw_external_pending());
        }
        for (auto& [ctx, t] : g_by_ctx) {            // the parked threads, where they wait
            if (t == h || t->zombie) continue;
            std::fprintf(stderr, "  thread %08X:\n", ctx);
            rt_backtrace(*t->cur, stderr, 7);
        }
        gx_report();
    }
}

void os_watch(int seconds) { std::thread(watch_main, seconds).detach(); }

#ifdef _WIN32
// WIIKIT_PROFILE=1: a sampling profiler of the thread holding the baton.
// About a thousand times a second that host thread is stopped for a moment:
// its registers and the top 64 KB of its stack are copied into buffers made
// in advance, and it goes on. Samples in the executable are reported by
// address (tools/profile_resolve.py names them with nm: the runtime, and
// recompiled functions by their guest names); a sample in a DLL (the OS
// waiting, the C library) is unwound on the stack copy to the first return
// address in the executable, and charged there with the DLL's name.
namespace {
bool exe_caller(CONTEXT c, const uint8_t* copy, uintptr_t copy_base, size_t copy_len,
                uintptr_t lo, uintptr_t hi, uintptr_t& out) {
    // rebase the stack pointers into the copy, then unwind frame by frame
    intptr_t delta = (intptr_t)(uintptr_t)copy - (intptr_t)copy_base;
    c.Rsp += delta;
    if (c.Rbp >= copy_base && c.Rbp < copy_base + copy_len) c.Rbp += delta;
    for (int depth = 0; depth < 32; ++depth) {
        if (c.Rip >= lo && c.Rip < hi) { out = c.Rip; return true; }
        if (c.Rsp < (uintptr_t)copy || c.Rsp + 8 > (uintptr_t)copy + copy_len) return false;
        DWORD64 image = 0;
        PRUNTIME_FUNCTION f = RtlLookupFunctionEntry(c.Rip, &image, nullptr);
        if (!f) {                                  // a leaf: the return address is on top
            c.Rip = *(DWORD64*)c.Rsp;
            c.Rsp += 8;
        } else {
            void* handler_data = nullptr;
            DWORD64 frame = 0;
            RtlVirtualUnwind(UNW_FLAG_NHANDLER, image, c.Rip, f, &c, &handler_data, &frame, nullptr);
        }
        if (!c.Rip) return false;
    }
    return false;
}
}  // namespace

void profile_main() {
    uintptr_t base = (uintptr_t)GetModuleHandleW(nullptr);
    auto* nt = (IMAGE_NT_HEADERS*)(base + ((IMAGE_DOS_HEADER*)base)->e_lfanew);
    uintptr_t lo = base, hi = base + nt->OptionalHeader.SizeOfImage;
    constexpr size_t kStack = 64 * 1024;
    std::vector<uint8_t> copy(kStack);
    std::unordered_map<uintptr_t, uint32_t> in_exe;                  // by address (image base 0x140000000)
    std::map<std::pair<HMODULE, uintptr_t>, uint32_t> via_dll;        // DLL, calling address
    uint64_t total = 0, dll = 0, lost = 0;
    timeBeginPeriod(1);
    auto next = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    auto norm = [&](uintptr_t ip) { return (ip - base + 0x140000000ull) & ~(uintptr_t)15; };
    for (;;) {
        Sleep(1);
        HANDLE t = nullptr;
        {
            std::lock_guard<std::mutex> lk(g_mx);
            if (g_running) t = g_running->handle;
        }
        if (!t) continue;
        CONTEXT ctx;
        ctx.ContextFlags = CONTEXT_FULL;
        if (SuspendThread(t) == (DWORD)-1) continue;
        BOOL ok = GetThreadContext(t, &ctx);
        size_t len = 0;
        if (ok) {                                   // the stack, as far as it is committed
            MEMORY_BASIC_INFORMATION mi;
            if (VirtualQuery((void*)ctx.Rsp, &mi, sizeof mi)) {
                uintptr_t end = (uintptr_t)mi.BaseAddress + mi.RegionSize;
                len = std::min<size_t>(kStack, end - ctx.Rsp);
                std::memcpy(copy.data(), (void*)ctx.Rsp, len);
            }
        }
        ResumeThread(t);
        if (!ok) continue;
        ++total;
        uintptr_t ip = (uintptr_t)ctx.Rip;
        if (ip >= lo && ip < hi) {
            ++in_exe[norm(ip)];
        } else {
            ++dll;
            HMODULE m = nullptr;
            GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                               (LPCWSTR)ip, &m);
            uintptr_t caller = 0;
            if (len && exe_caller(ctx, copy.data(), (uintptr_t)ctx.Rsp, len, lo, hi, caller)) ++via_dll[{m, norm(caller)}];
            else ++lost;
        }
        if (std::chrono::steady_clock::now() < next) continue;
        next += std::chrono::seconds(10);
        std::fprintf(stderr, "profile: %llu samples, %.1f%% in DLLs (%.1f%% of those not traced back)\n",
                     (unsigned long long)total, total ? 100.0 * dll / total : 0.0, dll ? 100.0 * lost / dll : 0.0);
        std::vector<std::pair<uint32_t, uintptr_t>> top;
        for (auto& [a, n] : in_exe) top.push_back({n, a});
        std::sort(top.rbegin(), top.rend());
        for (size_t k = 0; k < top.size() && k < 80; ++k)
            std::fprintf(stderr, "profile-raw: %llx %u\n", (unsigned long long)top[k].second, top[k].first);
        std::vector<std::pair<uint32_t, std::pair<HMODULE, uintptr_t>>> tops;
        for (auto& [k, n] : via_dll) tops.push_back({n, k});
        std::sort(tops.rbegin(), tops.rend());
        for (size_t k = 0; k < tops.size() && k < 40; ++k) {
            char name[MAX_PATH] = "?";
            GetModuleFileNameA(tops[k].second.first, name, sizeof name);
            const char* slash = std::strrchr(name, '\\');
            std::fprintf(stderr, "profile-raw: %llx %u %s\n", (unsigned long long)tops[k].second.second, tops[k].first,
                         slash ? slash + 1 : name);
        }
        in_exe.clear();
        via_dll.clear();
        total = dll = lost = 0;
    }
}
#endif

void os_profile() {
#ifdef _WIN32
    std::thread(profile_main).detach();
#endif
}

void os_start_main(uint32_t entry) {
    HostThread* h = new HostThread;
    h->c.msr = 0x00002032;                     // FP, IR, DR, RI; interrupts off, as the IPL leaves it
    {
        std::lock_guard<std::mutex> lk(g_mx);
        g_running = h;
    }
    spawn(h, entry);
}
