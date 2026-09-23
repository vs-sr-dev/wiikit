// Link check: every recompiled function, the dispatch table and the runtime
// services link into one executable. Nothing runs.
#include "ppc.h"
#include <cstdio>
#include <cstddef>

extern const PPCFuncEntry g_ppc_funcs[];
extern const size_t g_ppc_nfuncs;

int main() {
    std::printf("linked %zu recompiled functions; first %08X, last %08X\n", g_ppc_nfuncs,
                g_ppc_funcs[0].addr, g_ppc_funcs[g_ppc_nfuncs - 1].addr);
    return 0;
}
