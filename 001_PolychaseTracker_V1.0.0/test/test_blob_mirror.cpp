// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Peter Mercell
//
// Developed with assistance from Claude (Anthropic).

// =============================================================================
// test/test_blob_mirror.cpp — unit test for BlobMirror's PURE decision logic
// (pcn_blob_mirror.h): the cache compare, the "is this our own echo?" guard, and
// the would_write / needs_reload predicates that decide when a hidden String_knob
// blob is written or reloaded. DDImage-free path (PCN_BLOB_MIRROR_NO_DDIMAGE), so
// it runs anywhere:
//     g++ -std=c++17 -I../src test_blob_mirror.cpp -o test_blob_mirror && ./test_blob_mirror
// =============================================================================
#define PCN_BLOB_MIRROR_NO_DDIMAGE
#include "pcn_blob_mirror.h"

#include <cstdio>

using pcn::BlobMirror;

int main()
{
    BlobMirror m("mask_blob");
    int fail = 0;
    auto chk = [&](bool cond, const char* what){ if(!cond){ std::printf("  [FAIL] %s\n", what); ++fail; }
                                                 else      std::printf("  [ ok ] %s\n", what); };

    // Fresh mirror: empty cache.
    chk(m.would_write("abc"),   "new content would write");
    chk(m.needs_reload("abc"),  "external content needs reload");

    // Adopt the content as cache (== a successful save / load). Our own echo of
    // the SAME string is now neither a write nor a reload.
    m.set_cache("abc");
    chk(!m.would_write("abc"),  "unchanged content does NOT write");
    chk(!m.needs_reload("abc"), "own echo does NOT reload");

    // A genuinely different external string DOES reload (undo / redo / .nk load).
    chk(m.needs_reload("xyz"),  "changed external content reloads");

    // null live string is treated as empty.
    m.set_cache("");
    chk(!m.needs_reload(nullptr), "null live == empty cache, no reload");

    std::printf("\n%s\n", fail ? "FAIL" : "PASS");
    return fail ? 1 : 0;
}
