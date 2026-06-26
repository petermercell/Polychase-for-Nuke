// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Peter Mercell
//
// Developed with assistance from Claude (Anthropic).

// =============================================================================
// pcn_blob_mirror.h — BlobMirror: the shared round-trip machinery for the
// plugin's hidden String_knob "blobs" (pins, 3D mask, refine anchors, live
// pose). Each of those subsystems independently reimplemented the SAME three
// concerns:
//
//   1. a content cache (the last string we serialized) so our OWN asynchronous
//      set_text echo doesn't get re-imported and clobber a fresh in-memory edit;
//   2. a suppress flag held high while we write, so the re-entrant knob_changed
//      our set_text fires is a no-op;
//   3. the decision logic: write to the knob ONLY when the content actually
//      changed, and RELOAD from the knob ONLY when an external change
//      (undo / redo / .nk load) makes the live string differ from our cache.
//
// BlobMirror owns those three concerns; the subsystem keeps its own
// parse/serialize (they all differ). Writes go through ScopedFlags, so the
// suppress flag is exception-safe — a throw mid-write can't leave it stuck.
//
// The decision methods (needs_reload / would_write) are pure and DDImage-free,
// so they're unit-tested in test/test_blob_mirror.cpp. save()/the knob plumbing
// need DD::Image::Op and are exercised in-plugin.
// =============================================================================
#ifndef PCN_BLOB_MIRROR_H
#define PCN_BLOB_MIRROR_H

#include "pcn_scoped_guard.h"

#include <string>

// DDImage is only needed for the knob-writing convenience (save / current). The
// pure decision logic below compiles without it, which is what the unit test
// uses (it defines PCN_BLOB_MIRROR_NO_DDIMAGE before including this header).
#ifndef PCN_BLOB_MIRROR_NO_DDIMAGE
#include "DDImage/Op.h"
#include "DDImage/Knob.h"
#endif

namespace pcn {

class BlobMirror {
public:
    // `knob_name` is the hidden String_knob this mirror tracks; it must outlive
    // the mirror (always a string literal at the call sites).
    explicit BlobMirror(const char* knob_name) : name_(knob_name) {}

    const char*        knob_name() const { return name_; }
    const std::string& cache()     const { return cache_; }
    bool               suppressed() const { return suppress_; }

    // ---- pure decision logic (DDImage-free, unit-tested) -------------------

    // Would save() actually touch the knob for this content? (i.e. is it new?)
    bool would_write(const std::string& content) const { return content != cache_; }

    // Should on_*_changed() reload from this live knob string? True iff we are
    // NOT mid-write AND the live string differs from what we last serialized
    // (so genuine undo/redo/.nk-load changes reload, our own echo doesn't).
    // `live` may be null (treated as empty).
    bool needs_reload(const char* live) const
    {
        const std::string s(live ? live : "");
        return !suppress_ && s != cache_;
    }

    // Adopt `content` as the new cache without touching any knob. Used by the
    // load path (after a successful parse) and after a reload is consumed.
    void set_cache(const std::string& content) { cache_ = content; }
    void set_cache(const char* content) { cache_ = content ? content : ""; }

#ifndef PCN_BLOB_MIRROR_NO_DDIMAGE
    // ---- knob plumbing (in-plugin) -----------------------------------------

    // Write `content` to the knob iff it differs from the cache, suppressing our
    // own knob_changed echo for the duration of the write (exception-safe via
    // ScopedFlags). Updates the cache. No-op (and no undo entry) when unchanged.
    // Returns true if it wrote.
    bool save(DD::Image::Op* op, const std::string& content)
    {
        if (content == cache_) return false;
        cache_ = content;
        if (DD::Image::Knob* k = op->knob(name_)) {
            ScopedFlags guard(suppress_);
            k->set_text(content.c_str());
        }
        return true;
    }
#endif
    // NOTE: the reload path reads the knob's live value from the subsystem's
    // bound `const char*` member (e.g. mask_blob_), exactly as the hand-rolled
    // code did — `if (m.needs_reload(mask_blob_)) { ...parse...; m.set_cache(...);}`
    // — rather than via a Knob getter, so this helper makes no assumption about
    // which Knob read API the build exposes.

private:
    const char* name_;
    std::string cache_;
    bool        suppress_ = false;
};

} // namespace pcn

#endif // PCN_BLOB_MIRROR_H
