// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Peter Mercell
//
// Developed with assistance from Claude (Anthropic).

// =============================================================================
// pcn_scoped_guard.h — ScopedFlags: an RAII guard for the re-entrancy flags the
// plugin uses to swallow its own knob_changed() echoes
// (suppress_pin_blob_callback_, suppress_rot_callback_, suppress_trans_callback_,
// suppress_anchor_callback_, suppress_tracked_callback_,
// suppress_live_pose_callback_). The 3D-mask blob's equivalent flag now lives
// inside BlobMirror (pcn_blob_mirror.h), whose save() uses this same guard.
//
// WHY: every one of those flags used to be raised and lowered by hand —
//     flag = true;  ... set_value()/set_text() ...  flag = false;
// If anything between the two lines threw (Eigen, std::string, a Nuke call), the
// flag stayed stuck TRUE and the next genuine knob edit was silently ignored.
// Six flags × many call sites = six latent traps. ScopedFlags raises each flag
// in its constructor and RESTORES THE PRIOR VALUE in its destructor, so the
// flags come down on every path out of the scope — normal return, early return,
// or exception — and nesting composes correctly.
//
// This mirrors the UndoSuspend guard already used in tracker_gizmo.cpp, so the
// idiom is consistent across the plugin.
//
// USAGE — wrap the suppressed region in its own block so the guard lowers the
// flags exactly where the manual `= false;` used to sit:
//
//     {
//         ScopedFlags guard(suppress_trans_callback_, suppress_rot_callback_);
//         for (int i = 0; i < 7; ++i)
//             if (Knob* k = knob(kOffsetKnobNames[i])) k->set_value(s.v[i]);
//     }   // both flags restored here, even if a set_value above throws
//
// Up to four flags per guard (every current site needs at most two); bump kMax if
// a site ever needs more. Header-only, no allocation, no Nuke/Eigen dependency.
// =============================================================================
#ifndef PCN_SCOPED_GUARD_H
#define PCN_SCOPED_GUARD_H

#include <cstddef>
#include <initializer_list>

namespace pcn {

class ScopedFlags {
public:
    // Raise each flag (set true) now, remembering its prior value; the
    // destructor restores each prior value. Accepts 1..kMax bool lvalues.
    template <typename... Flags>
    explicit ScopedFlags(Flags&... flags) : n_(0)
    {
        static_assert(sizeof...(Flags) <= kMax,
                      "ScopedFlags: too many flags — raise kMax");
        // Fold over the pack via an initializer_list so evaluation order is
        // left-to-right and well-defined (pre-C++17-fold-expression friendly).
        (void)std::initializer_list<int>{ (raise(flags), 0)... };
    }

    ~ScopedFlags()
    {
        // Restore in reverse acquisition order (correct for nesting).
        for (std::size_t i = n_; i-- > 0; )
            *slot_[i] = prev_[i];
    }

    ScopedFlags(const ScopedFlags&)            = delete;
    ScopedFlags& operator=(const ScopedFlags&) = delete;
    ScopedFlags(ScopedFlags&&)                 = delete;
    ScopedFlags& operator=(ScopedFlags&&)      = delete;

private:
    static constexpr std::size_t kMax = 4;

    void raise(bool& f)
    {
        slot_[n_] = &f;
        prev_[n_] = f;
        f = true;
        ++n_;
    }

    bool*       slot_[kMax];
    bool        prev_[kMax];
    std::size_t n_;
};

} // namespace pcn

#endif // PCN_SCOPED_GUARD_H
