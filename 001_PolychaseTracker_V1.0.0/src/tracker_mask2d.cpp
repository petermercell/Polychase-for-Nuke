// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Peter Mercell
//
// Developed with assistance from Claude (Anthropic).

// =============================================================================
// tracker_mask2d.cpp — 2D occlusion mask ("mask plate") for the PolychaseTracker
// plugin. Samples the alpha of the node's 4th input ("mask", e.g. a Roto) per
// frame and hands Track/Refine a MaskPredicate (TrackerOptions/RefinerOptions::
// is_masked, added to polychase core in user_constraints.h) that excludes feature
// points falling in the masked region — for occluders that pass IN FRONT of the
// tracked object. See MASK_2D_PLAN.md.
//
// What lives here:
//   sample_mask2d_frame   — read the mask input's alpha at one frame into a
//                           packed Y-up bitset (set bit = excluded), per mode.
//   make_mask2d_predicate — pre-render a frame range into a shared bitset stack
//                           and return the std::function the solver calls.
//   mask2d_overlay_bits   — cached one-frame bitset for the viewer "Show Masked"
//                           tint (drawn in wireframe_knob.cpp).
//
// CONVENTION (the load-bearing part):
//   The optical-flow database keypoints are REAL-PIXEL, Y-DOWN (top-origin),
//   OpenCV — that's what the predicate receives. A Nuke Iop is Y-UP (origin
//   bottom-left). We therefore store the bitset in Nuke Y-up orientation (so the
//   viewer overlay, which draws in Y-up image-pixel space, maps to it 1:1 with no
//   flip), and the predicate flips the incoming y-down keypoint:
//       y_up = (h - 1) - round(y_down)
//   The mask format is assumed to match the plate (img) format; if it doesn't,
//   the row/col indices won't line up with the keypoints.
//
// OFF by default: mode 0 (None) or no mask input => empty predicate => the solve
// is bit-identical to before.
// =============================================================================
#include "polychase_tracker.h"

#include "DDImage/Hash.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

using namespace DD::Image;

namespace pcn {

namespace {

// Packed Y-up bitset stack for a contiguous frame range. Shared (shared_ptr) by
// the predicate lambda so it outlives make_mask2d_predicate and is cheap to copy.
struct Mask2DStack {
    int w = 0, h = 0;
    int first = 0, count = 0;
    int words_per_row = 0;
    std::vector<uint32_t> bits;   // count * h * words_per_row, Y-up, set = masked

    bool masked(int32_t frame, float x_ydown, float y_ydown) const {
        if (frame < first || frame >= first + count) return false;
        const int xi = (int)std::lround(x_ydown);
        // keypoints are Y-down (top origin); our bits are Y-up (bottom origin).
        const int yi = (h - 1) - (int)std::lround(y_ydown);
        if (xi < 0 || xi >= w || yi < 0 || yi >= h) return false;
        const uint32_t* row =
            &bits[((size_t)(frame - first) * h + yi) * words_per_row];
        return (row[xi >> 5] >> (xi & 31)) & 1u;
    }
};

}  // namespace


// -----------------------------------------------------------------------------
// sample_mask2d_frame — read the mask input's alpha at `frame` into a packed Y-up
// bitset (set bit = excluded). Returns false when the 2D mask is off / no input /
// the input has no usable format. Restores the mask op's OutputContext on the way
// out (same pattern as cam_world_at in tracker_track.cpp).
//
// VERIFY ON BUILD: the Row/get scanline read is the standard NDK way to sample an
// input's pixels outside engine(), but exact Row indexing can vary by SDK — if the
// overlay/solve mask comes out empty or shifted, check this loop first.
// -----------------------------------------------------------------------------
bool PolychaseTracker::sample_mask2d_frame(int frame, int& out_w, int& out_h,
                                           std::vector<uint32_t>& bits) const
{
    const int mode = mask2d_mode();
    Iop* m = input_mask();
    if (mode == 0 || !m) return false;

    // Sample at `frame` (swap the context, validate, restore at the end).
    OutputContext oc0 = m->outputContext();
    OutputContext oc  = oc0;
    oc.setFrame((double)frame);
    m->setOutputContext(oc);
    m->validate(true);

    const Format& fmt = m->info().format();
    const int w = fmt.width();
    const int h = fmt.height();
    if (w <= 0 || h <= 0) {
        m->setOutputContext(oc0);
        m->validate(true);
        return false;
    }

    const float thr    = (float)mask2d_threshold();
    const bool  invert = (mode == 2);   // "Mask Alpha Inverted": exclude alpha < thr
    const int   wpr    = (w + 31) / 32;
    bits.assign((size_t)wpr * h, 0u);

    ChannelSet chans(Mask_Alpha);
    m->request(0, 0, w, h, chans, 1);

    Row row(0, w);
    for (int y = 0; y < h; ++y) {                 // y is Nuke Y-up
        m->get(y, 0, w, chans, row);
        const float* a = row[Chan_Alpha];
        uint32_t* rb = &bits[(size_t)y * wpr];
        for (int x = 0; x < w; ++x) {
            const float av = a ? a[x] : 0.0f;
            const bool masked = invert ? (av < thr) : (av >= thr);
            if (masked) rb[x >> 5] |= (1u << (x & 31));
        }
    }

    m->setOutputContext(oc0);
    m->validate(true);

    out_w = w;
    out_h = h;
    return true;
}


// -----------------------------------------------------------------------------
// make_mask2d_predicate — pre-render [from,to] into a shared bitset stack and
// return the MaskPredicate the solver calls per correspondence. Empty (no-op)
// when the 2D mask is off / no input / nothing readable.
// -----------------------------------------------------------------------------
MaskPredicate PolychaseTracker::make_mask2d_predicate(int from, int to)
{
    if (mask2d_mode() == 0 || !input_mask()) return MaskPredicate();
    if (to < from) std::swap(from, to);

    auto stack = std::make_shared<Mask2DStack>();
    stack->first = from;
    stack->count = to - from + 1;

    int W = 0, H = 0, WPR = 0;
    bool sized = false;

    for (int f = from; f <= to; ++f) {
        int fw = 0, fh = 0;
        std::vector<uint32_t> b;
        const bool ok = sample_mask2d_frame(f, fw, fh, b);
        if (ok && !sized) {
            W = fw; H = fh; WPR = (W + 31) / 32;
            stack->w = W; stack->h = H; stack->words_per_row = WPR;
            stack->bits.assign((size_t)stack->count * H * WPR, 0u);
            sized = true;
        }
        if (ok && sized && fw == W && fh == H) {
            const size_t off = (size_t)(f - from) * H * WPR;
            std::copy(b.begin(), b.end(), stack->bits.begin() + (std::ptrdiff_t)off);
        }
        // A frame that failed to sample (or changed size) stays all-zero =
        // nothing masked there, which is the safe default.
    }

    if (!sized) return MaskPredicate();

    return [stack](int32_t frame, const Eigen::Vector2f& px) -> bool {
        return stack->masked(frame, px.x(), px.y());
    };
}


// -----------------------------------------------------------------------------
// mask2d_overlay_bits — cached one-frame Y-up bitset for the "Show Masked" viewer
// tint. Rebuilds only when the frame / mode / threshold / mask input changes, so
// repeated redraws on the same frame are free. Returns nullptr when off.
// -----------------------------------------------------------------------------
const std::vector<uint32_t>* PolychaseTracker::mask2d_overlay_bits(int frame,
                                                                   int& w, int& h)
{
    Iop* m = input_mask();
    if (mask2d_mode() == 0 || !m) {
        std::lock_guard<std::mutex> lock(mask2d_cache_mtx_);
        mask2d_cache_valid_ = false;
        return nullptr;
    }

    const int    mode  = mask2d_mode();
    const double thr   = mask2d_threshold();
    const void*  inptr = (const void*)m;

    // Identity of the mask input that actually affects the bits: the op pointer
    // (catches a different node wired in) AND its hash (catches the SAME node's
    // content/animation changing, and disambiguates a freed op reallocated at the
    // same address — that op carries a different hash). validate(true) populates
    // the hash at the current context; it's cheap/idempotent if already valid.
    m->validate(true);
    const uint64_t inhash = m->hash().value();

    std::lock_guard<std::mutex> lock(mask2d_cache_mtx_);

    if (mask2d_cache_valid_ && mask2d_cache_frame_ == frame &&
        mask2d_cache_mode_ == mode && mask2d_cache_thr_ == thr &&
        mask2d_cache_inptr_ == inptr && mask2d_cache_hash_ == inhash &&
        mask2d_cache_w_ > 0) {
        w = mask2d_cache_w_;
        h = mask2d_cache_h_;
        return &mask2d_cache_bits_;
    }

    int fw = 0, fh = 0;
    std::vector<uint32_t> b;
    if (!sample_mask2d_frame(frame, fw, fh, b)) {
        mask2d_cache_valid_ = false;
        return nullptr;
    }

    mask2d_cache_bits_  = std::move(b);
    mask2d_cache_frame_ = frame;
    mask2d_cache_mode_  = mode;
    mask2d_cache_thr_   = thr;
    mask2d_cache_inptr_ = inptr;
    mask2d_cache_hash_  = inhash;
    mask2d_cache_w_     = fw;
    mask2d_cache_h_     = fh;
    mask2d_cache_valid_ = true;

    w = fw;
    h = fh;
    return &mask2d_cache_bits_;
}

} // namespace pcn
