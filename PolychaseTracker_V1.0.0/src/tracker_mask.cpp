// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Peter Mercell
//
// Developed with assistance from Claude (Anthropic).

// =============================================================================
// tracker_mask.cpp — 3D mask storage + solver feeding for the PolychaseTracker
// plugin. Port of Polychase's per-triangle mask (cpp/geometry.h Mesh mask) into
// the Nuke NDK plugin. See MASKING_PLAN.md.
//
// What lives here (this pass): the persisted per-triangle bitset (mask_bits_),
// its hidden-knob round-trip (load/save), the bit ops, the topology-change guard
// (ensure_mask_sized), and build_mask_array — which hands the current mask to the
// AcceleratedMesh ctor at the Track / Refine build sites. The solver already
// passes check_mask=true to RayCast (tracker.cc / refiner.cc), so a populated
// bitset takes effect immediately; an EMPTY one yields an empty ArrayXu and is
// bit-identical to the pre-mask behaviour.
//
// Triangles are masked/unmasked from the 3D-viewer VERTEX selection
// (apply_3d_vertex_selection, via this node's GEOSELECT_KNOB): a triangle changes
// only when all three of its corner points are selected. The masked set tints in
// the 2D viewer (wireframe_knob.cpp). Mask Color / Clear Mask round out the UI.
//
// Storage format (mask_blob String_knob): the bitset words as concatenated
// 8-char hex (lowest-index word first). The triangle count travels separately in
// the INVISIBLE "mask_tri_count" Int_knob, so the blob is purely the bits.
//
// Masking affects Track and Refine (which RAYCAST feature points against the
// mesh). It deliberately does NOT touch the interactive pin solve: that's a
// vertex-pin PnP with no raycast, so there's no mask to consult there — pins are
// explicit user constraints by construction. (This refines MASKING_PLAN §3.3,
// which listed a pin-solve accel build that doesn't actually exist.)
// =============================================================================
#include "polychase_tracker.h"

#include "DDImage/GeoSelectKnobI.h"
#include "DDImage/ViewerContext.h"

#ifdef PCN_NEW_3D
// New-system (usg/USD) vertex selection lives in a separate singleton, not the
// classic GeoSelect knob. Exported in libDDImage (already linked).
#include "ndk/geo/selection/GlobalGeoSelection.h"  // ndk::Selection::GlobalGeoSelection
#include "ndk/geo/selection/GeoSelection.h"         // GeoSelection / WeightedPartSelection / PartType
#endif

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

using namespace DD::Image;

namespace pcn {

// -----------------------------------------------------------------------------
// Bit ops. Index by triangle order; out-of-range indices are ignored (the brush
// can momentarily produce a stale index between geo edits — never UB).
// -----------------------------------------------------------------------------
void PolychaseTracker::mask_triangle(uint32_t tri)
{
    const size_t w = tri >> 5;
    if (w < mask_bits_.size()) { mask_bits_[w] |= (1u << (tri & 31u)); ++mask_version_; }
}

void PolychaseTracker::unmask_triangle(uint32_t tri)
{
    const size_t w = tri >> 5;
    if (w < mask_bits_.size()) { mask_bits_[w] &= ~(1u << (tri & 31u)); ++mask_version_; }
}

bool PolychaseTracker::is_triangle_masked(uint32_t tri) const
{
    const size_t w = tri >> 5;
    return (w < mask_bits_.size()) && ((mask_bits_[w] >> (tri & 31u)) & 1u);
}


// -----------------------------------------------------------------------------
// Persistence — hex round-trip through the hidden "mask_blob" String_knob, with
// the triangle count in the "mask_tri_count" Int_knob. In-memory mask_bits_ is
// authoritative for the session once loaded/edited; the reload path
// (knob_changed) compares content vs. cache so our own async set_text echo is
// ignored — same guard as the pins blob.
// -----------------------------------------------------------------------------
void PolychaseTracker::load_mask_from_knob()
{
    mask_bits_.clear();

    // Triangle count comes from its own bound knob/member (set by Nuke on load).
    if (DD::Image::Knob* k = knob("mask_tri_count"))
        mask_tri_count_ = (int)k->get_value();

    const std::string hex(mask_blob_ ? mask_blob_ : "");
    const size_t nwords = hex.size() / 8;             // 8 hex chars per uint32
    mask_bits_.reserve(nwords);
    for (size_t i = 0; i + 8 <= hex.size(); i += 8) {
        uint32_t word = 0;
        for (size_t j = 0; j < 8; ++j) {
            const char c = hex[i + j];
            uint32_t nib = 0;
            if      (c >= '0' && c <= '9') nib = (uint32_t)(c - '0');
            else if (c >= 'a' && c <= 'f') nib = (uint32_t)(c - 'a' + 10);
            else if (c >= 'A' && c <= 'F') nib = (uint32_t)(c - 'A' + 10);
            word = (word << 4) | nib;
        }
        mask_bits_.push_back(word);
    }

    // Keep the bitset consistent with the recorded triangle count when known.
    if (mask_tri_count_ > 0) {
        const size_t words = ((size_t)mask_tri_count_ + 31u) / 32u;
        if (mask_bits_.size() != words) mask_bits_.resize(words, 0u);
    }

    mask_mirror_.set_cache(hex);
    mask_loaded_     = true;
    ++mask_version_;
}

void PolychaseTracker::save_mask_to_knob()
{
    std::string hex;
    hex.reserve(mask_bits_.size() * 8);
    char buf[9];
    for (uint32_t word : mask_bits_) {
        std::snprintf(buf, sizeof(buf), "%08x", word);
        hex.append(buf, 8);
    }

    // Write the blob through the mirror: it no-ops when unchanged (no spurious
    // undo entry) and holds the suppress flag via ScopedFlags for the duration of
    // the set_text, so a throw mid-write can't leave the callback gate stuck.
    mask_mirror_.save(this, hex);

    // The count is part of the persisted state too (mismatch guard reads it).
    if (DD::Image::Knob* k = knob("mask_tri_count")) k->set_value((double)mask_tri_count_);
}


// -----------------------------------------------------------------------------
// ensure_mask_sized — make mask_bits_ exactly ceil(num_triangles/32) words for
// the live mesh. If a mask was painted against a DIFFERENT triangle count, its
// indices no longer line up, so drop it (and log) — Blender carries the same
// implicit topology assumption. Safe to call every solve / paint.
// -----------------------------------------------------------------------------
void PolychaseTracker::ensure_mask_sized(uint32_t num_triangles)
{
    if (!mask_loaded_) load_mask_from_knob();

    const size_t words = ((size_t)num_triangles + 31u) / 32u;

    if (mask_tri_count_ != 0 && mask_tri_count_ != (int)num_triangles) {
        PCN_LOG("[mask] mesh triangle count changed (" << mask_tri_count_
                << " -> " << num_triangles << "); dropping the stale mask\n");
        mask_bits_.assign(words, 0u);
        mask_tri_count_ = (int)num_triangles;
        save_mask_to_knob();
        return;
    }

    if (mask_bits_.size() != words) mask_bits_.resize(words, 0u);
    mask_tri_count_ = (int)num_triangles;
}


// -----------------------------------------------------------------------------
// build_mask_array — the bridge into the solver. Reconciles the mask to the live
// mesh, then returns the per-triangle ArrayXu the AcceleratedMesh ctor wants.
// Returns an EMPTY array when nothing is masked, so the common case keeps the
// pre-mask fast path (and identical results). Sized to exactly ceil/32 otherwise,
// so the Mesh ctor's length CHECK is satisfied.
// -----------------------------------------------------------------------------
ArrayXu PolychaseTracker::build_mask_array(uint32_t num_triangles)
{
    ensure_mask_sized(num_triangles);

    bool any = false;
    for (uint32_t w : mask_bits_) { if (w) { any = true; break; } }
    if (!any) return ArrayXu();        // empty => no-op fast path

    // The core Mesh ctor (geometry.h) stores the per-triangle mask as a packed
    // uint32 bitset PADDED up to a multiple of 4 ints (mask_num_ints_padded) and
    // asserts masked_triangles.rows() >= that width. Our bitset is the UNPADDED
    // ceil(nt/32) words, so pad the handed-over copy up to the SAME multiple,
    // zero-filling the tail. Mirrors geometry.h exactly:
    //     mask_num_ints        = (nt + 31) / 32;          // == mask_bits_.size()
    //     mask_num_ints_padded = n + (4 - n % 4) % 4;     // round up to mult of 4
    // The pad words cover triangle indices past num_triangles (which don't
    // exist), so they are correctly unmasked (0).
    constexpr Eigen::Index kMaskIntPad = 4;
    const Eigen::Index nwords = (Eigen::Index)mask_bits_.size();
    const Eigen::Index padded = ((nwords + kMaskIntPad - 1) / kMaskIntPad) * kMaskIntPad;

    ArrayXu masked(padded);
    masked.setZero();
    for (Eigen::Index i = 0; i < nwords; ++i)
        masked(i) = mask_bits_[(size_t)i];
    return masked;
}


// -----------------------------------------------------------------------------
// clear_mask — wipe every bit (button + topology-mismatch reset). Keeps the
// bitset sized to the current count so a subsequent paint still lines up.
// -----------------------------------------------------------------------------
void PolychaseTracker::clear_mask()
{
    if (!mask_loaded_) load_mask_from_knob();
    std::fill(mask_bits_.begin(), mask_bits_.end(), 0u);
    ++mask_version_;
    save_mask_to_knob();
    set_status("[" + timestamp() + "] Mask cleared.");
    invalidate();    // dirty the op so the tint clears from the overlay immediately
    asapUpdate();
}


// -----------------------------------------------------------------------------
// apply_3d_vertex_selection — apply the 3D-viewer VERTEX selection to the mask.
// The node hosts a GEOSELECT_KNOB and is itself the 3D selection target, so
// getSelectedItems(eSelect3DVertex) hands back the selected point indices per
// object. Walking the SAME fan as extract_first_object_mesh (so the triangle
// index == the mask_bits_ bit == RayHit::primitive_id), a triangle is changed
// only when ALL THREE of its corner points are selected — STRICT, no spill past
// the selected region. erase=false masks; erase=true unmasks (the two buttons).
// One Undo step (save_mask_to_knob). The masked-triangle tint is the confirmation.
// -----------------------------------------------------------------------------
void PolychaseTracker::apply_3d_vertex_selection(bool erase)
{
    Knob* k = knob("geo_select");
    if (!k) {
        PCN_LOG("[mask] selected: no 'geo_select' knob on this node\n");
        set_status("[" + timestamp() + "] Mask Selected: no geo_select knob.");
        return;
    }
    GeoSelect_KnobI* gsk = k->geoSelectKnob();
    if (!gsk) {
        PCN_LOG("[mask] selected: 'geo_select' is not a GEOSELECT_KNOB\n");
        set_status("[" + timestamp() + "] Mask Selected: geo_select has no "
                   "GeoSelect interface.");
        return;
    }

    // Read the per-vertex 3D selection routed to this node (point indices per object).
    Op::ItemSelectionList items;   // map<Hash, vector<unsigned pointIndex>>
    gsk->getSelectedItems(eSelect3DVertex, items);

    // Mesh for object 0 — geometry-agnostic: a classic GeoOp or a new-system
    // GeomOp (GeoCube) both yield the same GeoMesh (LOCAL verts + fan triangles),
    // and triangle index == mask bit == RayHit::primitive_id for either path.
    Op* geo = input_geo_op();
    if (!geo) { set_status("[" + timestamp() + "] Mask Selected: no geo input."); return; }
    geo->validate(true);
    GeoMesh gm;
    if (!extract_mesh(geo, gm)) {
        set_status("[" + timestamp() + "] Mask Selected: could not extract a mesh "
                   "from the geo input.");
        return;
    }
    const Eigen::Index ntri = gm.triangles.rows();
    if (ntri == 0) {
        set_status("[" + timestamp() + "] Mask Selected: geo has no triangles.");
        return;
    }

    // Resolve the selected vertex indices (same index space as gm.triangles /
    // gm.local_vertices rows). There are TWO selection systems, picked by geo type:
    //   • Classic GeoOp  -> this node's GeoSelect knob (point indices per object,
    //                       disambiguated via geoID; multi-object safe).
    //   • New-system usg -> the ndk::Selection::GlobalGeoSelection singleton (keyed
    //                       by usg::Path). The viewer routes GeoCube/USD vertex
    //                       selection THERE, not into the classic knob, so we read it
    //                       directly. Single GeoCube => one path, and its getPoints
    //                       index order == our extract_mesh vertex order, so selected
    //                       vertex i maps straight onto gm.local_vertices row i.
    std::unordered_set<unsigned> sel;

    if (GeoOp* cgeo = dynamic_cast<GeoOp*>(geo)) {            // classic geometry
        const std::vector<unsigned>* verts = nullptr;
        Scene scene;
        GeometryList glist;
        cgeo->get_geometry(scene, glist);
        if (glist.objects() > 0) {
            const Hash id = GeoSelection::geoID(glist[0]);
            auto it = items.find(id);
            if (it == items.end() && items.size() == 1) it = items.begin();
            if (it != items.end()) verts = &it->second;
        }
        if (!verts && items.size() == 1) verts = &items.begin()->second;
        if (verts) sel.insert(verts->begin(), verts->end());
    }
#ifdef PCN_NEW_3D
    else {                                                    // new-system (usg/USD) geometry
        namespace ndksel = ndk::Selection;
        const ndksel::GeoSelection& gsel =
            ndksel::GlobalGeoSelection::GetInstance().getCurrent();
        for (auto it = gsel.begin(); it != gsel.end(); ++it) {
            const ndksel::WeightedPartSelection& vsel =
                it->second.getPartSelection(ndksel::PartType::Vertices);
            const size_t n = vsel.selections.size();
            for (size_t i = 0; i < n; ++i)
                if (vsel.selections[i] != 0.0f) sel.insert((unsigned)i);
        }
    }
#endif

    if (sel.empty()) {
        set_status("[" + timestamp() + "] Mask Selected: no vertices selected on this "
                   "node. Select vertices in the 3D viewer with this node active.");
        PCN_LOG("[mask] selected: no vertices (classic knob entries=" << items.size() << ")\n");
        return;
    }


    ensure_mask_sized((uint32_t)ntri);

    // STRICT all-3: a triangle changes only when all three of its corner point
    // indices are selected — no spill past the selected region. The triangle's
    // running index == its mask bit (gm.triangles is the same fan the solver rays).
    size_t changed = 0;
    for (Eigen::Index ti = 0; ti < ntri; ++ti) {
        const unsigned a = (unsigned)gm.triangles(ti, 0);
        const unsigned b = (unsigned)gm.triangles(ti, 1);
        const unsigned c = (unsigned)gm.triangles(ti, 2);
        if (sel.count(a) && sel.count(b) && sel.count(c)) {
            if (erase) unmask_triangle((uint32_t)ti);
            else       mask_triangle((uint32_t)ti);
            ++changed;
        }
    }

    save_mask_to_knob();
    // A NoIop's image output doesn't change when the mask does, so asapUpdate()
    // alone leaves the viewer with no reason to re-cook — the masked-triangle tint
    // stays stale until a frame change forces a redraw. invalidate() dirties the
    // op so the overlay repaints immediately (same effect a hashing-knob edit has).
    invalidate();
    asapUpdate();

    unsigned max_sel = 0;
    for (unsigned v : sel) max_sel = std::max(max_sel, v);

    std::ostringstream msg;
    msg << "[" << timestamp() << "] Mask Selected: " << sel.size()
        << " vertices -> " << changed << " triangle(s) "
        << (erase ? "unmasked." : "masked.");
    set_status(msg.str());
    PCN_LOG("[mask] selected(vtx): verts=" << sel.size() << " maxIdx=" << max_sel
            << " tris=" << ntri << " changed=" << changed
            << (erase ? " unmasked\n" : " masked\n"));
}

} // namespace pcn