// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Peter Mercell
//
// Developed with assistance from Claude (Anthropic).

// =============================================================================
// tracker_usertracks.cpp — User (helper) tracks for the PolychaseTracker plugin.
// 2D tracks made with a Nuke Tracker node are fed into the solve as extra
// 2D<->3D correspondences. The polychase core consumes them via
// TrackerOptions/RefinerOptions::user_tracks (see user_constraints.h);
// This file ingests + anchors them. See USER_TRACKS_PLAN.md.
//
// Pipeline:
//   1. Load Tracks  -> a Python callback (_polychase_collect_user_tracks in
//      menu.py) reads the named Tracker node and writes the RAW 2D tracks (Nuke
//      y-up px) into the hidden 'user_tracks_blob'.
//   2. rebuild_user_tracks() parses the blob and, on the Reference Frame, RAY-CASTS
//      each track's position onto the mesh to get a rigid OBJECT-space anchor.
//      A track whose ray MISSES the mesh is "not on the object" and is dropped —
//      "only trackers on the object count".
//   3. Each track becomes a UserTrack { object_point, weight, observations[] } with
//      its per-frame positions converted to the solver convention; Track/Refine
//      add them as constraints.
//
// CONVENTION BRIDGE — same as Track (tracker_track.cpp). The optical-flow DB and
// the solver use OpenCV / Y-DOWN / real-pixel coords; a Nuke Tracker reports Y-UP
// pixels. The flip is  y_cv = h - y_nuke  (image-centre maps to image-centre, the
// same centred principal point tracker_intrinsics assumes). The anchor ray-cast
// uses the identical OpenCV intrinsics + Y/Z-flipped view that SolveFrame uses, so
// a track anchors to the same surface point the optical-flow points would.
//
// VERIFY ON BUILD: anchoring uses RayCast + the file-local convention helpers
// (duplicated from tracker_track.cpp, as tracker_refine.cpp also does). If tracks
// anchor to the wrong place, the y-flip (h vs h-1) and the GL->CV view flip are the
// first suspects — flip them together with the Track path.
// =============================================================================
#include "polychase_tracker.h"

// Shared OpenGL<->OpenCV convention bridge (one definition for Track/Refine/
// user-track anchoring); aliased to the uts_ names below.
#include "pcn_convention.h"

#include <Eigen/LU>          // RowMajorMatrix4f::inverse() (via nuke_to_eigen + RayCast)

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

using namespace DD::Image;

namespace pcn {

// -----------------------------------------------------------------------------
// Convention helpers. The GL<->CV bridge is shared (pcn_convention.h); aliased
// here under the file's uts_ names so the call sites are unchanged and the math
// can never drift from Track / Refine. uts_cam_basis_at stays local (it samples
// a camera at a frame, restoring its OutputContext) since user-track anchoring
// needs both projection() and imatrix() together.
// -----------------------------------------------------------------------------
namespace {

constexpr auto uts_gl_to_cv_flip          = &conv::gl_to_cv_flip;
constexpr auto uts_tracker_intrinsics     = &conv::tracker_intrinsics;
constexpr auto uts_build_local_accel_mesh = &conv::build_local_accel_mesh;

// Sample a camera's projection + world-to-camera (imatrix) at one frame, restoring
// the op's OutputContext afterwards (mirrors cam_world_at in tracker_track.cpp).
void uts_cam_basis_at(DD::Image::CameraOp* cam, double frame,
                      DD::Image::Matrix4& proj, DD::Image::Matrix4& imat)
{
    OutputContext orig = cam->outputContext();
    OutputContext c    = orig;
    c.setFrame(frame);
    cam->setOutputContext(c);
    cam->validate(true);
    proj = cam->projection();
    imat = cam->imatrix();
    cam->setOutputContext(orig);
    cam->validate(true);

    // FOCAL FROM THE ANIMATION CURVE (not the cooked projection). cam->projection()
    // does NOT re-evaluate an ANIMATED focal from a forced OutputContext — it returns
    // the panel-frame value frozen (same bug fixed in track_via_pins; diagnosed as a
    // flat/ wandering a00 while the focal was actually animated). The anchoring builds
    // intrinsics from this proj via uts_tracker_intrinsics (fx = |a00|*w/2), so the
    // stale focal mis-rays the ref-frame anchor points and tracks intermittently DROP
    // (ray-miss). Read the camera's focal/haperture straight off the knobs with
    // get_value_at(frame) — no cook — and rewrite the diagonal projection terms:
    // a00 = a11 = 2*focal_mm/haperture in this uniform-(w/2)-scale convention. Sign is
    // preserved (uts_tracker_intrinsics takes abs anyway). a02/a12 (principal/lens
    // shift) are left untouched. Static lens => identical value, so non-zoom anchoring
    // is unchanged. (imatrix is correct for a static camera; animated camera POSITION
    // would need its own curve-read and is out of scope here.)
    double foc_mm = 0.0, hap = 24.576;
    if (Knob* fk = cam->knob("focal"))     foc_mm = fk->get_value_at(frame);
    if (Knob* hk = cam->knob("haperture")) hap    = hk->get_value_at(frame);
    if (std::isfinite(foc_mm) && foc_mm > 1e-6 && hap > 1e-6) {
        const float a = (float)(2.0 * foc_mm / hap);
        proj.a00 = (proj.a00 < 0.0f) ? -a : a;
        proj.a11 = (proj.a11 < 0.0f) ? -a : a;
    }
}

// Raw (pre-anchor) track parsed from the blob: a stable name + per-frame Nuke
// y-up pixel positions.
struct RawObs  { int frame; float x; float y; };
struct RawTrack { std::string name; std::vector<RawObs> obs; };

// Parse the blob: tracks separated by ';'. Each track is "name|obs obs ..." where
// obs is "frame,x,y" (separated by ' '). The "name|" prefix is the stable link id;
// an older blob with no '|' parses as an unnamed track (name stays empty).
std::vector<RawTrack> uts_parse_blob(const char* s)
{
    std::vector<RawTrack> out;
    if (!s || !*s) return out;
    std::string text(s);

    size_t tstart = 0;
    while (tstart <= text.size()) {
        size_t tend = text.find(';', tstart);
        if (tend == std::string::npos) tend = text.size();
        std::string track = text.substr(tstart, tend - tstart);

        RawTrack rt;
        // Optional "name|" prefix (first '|' splits the name from the observations).
        const size_t bar = track.find('|');
        if (bar != std::string::npos) {
            rt.name = track.substr(0, bar);
            track   = track.substr(bar + 1);
        }

        size_t ostart = 0;
        while (ostart <= track.size()) {
            size_t oend = track.find(' ', ostart);
            if (oend == std::string::npos) oend = track.size();
            const std::string ob = track.substr(ostart, oend - ostart);
            if (!ob.empty()) {
                // "frame,x,y"
                const size_t c1 = ob.find(',');
                const size_t c2 = (c1 == std::string::npos) ? std::string::npos
                                                            : ob.find(',', c1 + 1);
                if (c1 != std::string::npos && c2 != std::string::npos) {
                    try {
                        RawObs o;
                        o.frame = std::stoi(ob.substr(0, c1));
                        o.x     = std::stof(ob.substr(c1 + 1, c2 - c1 - 1));
                        o.y     = std::stof(ob.substr(c2 + 1));
                        rt.obs.push_back(o);
                    } catch (...) { /* skip malformed obs */ }
                }
            }
            if (oend == track.size()) break;
            ostart = oend + 1;
        }
        if (!rt.obs.empty()) out.push_back(std::move(rt));

        if (tend == text.size()) break;
        tstart = tend + 1;
    }
    return out;
}

}  // namespace


// -----------------------------------------------------------------------------
// rebuild_user_tracks — parse the blob and anchor each track to the mesh at the
// Reference Frame (RayCast). Off-object tracks (ray miss / no key on the ref
// frame) are dropped. Fills user_tracks_ and the total / on-object counts.
// -----------------------------------------------------------------------------
void PolychaseTracker::rebuild_user_tracks()
{
    user_tracks_.clear();
    user_track_names_.clear();
    user_tracks_total_     = 0;
    user_tracks_on_object_ = 0;

    // Read the blob from the LIVE knob, not the bound member. Python's setValue
    // (from Load Tracks' script_command) updates the knob immediately, but the
    // bound user_tracks_blob_ pointer only refreshes on the next store() — so
    // right after a Load it is still the stale (old/empty) value, which made the
    // rebuild parse 0 tracks. get_text() returns the current text (it pairs with
    // the set_text() this file already uses on the same knob); the member is a
    // fallback for any knob that doesn't supply text.
    std::string blob;
    if (DD::Image::Knob* kb = knob("user_tracks_blob")) {
        if (const char* t = kb->get_text()) blob = t;
    }
    if (blob.empty() && user_tracks_blob_) blob = user_tracks_blob_;

    std::vector<RawTrack> raw = uts_parse_blob(blob.c_str());
    user_tracks_total_ = (int)raw.size();
    if (raw.empty()) { refresh_user_tracks_label(); return; }

    CameraOp* cam = input_cam();
    Op*       geo = input_geo_op();
    Iop*      img = input_img();
    if (!cam || !geo) { refresh_user_tracks_label(); return; }   // need cam + geo to anchor
    cam->validate(true);
    geo->validate(true);
    if (img) img->validate(true);

    float w = 0.0f, h = 0.0f;
    if (img) {
        const Format& fmt = img->info().format();
        w = (float)fmt.width();
        h = (float)fmt.height();
    }
    if (w <= 0.0f || h <= 0.0f) { refresh_user_tracks_label(); return; }   // need a format

    GeoMesh gm;
    if (!extract_mesh(geo, gm) || gm.local_vertices.rows() < 3) {
        refresh_user_tracks_label();
        return;
    }
    std::shared_ptr<AcceleratedMesh> accel =
        uts_build_local_accel_mesh(gm, build_mask_array((uint32_t)gm.triangles.rows()));

    const int ref = user_track_ref_;
    DD::Image::Matrix4 proj, imat;
    uts_cam_basis_at(cam, (double)ref, proj, imat);

    const RowMajorMatrix4f flip    = uts_gl_to_cv_flip();
    const CameraIntrinsics intr    = uts_tracker_intrinsics(proj, w, h);
    const RowMajorMatrix4f view_cv = flip * nuke_to_eigen_m4(imat);
    const RowMajorMatrix4f model0  = nuke_to_eigen_m4(pose_matrix_to_nuke((double)ref));

    SceneTransformations st;
    st.model_matrix = model0;
    st.view_matrix  = view_cv;
    st.intrinsics   = intr;

    int ti_dbg = -1;
    int drop_no_ref = 0, drop_ray_miss = 0, recovered_overscan = 0;
    const float overscan = (float)std::max(0, user_track_overscan_);
    for (const RawTrack& rt : raw) {
        ++ti_dbg;
        // The anchor is the track's position on the reference frame. No key there
        // => can't anchor this track.
        const RawObs* refobs = nullptr;
        for (const RawObs& o : rt.obs) {
            if (o.frame == ref) { refobs = &o; break; }
        }
        if (!refobs) {
            ++drop_no_ref;
            // Show the track's actual frame span so it's obvious whether the
            // Reference Frame simply falls outside this track's life.
            int f0 = rt.obs.empty() ? 0 : rt.obs.front().frame;
            int f1 = rt.obs.empty() ? 0 : rt.obs.back().frame;
            PCN_LOG("[utracks] track " << ti_dbg << " DROP no key @ ref=" << ref
                    << " (track spans " << f0 << ".." << f1
                    << ", " << rt.obs.size() << " keys)\n");
            continue;
        }

        // Nuke y-up -> OpenCV y-down, then ray-cast onto the mesh (check_mask=true,
        // so a track over 3D-masked geometry is treated as off-object).
        const Eigen::Vector2f kp(refobs->x, h - refobs->y);
        std::optional<RayHit> hit = RayCast(*accel, st, kp, true);

        // Overscan: a track sitting just off a tight silhouette ray-misses. Probe
        // outward in rings (~2px steps, 12 directions) and snap to the nearest
        // surface point within `overscan` pixels, so edge tracks still anchor.
        if (!hit && overscan > 0.0f) {
            const int rings = std::max(1, (int)std::ceil(overscan / 2.0f));
            const int dirs  = 12;
            for (int ri = 1; ri <= rings && !hit; ++ri) {
                const float rad = overscan * (float)ri / (float)rings;
                for (int di = 0; di < dirs; ++di) {
                    const float ang = (float)(2.0 * M_PI * di / dirs);
                    const Eigen::Vector2f probe(kp.x() + rad * std::cos(ang),
                                                kp.y() + rad * std::sin(ang));
                    if (auto h2 = RayCast(*accel, st, probe, true)) { hit = h2; break; }
                }
            }
            if (hit) {
                ++recovered_overscan;
                PCN_LOG("[utracks] track " << ti_dbg << " RECOVERED via overscan ("
                        << (int)overscan << "px) @ ref=" << ref << "\n");
            }
        }

        if (!hit) {
            ++drop_ray_miss;
            PCN_LOG("[utracks] track " << ti_dbg << " DROP ray miss @ ref=" << ref
                    << " kp_nuke=(" << refobs->x << "," << refobs->y
                    << ") kp_cv=(" << kp.x() << "," << kp.y() << ")\n");
            continue;   // off the object
        }
        PCN_LOG("[utracks] track " << ti_dbg << " ANCHOR @ ref=" << ref
                << " kp_nuke=(" << refobs->x << "," << refobs->y
                << ") -> obj(" << hit->pos.x() << "," << hit->pos.y()
                << "," << hit->pos.z() << ")\n");

        UserTrack ut;
        ut.object_point = hit->pos;   // OBJECT/local space (model applied by the solver)
        ut.weight       = 1.0f;       // per-track; the global weight is opts.user_track_weight
        ut.observations.reserve(rt.obs.size());
        for (const RawObs& o : rt.obs) {
            UserTrackObservation ob;
            ob.frame_id    = o.frame;
            ob.image_point = Eigen::Vector2f(o.x, h - o.y);   // y-down
            ut.observations.push_back(ob);
        }
        user_tracks_.push_back(std::move(ut));
        user_track_names_.push_back(rt.name);   // parallel: stable id for this anchored track
        ++user_tracks_on_object_;
    }

    PCN_LOG("[utracks] anchored " << user_tracks_on_object_ << "/" << raw.size()
            << " @ ref=" << ref << "  (dropped: " << drop_no_ref
            << " no-ref-key, " << drop_ray_miss << " ray-miss; recovered "
            << recovered_overscan << " via overscan)\n");

    // The anchored set just changed — re-bind every pin's stable link name to its
    // current index here (or -1 if its track is no longer in the set).
    reresolve_pin_links();

    refresh_user_tracks_label();
}


// -----------------------------------------------------------------------------
// reresolve_pin_links — map each pin's STABLE linked_track_name to the current
// anchored-set index (pin.linked_track). Called after every user-track rebuild so
// reloading / reordering / dropping tracks can never leave a pin bound to the
// wrong track: a name that's no longer present resolves to -1 (pin shows
// unlinked), and a name that moved follows to its new index. Pins that predate
// named links (empty name but a stored index) are left as-is for back-compat.
// -----------------------------------------------------------------------------
void PolychaseTracker::reresolve_pin_links()
{
    // Need BOTH sides present. If pins aren't loaded yet, nothing to bind; if the
    // anchored set is empty (tracks not built yet, or none anchored), DON'T clobber
    // stored links to -1 — a stale index against an empty set is harmless
    // (resolve_pin_2d ignores it), and the next rebuild with real names re-binds.
    if (pins_.empty() || user_track_names_.empty()) return;
    int rebound = 0, lost = 0;
    for (Pin& p : pins_) {
        if (p.linked_track_name.empty()) continue;     // legacy index-only link: leave it
        int found = -1;
        for (size_t i = 0; i < user_track_names_.size(); ++i)
            if (user_track_names_[i] == p.linked_track_name) { found = (int)i; break; }
        if (found != p.linked_track) {
            if (found < 0) ++lost; else ++rebound;
            p.linked_track = found;
        }
    }
    if (rebound || lost)
        PCN_LOG("[links] reresolved pin links: " << rebound << " rebound, "
                << lost << " lost (track name gone)\n");
}


// -----------------------------------------------------------------------------
// Cache signature — the anchors depend on the raw tracks, the reference frame, the
// 3D mask, and the connected cam/geo/img. (Geometry edits that keep the same op
// pointer but deform the mesh won't bump this; proxies are rigid, so that's fine —
// re-Load if you reshape the proxy.)
// -----------------------------------------------------------------------------
std::string PolychaseTracker::user_tracks_signature() const
{
    std::ostringstream o;
    o << (user_tracks_blob_ ? user_tracks_blob_ : "")
      << "|r" << user_track_ref_
      << "|o" << user_track_overscan_
      << "|m" << mask_version_
      << "|c" << (const void*)input_cam()
      << "|g" << (const void*)input_geo_op()
      << "|i" << (const void*)input_img();
    return o.str();
}

void PolychaseTracker::ensure_user_tracks_built()
{
    const std::string sig = user_tracks_signature();
    if (user_tracks_valid_ && sig == user_tracks_sig_) return;
    rebuild_user_tracks();
    user_tracks_sig_   = sig;
    user_tracks_valid_ = true;
}

const UserTracks& PolychaseTracker::user_tracks_for_solve()
{
    ensure_user_tracks_built();
    return user_tracks_;
}


void PolychaseTracker::refresh_user_tracks_label()
{
    std::ostringstream o;
    if (user_tracks_total_ == 0) {
        o << "No user tracks loaded. Name a Tracker node and press Load Tracks.";
    } else {
        o << user_tracks_total_ << " track(s) loaded; "
          << user_tracks_on_object_ << " anchored on the object (used by the solve).";
        if (user_tracks_on_object_ == 0)
            o << "\n  None hit the mesh on the Reference Frame — check the ref frame "
                 "and that the proxy is aligned over the tracked features.";
    }
    if (DD::Image::Knob* k = knob("user_track_list")) k->set_text(o.str().c_str());
}


// -----------------------------------------------------------------------------
// on_load_user_tracks — pull the raw 2D tracks from the named Tracker node into
// the hidden blob via the menu.py Python callback, then rebuild the anchors. The
// callback returns a short status string (script_result).
// -----------------------------------------------------------------------------
void PolychaseTracker::on_load_user_tracks()
{
    const std::string src(user_track_src_ ? user_track_src_ : "");
    if (src.empty()) {
        set_status("[" + timestamp() + "] Load Tracks: type a Tracker node name in "
                   "'Tracker Node' first.");
        return;
    }

    // Nuke 17: Op::node_name() returns std::string (was const char* in 16 and
    // earlier). Take it by value — the empty check and concatenation below are
    // unchanged.
    const std::string me = node_name();
    if (me.empty()) {
        set_status("[" + timestamp() + "] Load Tracks: could not resolve this node's name.");
        return;
    }
    const std::string cmd =
        "_polychase_collect_user_tracks('" + me + "','" + src + "')";

    std::string res;
    const bool ok = this->script_command(cmd.c_str());   // py=true, eval=true
    if (const char* r = Op::script_result()) res = r;
    Op::script_unlock();

    // Force a rebuild; the blob knob_changed will also fire and rebuild, but do it
    // here too so the summary/counts are current right after the click.
    user_tracks_valid_ = false;
    ensure_user_tracks_built();

    std::ostringstream o;
    o << "[" << timestamp() << "] Load Tracks";
    const bool looks_error = !ok ||
        res.find("Error")     != std::string::npos ||
        res.find("Traceback") != std::string::npos ||
        res.find("not found") != std::string::npos ||
        res.find("no tracks") != std::string::npos;
    if (looks_error) {
        o << " [WARN] " << (res.empty() ? "could not read tracks (is menu.py loaded?)" : res);
    } else {
        o << ": " << res << "\n  "
          << user_tracks_total_ << " parsed, " << user_tracks_on_object_
          << " anchored on the object.";
    }
    set_status(o.str());
    asapUpdate();
}


void PolychaseTracker::clear_user_tracks()
{
    if (DD::Image::Knob* k = knob("user_tracks_blob")) k->set_text("");
    user_tracks_.clear();
    user_track_names_.clear();
    user_tracks_total_     = 0;
    user_tracks_on_object_ = 0;
    user_tracks_valid_     = false;
    refresh_user_tracks_label();
    set_status("[" + timestamp() + "] User tracks cleared.");
    asapUpdate();
}


// The solve set for flow Track and Refine, sourced from the PIN MODEL (Phase 6).
// Each pin LINKED to a track becomes one helper track, anchored at its EXACT mesh
// vertex (ground-truth 3D) instead of the raycast surface hit, with per-frame 2D
// from resolve_pin_2d (a manual key on a frame overrides the track there, else the
// linked track's observation). Stored Y-DOWN to match the solver/optical-flow
// convention. Keyed-but-unlinked pins are NOT included here — they drive the pose
// directly (live solve / track-via-pins), not as optical-flow helpers.
//
// Fallback: if NO pin is linked (the pin model has no links), return the full
// anchored loaded-track set, so "Load Tracks" without Connect still helps the
// solve, exactly as before.
UserTracks PolychaseTracker::build_solve_tracks()
{
    const UserTracks& base = user_tracks_for_solve();   // ensures user_tracks_ built
    ensure_pins_loaded();

    // Mesh for exact vertex anchors.
    GeoMesh gm;
    DD::Image::Op* geo = input_geo_op();
    bool have_mesh = false;
    if (geo) { geo->validate(true); have_mesh = extract_mesh(geo, gm); }
    const Eigen::Index nverts = have_mesh ? gm.local_vertices.rows() : 0;

    // Format height for the Y-up -> Y-down flip (cancels for a linked pin's track
    // obs, matters only for a manual-key override). Fall back to 1080 if no input.
    float fmt_h = 1080.0f;
    if (Iop* img = input_img()) { img->validate(true); fmt_h = (float)img->info().format().height(); }

    UserTracks out;
    for (const Pin& p : pins_) {
        if (p.linked_track < 0 || (size_t)p.linked_track >= base.size()) continue;  // links only
        if (!have_mesh || (Eigen::Index)p.vertex_idx >= nverts) continue;

        UserTrack ut;
        ut.object_point = Eigen::Vector3f(
            gm.local_vertices((Eigen::Index)p.vertex_idx, 0),
            gm.local_vertices((Eigen::Index)p.vertex_idx, 1),
            gm.local_vertices((Eigen::Index)p.vertex_idx, 2));  // EXACT vertex
        ut.weight = base[(size_t)p.linked_track].weight;

        for (int f = first_frame_; f <= last_frame_; ++f) {
            const std::optional<Eigen::Vector2f> r2 =
                resolve_pin_2d(p, f, fmt_h, /*allow_interp=*/false);  // key override, else track
            if (!r2) continue;
            UserTrackObservation ob;
            ob.frame_id    = f;
            ob.image_point = Eigen::Vector2f(r2->x(), fmt_h - r2->y());  // Y-up -> Y-down
            ut.observations.push_back(ob);
        }
        if (ut.observations.size() >= 2) out.push_back(std::move(ut));
    }

    if (out.empty()) {
        PCN_LOG("[solve-tracks] no linked pins; falling back to "
                << base.size() << " anchored loaded track(s)\n");
        return base;   // copy; pre-Connect behaviour preserved
    }
    PCN_LOG("[solve-tracks] from pins: " << out.size()
            << " linked helper track(s) at exact vertices\n");
    return out;
}


} // namespace pcn