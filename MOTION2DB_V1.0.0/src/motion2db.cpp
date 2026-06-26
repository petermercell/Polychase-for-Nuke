// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Peter Mercell
//
// Developed with assistance from Claude (Anthropic)

// =============================================================================
// motion2db.cpp — Motion2DB Op (Path A): motion-vector EXRs on disk -> polychase
// optical-flow .db. FAITHFUL 1:1 port of the standalone mvflow_to_db.cc, wrapped
// as a Nuke node. See motion2db.h / MOTION2DB_PLAN.md.
//
// Per adjacent frame pair (prev -> cur), the motion EXRs hold:
//   motion.forward.u/v   displacement f -> f+1   (Nuke pixels, bottom-origin)
//   motion.backward.u/v  displacement f -> f-1
// The forward pair (prev -> cur) samples prev's FORWARD field at each grid point;
// its per-match error is the forward/backward consistency residual checked
// against cur's BACKWARD field (the NEIGHBOUR frame's opposite-direction motion):
//   q   = prev_grid + forward(prev)               (target on cur)
//   r   = q        + backward(cur) sampled at q   (round-trip back onto prev)
//   err = || r - prev_grid ||   (pixels)          (stored as the solver weight)
// Out-of-frame targets / FB samples are DROPPED, matching the CLI.
//
// CONVENTIONS: the DB is TOP-ORIGIN real pixels keyed by 1-based Nuke frames;
// Nuke vectors are bottom-origin (y up) so target_y = y - v by default (flip_v).
//
// SPDX-License-Identifier: GPL-3.0-or-later  (links libpolychase, GPL-3.0)
// =============================================================================

// --- TinyEXR (header-only). Define the implementation in exactly ONE TU. ----
#define TINYEXR_IMPLEMENTATION
#define TINYEXR_USE_MINIZ 0
#define TINYEXR_USE_STB_ZLIB 0
#include <zlib.h>
#include "tinyexr.h"

#include "motion2db.h"
#include "database.h"  // Database, Keypoints, KeypointsIndices, FlowErrors, ImagePairFlow

#include "DDImage/OutputContext.h"  // per-frame File_knob evaluation

// Optional progress dialog (Qt, isolated in analyze_progress.cpp). Compiled in
// only when the build defines MOTION2DB_HAVE_QT (CMake option MOTION2DB_PROGRESS);
// otherwise the shim below is a no-op so the Qt-free, fully-static build still
// works. AnalyzeProgress is itself a no-op in headless/terminal Nuke.
#ifdef MOTION2DB_HAVE_QT
#include "analyze_progress.h"
#endif

#include <Eigen/Core>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <system_error>
#include <vector>

using namespace DD::Image;
namespace fs = std::filesystem;

namespace pcn {

const char* const Motion2DB::kClass = "Motion2DB";

namespace {

// Thin progress wrapper: real QProgressDialog when built with Qt, otherwise a
// no-op so the same on_convert code compiles in the Qt-free static build.
struct Progress {
#ifdef MOTION2DB_HAVE_QT
    pcn::AnalyzeProgress impl;
    explicit Progress(const char* title) : impl(title) {}
    void set(int pct) { impl.set_percent(pct); }
    bool cancelled() const { return impl.cancelled(); }
#else
    explicit Progress(const char*) {}
    void set(int) {}
    bool cancelled() const { return false; }
#endif
};

// ---- small helpers -------------------------------------------------------

std::string timestamp() {
    std::time_t t = std::time(nullptr);
    std::tm tm{};
#if defined(_WIN32)
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    char buf[16];
    std::snprintf(buf, sizeof buf, "%02d:%02d:%02d", tm.tm_hour, tm.tm_min, tm.tm_sec);
    return buf;
}

const char* nz(const char* s, const char* d) { return (s && s[0]) ? s : d; }

// Substitute the frame number into a sequence pattern: ####, %04d / %d, or a
// literal (returned unchanged).
std::string substitute_frame(const std::string& pattern, int frame) {
    const size_t h0 = pattern.find('#');
    if (h0 != std::string::npos) {
        size_t h1 = h0;
        while (h1 < pattern.size() && pattern[h1] == '#') ++h1;
        const int width = static_cast<int>(h1 - h0);
        char buf[32];
        std::snprintf(buf, sizeof buf, "%0*d", width, frame);
        return pattern.substr(0, h0) + buf + pattern.substr(h1);
    }
    if (pattern.find('%') != std::string::npos) {
        char buf[2048];
        std::snprintf(buf, sizeof buf, pattern.c_str(), frame);
        return std::string(buf);
    }
    return pattern;
}

// Last two dot-tokens of a channel name ("motion.forward.u" ~ "forward.u").
std::string leaf2(const std::string& n) {
    const size_t p1 = n.rfind('.');
    if (p1 == std::string::npos) return n;
    const size_t p0 = n.rfind('.', p1 - 1);
    return (p0 == std::string::npos) ? n : n.substr(p0 + 1);
}

struct Planes {
    int w = 0, h = 0;
    std::vector<float> fu, fv, bu, bv;  // top-origin float rasters
};

// Read the four named flow channels into top-origin float planes (TinyEXR).
bool read_motion(const std::string& path, const char* fwd_u, const char* fwd_v,
                 const char* bwd_u, const char* bwd_v, Planes& out,
                 std::string& err) {
    EXRVersion ver;
    if (ParseEXRVersionFromFile(&ver, path.c_str()) != 0) {
        err = "cannot parse EXR version: " + path;
        return false;
    }
    EXRHeader hdr;
    InitEXRHeader(&hdr);
    const char* e = nullptr;
    if (ParseEXRHeaderFromFile(&hdr, &ver, path.c_str(), &e) != 0) {
        err = e ? e : "header parse failed";
        if (e) FreeEXRErrorMessage(e);
        return false;
    }
    for (int i = 0; i < hdr.num_channels; ++i)
        hdr.requested_pixel_types[i] = TINYEXR_PIXELTYPE_FLOAT;

    EXRImage img;
    InitEXRImage(&img);
    if (LoadEXRImageFromFile(&img, &hdr, path.c_str(), &e) != 0) {
        err = e ? e : "image load failed";
        if (e) FreeEXRErrorMessage(e);
        FreeEXRHeader(&hdr);
        return false;
    }

    auto find = [&](const std::string& want) -> int {
        const std::string wl = leaf2(want);
        int exact = -1, leaf = -1;
        for (int i = 0; i < hdr.num_channels; ++i) {
            const std::string nm = hdr.channels[i].name;
            if (nm == want)
                exact = i;
            else if (leaf2(nm) == wl)
                leaf = i;
        }
        return exact >= 0 ? exact : leaf;
    };
    const int cfu = find(fwd_u), cfv = find(fwd_v);
    const int cbu = find(bwd_u), cbv = find(bwd_v);
    if (cfu < 0 || cfv < 0 || cbu < 0 || cbv < 0) {
        err = "missing flow channel(s) in " + path + ". Channels present:";
        for (int i = 0; i < hdr.num_channels; ++i) {
            err += ' ';
            err += hdr.channels[i].name;
        }
        FreeEXRImage(&img);
        FreeEXRHeader(&hdr);
        return false;
    }

    out.w = img.width;
    out.h = img.height;
    const size_t n = static_cast<size_t>(out.w) * out.h;
    auto grab = [&](int c, std::vector<float>& dst) {
        dst.resize(n);
        std::copy_n(reinterpret_cast<const float*>(img.images[c]), n, dst.data());
    };
    grab(cfu, out.fu);
    grab(cfv, out.fv);
    grab(cbu, out.bu);
    grab(cbv, out.bv);

    FreeEXRImage(&img);
    FreeEXRHeader(&hdr);
    return true;
}

// Plate luma (top-origin) for the gradient gate (Rec.601, matching the CLI).
bool read_luma(const std::string& path, std::vector<float>& luma, int& w, int& h) {
    float* rgba = nullptr;
    const char* e = nullptr;
    if (LoadEXR(&rgba, &w, &h, path.c_str(), &e) != TINYEXR_SUCCESS) {
        if (e) FreeEXRErrorMessage(e);
        return false;
    }
    const size_t n = static_cast<size_t>(w) * h;
    luma.resize(n);
    for (size_t i = 0; i < n; ++i)
        luma[i] = 0.299f * rgba[i * 4] + 0.587f * rgba[i * 4 + 1] +
                  0.114f * rgba[i * 4 + 2];
    free(rgba);
    return true;
}

// Bilinear sample; returns false (no clamp) if (x,y) is out of frame.
inline bool sample(const std::vector<float>& P, int W, int H, float x, float y,
                   float& o) {
    if (x < 0 || y < 0 || x > W - 1 || y > H - 1) return false;
    const int x0 = (int)std::floor(x), y0 = (int)std::floor(y);
    const int x1 = std::min(x0 + 1, W - 1), y1 = std::min(y0 + 1, H - 1);
    const float tx = x - x0, ty = y - y0;
    const float a = P[(size_t)y0 * W + x0], b = P[(size_t)y0 * W + x1];
    const float c = P[(size_t)y1 * W + x0], d = P[(size_t)y1 * W + x1];
    o = (a * (1 - tx) + b * tx) * (1 - ty) + (c * (1 - tx) + d * tx) * ty;
    return true;
}

// Apply the sign/origin convention to a (u,v) at (x,y) -> target pixel.
inline void target_of(float x, float y, float u, float v, bool flip_v,
                      bool negate, float& tx, float& ty) {
    const float s = negate ? -1.0f : 1.0f;
    tx = x + s * u;
    ty = flip_v ? (y - s * v) : (y + s * v);
}

// Luma gradient magnitude at integer (x,y) (central difference; matches CLI).
inline float grad_mag(const std::vector<float>& L, int W, int H, int x, int y) {
    const int xm = std::max(x - 1, 0), xp = std::min(x + 1, W - 1);
    const int ym = std::max(y - 1, 0), yp = std::min(y + 1, H - 1);
    const float gx = L[(size_t)y * W + xp] - L[(size_t)y * W + xm];
    const float gy = L[(size_t)yp * W + x] - L[(size_t)ym * W + x];
    return std::sqrt(gx * gx + gy * gy);
}

// Grid keypoints (start at stride/2, optionally gated by plate luma gradient).
Keypoints make_keypoints(int W, int H, const std::vector<float>* luma, int stride,
                         double grad_gate) {
    Keypoints kp;
    const int s = std::max(1, stride);
    for (int y = s / 2; y < H; y += s)
        for (int x = s / 2; x < W; x += s) {
            if (grad_gate > 0.0 && luma &&
                grad_mag(*luma, W, H, x, y) < (float)grad_gate)
                continue;
            kp.emplace_back((float)x, (float)y);
        }
    return kp;
}

// Build one ImagePairFlow: sample `mu/mv` (the source frame's motion) at each
// src keypoint -> target in the neighbour; FB-check against `cu/cv` (the
// neighbour frame's opposite-direction motion). Drops correspondences whose
// target or FB sample falls out of frame.
ImagePairFlow build_pair(int from_id, int to_id, const Keypoints& src_kps,
                         const std::vector<float>& mu, const std::vector<float>& mv,
                         const std::vector<float>& cu, const std::vector<float>& cv,
                         int W, int H, bool flip_v, bool negate, double max_error,
                         double& err_sum, size_t& err_cnt, long long& dropped) {
    ImagePairFlow pf;
    pf.image_id_from = from_id;
    pf.image_id_to = to_id;
    for (uint32_t i = 0; i < src_kps.size(); ++i) {
        const float x = src_kps[i].x(), y = src_kps[i].y();
        float u, v;
        if (!sample(mu, W, H, x, y, u) || !sample(mv, W, H, x, y, v)) {
            ++dropped;
            continue;
        }
        float qx, qy;
        target_of(x, y, u, v, flip_v, negate, qx, qy);
        if (qx < 0 || qy < 0 || qx > W - 1 || qy > H - 1) {
            ++dropped;
            continue;
        }
        float bu, bv;
        if (!sample(cu, W, H, qx, qy, bu) || !sample(cv, W, H, qx, qy, bv)) {
            ++dropped;
            continue;
        }
        float rx, ry;
        target_of(qx, qy, bu, bv, flip_v, negate, rx, ry);
        const float err = std::sqrt((rx - x) * (rx - x) + (ry - y) * (ry - y));
        if (max_error > 0.0 && err > (float)max_error) {
            ++dropped;
            continue;
        }
        pf.src_kps_indices.push_back(i);
        pf.tgt_kps.emplace_back(qx, qy);
        pf.flow_errors.push_back(err);
        err_sum += err;
        ++err_cnt;
    }
    return pf;
}

// Invert a forward pair (from=A, to=B) into the backward pair (from=B, to=A):
//   forward:   A_grid[ fwd.src_kps_indices[j] ]  ->  fwd.tgt_kps[j]   (= Q on B)
//   backward:  Q (on B)                          ->  A_grid[...]      (= P on A)
// Q is indexed into B's stored keypoints; the forward targets were appended to
// B's keypoints starting at q_base, so Q[j] lives at index q_base + j. Same
// per-match error. A timeline-backward Track then reads the reverse of the SAME
// correspondence a forward Track read, instead of the noisier backward field.
ImagePairFlow invert_pair(int from_id, int to_id, const ImagePairFlow& fwd,
                          const Keypoints& from_grid, uint32_t q_base) {
    ImagePairFlow pf;
    pf.image_id_from = from_id;
    pf.image_id_to = to_id;
    const size_t n = std::min(fwd.tgt_kps.size(), fwd.src_kps_indices.size());
    pf.src_kps_indices.reserve(n);
    pf.tgt_kps.reserve(n);
    pf.flow_errors.reserve(n);
    for (size_t j = 0; j < n; ++j) {
        pf.src_kps_indices.push_back(q_base + (uint32_t)j);       // Q on B
        pf.tgt_kps.push_back(from_grid[fwd.src_kps_indices[j]]);  // P on A
        pf.flow_errors.push_back(j < fwd.flow_errors.size() ? fwd.flow_errors[j] : 0.0f);
    }
    return pf;
}

}  // namespace

// ---- knobs ---------------------------------------------------------------

void Motion2DB::knobs(Knob_Callback f) {
    Divider(f, "Source");
    static const char* const kSource[] = {"File on disk", "Input (live) — not available", nullptr};
    Enumeration_knob(f, &source_, kSource, "source", "Source");
    Tooltip(f, "Where the motion vectors come from. Only \"File on disk\" is "
               "implemented (Path A).");

    File_knob(f, &motion_path_, "motion_path", "Motion EXRs");
    Tooltip(f, "Motion-vector EXR sequence, e.g. /path/motion.####.exr (also "
               "accepts %04d / %d). Must contain the forward.u/v and "
               "backward.u/v channels named below.");

    Divider(f, "Frames");
    Int_knob(f, &first_frame_, "first_frame", "First Frame");
    Tooltip(f, "First frame (DB image_id). 1-based Nuke frame numbers.");
    Int_knob(f, &last_frame_, "last_frame", "Last Frame");
    ClearFlags(f, Knob::STARTLINE);

    Divider(f, "Output");
    File_knob(f, &db_path_, "db_path", "Database");
    Tooltip(f, "Output polychase .db. Built by this node; the tracker reads it.");
    Bool_knob(f, &overwrite_, "overwrite", "Overwrite");
    Tooltip(f, "Delete an existing .db before writing (required to re-run into "
               "the same path).");

    Divider(f, "Channels");
    String_knob(f, &fwd_u_, "fwd_u", "Forward U");
    Tooltip(f, "Channel name for forward X displacement (leaf-suffix match).");
    String_knob(f, &fwd_v_, "fwd_v", "Forward V");
    ClearFlags(f, Knob::STARTLINE);
    String_knob(f, &bwd_u_, "bwd_u", "Backward U");
    String_knob(f, &bwd_v_, "bwd_v", "Backward V");
    ClearFlags(f, Knob::STARTLINE);

    Divider(f, "Conventions");
    Bool_knob(f, &flip_v_, "flip_v", "Flip V (y - v)");
    Tooltip(f, "target_y = y - v (DB top-origin vs Nuke y-up).");
    Bool_knob(f, &negate_, "negate", "Negate");
    ClearFlags(f, Knob::STARTLINE);
    Tooltip(f, "Negate vectors — flow stores 'came-from' rather than 'goes-to'.");

    Divider(f, "Keypoints");
    Int_knob(f, &grid_stride_, "grid_stride", "Grid stride");
    SetRange(f, 2, 64);
    Tooltip(f, "Grid spacing in pixels (start at stride/2). ~15k kp/frame at 12 on 2K.");
    Double_knob(f, &grad_gate_, "grad_gate", "Gradient gate");
    SetRange(f, 0.0, 1.0);
    ClearFlags(f, Knob::STARTLINE);
    Tooltip(f, "Skip grid points whose plate-luma gradient is below this "
               "(0 = keep all). Needs a Plate EXRs sequence.");
    File_knob(f, &plate_path_, "plate_path", "Plate EXRs");
    Tooltip(f, "Optional plate EXR sequence (same numbering) used only for the "
               "gradient gate. Leave empty to keep every grid point.");

    Divider(f, "Pairs");
    Bool_knob(f, &write_backward_, "write_backward", "Write backward");
    Tooltip(f, "Also write (f, f-1) pairs so a timeline-backward Track works.");
    Bool_knob(f, &invert_backward_, "invert_backward", "Invert backward");
    ClearFlags(f, Knob::STARTLINE);
    Tooltip(f, "ON (default, matches the mvflow_to_db base tool): backward pairs "
               "are the exact inverse of forward pairs, appending the forward "
               "targets to each frame's keypoints (grid+appended, ~2x kp/frame) so "
               "a reverse Track reads the SAME correspondences as forward. "
               "OFF: grid-only keypoints + a separately-sampled backward NN field "
               "(~15k kp/frame, noisier reverse track).");
    Double_knob(f, &max_error_, "max_error", "Max FB error");
    SetRange(f, 0.0, 10.0);
    Tooltip(f, "Drop correspondences whose forward/backward residual exceeds this "
               "many pixels (0 = keep all; the residual is stored as the weight).");

    Divider(f, "Convert");
    Button(f, "convert", "Convert");
    Tooltip(f, "Read the motion EXRs and write the .db. Synchronous — the UI "
               "blocks; per-frame counts print to the terminal Nuke was launched from.");

    Divider(f, "Status");
    Multiline_String_knob(f, &status_cstr_, "status", "Status");
    SetFlags(f, Knob::READ_ONLY | Knob::STARTLINE);
}

int Motion2DB::knob_changed(Knob* k) {
    if (k == &Knob::showPanel) return 1;
    if (k && k->is("convert")) {
        on_convert();
        return 1;
    }
    return NoIop::knob_changed(k);
}

void Motion2DB::set_status(const std::string& s) {
    status_store_ = s;
    if (Knob* k = knob("status")) k->set_text(status_store_.c_str());
}

// ---- the conversion (faithful port of mvflow_to_db.cc's main loop) -------

void Motion2DB::on_convert() {
    std::ostringstream oss;
    oss << "[" << timestamp() << "] Motion2DB\n";

    if (source_ != 0) {
        oss << "  [FAIL] only \"File on disk\" source is implemented.";
        set_status(oss.str());
        return;
    }
    if (!motion_path_ || !motion_path_[0]) {
        oss << "  [FAIL] Motion EXRs path not set.";
        set_status(oss.str());
        return;
    }
    if (!db_path_ || !db_path_[0]) {
        oss << "  [FAIL] Database path not set.";
        set_status(oss.str());
        return;
    }

    const int lo = std::min(first_frame_, last_frame_);
    const int hi = std::max(first_frame_, last_frame_);
    const int stride = std::max(2, grid_stride_);
    const bool gate = (grad_gate_ > 0.0) && plate_path_ && plate_path_[0];

    const char* fu_name = nz(fwd_u_, "motion.forward.u");
    const char* fv_name = nz(fwd_v_, "motion.forward.v");
    const char* bu_name = nz(bwd_u_, "motion.backward.u");
    const char* bv_name = nz(bwd_v_, "motion.backward.v");

    {
        std::ostringstream pre;
        pre << "[" << timestamp() << "] Motion2DB starting\n"
            << "  range   = " << lo << ".." << hi << "  (" << (hi - lo + 1) << " frames)\n"
            << "  motion  = " << motion_path_ << "\n"
            << "  output  = " << db_path_ << "\n"
            << "  stride  = " << stride << "   flip_v=" << (flip_v_ ? 1 : 0)
            << "  negate=" << (negate_ ? 1 : 0)
            << "  backward=" << (write_backward_ ? (invert_backward_ ? "invert" : "field") : "off") << "\n"
            << "  Synchronous — the UI blocks until done. Per-frame counts print\n"
            << "  to the terminal Nuke was launched from.";
        set_status(pre.str());
    }
    std::cout.flush();

    // ---- stats ----
    int frames_done = 0, fwd_pairs = 0, bwd_pairs = 0;
    long long kp_total = 0, corr_total = 0, dropped = 0;
    double err_sum = 0.0;
    size_t err_cnt = 0;
    int ref_w = 0, ref_h = 0;

    const auto t0 = std::chrono::steady_clock::now();
    bool error = false;
    std::string emsg;
    bool cancelled = false;
    bool plate_warned = false;

    // Resolve a File_knob's sequence for a SPECIFIC frame. The bound const char*
    // (motion_path_/plate_path_) is evaluated at the node's current viewer frame,
    // so it can collapse to one concrete file; asking the knob with an explicit
    // OutputContext re-evaluates the ####/%04d pattern per frame. substitute_frame
    // is a belt-and-suspenders pass (idempotent once the path is already concrete).
    auto seq_path = [&](const char* kname, const char* bound, int fr) -> std::string {
        if (Knob* k = this->knob(kname)) {
            DD::Image::OutputContext oc;
            oc.setFrame((double)fr);
            const char* t = k->get_text(&oc);
            if (t && t[0]) return substitute_frame(std::string(t), fr);
        }
        return substitute_frame(std::string(bound ? bound : ""), fr);
    };

    // Loads frame fr's motion planes + grid keypoints (gated by its plate luma).
    auto load_frame = [&](int fr, Planes& pl, Keypoints& kp, std::string& err) -> bool {
        const std::string mpath = seq_path("motion_path", motion_path_, fr);
        if (!read_motion(mpath, fu_name, fv_name, bu_name, bv_name, pl, err))
            return false;
        const std::vector<float>* luma = nullptr;
        std::vector<float> lbuf;
        int lw = 0, lh = 0;
        if (gate) {
            const std::string ppath = seq_path("plate_path", plate_path_, fr);
            if (read_luma(ppath, lbuf, lw, lh) && lw == pl.w && lh == pl.h) {
                luma = &lbuf;
            } else if (!plate_warned) {
                std::cout << "[Motion2DB]   plate gate disabled (read/size) — "
                             "keeping all grid points" << std::endl;
                plate_warned = true;
            }
        }
        kp = make_keypoints(pl.w, pl.h, luma, stride, grad_gate_);
        return true;
    };

    try {
        if (overwrite_) {
            std::error_code ec;
            fs::remove(fs::path(std::string(db_path_)), ec);  // ignore "didn't exist"
        }

        // Sequence sanity: if consecutive frames resolve to the SAME file, the
        // path isn't a real sequence (no ####/%04d) and every frame would read
        // the same flow. Fail loudly instead of silently producing a flat DB.
        if (hi > lo) {
            const std::string p0 = seq_path("motion_path", motion_path_, lo);
            const std::string p1 = seq_path("motion_path", motion_path_, lo + 1);
            std::cout << "[Motion2DB]   resolved frame " << lo << " -> " << p0 << "\n"
                      << "[Motion2DB]   resolved frame " << (lo + 1) << " -> " << p1
                      << std::endl;
            if (p0 == p1)
                throw std::runtime_error(
                    "motion path resolves to the SAME file for every frame (\"" + p0 +
                    "\"). Use a sequence pattern with a frame token, e.g. "
                    "CUBEMOTION.####.exr or CUBEMOTION.%04d.exr — not a single "
                    "concrete frame.");
        }

        Database db{std::string(db_path_)};  // fresh path -> tables created

        Planes prev, cur;
        Keypoints kp_prev, kp_cur;
        std::string rerr;

        if (!load_frame(lo, prev, kp_prev, rerr)) throw std::runtime_error(rerr);
        ref_w = prev.w;
        ref_h = prev.h;
        const int W = ref_w, H = ref_h;

        db.WriteKeypoints(lo, kp_prev);  // first frame: grid only
        kp_total += (long long)kp_prev.size();
        ++frames_done;
        std::cout << "[Motion2DB]   frame " << lo << "  grid=" << kp_prev.size()
                  << "  keypoints=" << kp_prev.size() << std::endl;

        // Progress dialog (GUI Nuke only; no-op headless / without Qt). The total
        // is the number of cur-frames we iterate; the first frame is already done.
        Progress prog("Motion2DB - converting motion vectors");
        const int total = std::max(1, hi - lo);
        prog.set((int)((double)(frames_done - 1) * 100.0 / total));

        for (int fr = lo + 1; fr <= hi; ++fr) {
            if (prog.cancelled()) { cancelled = true; break; }
            if (!load_frame(fr, cur, kp_cur, rerr)) throw std::runtime_error(rerr);
            if (cur.w != W || cur.h != H) {
                std::ostringstream m;
                m << "frame " << fr << " size " << cur.w << "x" << cur.h
                  << " != " << W << "x" << H;
                throw std::runtime_error(m.str());
            }
            const int prev_id = fr - 1;
            const int cur_id = fr;

            // forward pair (prev -> cur): prev.forward, FB against cur.backward.
            ImagePairFlow fwd =
                build_pair(prev_id, cur_id, kp_prev, prev.fu, prev.fv, cur.bu, cur.bv,
                           W, H, flip_v_, negate_, max_error_, err_sum, err_cnt, dropped);

            if (invert_backward_) {
                // cur's stored keypoints = cur grid ++ the forward targets Q.
                const uint32_t q_base = (uint32_t)kp_cur.size();
                Keypoints kp_cur_stored = kp_cur;
                kp_cur_stored.insert(kp_cur_stored.end(), fwd.tgt_kps.begin(),
                                     fwd.tgt_kps.end());
                db.WriteKeypoints(cur_id, kp_cur_stored);  // FK parent first
                kp_total += (long long)kp_cur_stored.size();

                db.WriteImagePairFlow(fwd);  // (prev -> cur), from=prev exists
                ++fwd_pairs;
                corr_total += (long long)fwd.tgt_kps.size();

                if (write_backward_) {  // (cur -> prev) = inverse of forward
                    ImagePairFlow bwd =
                        invert_pair(cur_id, prev_id, fwd, kp_prev, q_base);
                    db.WriteImagePairFlow(bwd);
                    ++bwd_pairs;
                    corr_total += (long long)bwd.tgt_kps.size();
                }
            } else {
                // Field-sampled: grid-only keypoints + a backward pair from cur's
                // backward NN field (this reproduces the ~15k-kp/frame nn DB).
                db.WriteKeypoints(cur_id, kp_cur);  // FK parent first
                kp_total += (long long)kp_cur.size();

                db.WriteImagePairFlow(fwd);
                ++fwd_pairs;
                corr_total += (long long)fwd.tgt_kps.size();

                if (write_backward_) {
                    ImagePairFlow bwd = build_pair(
                        cur_id, prev_id, kp_cur, cur.bu, cur.bv, prev.fu, prev.fv,
                        W, H, flip_v_, negate_, max_error_, err_sum, err_cnt, dropped);
                    db.WriteImagePairFlow(bwd);
                    ++bwd_pairs;
                    corr_total += (long long)bwd.tgt_kps.size();
                }
            }
            ++frames_done;

            std::cout << "[Motion2DB]   frame " << cur_id
                      << "  grid=" << kp_cur.size()
                      << "  fwd corr=" << fwd.tgt_kps.size() << std::endl;

            prog.set((int)((double)(frames_done - 1) * 100.0 / total));

            std::swap(prev, cur);
            std::swap(kp_prev, kp_cur);
        }
    } catch (const std::exception& ex) {
        error = true;
        emsg = ex.what();
    } catch (...) {
        error = true;
        emsg = "(unknown exception type)";
    }

    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - t0)
                        .count();

    // ---- summary ----
    std::ostringstream done;
    if (error) {
        done << "[" << timestamp() << "] Motion2DB FAILED\n"
             << "  " << emsg << "\n"
             << "  frames written before failure: " << frames_done << "\n"
             << "  elapsed: " << (ms / 1000.0) << "s";
    } else {
        const double avg_kp = frames_done ? (double)kp_total / frames_done : 0.0;
        const double mean_err = err_cnt ? (err_sum / (double)err_cnt) : 0.0;
        done << "[" << timestamp() << "] Motion2DB "
             << (cancelled ? "CANCELLED (partial DB written)" : "complete") << "\n"
             << "  [" << (cancelled ? "PARTIAL" : "PASS") << "] wrote " << db_path_ << "\n"
             << "  frames        = " << lo << ".." << hi << "  (" << frames_done << ")\n"
             << "  keypoints     = " << kp_total << "  (avg " << std::fixed
             << std::setprecision(1) << avg_kp << "/frame)\n"
             << "  pairs         = " << fwd_pairs << " forward, " << bwd_pairs
             << " backward  (" << (write_backward_ ? (invert_backward_ ? "inverted" : "field") : "none")
             << ")\n"
             << "  correspondences = " << corr_total;
        if (max_error_ > 0.0) done << "  (dropped " << dropped << " over max error / out-of-frame)";
        else if (dropped) done << "  (dropped " << dropped << " out-of-frame)";
        done << "\n";
        if (err_cnt)
            done << "  mean FB error = " << std::setprecision(3) << mean_err << " px\n";
        done << "  grid stride " << stride << ", flip_v " << (flip_v_ ? 1 : 0)
             << ", negate " << (negate_ ? 1 : 0) << "\n"
             << "  elapsed " << std::setprecision(3) << (ms / 1000.0) << "s\n"
             << "  Inspect with VisualizeFlowDB on the same plate, then point\n"
             << "  PolychaseTracker's Database at this .db and TrackIt.";
    }
    std::cout << done.str() << std::endl;
    set_status(done.str());
}

// ---- registration --------------------------------------------------------

static Op* build(Node* node) { return new Motion2DB(node); }
const Op::Description Motion2DB::description(Motion2DB::kClass, build);

}  // namespace pcn
