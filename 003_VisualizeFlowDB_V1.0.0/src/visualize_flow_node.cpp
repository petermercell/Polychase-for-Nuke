// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Peter Mercell
//
// Developed with assistance from Claude (Anthropic)

// =============================================================================
// visualize_flow_node.cpp — VisualizeFlowDB Op + its 2D-viewer overlay knob.
// See visualize_flow_node.h. All polychase/SQLite access is confined here.
// =============================================================================
#include "visualize_flow_node.h"

#include "database.h"          // Database, Keypoints, ImagePairFlow, KeypointsIndices

#include <GL/gl.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

using namespace DD::Image;

namespace pcn {

namespace {

// Deterministic per-(frame,index) colour so a feature keeps its colour as you
// scrub, and source/target share a colour. 0..1 RGB, floored at 0.35 so nothing
// is too dark on a dark plate.
void index_color(int frame, size_t idx, float& r, float& g, float& b)
{
    uint64_t h = (uint64_t)((uint32_t)frame * 2654435761u) ^ (idx * 40503u + 0x9E37u);
    h ^= h >> 13; h *= 0xC2B2AE35u; h ^= h >> 16;
    r = 0.35f + 0.65f * (float)( h        & 0xFF) / 255.0f;
    g = 0.35f + 0.65f * (float)((h >> 8)  & 0xFF) / 255.0f;
    b = 0.35f + 0.65f * (float)((h >> 16) & 0xFF) / 255.0f;
}

const char* const kFlowDirs[] = { "forward", "backward", "both", nullptr };

} // namespace


VisualizeFlowDB::VisualizeFlowDB(Node* node) : NoIop(node) {}

VisualizeFlowDB::~VisualizeFlowDB() { close_db_(); }

void VisualizeFlowDB::close_db_()
{
    if (db_) { delete static_cast<Database*>(db_); db_ = nullptr; }
    db_open_path_.clear();
    cached_frame_ = -2147483647;
    cache_dirty_  = true;
    cache_points_.clear();
    cache_segs_.clear();
}


void VisualizeFlowDB::knobs(Knob_Callback f)
{
    File_knob(f, &db_file_, "database", "Flow DB");
    Tooltip(f, "Path to the polychase optical-flow .db to visualize. Wire the "
               "same footage the DB was built from into input 0.");

    Bool_knob(f, &show_keypoints_, "show_keypoints", "Keypoints");
    Tooltip(f, "Draw a dot at each detected keypoint on the current frame.");

    Bool_knob(f, &show_vectors_, "show_vectors", "Flow vectors");
    Tooltip(f, "Draw a line from each keypoint to where it flows in the "
               "neighbour frame(s). Coherent lines = good matches; scattered "
               "lines jumping around = ambiguous (e.g. repetitive texture).");

    Enumeration_knob(f, &flow_dir_, kFlowDirs, "flow_dir", "Vectors to");
    Tooltip(f, "Which neighbours to draw flow vectors toward: later frames "
               "(forward), earlier frames (backward), or both.");

    Double_knob(f, &point_size_, "point_size", "Point size");
    SetRange(f, 1.0, 20.0);

    Double_knob(f, &line_width_, "line_width", "Line width");
    SetRange(f, 0.5, 5.0);

    Bool_knob(f, &color_by_error_, "color_by_error", "Colour by error");
    Tooltip(f, "Colour each flow vector by its match error instead of a "
               "per-feature colour: green = low error (good match), red = high "
               "error (ambiguous/bad). On repetitive texture the spiderweb "
               "lights up red. The per-frame error range is printed to the "
               "terminal so you can set Error scale / threshold sensibly.");

    Double_knob(f, &error_scale_, "error_scale", "Error scale");
    SetRange(f, 0.1, 20.0);
    Tooltip(f, "Match error mapped to full red. Lower = more sensitive "
               "(more vectors go red). Check the terminal print for the "
               "actual error magnitudes in this DB.");

    Double_knob(f, &error_threshold_, "error_threshold", "Hide error >");
    SetRange(f, 0.0, 10.0);
    Tooltip(f, "0 = show all. Above 0, hide every vector whose match error "
               "exceeds this value, leaving only the trustworthy matches.");

    // The overlay itself (INVISIBLE: it's a viewer handle, not a UI row).
    CustomKnob1(VisualizeFlowOverlayKnob, f, this, "flow_overlay");
    SetFlags(f, Knob::INVISIBLE);
}


int VisualizeFlowDB::knob_changed(Knob* k)
{
    if (k == &Knob::showPanel) return 1;
    if (k) {
        if (k->is("database")) { close_db_(); asapUpdate(); return 1; }
        if (k->is("flow_dir")) { cache_dirty_ = true; asapUpdate(); return 1; }
        if (k->is("show_keypoints") || k->is("show_vectors") ||
            k->is("point_size")     || k->is("line_width")   ||
            k->is("color_by_error") || k->is("error_scale")  ||
            k->is("error_threshold")) {
            asapUpdate(); return 1;   // draw-style only; collected data unchanged
        }
    }
    return NoIop::knob_changed(k);
}


void VisualizeFlowDB::collect_overlay(int frame,
                                      std::vector<FlowPoint>& points,
                                      std::vector<FlowSeg>& segs)
{
    points.clear();
    segs.clear();

    const std::string path = db_file_ ? db_file_ : "";
    if (path.empty()) return;

    // (Re)open on first use or path change. Kept open across redraws.
    if (path != db_open_path_ || !db_) {
        close_db_();
        try {
            db_ = new Database(path);
            db_open_path_ = path;
        } catch (const std::exception& e) {
            std::cout << "[VisualizeFlowDB] could not open '" << path << "': "
                      << e.what() << std::endl;
            db_ = nullptr; db_open_path_.clear();
            return;
        } catch (...) {
            db_ = nullptr; db_open_path_.clear();
            return;
        }
        cache_dirty_ = true;
    }
    if (!db_) return;

    // Serve the cache unless the frame or a relevant knob changed.
    if (frame == cached_frame_ && !cache_dirty_) {
        points = cache_points_;
        segs   = cache_segs_;
        return;
    }

    cache_points_.clear();
    cache_segs_.clear();

    try {
        Database* db = static_cast<Database*>(db_);
        const Keypoints kps = db->ReadKeypoints(frame);
        if (!kps.empty()) {
            std::vector<std::array<float, 3>> col(kps.size());
            cache_points_.reserve(kps.size());
            for (size_t i = 0; i < kps.size(); ++i) {
                float r, g, b;
                index_color(frame, i, r, g, b);
                col[i] = { r, g, b };
                cache_points_.push_back({ kps[i].x(), kps[i].y(), r, g, b });
            }

            const std::vector<int32_t> nb = db->FindOpticalFlowsFromImage(frame);
            float emin = 1e30f, emax = -1e30f; double esum = 0.0; size_t ecnt = 0;
            for (int32_t id2 : nb) {
                if (flow_dir_ == 0 && !(id2 > frame)) continue;   // forward only
                if (flow_dir_ == 1 && !(id2 < frame)) continue;   // backward only
                const ImagePairFlow flow = db->ReadImagePairFlow(frame, id2);
                const size_t n = std::min(flow.tgt_kps.size(), flow.src_kps_indices.size());
                const bool have_err = (flow.flow_errors.size() == flow.tgt_kps.size());
                for (size_t i = 0; i < n; ++i) {
                    const uint32_t si = (uint32_t)flow.src_kps_indices[i];
                    if (si >= kps.size()) continue;
                    const std::array<float, 3>& c = col[si];
                    const float err = have_err ? flow.flow_errors[i] : -1.0f;
                    if (err >= 0.0f) {
                        emin = std::min(emin, err); emax = std::max(emax, err);
                        esum += err; ++ecnt;
                    }
                    cache_segs_.push_back({ kps[si].x(),        kps[si].y(),
                                            flow.tgt_kps[i].x(), flow.tgt_kps[i].y(),
                                            c[0], c[1], c[2], err });
                }
            }

            // Print the error scale so the Error scale / threshold knobs can be
            // set meaningfully (units are whatever Analyze wrote into the DB).
            if (ecnt)
                std::cout << "[VisualizeFlowDB] frame " << frame << ": "
                          << cache_segs_.size() << " vectors, err min/mean/max = "
                          << emin << " / " << (esum / (double)ecnt) << " / "
                          << emax << std::endl;
            else if (!cache_segs_.empty())
                std::cout << "[VisualizeFlowDB] frame " << frame << ": "
                          << cache_segs_.size() << " vectors, no error data in DB"
                          << std::endl;
        }
    } catch (const std::exception& e) {
        std::cout << "[VisualizeFlowDB] read failed @ frame " << frame << ": "
                  << e.what() << std::endl;
    } catch (...) {
        // leave whatever was collected
    }

    cached_frame_ = frame;
    cache_dirty_  = false;
    points = cache_points_;
    segs   = cache_segs_;
}


// -----------------------------------------------------------------------------
// VisualizeFlowOverlayKnob::draw_handle — 2D viewer only, DRAW_OPAQUE only.
// Image-pixel coords go straight to glVertex2f (the viewer's MV/PJ apply
// pan+zoom); we flip Y (fmt_h - y) from the DB's top-origin to the viewer's
// bottom-origin convention.
// -----------------------------------------------------------------------------
void VisualizeFlowOverlayKnob::draw_handle(ViewerContext* ctx)
{
    if (!owner_) return;
    if (ctx->transform_mode() != 0) return;     // 2D viewer only
    if (ctx->event() != DRAW_OPAQUE) return;

    Iop* img = dynamic_cast<Iop*>(owner_->Op::input(0));
    if (!img) return;                            // overlay needs the plate (format)
    img->validate(true);
    const Format& fmt = img->info().format();
    const float fmt_h = (float)fmt.height();

    const int frame = (int)std::lround(uiContext().frame());

    std::vector<FlowPoint> points;
    std::vector<FlowSeg>   segs;
    owner_->collect_overlay(frame, points, segs);
    if (points.empty() && segs.empty()) return;

    glPushAttrib(GL_LINE_BIT | GL_POINT_BIT | GL_CURRENT_BIT
                 | GL_ENABLE_BIT | GL_DEPTH_BUFFER_BIT);
    glDisable(GL_LIGHTING);
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_DEPTH_TEST);

    if (owner_->show_vectors() && !segs.empty()) {
        const bool  by_err = owner_->color_by_error();
        const float scale  = std::max(1e-4f, owner_->error_scale());
        const float thr    = owner_->error_threshold();
        glLineWidth(owner_->line_width());
        glBegin(GL_LINES);
        for (const FlowSeg& s : segs) {
            if (thr > 0.0f && s.err >= 0.0f && s.err > thr) continue;  // hide high-error
            if (by_err) {
                if (s.err < 0.0f) {
                    glColor4f(0.6f, 0.6f, 0.6f, 0.9f);                 // no error data
                } else {
                    // green -> yellow -> red heatmap (one channel always 1).
                    const float t = std::min(1.0f, s.err / scale);
                    const float r = (t < 0.5f) ? (2.0f * t)         : 1.0f;
                    const float g = (t < 0.5f) ? 1.0f               : (2.0f * (1.0f - t));
                    glColor4f(r, g, 0.0f, 0.9f);
                }
            } else {
                glColor4f(s.r, s.g, s.b, 0.9f);
            }
            glVertex2f(s.x0, fmt_h - s.y0);
            glVertex2f(s.x1, fmt_h - s.y1);
        }
        glEnd();
    }

    if (owner_->show_keypoints() && !points.empty()) {
        glPointSize(owner_->point_size());
        glEnable(GL_POINT_SMOOTH);
        glBegin(GL_POINTS);
        for (const FlowPoint& p : points) {
            glColor4f(p.r, p.g, p.b, 1.0f);
            glVertex2f(p.x, fmt_h - p.y);
        }
        glEnd();
    }

    glPopAttrib();
}


// ---- Plugin registration ----
static Op* build(Node* node) { return new VisualizeFlowDB(node); }
Op::Description VisualizeFlowDB::description("VisualizeFlowDB", build);

} // namespace pcn
