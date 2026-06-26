// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Peter Mercell
//
// Developed with assistance from Claude (Anthropic)

// =============================================================================
// visualize_flow_node.h — standalone "VisualizeFlowDB" NDK node.
//
// A NoIop that passes input 0 (the plate) through to the 2D viewer and draws
// the contents of a polychase optical-flow .db on top of it: a dot at each
// keypoint on the current frame, and a line from each keypoint to where it
// flows in the neighbour frame(s). No tracking, no solve — pure inspection, the
// in-viewer cousin of polychase's visualize_flow.cc.
//
// Architecture mirrors PolychaseWireframeKnob: a CustomKnob1 subclass owns the
// 2D-viewer draw_handle (the only place GL is valid), and reaches back into the
// Op for the per-frame geometry. All Database access lives in the .cpp so this
// header stays free of polychase/SQLite headers.
//
// Coordinate note: the DB stores REAL-PIXEL, TOP-ORIGIN keypoints (the same
// orientation Analyze fed it). The 2D viewer overlay draws in BOTTOM-ORIGIN
// image pixels, so draw_handle emits (x, fmt_h - y).
// =============================================================================
#ifndef PCN_VISUALIZE_FLOW_NODE_H
#define PCN_VISUALIZE_FLOW_NODE_H

// DDImage headers first: NoIop.h transitively pulls GL/glew.h, which hard-errors
// if a plain GL/gl.h was seen first. ViewerContext.h (needed complete for the
// inline build_handle below) pulls gl.h, so it must come AFTER NoIop.h.
#include "DDImage/NoIop.h"
#include "DDImage/Iop.h"
#include "DDImage/Op.h"
#include "DDImage/Knobs.h"
#include "DDImage/Knob.h"
#include "DDImage/Channel.h"
#include "DDImage/ViewerContext.h"

#include <string>
#include <vector>

namespace pcn {

using namespace DD::Image;

class VisualizeFlowDB;   // fwd: the knob holds an owner pointer to it

// Overlay primitives, in TOP-ORIGIN image pixels with 0..1 RGB. The knob flips
// Y to the viewer's bottom-origin space at draw time.
struct FlowPoint { float x,  y;          float r, g, b; };
struct FlowSeg   { float x0, y0, x1, y1; float r, g, b; float err; }; // err<0 = no data

// -----------------------------------------------------------------------------
// VisualizeFlowOverlayKnob — 2D-viewer overlay (draws only; no interaction).
// -----------------------------------------------------------------------------
class VisualizeFlowOverlayKnob : public Knob {
public:
    VisualizeFlowOverlayKnob(Knob_Closure* kc, void* pointer, const char* name)
        : Knob(kc, name)
        , owner_(static_cast<VisualizeFlowDB*>(pointer)) {}

    const char* Class() const override { return "VisualizeFlowOverlay"; }
    bool build_handle(ViewerContext* ctx) override { return ctx->transform_mode() == 0; }
    void draw_handle(ViewerContext* ctx) override;

private:
    VisualizeFlowDB* owner_;
};

// -----------------------------------------------------------------------------
// VisualizeFlowDB — NoIop passthrough of input 0 + the flow overlay.
// -----------------------------------------------------------------------------
class VisualizeFlowDB : public NoIop {
public:
    explicit VisualizeFlowDB(Node* node);
    ~VisualizeFlowDB() override;

    const char* Class()     const override { return description.name; }
    const char* node_help() const override {
        return "Overlay a polychase optical-flow database on the plate. Set the "
               "Flow DB path and wire the same footage into input 0; scrub to "
               "inspect keypoints and per-frame flow vectors.";
    }

    int  minimum_inputs() const override { return 0; }
    int  maximum_inputs() const override { return 1; }
    bool test_input(int idx, Op* op) const override {
        if (!op) return true;
        return idx == 0 && dynamic_cast<Iop*>(op) != nullptr;
    }
    const char* input_label(int idx, char* /*buf*/) const override {
        return idx == 0 ? "img" : "";
    }

    // NoIop's default _validate copies info from input(0) and NULL-derefs when
    // it's unwired — guard it (same pattern as PolychaseTracker).
    void _validate(bool for_real) override {
        if (input(0)) NoIop::_validate(for_real);
        else          set_out_channels(Mask_None);
    }

    void knobs(Knob_Callback f) override;
    int  knob_changed(Knob* k) override;

    // Called by the overlay knob each DRAW_OPAQUE. Fills markers + vectors for
    // `frame` in TOP-ORIGIN pixels. Cached so pan/zoom redraws don't re-query
    // SQLite; the cache invalidates on frame change, DB-path change, or a
    // vector-direction change.
    void collect_overlay(int frame,
                         std::vector<FlowPoint>& points,
                         std::vector<FlowSeg>& segs);

    // Draw-style accessors read live by the knob.
    bool  show_keypoints() const { return show_keypoints_; }
    bool  show_vectors()   const { return show_vectors_; }
    float point_size()     const { return (float)point_size_; }
    float line_width()     const { return (float)line_width_; }
    bool  color_by_error()  const { return color_by_error_; }
    float error_scale()     const { return (float)error_scale_; }
    float error_threshold() const { return (float)error_threshold_; }

    static Op::Description description;

private:
    void close_db_();

    // ---- knob-bound state ----
    const char* db_file_        = nullptr;   // File_knob
    bool        show_keypoints_ = true;
    bool        show_vectors_   = true;
    int         flow_dir_       = 0;         // 0=forward, 1=backward, 2=both
    double      point_size_     = 7.0;
    double      line_width_     = 1.0;
    bool        color_by_error_  = false;  // colour vectors by match error
    double      error_scale_     = 2.0;    // error value mapped to full red
    double      error_threshold_ = 0.0;    // 0 = off; >0 hides err > threshold

    // ---- opaque Database handle (kept open across redraws) ----
    void*       db_ = nullptr;               // Database* (defined in the .cpp)
    std::string db_open_path_;

    // ---- per-frame draw cache ----
    int                    cached_frame_ = -2147483647;
    bool                   cache_dirty_  = true;
    std::vector<FlowPoint> cache_points_;
    std::vector<FlowSeg>   cache_segs_;
};

} // namespace pcn

#endif // PCN_VISUALIZE_FLOW_NODE_H
