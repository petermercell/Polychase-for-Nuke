// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Peter Mercell
//
// Developed with assistance from Claude (Anthropic)

// =============================================================================
// motion2db.h — Motion2DB Op: build a polychase optical-flow .db from a
// pre-rendered Nuke motion-vector EXR sequence on disk.
//
// Path A of MOTION2DB_PLAN.md: a no-input, button-driven NoIop that reads the
// motion EXRs frame-by-frame with TinyEXR (so there is NO Nuke frame-stepping
// involved — the bug that killed the in-node Analyze), resamples the dense flow
// at a grid of keypoints, and writes the exact records PolychaseTracker reads
// and VisualizeFlowDB inspects. It is the standalone mvflow_to_db CLI wrapped in
// a node with a Convert button and a Status line.
//
// All polychase/SQLite/TinyEXR access is confined to motion2db.cpp.
// =============================================================================
#pragma once

#include "DDImage/NoIop.h"
#include "DDImage/Knobs.h"
#include "DDImage/Knob.h"

#include <string>

namespace pcn {

class Motion2DB : public DD::Image::NoIop {
   public:
    explicit Motion2DB(Node* node) : DD::Image::NoIop(node) {}

    // Everything is read from disk; the single NoIop input is unused (Nuke fills
    // it with black when unconnected, which keeps _validate safe). Leave it
    // disconnected — the converter never touches input pixels.

    void knobs(DD::Image::Knob_Callback f) override;
    int knob_changed(DD::Image::Knob* k) override;

    const char* Class() const override { return kClass; }
    const char* node_help() const override {
        return "Motion2DB — build a polychase optical-flow database (.db) from a "
               "pre-rendered motion-vector EXR sequence (e.g. NNFlowVector's "
               "\"motion\" layer: forward.u/v + backward.u/v). Point it at the "
               "motion files, set the frame range and the output .db, press "
               "Convert. The result drives PolychaseTracker and can be inspected "
               "with VisualizeFlowDB. Reads frames straight from disk — no Nuke "
               "render/frame-stepping involved.";
    }

    static const char* const kClass;
    static const DD::Image::Op::Description description;

   private:
    void on_convert();
    void set_status(const std::string& s);

    // ---- knob storage --------------------------------------------------------
    // Source
    const char* motion_path_ = "";          // File_knob: …/motion.####.exr
    int         source_      = 0;            // 0 = File on disk (only mode for now)

    // Frames (DB image_ids are these 1-based Nuke frame numbers)
    int first_frame_ = 1;
    int last_frame_  = 100;

    // Output
    const char* db_path_   = "";            // File_knob: output .db
    bool        overwrite_ = true;          // fs::remove existing db first

    // Channel names (leaf-suffix fallback in the reader)
    const char* fwd_u_ = "motion.forward.u";
    const char* fwd_v_ = "motion.forward.v";
    const char* bwd_u_ = "motion.backward.u";
    const char* bwd_v_ = "motion.backward.v";

    // Conventions
    bool flip_v_ = true;                    // target_y = y - v (DB top-origin)
    bool negate_ = false;                   // flow stores "came-from" not "goes-to"

    // Keypoints
    int         grid_stride_ = 12;
    double      grad_gate_   = 0.0;         // skip grid pts below this luma gradient
    const char* plate_path_  = "";          // optional plate EXR seq for the gate

    // Pairs / quality
    bool   write_backward_  = true;
    bool   invert_backward_ = true;         // matches mvflow_to_db's default: backward
                                            // pairs = exact inverse of forward; appends
                                            // forward targets to keypoints (grid+appended)
    double max_error_       = 0.0;          // drop corrs with FB error above this

    // Status
    const char* status_cstr_ = "";
    std::string status_store_;
};

}  // namespace pcn
