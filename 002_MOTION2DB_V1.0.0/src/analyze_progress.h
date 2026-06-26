// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Peter Mercell
//
// Developed with assistance from Claude (Anthropic)

// =============================================================================
// analyze_progress.h — minimal progress dialog for the Analyze stage.
//
// Deliberately Qt-FREE in the header so PolychaseTracker.cpp (which pulls in
// DDImage's GL/glew headers) never includes Qt and never hits the Qt<->glew
// header clash. All Qt lives in analyze_progress.cpp.
//
// Usage (on the main/GUI thread, which is where Analyze runs):
//     pcn::AnalyzeProgress prog("Polychase — analyzing optical flow");
//     ... in the per-frame progress callback ...
//         prog.set_percent(pct);
//         if (prog.cancelled()) return false;   // stop generation
//     // dialog closes when `prog` goes out of scope (RAII)
//
// In headless/terminal Nuke (no QApplication) every call is a silent no-op, so
// command-line renders are unaffected.
// =============================================================================
#ifndef PCN_ANALYZE_PROGRESS_H
#define PCN_ANALYZE_PROGRESS_H

#include <string>

namespace pcn {

class AnalyzeProgress {
public:
    explicit AnalyzeProgress(const std::string& title);
    ~AnalyzeProgress();

    AnalyzeProgress(const AnalyzeProgress&) = delete;
    AnalyzeProgress& operator=(const AnalyzeProgress&) = delete;

    // Update the bar (0..100) and pump the event loop so it repaints and the
    // Cancel button stays responsive while Analyze blocks the main thread.
    void set_percent(int pct);

    // True once the user has clicked Cancel.
    bool cancelled() const;

private:
    void* dlg_ = nullptr;   // opaque QProgressDialog* (nullptr when headless)
};

}  // namespace pcn

#endif  // PCN_ANALYZE_PROGRESS_H
