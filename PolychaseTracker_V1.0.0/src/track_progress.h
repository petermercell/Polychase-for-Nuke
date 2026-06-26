// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Peter Mercell
//
// Developed with assistance from Claude (Anthropic).

// =============================================================================
// track_progress.h — minimal progress dialog for the Track (TrackIt) stage.
//
// Deliberately Qt-FREE in the header so tracker_track.cpp (which transitively
// pulls DDImage's GL/glew headers via polychase_tracker.h) never includes Qt and
// never hits the Qt<->glew header clash. ALL Qt lives in track_progress.cpp.
// Mirrors Motion2DB's AnalyzeProgress, but isolated to this plugin.
//
// Usage (on the main/GUI thread, where on_track runs synchronously):
//     pcn::TrackProgress prog("Polychase — tracking forward");
//     ... inside the per-frame TrackSequence callback ...
//         prog.set_percent(pct);
//         if (prog.cancelled()) tracking_cancel_ = true;   // stop the solve
//     // dialog closes when `prog` goes out of scope (RAII)
//
// In headless/terminal Nuke (no QApplication) every call is a silent no-op, so
// command-line renders are unaffected. When the plugin is built WITHOUT Qt
// (POLYCHASE_PROGRESS=OFF, so POLYCHASE_HAVE_QT undefined) track_progress.cpp
// compiles a no-op body instead — the call sites stay identical and need no
// #ifdef. Same portability story as the rest of the plugin.
// =============================================================================
#ifndef PCN_TRACK_PROGRESS_H
#define PCN_TRACK_PROGRESS_H

#include <string>

namespace pcn {

class TrackProgress {
public:
    explicit TrackProgress(const std::string& title);
    ~TrackProgress();

    TrackProgress(const TrackProgress&)            = delete;
    TrackProgress& operator=(const TrackProgress&) = delete;

    // Update the bar (0..100) and pump the event loop so it repaints and the
    // Cancel button stays responsive while the synchronous track blocks the
    // main thread. Clamped to [0,100] internally.
    void set_percent(int pct);

    // True once the user has clicked Cancel. Always false when headless / Qt-free.
    bool cancelled() const;

private:
    void* dlg_ = nullptr;   // opaque QProgressDialog* (nullptr when headless/Qt-free)
};

}  // namespace pcn

#endif  // PCN_TRACK_PROGRESS_H
