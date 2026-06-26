// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Peter Mercell
//
// Developed with assistance from Claude (Anthropic)

// =============================================================================
// analyze_progress.cpp — QProgressDialog implementation. The ONLY TU that
// includes Qt. No Q_OBJECT / signals / slots, so no moc step is needed and we
// only touch long-stable API — headers from a newer Qt6 still run correctly
// against Nuke's bundled Qt6 6.5.3 (whose libs are already loaded at runtime).
// =============================================================================
#include "analyze_progress.h"

#include <QApplication>
#include <QProgressDialog>
#include <QString>
#include <Qt>

namespace pcn {

AnalyzeProgress::AnalyzeProgress(const std::string& title)
{
    // No QApplication => headless/terminal Nuke. Stay a no-op.
    if (QApplication::instance() == nullptr)
        return;

    auto* d = new QProgressDialog(
        QString::fromStdString(title),
        QStringLiteral("Cancel"),
        0, 100);

    // App-modal so the artist can't re-trigger Analyze mid-run; the UI was
    // already blocked by the synchronous generation, so this only adds a
    // visible bar + a working Cancel.
    d->setWindowModality(Qt::ApplicationModal);
    d->setMinimumDuration(0);     // show immediately rather than after 4s
    d->setAutoClose(false);
    d->setAutoReset(false);
    d->setValue(0);
    d->show();
    QApplication::processEvents();

    dlg_ = d;
}

AnalyzeProgress::~AnalyzeProgress()
{
    if (dlg_ == nullptr)
        return;
    auto* d = static_cast<QProgressDialog*>(dlg_);
    d->hide();
    delete d;                     // not inside a Qt slot, so direct delete is safe
    dlg_ = nullptr;
    QApplication::processEvents();
}

void AnalyzeProgress::set_percent(int pct)
{
    if (dlg_ == nullptr)
        return;
    if (pct < 0)   pct = 0;
    if (pct > 100) pct = 100;
    auto* d = static_cast<QProgressDialog*>(dlg_);
    d->setValue(pct);
    QApplication::processEvents();   // repaint + keep Cancel responsive
}

bool AnalyzeProgress::cancelled() const
{
    if (dlg_ == nullptr)
        return false;
    return static_cast<QProgressDialog*>(dlg_)->wasCanceled();
}

}  // namespace pcn
