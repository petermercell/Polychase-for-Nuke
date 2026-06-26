// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Peter Mercell
//
// Developed with assistance from Claude (Anthropic).

// =============================================================================
// test/test_pose_math.cpp — regression test for the TRS compose/decompose pair
// that bridges the solved object pose and the animated translate/rotate/scale
// knob curves (polychase_pose_math.h). DDImage-free, so it builds and runs on
// any machine with just Eigen on the include path:
//
//     g++ -std=c++17 -I<eigen> -I.. test_pose_math.cpp -o test_pose_math && ./test_pose_math
//
// (CMake target `pose_math_test`; `ctest` / `ctest -R pose_math` runs it.)
//
// What it actually checks — the MATRIX round-trip, not the raw Euler triple:
//   compose_trs(decompose_trs(M)) == M   over a grid of T/R/S poses,
// because at the ry = ±90° gimbal-lock singularity decompose legitimately
// collapses rz into rx (rz := 0) — a DIFFERENT triple that recomposes to the
// SAME matrix. Comparing matrices is the convention-agnostic correctness gate;
// comparing angles would flag a correct gimbal-lock branch as a failure.
//
// Tolerance: the knobs/storage are float, so we compose in float and allow a
// few-ULP epsilon (1e-4 absolute on a unit-ish matrix). Translations up to 1e4
// are tested with a relative bound so large offsets don't trip the absolute eps.
// =============================================================================
#include "polychase_pose_math.h"

#include <Eigen/Core>

#include <cmath>
#include <cstdio>
#include <array>
#include <algorithm>
#include <string>
#include <vector>

using pcn::compose_trs;
using pcn::decompose_trs;
using Mat4 = pcn::RowMajorMatrix4f;

namespace {

int    g_failures = 0;
int    g_checks   = 0;

// Max absolute element difference between two matrices, with a relative slack on
// the translation column so a 10,000-unit offset isn't held to the same absolute
// eps as a rotation cosine.
double matrix_max_err(const Mat4& a, const Mat4& b)
{
    double worst = 0.0;
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c) {
            const double av = a((Eigen::Index)r, (Eigen::Index)c);
            const double bv = b((Eigen::Index)r, (Eigen::Index)c);
            const double scale = (c == 3) ? std::max(1.0, std::abs(av)) : 1.0;
            worst = std::max(worst, std::abs(av - bv) / scale);
        }
    return worst;
}

void check_roundtrip(const char* what,
                     const double t[3], const double r[3], const double s[3],
                     double eps = 1e-4)
{
    ++g_checks;
    const Mat4 M0 = compose_trs(t, r, s);

    double t1[3], r1[3], s1[3];
    decompose_trs(M0, t1, r1, s1);
    const Mat4 M1 = compose_trs(t1, r1, s1);

    bool finite = true;
    for (int r = 0; r < 4 && finite; ++r)
        for (int c = 0; c < 4; ++c)
            if (!std::isfinite(M1((Eigen::Index)r, (Eigen::Index)c))) { finite = false; break; }

    const double err = matrix_max_err(M0, M1);
    if (!(err <= eps) || !finite) {
        ++g_failures;
        std::printf("  [FAIL] %-28s  max_err=%.3e (eps=%.0e)\n", what, err, eps);
        std::printf("         in : T(%.3f %.3f %.3f) R(%.3f %.3f %.3f) S(%.3f %.3f %.3f)\n",
                    t[0],t[1],t[2], r[0],r[1],r[2], s[0],s[1],s[2]);
        std::printf("         out: T(%.3f %.3f %.3f) R(%.3f %.3f %.3f) S(%.3f %.3f %.3f)\n",
                    t1[0],t1[1],t1[2], r1[0],r1[1],r1[2], s1[0],s1[1],s1[2]);
    } else {
        std::printf("  [ ok ] %-28s  max_err=%.3e\n", what, err);
    }
}

// Identity must decompose to T=0, R=0, S=1 EXACTLY (a stronger, absolute check —
// the neutral pose is what the overlay falls back to, so any drift here is a
// real bug, not a float artefact).
void check_identity()
{
    ++g_checks;
    Mat4 I = Mat4::Identity();
    double t[3], r[3], s[3];
    decompose_trs(I, t, r, s);
    const bool ok =
        std::abs(t[0]) < 1e-6 && std::abs(t[1]) < 1e-6 && std::abs(t[2]) < 1e-6 &&
        std::abs(r[0]) < 1e-4 && std::abs(r[1]) < 1e-4 && std::abs(r[2]) < 1e-4 &&
        std::abs(s[0]-1.0) < 1e-6 && std::abs(s[1]-1.0) < 1e-6 && std::abs(s[2]-1.0) < 1e-6;
    if (!ok) {
        ++g_failures;
        std::printf("  [FAIL] identity decompose      T(%.3g %.3g %.3g) R(%.3g %.3g %.3g) S(%.3g %.3g %.3g)\n",
                    t[0],t[1],t[2], r[0],r[1],r[2], s[0],s[1],s[2]);
    } else {
        std::printf("  [ ok ] identity decompose\n");
    }
}

} // namespace

int main()
{
    std::printf("== pose-math round-trip test (compose_trs / decompose_trs) ==\n");

    check_identity();

    // ---- a spread of translations (incl. large offsets the export sees) ----
    const std::vector<std::array<double,3>> trans = {
        {0,0,0}, {1,2,3}, {-5,10,-15}, {1234.5, -6789.0, 4321.0}, {0,0,9999.0}
    };
    // ---- rotations: axis-aligned, compound, near-lock, AT lock ----
    const std::vector<std::array<double,3>> rots = {
        {0,0,0}, {30,0,0}, {0,45,0}, {0,0,60}, {15,30,45}, {-23,67,-89},
        {10,89.0,10}, {0,90,0}, {0,-90,0}, {180,0,0}, {0,179,0}, {-179,12,179}
    };
    // ---- scales: uniform + a touch non-uniform + reflected (negative). A
    //      TransformGeo can carry a negative axis scale (det<0); decompose must
    //      fold the sign into one scale axis so the matrix still round-trips. ----
    const std::vector<std::array<double,3>> scales = {
        {1,1,1}, {2,2,2}, {0.5,0.5,0.5}, {1.5,0.75,2.25},
        {-1,1,1}, {1,-1,1}, {1,1,-1}, {-2,1.5,0.5}
    };

    for (const auto& T : trans)
        for (const auto& R : rots)
            for (const auto& S : scales) {
                char name[96];
                std::snprintf(name, sizeof(name), "T%.0f,%.0f,%.0f R%.0f,%.0f,%.0f S%.2g",
                              T[0],T[1],T[2], R[0],R[1],R[2], S[0]);
                check_roundtrip(name, T.data(), R.data(), S.data());
            }

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    if (g_failures == 0) std::printf("PASS\n");
    else                 std::printf("FAIL\n");
    return g_failures == 0 ? 0 : 1;
}
