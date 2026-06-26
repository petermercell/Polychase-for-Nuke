// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Peter Mercell
//
// Developed with assistance from Claude (Anthropic).

// =============================================================================
// polychase_pose_math.h — the DDImage-FREE pose math (TRS compose / decompose)
// extracted from polychase_util.h so it can be unit-tested standalone.
//
// These two functions are the load-bearing bridge between the solved 4x4 object
// pose and the animated translate / rotate / scale knob curves (the export
// path). A regression here silently corrupts every baked shot, and there was
// already one historical convention bug in this area — hence the dedicated test
// (test/test_pose_math.cpp) that round-trips them over a grid of poses.
//
// Nothing here touches Nuke: only <Eigen/Core> + <cmath>. polychase_util.h pulls
// this in (with PCN_POSE_MATH_EXTERNAL_TYPEDEFS defined so RowMajorMatrix4f stays
// the project typedef from eigen_typedefs.h); the test pulls it in WITHOUT that
// macro, so the header supplies its own identical typedef and compiles with just
// Eigen on the include path. Both consumers therefore exercise the SAME code.
//
// Convention (unchanged from the original util.h definition):
//   Rotation order R = Rx * Ry * Rz, angles in DEGREES, XYZ.
//   compose_trs / decompose_trs round-trip exactly outside gimbal lock; at the
//   ry = ±90° singularity decompose collapses rz into rx (rz := 0), which still
//   recomposes to the same matrix — the test checks the MATRIX round-trip, not
//   the raw Euler triple, precisely because of that.
// =============================================================================
#ifndef POLYCHASE_POSE_MATH_H
#define POLYCHASE_POSE_MATH_H

#include <Eigen/Core>

#include <algorithm>
#include <cmath>

namespace pcn {

// Single source of pi for this header (was duplicated as a literal in both
// compose_trs and decompose_trs).
inline constexpr double kPi = 3.14159265358979323846;

// In the plugin build RowMajorMatrix4f is the project typedef from
// eigen_typedefs.h (polychase_util.h defines PCN_POSE_MATH_EXTERNAL_TYPEDEFS
// before including this header, so we DON'T redefine it and there's no risk of a
// clashing typedef). In a standalone test build the macro is absent and we
// supply the identical type ourselves.
#ifndef PCN_POSE_MATH_EXTERNAL_TYPEDEFS
using RowMajorMatrix4f = Eigen::Matrix<float, 4, 4, Eigen::RowMajor>;
#endif

// -----------------------------------------------------------------------------
// Translate / Rotate(deg, XYZ: R = Rx*Ry*Rz) / Scale  <->  row-major 4x4.
// Used to store the solved object pose as animated translate/rotate/scale
// knobs (so it keyframes on the timeline and persists in the .nk). The
// compose/decompose pair round-trips exactly (verified by test_pose_math).
// -----------------------------------------------------------------------------
inline RowMajorMatrix4f compose_trs(const double t[3], const double r_deg[3],
                                    const double s[3])
{
    const double d2r = kPi / 180.0;
    const double rx = r_deg[0]*d2r, ry = r_deg[1]*d2r, rz = r_deg[2]*d2r;
    const double cx = std::cos(rx), sx = std::sin(rx);
    const double cy = std::cos(ry), sy = std::sin(ry);
    const double cz = std::cos(rz), sz = std::sin(rz);
    double R[3][3];
    R[0][0]= cy*cz;             R[0][1]=-cy*sz;             R[0][2]= sy;
    R[1][0]= sx*sy*cz + cx*sz;  R[1][1]=-sx*sy*sz + cx*cz;  R[1][2]=-sx*cy;
    R[2][0]=-cx*sy*cz + sx*sz;  R[2][1]= cx*sy*sz + sx*cz;  R[2][2]= cx*cy;
    RowMajorMatrix4f M;
    M.setIdentity();
    for (int row = 0; row < 3; ++row) {
        M((Eigen::Index)row, 0) = (float)(R[row][0]*s[0]);
        M((Eigen::Index)row, 1) = (float)(R[row][1]*s[1]);
        M((Eigen::Index)row, 2) = (float)(R[row][2]*s[2]);
        M((Eigen::Index)row, 3) = (float)t[row];
    }
    return M;
}

inline void decompose_trs(const RowMajorMatrix4f& M, double t[3],
                          double r_deg[3], double s[3])
{
    const double r2d = 180.0 / kPi;
    for (int i = 0; i < 3; ++i) t[i] = M((Eigen::Index)i, 3);

    double col[3][3];
    for (int c = 0; c < 3; ++c) {
        col[c][0] = M(0, (Eigen::Index)c);
        col[c][1] = M(1, (Eigen::Index)c);
        col[c][2] = M(2, (Eigen::Index)c);
        s[c] = std::sqrt(col[c][0]*col[c][0] + col[c][1]*col[c][1] + col[c][2]*col[c][2]);
    }
    double Rm[3][3];
    for (int c = 0; c < 3; ++c) {
        const double inv = (s[c] > 1e-12) ? 1.0/s[c] : 0.0;
        Rm[0][c] = col[c][0]*inv;
        Rm[1][c] = col[c][1]*inv;
        Rm[2][c] = col[c][2]*inv;
    }

    // Reflection / negative scale. Column norms are unsigned, so a matrix with a
    // negative determinant (an odd number of mirrored axes — e.g. a TransformGeo
    // with scale -1 on one axis) would otherwise decompose to all-positive scale
    // with the sign smeared into the rotation block, and Rm would be improper
    // (det = -1), which the proper-rotation Euler extraction below cannot
    // represent — so it would NOT round-trip. Fold the sign into a single scale
    // axis (X) and negate the matching rotation column to restore a proper
    // rotation (det = +1); compose_trs reproduces the reflection via the negative
    // s[0]. For the common det>0 case this is a no-op.
    const double det =
        Rm[0][0]*(Rm[1][1]*Rm[2][2] - Rm[1][2]*Rm[2][1]) -
        Rm[0][1]*(Rm[1][0]*Rm[2][2] - Rm[1][2]*Rm[2][0]) +
        Rm[0][2]*(Rm[1][0]*Rm[2][1] - Rm[1][1]*Rm[2][0]);
    if (det < 0.0) {
        s[0] = -s[0];
        Rm[0][0] = -Rm[0][0];
        Rm[1][0] = -Rm[1][0];
        Rm[2][0] = -Rm[2][0];
    }

    // Inverse of R = Rx*Ry*Rz.
    const double clampv = std::max(-1.0, std::min(1.0, Rm[0][2]));
    const double ry = std::asin(clampv);
    const double cy = std::cos(ry);
    double rx, rz;
    if (std::abs(cy) > 1e-6) {
        rx = std::atan2(-Rm[1][2], Rm[2][2]);
        rz = std::atan2(-Rm[0][1], Rm[0][0]);
    } else {                       // gimbal lock
        rx = std::atan2(Rm[2][1], Rm[1][1]);
        rz = 0.0;
    }
    r_deg[0] = rx*r2d; r_deg[1] = ry*r2d; r_deg[2] = rz*r2d;
}

} // namespace pcn

#endif // POLYCHASE_POSE_MATH_H
