# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- **PoseLib dependency** (v2.0.4, BSD-3-Clause) via vcpkg, providing minimal
  solvers for calibrated camera pose estimation. Depends on eigen3 (already a
  project dependency). Used by the tracker's PnP / pose-solving path.
- `gflags` dependency for the command-line example tools under `cpp/examples`.

### Changed
- Pinned vcpkg `builtin-baseline` to
  `84bab45d415d22042bd0b9081aea57f362da3f35` for reproducible dependency
  resolution across machines.
- Pinned `tbb` to 2021.13.0 via `overrides` to keep the Embree TBB tasking
  backend on a known-good version.
