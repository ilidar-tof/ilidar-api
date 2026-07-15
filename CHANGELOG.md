# Changelog

All notable changes to the iLidar C++ API examples and Python wrappers are
documented in this file.

## [2.0.0] - 2026-07-15

Initial V2 release.

### Added

- iTFS and iTFS-LITE C++ receive APIs with data, status, and info callbacks.
- Helloworld, OpenCV, and PCL examples for both product families.
- Local-build Python bindings with Helloworld, OpenCV, and Open3D examples.
- Multi-device receive examples using reusable per-device frame storage.
- LITE depth, amplitude, intensity, confidence, and XYZ conversion support.
- iTFS direction-table resources for calibrated point-cloud generation.
- Linux build orchestration and Windows CMake/build documentation.

### Documentation

- Added product-specific C++ and Python integration guides.
- Documented callback lifetime, latest-frame processing, dependencies, and
  supported direct-build workflows.
- Standardized the project version as V2.0.0 and separated optional third-party
  dependency notices from the MIT project license.
