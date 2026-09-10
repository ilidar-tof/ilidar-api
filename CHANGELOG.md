# Changelog

All notable changes to the iLidar C++ API examples and Python wrappers are
documented in this file.

## [2.0.1] - 2026-09-09

LITE firmware V1.2.6 compatibility.

- Added the `advanced_trim` packet definition; internal `ilidar_lite_pack_ver` and `ilidar_lite_lib_ver` are now 2.0.2.
- Added 100 MHz F1 Single depth LOG8 decoding with a dedicated lookup table.
- Unified RAW/Linear8 depth and XYZ conversion in the PCL example and Python
  point-cloud paths with firmware fixed-point scaling: F1 uses 1499 mm and
  F2/Dual uses 7495 mm. All modes use divisors 65536/32768 for Z/XY RAW and
  256/128 for Z/XY Linear8. Millimeter output handling remains unchanged.
- Clarified independent Edge Filter and Post Cutoff settings. Post Cutoff
  removes depth where raw amplitude is below Capture Limit; the new firmware
  default is 127, while Post Cutoff remains OFF. Existing saved limits are retained.
- Documented F1/F2 Single exposure limits: 1200 us at capture periods of 50 ms
  or less, and 1600 us at longer periods.
- Corrected the distortion model used for iTFS-LITE depth-to-point-cloud
  reconstruction, improving agreement with native sensor XYZ output.

SDK version: 2.0.1. Sensor firmware version: 1.2.6.

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
