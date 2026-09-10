# Third-Party Notices

Version: `V2.0.1`

The iLidar SDK and example source code are licensed under the MIT License; see
`LICENSE`. The optional example and binding dependencies below are maintained
by their respective projects and retain their own licenses.

|Dependency|Used by|License|Project|
|:---|:---|:---|:---|
|OpenCV|C++ and Python image examples|Apache License 2.0|https://opencv.org/|
|Point Cloud Library (PCL)|C++ point-cloud examples|BSD 3-Clause|https://pointclouds.org/|
|VTK|PCL visualization dependency|BSD 3-Clause|https://vtk.org/|
|Open3D|Python point-cloud examples|MIT License|https://www.open3d.org/|
|pybind11|Python native bindings|BSD-style license|https://github.com/pybind/pybind11|
|NumPy|Python array interface|BSD 3-Clause|https://numpy.org/|

These libraries are not vendored into this source tree. They are installed
separately by the user or operating-system package manager. When distributing
binaries that bundle or redistribute any dependency, include the exact license
and notice files supplied by that dependency version.
