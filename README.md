# iLidar API Example Projects

The iLidar API provides C++ and Python receive examples for the **iTFS** and
**iTFS-LITE** product families.

Version: `V2.0.0`

## Quick Start

Before running an example, connect the sensor and host to mutually reachable
IPv4 networks. Configure the sensor's destination IP address and output mode
with the iLidar configuration tool so that data is sent to the host.

### Ubuntu

#### C++

```bash
sudo apt update
sudo apt install build-essential cmake libopencv-dev libpcl-dev
./build.sh --itfs --lite

# iTFS
./itfs/bin/itfs_helloworld_cmake

# iTFS-LITE
./lite/bin/lite_helloworld_cmake
```

#### Python

```bash
sudo apt update
sudo apt install build-essential python3-dev python3-venv
python3 -m venv .venv
source .venv/bin/activate
python -m pip install "setuptools>=64" wheel "pybind11>=2.10" "numpy>=1.20"
./build.sh --itfs_pywrapper --lite_pywrapper

# iTFS
python itfs_pywrapper/helloworld.py

# iTFS-LITE
python lite_pywrapper/lite_helloworld.py
```

### Windows

#### C++

Install Visual Studio 2022 with the **Desktop development with C++** workload,
CMake, and Git. The following PowerShell example installs OpenCV and PCL with
vcpkg, then builds both product families:

```powershell
git clone https://github.com/microsoft/vcpkg.git C:\vcpkg
C:\vcpkg\bootstrap-vcpkg.bat
C:\vcpkg\vcpkg.exe install opencv4:x64-windows pcl:x64-windows

cmake -S itfs -B itfs/build `
  -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake `
  -DVCPKG_TARGET_TRIPLET=x64-windows `
  -DOpenCV_DIR=C:/vcpkg/installed/x64-windows/share/opencv4 `
  -DPCL_DIR=C:/vcpkg/installed/x64-windows/share/pcl
cmake --build itfs/build --config Release

cmake -S lite -B lite/build `
  -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake `
  -DVCPKG_TARGET_TRIPLET=x64-windows `
  -DOpenCV_DIR=C:/vcpkg/installed/x64-windows/share/opencv4 `
  -DPCL_DIR=C:/vcpkg/installed/x64-windows/share/pcl
cmake --build lite/build --config Release

# iTFS
.\itfs\bin\Release\itfs_helloworld_cmake.exe

# iTFS-LITE
.\lite\bin\Release\lite_helloworld_cmake.exe
```

#### Python

Install Visual Studio 2022 with the C++ workload and Python 3, then run:

```powershell
py -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install "setuptools>=64" wheel "pybind11>=2.10" "numpy>=1.20"

cd itfs_pywrapper
python setup.py build_ext --inplace
python helloworld.py

cd ..\lite_pywrapper
python setup.py build_ext --inplace
python lite_helloworld.py
```

Only one application should receive a sensor's data stream at a time. Close
other viewers or examples that use the same sensor before starting a new one.

## Project Overview

### Directory Layout

|Directory|Language|Target|Description|
|:---|:---:|:---:|:---|
|`itfs/`|C++|iTFS|`iTFS::LiDAR` examples|
|`itfs_pywrapper/`|Python|iTFS|Python examples and local `ilidar` package|
|`lite/`|C++|iTFS-LITE|`iTFS::LITE` examples|
|`lite_pywrapper/`|Python|iTFS-LITE|Python examples and local `ilidar_lite` package|
|`src/`|C++|SDK core|API implementation and pybind11 bindings|

### Examples

|Example|Language / Library|Description|
|:---|:---:|:---|
|Helloworld|C++ / Python|Receives data, status, and device information|
|OpenCV|C++ / Python, [OpenCV]|Displays or saves 2D sensor images|
|PCL|C++, [PCL]|Builds a 3D point cloud from depth data|
|Open3D|Python, [Open3D]|Displays a 3D point cloud from Python|

### API Families

The two APIs share a callback-oriented receive flow, but their packet and image
formats are different. Use the family matching the connected product.

|C++ API|Python package|Live callback object|Reusable image copy|
|:---|:---|:---|:---|
|`iTFS::LiDAR`|`ilidar`|`Device` (`device_t`)|`Img` (`img_t`)|
|`iTFS::LITE`|`ilidar_lite`|`LiteDevice` (`lite_device_t`)|`LiteImgCpy` (`lite_img_cpy_t`)|

## Integration Guides

Use the guide matching the product and language for callback lifetime rules,
packet-specific behavior, and integration examples:

- [iTFS C++ guide](itfs/README.md)
- [iTFS Python guide](itfs_pywrapper/README.md)
- [iTFS-LITE C++ guide](lite/README.md)
- [iTFS-LITE Python guide](lite_pywrapper/README.md)

The wrappers are intended primarily for receive-side integration. Use the
dedicated product tools for sensor configuration, firmware, flash, and
production maintenance operations.

## Changelog

See [CHANGELOG.md](CHANGELOG.md) for release history.

## License

The iLidar SDK and examples are licensed under the MIT License. Copyright
2022-Present HYBO Inc. See [LICENSE](LICENSE).

Optional dependencies retain their own licenses. See
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) before redistributing binaries
that include those dependencies.

[OpenCV]: https://opencv.org/
[PCL]: https://pointclouds.org/
[Open3D]: http://www.open3d.org/
