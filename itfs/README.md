# iTFS C++ Examples

C++ receive examples for the `iTFS::LiDAR` API.

Version: `V2.0.0`

## Compatibility

|Sensor|Minimum firmware|API language|
|:---|:---:|:---:|
|iTFS-110|V1.5.0|C++17|
|iTFS-80|V1.5.0|C++17|

## Examples

|File|Description|
|:---|:---|
|`helloworld.cpp`|Helloworld receive loop for data, status, and info events|
|`opencv_example.cpp`|Depth and intensity image display with OpenCV|
|`pcl_example.cpp`|Calibrated 3D point-cloud display with PCL|

## Quick Start

Connect and configure the sensor so that it sends data to the host. The CMake
project configures all examples together, so OpenCV and PCL are required even
when only Helloworld will be run. Run the following commands from the
repository root.

### Ubuntu

#### Requirements

```bash
sudo apt update
sudo apt install build-essential cmake libopencv-dev libpcl-dev
```

#### Build and Run

```bash
./build.sh --itfs
./itfs/bin/itfs_helloworld_cmake
```

### Windows

#### Requirements

Install Visual Studio 2022 with the **Desktop development with C++** workload,
CMake, and Git. Install OpenCV and PCL with vcpkg:

```powershell
git clone https://github.com/microsoft/vcpkg.git C:\vcpkg
C:\vcpkg\bootstrap-vcpkg.bat
C:\vcpkg\vcpkg.exe install opencv4:x64-windows pcl:x64-windows
```

#### Build and Run

```powershell
cmake -S itfs -B itfs/build `
  -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake `
  -DVCPKG_TARGET_TRIPLET=x64-windows `
  -DOpenCV_DIR=C:/vcpkg/installed/x64-windows/share/opencv4 `
  -DPCL_DIR=C:/vcpkg/installed/x64-windows/share/pcl
cmake --build itfs/build --config Release
.\itfs\bin\Release\itfs_helloworld_cmake.exe
```

Each executable checks the API and packet versions before starting. The build
copies `iTFS-110.dat` and `iTFS-80.dat` beside the PCL executable; select the
calibration table matching the connected sensor optics in `pcl_example.cpp`.

## Data Contract

|Data|Representation|Unit / validity|
|:---|:---|:---|
|Depth|Unsigned 16-bit image|Millimeters; zero means invalid/no range|
|Intensity or mode-0 gray|Unsigned 16-bit image|Original sensor value|
|Point cloud|XYZ floating point|Meters after calibrated direction-table conversion|
|Status|`status_t` or `status_full_t`|Copied sensor status fields|
|Info|`info_t` or `info_v2_t`|Sensor identity and configured layout|

The maximum image resolution is `320 x 160`. Capture mode and row settings
determine the active region. A frame with missing rows is still delivered with
`frame_status == iTFS::status_missing_rows` (`0x10`).

## Callback Lifetime

One `iTFS::LiDAR` instance receives every discovered iTFS sensor up to
`iTFS::max_device`.

|Object|Owner|Lifetime rule|
|:---|:---|:---|
|`device_t *` callback argument|SDK|Valid only during the callback|
|`device->data`|SDK|May be overwritten by a later packet|
|Application image copy|Application|Valid until the application overwrites or releases it|
|Copied status/info values|Application|May be retained after the callback|

Keep callbacks short. Copy required data during the callback and perform image
conversion, visualization, logging, or storage in another thread.

## Integration Pattern

The examples allocate one reusable destination per device and queue only the
device index. This keeps packet reception bounded and implements a live
latest-frame policy: if processing falls behind, stale notifications are
discarded and the newest copied frame is processed.

```cpp
static iTFS::img_t images[iTFS::max_device];

static void lidar_data_handler(iTFS::device_t *device) {
    std::memcpy(&images[device->idx], &device->data, sizeof(device->data));
    // Queue device->idx and return quickly.
}

static void status_packet_handler(iTFS::device_t *device) {
    // Copy or inspect device->status / device->status_full.
}

static void info_packet_handler(iTFS::device_t *device) {
    // Copy or inspect device->info / device->info_v2.
}
```

Use `lidar_data_handler()` for copying image data,
`status_packet_handler()` for health monitoring, and `info_packet_handler()`
for device identification. The example keyboard threads demonstrate optional
configuration commands independently of the receive callback pattern.

## Changelog and License

See [`../CHANGELOG.md`](../CHANGELOG.md) for release history. The SDK and
examples are licensed under the MIT License in [`../LICENSE`](../LICENSE).
Optional dependencies retain their own licenses; see
[`../THIRD_PARTY_NOTICES.md`](../THIRD_PARTY_NOTICES.md).
