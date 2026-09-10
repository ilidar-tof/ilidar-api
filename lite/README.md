# iTFS-LITE C++ Examples

C++ receive examples for the `iTFS::LITE` API.

Version: `V2.0.1`

## Compatibility

|Sensor|Minimum firmware|API language|
|:---|:---:|:---:|
|iTFS-LITE|V1.0.0|C++17|

## Examples

|File|Description|
|:---|:---|
|`lite_helloworld.cpp`|Helloworld receive loop for data, status, and info events|
|`lite_opencv_example.cpp`|Depth, amplitude, intensity, and confidence display with OpenCV|
|`lite_pcl_example.cpp`|3D point-cloud display from depth or native XYZ data with PCL|

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
./build.sh --lite
./lite/bin/lite_helloworld_cmake
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
cmake -S lite -B lite/build `
  -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake `
  -DVCPKG_TARGET_TRIPLET=x64-windows `
  -DOpenCV_DIR=C:/vcpkg/installed/x64-windows/share/opencv4 `
  -DPCL_DIR=C:/vcpkg/installed/x64-windows/share/pcl
cmake --build lite/build --config Release
.\lite\bin\Release\lite_helloworld_cmake.exe
```

Each executable checks the LITE API and packet versions before starting.

## Data Contract

The active image classes depend on `info_v3.data_output`. Check that a class is
enabled before accessing it.

|Data class|Possible sensor representation|Application value|
|:---|:---|:---|
|Depth / XYZ-Z|MM16, raw Q-format, linear 8-bit, logarithmic 8-bit|Examples convert to unsigned millimeters; zero is invalid/no range|
|Amplitude|RAW16, LIN8, LOG8|Raw or restored unsigned sensor scale|
|Intensity|RAW16, LIN8, LOG8|Raw or restored unsigned sensor scale|
|Confidence|RAW16 or MASK1|Raw confidence or binary validity|
|Point X/Y|Native XYZ modes|Sensor-coordinate components used with Z|
|Point cloud|Converted XYZ|Floating-point meters; invalid points are omitted or marked invalid by the example|

LITE images are up to `320 x 240`. An enabled stream can use 8-bit or 16-bit
storage, so do not treat every slot as a 16-bit millimeter image. Frames with
missing rows are still delivered with
`frame_status == iTFS::status_missing_rows` (`0x10`).

## Callback Lifetime

One `iTFS::LITE` instance receives every discovered LITE sensor up to
`iTFS::max_device`.

|Object|Owner|Lifetime rule|
|:---|:---|:---|
|`lite_device_t *` callback argument|SDK|Valid only during the callback|
|`device->data` active slots|SDK|May be overwritten by a later packet|
|`lite_img_cpy_t`|Application|Reusable deep-copy target that may be retained|
|Copied status/info values|Application|May be retained after the callback|

Keep callbacks short. Copy active slots during the callback and perform
conversion, visualization, logging, or storage in another thread.

## Integration Pattern

Copy metadata first, then copy only the enabled packed slots. Access images
through the SDK helpers so the configured data class and storage width are
respected.

```cpp
static iTFS::lite_img_cpy_t images[iTFS::max_device];

static void lidar_data_handler(iTFS::lite_device_t *device) {
    const iTFS::lite_img_t *src = &device->data;
    iTFS::lite_img_cpy_t *dst = &images[device->idx];

    iTFS::copy_img_metadata(dst, src);
    std::memcpy(dst->data, src->data,
                sizeof(iTFS::lite_img_slot_t) * dst->data_class_count);
    // Queue device->idx and return quickly.
}
```

The complete examples reuse one destination per device and implement a live
latest-frame policy. Use `iTFS::get_img_u16_ptr()` and related helpers to find
enabled classes. The OpenCV and PCL examples inspect `data_output` and select
the matching restoration or XYZ conversion path.

## Changelog and License

See [`../CHANGELOG.md`](../CHANGELOG.md) for release history. The SDK and
examples are licensed under the MIT License in [`../LICENSE`](../LICENSE).
Optional dependencies retain their own licenses; see
[`../THIRD_PARTY_NOTICES.md`](../THIRD_PARTY_NOTICES.md).
