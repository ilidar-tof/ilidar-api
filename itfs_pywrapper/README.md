# iTFS Python Examples

Python examples and native bindings for the `iTFS::LiDAR` API.

Version: `V2.0.0`

## Compatibility

|Sensor|Minimum firmware|Python|
|:---|:---:|:---:|
|iTFS-110|V1.5.0|3.8 or newer|
|iTFS-80|V1.5.0|3.8 or newer|

The package supports Linux and Windows native builds. Its compiled extension is
specific to the operating system, CPU architecture, Python ABI, and compiler
runtime used during the build.

## Examples

|File|Description|
|:---|:---|
|`ilidar_wrapper.py`|Shared event receiver and argument helpers|
|`helloworld.py`|Helloworld receive loop for data, status, and info events|
|`opencv_example.py`|Depth and intensity display or headless image output|
|`open3d_example.py`|Calibrated point-cloud display with Open3D|

## Quick Start

Connect and configure the sensor so that it sends data to the host. The native
extension requires Python 3.8 or newer and a C++17 compiler. Run the following
commands from the repository root.

### Ubuntu

#### Requirements

```bash
sudo apt update
sudo apt install build-essential python3-dev python3-venv
python3 -m venv .venv
source .venv/bin/activate
python -m pip install "setuptools>=64" wheel "pybind11>=2.10" \
    "numpy>=1.20" opencv-python open3d
```

#### Build and Run

```bash
./build.sh --itfs_pywrapper
python itfs_pywrapper/helloworld.py
python itfs_pywrapper/opencv_example.py
python itfs_pywrapper/open3d_example.py
```

### Windows

#### Requirements

Install Visual Studio 2022 with the **Desktop development with C++** workload
and Python 3, then create an environment from the repository root:

```powershell
py -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install "setuptools>=64" wheel "pybind11>=2.10" `
    "numpy>=1.20" opencv-python open3d
```

#### Build and Run

```powershell
cd itfs_pywrapper
python setup.py build_ext --inplace
python helloworld.py
python opencv_example.py
python open3d_example.py
```

The wrapper is source-distributed with the complete `ilidar-api` tree and
requires the sibling `../src/` directory. Rebuild after changing Python ABI,
operating system, CPU architecture, or compiler runtime. Each example also
attempts an in-place build when its native module is missing.

### Common Options

Run a script with `--help` for its complete option list.

|Option|Used by|Purpose|
|:---|:---|:---|
|`--broadcast-ip <IP>`|All examples|Select the SDK broadcast interface|
|`--listening-ip <IP>`|All examples|Select the local receive interface|
|`--timeout <seconds>`|All examples|Set the sensor wait timeout|
|`--headless`|OpenCV|Run without display windows|
|`--save-prefix <path>`|OpenCV|Save output images using the supplied prefix|
|`--intrinsic <file>`|Open3D|Select the iTFS direction table|

## Public Types and Lifetime

|Python type|C++ counterpart|Lifetime rule|
|:---|:---|:---|
|`ilidar.LiDAR`|`iTFS::LiDAR`|Owns the native receive runtime|
|`ilidar.Device`|`iTFS::device_t`|Non-owning; valid only during its callback|
|`ilidar.Img`|`iTFS::img_t`|Reusable Python-owned image copy|
|Status/info dictionary|Native packet snapshot|Python copy that may be retained|

Do not store a `Device` for later use. Call `device.copy_image()` during the
native callback and retain the resulting `Img`. `Img.depth`, `Img.intensity`,
and `Img.raw_image` are NumPy views over a reusable buffer; the next frame for
the same device overwrites them. Use `numpy.copy()` when a frame must be kept.

## Data Contract

|Data|Python representation|Unit / validity|
|:---|:---|:---|
|Depth|`Img.depth` NumPy view|Unsigned millimeters; zero means invalid/no range|
|Intensity / gray|`Img.intensity` NumPy view|Original unsigned sensor value|
|Point cloud|`Img.point_cloud()`|XYZ floating point in meters after calibrated conversion|
|Status|Dictionary|Copied sensor health fields|
|Info|Dictionary|Copied identity and configured layout|

Images have a maximum resolution of `320 x 160`. Missing-row frames remain
available with `frame_status == 0x10`.

## Event Handling and Multiple Sensors

`ItfsEventReceiver` creates one reusable `Img` per possible device, copies the
current frame during the callback, and queues only its device index. It is a
live latest-frame design rather than a lossless recording queue.

```python
ilidar_module = load_ilidar()
lidar = ilidar_module.LiDAR()
events = ItfsEventReceiver(ilidar_module, lidar)

img = events.read_frame(timeout=1.0)        # Newest available device
img = events.read_frame(timeout=1.0, idx=1) # Selected runtime index
```

Device indexes follow discovery order. Use `find_device()` with an IP address
or serial number when persistent identity is required. OpenCV creates separate
windows per sensor; Open3D creates separate point-cloud objects. Replace the
example's zero rotation and fixed spacing placeholders with measured sensor
extrinsics in a real multi-sensor installation.

The binding is receive-oriented and exposes only the synchronization operation
needed by the examples. Use the dedicated product tools for configuration,
firmware, flash, and maintenance operations.

## Changelog and License

See [`../CHANGELOG.md`](../CHANGELOG.md) for release history. The SDK and
bindings are licensed under the MIT License in [`../LICENSE`](../LICENSE).
Optional dependencies retain their own licenses; see
[`../THIRD_PARTY_NOTICES.md`](../THIRD_PARTY_NOTICES.md).
