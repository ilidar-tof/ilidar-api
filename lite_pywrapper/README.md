# iTFS-LITE Python Examples

Python examples and native bindings for the `iTFS::LITE` API.

Version: `V2.0.1`

## Compatibility

|Sensor|Minimum firmware|Python|
|:---|:---:|:---:|
|iTFS-LITE|V1.0.0|3.8 or newer|

The package supports Linux and Windows native builds. Its compiled extension is
specific to the operating system, CPU architecture, Python ABI, and compiler
runtime used during the build.

## Examples

|File|Description|
|:---|:---|
|`ilidar_lite_wrapper.py`|Shared event receiver and argument helpers|
|`lite_helloworld.py`|Helloworld receive loop for data, status, and info events|
|`lite_opencv_example.py`|LITE image display or headless image output|
|`lite_open3d_example.py`|LITE point-cloud display with Open3D|

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
./build.sh --lite_pywrapper
python lite_pywrapper/lite_helloworld.py
python lite_pywrapper/lite_opencv_example.py
python lite_pywrapper/lite_open3d_example.py
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
cd lite_pywrapper
python setup.py build_ext --inplace
python lite_helloworld.py
python lite_opencv_example.py
python lite_open3d_example.py
```

The wrapper is source-distributed with the complete `ilidar-api` tree and
requires the sibling `../src/` directory. Rebuild after changing Python ABI,
operating system, CPU architecture, or compiler runtime. Each example also
attempts an in-place build when its native module is missing. Open3D wheel
availability on Raspberry Pi depends on the Python version, CPU architecture,
and desktop/OpenGL environment.

### Common Options

Run a script with `--help` for its complete option list.

|Option|Used by|Purpose|
|:---|:---|:---|
|`--broadcast-ip <IP>`|All examples|Select the SDK broadcast interface|
|`--listening-ip <IP>`|All examples|Select the local receive interface|
|`--timeout <seconds>`|All examples|Set the sensor wait timeout|
|`--headless`|OpenCV|Run without display windows|
|`--save-prefix <path>`|OpenCV|Save output images using the supplied prefix|

## Public Types and Lifetime

|Python type|C++ counterpart|Lifetime rule|
|:---|:---|:---|
|`ilidar_lite.LITE`|`iTFS::LITE`|Owns the native receive runtime|
|`ilidar_lite.LiteDevice`|`iTFS::lite_device_t`|Non-owning; valid only during its callback|
|`ilidar_lite.LiteImgCpy`|`iTFS::lite_img_cpy_t`|Reusable Python-owned active-image copy|
|Status/info dictionary|Native packet snapshot|Python copy that may be retained|

Do not store a `LiteDevice` for later use. Call `device.copy_image()` during
the callback and retain the destination `LiteImgCpy`. Accessing an expired
`LiteDevice` raises `RuntimeError`. NumPy image properties view the reusable
copy buffer; use `numpy.copy()` when a frame must be kept across later events.

## Data Contract

The available properties and their storage width depend on
`info_v3.data_output`.

|Property|Possible input|Meaning|
|:---|:---|:---|
|`depth`|MM16, raw Q-format, LIN8, LOG8, or XYZ-Z|Configured depth/Z representation|
|`amplitude`|RAW16, LIN8, or LOG8|Configured amplitude representation|
|`intensity`|RAW16, LIN8, or LOG8|Configured intensity representation|
|`confidence`|RAW16 or MASK1|Configured confidence representation|
|`point_x`, `point_y`|Native XYZ modes|Sensor-coordinate X/Y components|
|`point_cloud()`|Depth or XYZ input|Converted floating-point XYZ point cloud in meters|

An image property is absent when its class is disabled. Display helpers restore
stable viewable images, while `point_cloud()` performs the required depth/XYZ
conversion in native C++. LITE images are up to `320 x 240`; missing-row frames
remain available with `frame_status == 0x10`.

## Event Handling and Multiple Sensors

`LiteEventReceiver` creates one reusable `LiteImgCpy` per possible device,
copies active image slots during the callback, and queues only its device
index. It is a live latest-frame design rather than a lossless recording queue.

```python
lite_module = load_ilidar_lite()
lite = lite_module.LITE()
events = LiteEventReceiver(lite_module, lite)

img = events.read_frame(timeout=1.0)        # Newest available device
img = events.read_frame(timeout=1.0, idx=1) # Selected runtime index
```

Device indexes follow discovery order. Use `find_device()` with an IP address
or serial number when persistent identity is required. OpenCV creates separate
windows per sensor; Open3D creates separate point-cloud objects. Replace the
example's zero rotation and fixed spacing placeholders with measured sensor
extrinsics in a real multi-sensor installation.

The binding is receive-oriented. Use the dedicated product tools for sensor
configuration, firmware, flash, lock/unlock, and maintenance operations.

## Changelog and License

See [`../CHANGELOG.md`](../CHANGELOG.md) for release history. The SDK and
bindings are licensed under the MIT License in [`../LICENSE`](../LICENSE).
Optional dependencies retain their own licenses; see
[`../THIRD_PARTY_NOTICES.md`](../THIRD_PARTY_NOTICES.md).
