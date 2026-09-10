"""Core helpers for using the local iTFS-LITE Python wrapper scripts.

The example scripts import this module, and users can also import it from their
own scripts when they want the same local-build and receive-loop behavior.
It provides a small, focused layer around the native `ilidar_lite` package:

* make the local pybind11 extension importable;
* build the extension in-place when it has not been built yet;
* open the C++ iTFS::LITE receive runtime with the same network arguments;
* receive native data/status/info callbacks through `LiteDevice`;
* copy active image slots into Python-owned `LiteImgCpy` objects;
* print status/info packets in the same format as the C++ LITE examples.

This module does not configure the sensor. Keep command/configuration flows in
`ilidar_tool_lite.py`; these helpers only read status, info, and data already
emitted by the C++ LITE receive path.
"""

import importlib.util
from collections import deque
import shutil
import subprocess
import sys
import sysconfig
import threading
import time
from pathlib import Path


__version__ = "2.0.1"

INFO_STATUS_FALLBACK_COUNT = 10

PYWRAPPER_DIR = Path(__file__).resolve().parent
ROOT = PYWRAPPER_DIR.parent

try:
    sys.stdout.reconfigure(line_buffering=True)
except Exception:
    pass


def extension_available(module_name, package_name):
    """Return whether a native submodule can be imported without raising."""
    try:
        return importlib.util.find_spec(module_name) is not None
    except (ImportError, ModuleNotFoundError):
        sys.modules.pop(package_name, None)
        return False


def load_ilidar_lite():
    """Import the local `ilidar_lite` package, building the extension if needed.

    The wrapper is a native pybind11 extension, so the compiled file name is
    tied to the local Python ABI, for example `cpython-312-x86_64-linux-gnu`.
    This function keeps example execution simple:

    1. Add `ilidar-api/lite_pywrapper` to `sys.path`.
    2. Check whether `ilidar_lite._itfs_lite` can be imported.
    3. If it is missing, run `setup.py build_ext --inplace` in `lite_pywrapper`.
    4. Import and return the `ilidar_lite` module.

    Use this in scripts instead of importing `ilidar_lite` directly when the
    script should work on a fresh checkout without a manual build step.
    """
    sys.path.insert(0, str(PYWRAPPER_DIR))
    if not extension_available("ilidar_lite._itfs_lite", "ilidar_lite"):
        print("building local ilidar_lite extension...")
        subprocess.check_call([sys.executable, "setup.py", "build_ext", "--inplace"], cwd=PYWRAPPER_DIR)
        if not extension_available("ilidar_lite._itfs_lite", "ilidar_lite"):
            copy_built_extension()

    import ilidar_lite

    return ilidar_lite


def copy_built_extension():
    """Copy a built `_itfs_lite*.so` from `build/` into `ilidar_lite/`.

    `setup.py build_ext --inplace` normally copies the compiled extension to the
    package directory by itself. Some setuptools/platform combinations leave the
    file under `build/lib...` only. This fallback searches for an extension whose
    suffix matches the current Python ABI first, then falls back to any
    `_itfs_lite*.so`.

    The function raises `ImportError` when the build completed but no extension
    file was found, because importing the wrapper cannot succeed in that state.
    """
    suffix = sysconfig.get_config_var("EXT_SUFFIX") or ".so"
    built_extensions = sorted(
        path for path in (PYWRAPPER_DIR / "build").rglob("_itfs_lite*")
        if "ilidar_lite" in path.parts and path.name.endswith(suffix)
    )
    if not built_extensions:
        built_extensions = sorted(
            path for path in (PYWRAPPER_DIR / "build").rglob("_itfs_lite*")
            if "ilidar_lite" in path.parts
        )
    if not built_extensions:
        raise ImportError("build finished but _itfs_lite*.so was not found under build/")

    dst_dir = PYWRAPPER_DIR / "ilidar_lite"
    dst_dir.mkdir(parents=True, exist_ok=True)
    dst = dst_dir / built_extensions[-1].name
    shutil.copy2(built_extensions[-1], dst)
    print(f"copied {built_extensions[-1]} -> {dst}")


def add_lite_args(parser):
    """Add the common network/timeout options used by all LITE examples.

    The defaults mirror the C++ examples: if IP addresses are omitted, the C++
    LITE runtime uses its default broadcast/listening behavior.

    Arguments added:
    * `--broadcast-ip`: optional IPv4 broadcast target for discovery/commands.
    * `--listening-ip`: optional local IPv4 address to bind the UDP receiver.
    * `--timeout`: seconds to wait when an example is waiting for a frame.

    These are receive/open options only; they do not change sensor output mode.
    """
    parser.add_argument("--broadcast-ip", default=None)
    parser.add_argument("--listening-ip", default=None)
    parser.add_argument("--timeout", type=float, default=10.0)


def open_lite(ilidar_lite, args, queue_size=8):
    """Create an `ilidar_lite.LITE` instance from parsed example arguments.

    The returned object owns the native C++ `iTFS::LITE` receive runtime. It
    starts the UDP receive path and calls the C++ data/status/info handlers.
    Attach `LiteEventReceiver` to receive direct `LiteDevice` callbacks and
    copy active image slots into Python-owned `LiteImgCpy` objects.

    Use it as a context manager:

        with open_lite(ilidar_lite, args) as lite:
            ...

    `queue_size` limits native status/info queues. `LiteEventReceiver` owns the
    image overwrite-and-flush path used by these examples.
    """
    return ilidar_lite.LITE(
        queue_size=queue_size,
        broadcast_ip=args.broadcast_ip,
        listening_ip=args.listening_ip,
    )


class LiteEventReceiver:
    """Receive native LITE events and copy image data only on data events.

    `LiteDevice` directly references the SDK-owned `lite_device_t` and is valid
    only while the native callback is running. One Python-owned `LiteImgCpy` is
    created per possible device and overwritten by `lidar_data_handler`, matching the C++
    OpenCV example. Status and info events are converted to dictionaries and
    passed directly to their Python handlers without entering the image queue.
    """

    def __init__(
        self,
        ilidar_lite,
        lite,
        status_handler=None,
        info_handler=None,
    ):
        """Allocate reusable per-device copies and register native handlers."""
        self._module = ilidar_lite
        self.lite = lite
        self._status_handler = status_handler or print_lite_status
        self._info_handler = info_handler or print_lite_info
        self._info_printed = set()
        self._status_count = [
            0 for _ in range(int(self._module.max_device))
        ]
        self.lidar_cv = threading.Condition()
        self.lite_img_data = [
            self._module.LiteImgCpy()
            for _ in range(int(self._module.max_device))
        ]
        self.lidar_q = deque()
        self._closed = False
        lite.set_event_handlers(
            on_data=self.lidar_data_handler,
            on_status=self.status_packet_handler,
            on_info=self.info_packet_handler,
        )

    def lidar_data_handler(self, device):
        """Deep-copy active image slots and notify the main thread."""
        idx = device.idx
        img = self.lite_img_data[idx]
        device.copy_image(img)
        with self.lidar_cv:
            if self._closed:
                return
            self.lidar_q.append(idx)
            self.lidar_cv.notify_all()

    def status_packet_handler(self, device):
        """Print status and recover missed startup info after ten packets."""
        self._status_handler(device.status)
        idx = device.idx
        self._status_count[idx] += 1
        if (
            self._status_count[idx] >= INFO_STATUS_FALLBACK_COUNT
            and idx not in self._info_printed
        ):
            info = device.info
            if info.get("sensor_hw_id"):
                self._info_handler(info)
                self._info_printed.add(idx)

    def info_packet_handler(self, device):
        """Print the first valid info packet received from each sensor."""
        idx = device.idx
        if idx not in self._info_printed:
            # The native callback means a complete info_v3 packet was decoded.
            # A serial number of zero is valid on sensors without an assigned SN.
            self._info_handler(device.info)
            self._info_printed.add(idx)

    def read_frame(self, timeout=1.0, idx=None):
        """Return the newest overwritten device image, flushing any backlog."""
        deadline = None if timeout < 0 else time.monotonic() + timeout
        with self.lidar_cv:
            while True:
                if self.lidar_q:
                    received = [self.lidar_q.popleft()]
                    while self.lidar_q:
                        received.append(self.lidar_q.popleft())
                    if len(received) > 1:
                        print(
                            "[WARNING] iTFS::LITE The main loop seems to be slower "
                            "than the LITE data reception handler."
                        )
                    if idx is None:
                        return self.lite_img_data[received[-1]]
                    for recv_device_idx in reversed(received):
                        if recv_device_idx == idx:
                            return self.lite_img_data[recv_device_idx]
                if self._closed:
                    return None
                if deadline is None:
                    self.lidar_cv.wait()
                    continue
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self.lidar_cv.wait(remaining)

    def close(self):
        """Detach native callbacks and wake blocked readers."""
        with self.lidar_cv:
            if self._closed:
                return
            self._closed = True
            self.lidar_cv.notify_all()
        self.lite.set_event_handlers()


def wait_for_devices(lite, count=1, timeout=10.0):
    """Wait until at least `count` LITE sensors have been discovered.

    `lite.ready` only reports that the native receive runtime is ready. Use
    this helper when an application needs to wait for a known sensor count.
    The returned list contains every device discovered before the timeout.
    """
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if lite.device_count >= count:
            return lite.devices()
        time.sleep(0.05)
    return lite.devices()


def find_device(lite, *, ip=None, sensor_sn=None):
    """Return a discovered LITE device matching an IP or serial number.

    Device indexes follow discovery order and are stable only for the current
    runtime. Use this helper to resolve a persistent IP address or sensor
    serial number to its current `idx`.
    """
    if ip is None and sensor_sn is None:
        raise ValueError("ip or sensor_sn is required")
    for device in lite.devices():
        info = device.get("info", {})
        status = device.get("status", {})
        if ip is not None and device.get("ip") != ip:
            continue
        discovered_sn = info.get("sensor_sn") or status.get("sensor_sn")
        if sensor_sn is not None and discovered_sn != sensor_sn:
            continue
        return device
    return None


def wait_image_frame(source, timeout, idx=None):
    """Wait until a frame with at least one image/data class is received.

    `LiteEventReceiver.read_frame()` can return early images while output
    settings are settling. The image examples need a copy that contains real
    payload classes such as depth or amplitude, so this helper loops until
    `img.data_classes` is not
    empty or until `timeout` seconds pass.

    Returns:
        A `LiteImgCpy` when image data is available, otherwise `None`.
    """
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        img = source.read_frame(
            timeout=min(1.0, max(0.0, deadline - time.monotonic())),
            idx=idx,
        )
        if img is not None and img.data_classes:
            return img
    return None


def drain_lite_status_info(
    lite,
    idx=None,
    status_handler=None,
    info_handler=None,
):
    """Drain status/info for every sensor or one selected sensor."""
    status_handler = status_handler or print_lite_status
    info_handler = info_handler or print_lite_info
    while True:
        status = lite.read_status(timeout=0.0, idx=idx)
        if status is None:
            break
        status_handler(status)

    while True:
        info = lite.read_info(timeout=0.0, idx=idx)
        if info is None:
            break
        info_handler(info)


def frame_summary(img):
    """Return a compact one-line summary for warning/debug messages.

    This is deliberately short so it can be appended to warning messages such as
    "no point cloud from frame". It does not print pixel data or arrays.
    """
    return (
        f"D# {img.idx} "
        f"{img.ip}:{img.port} "
        f"M {img.mode} "
        f"F# {img.frame} "
        f"STATUS {img.frame_status} "
        f"CLASSES {list(img.data_classes)}"
    )


def yes_no(value):
    """Convert a LITE bit/flag value to the C++ example's ON/OFF text."""
    return "ON" if value else "OFF"


def depth_output_name(mode):
    """Return the symbolic name for the depth/XYZ part of `info_v3.data_output`.

    The low nibble of `data_output` selects the depth representation. In XYZ
    modes, the Z image is carried in the depth slot and X/Y are separate point
    image classes.
    """
    return {
        0x00: "OFF",
        0x01: "DEPTH_MM_16",
        0x02: "DEPTH_RAW_Q16",
        0x03: "DEPTH_LIN_8",
        0x04: "DEPTH_LOG_8",
        0x05: "XYZ_MM_16",
        0x06: "XYZ_RAW_Q15_Q16",
        0x07: "XYZ_LIN_8",
    }.get(mode, "UNKNOWN")


def stream_output_name(mode):
    """Return the symbolic name for amplitude or intensity output mode.

    Amplitude and intensity share the same 2-bit stream mode encoding after
    their bit fields are shifted down to zero.
    """
    return {
        0x00: "OFF",
        0x01: "RAW_16",
        0x02: "LIN_8",
        0x03: "LOG_8",
    }.get(mode, "UNKNOWN")


def confidence_output_name(mode):
    """Return the symbolic name for the confidence output mode."""
    return {
        0x00: "OFF",
        0x01: "RAW_16",
        0x02: "MASK_1",
    }.get(mode, "UNKNOWN")


def capture_freq_name(mode):
    """Decode the frequency field from `info_v3.capture_mode`.

    Bits 7:6 hold the frequency selection used by the C++ LITE examples.
    """
    return {
        0: "DUAL",
        1: "F1_SINGLE",
        2: "F2_SINGLE",
    }.get((mode >> 6) & 0x03, "RESERVED")


def capture_exposure_name(mode):
    """Decode the exposure field from `info_v3.capture_mode`.

    Bits 3:2 select auto/fixed/HDR exposure behavior.
    """
    return {
        0: "AUTO",
        1: "FIXED",
        2: "HDR_LV2",
        3: "HDR_LV3",
    }.get((mode >> 2) & 0x03, "HDR_LV3")


def capture_tof_name(mode):
    """Decode the ToF sampling/binning field from `info_v3.capture_mode`.

    Bits 1:0 select GRAY/NORMAL/V_BIN/HV_BIN.
    """
    return {
        0: "GRAY",
        1: "NORMAL",
        2: "V_BIN",
        3: "HV_BIN",
    }.get(mode & 0x03, "HV_BIN")


def split_data_output(data_output):
    """Split `info_v3.data_output` into normalized per-class mode values.

    The raw field packs four independent outputs:

    * bits 3:0   depth or XYZ mode;
    * bits 5:4   amplitude mode;
    * bits 7:6   intensity mode;
    * bits 9:8   confidence mode.

    The returned values are shifted down so they can be passed directly to
    `depth_output_name`, `stream_output_name`, and `confidence_output_name`.
    """
    return {
        "depth": data_output & 0x000F,
        "amplitude": (data_output >> 4) & 0x0003,
        "intensity": (data_output >> 6) & 0x0003,
        "confidence": (data_output >> 8) & 0x0003,
    }


def print_lite_status(status):
    """Print a status snapshot in the same shape as the C++ status handler.

    `status` is the dictionary returned by `lite.read_status()` or
    `lite.latest_status()`. It is copied from the C++ `status_packet_handler`
    path, so it is safe to keep or print after the native callback returns.

    This function only displays status information; it does not acknowledge,
    configure, or command the sensor.
    """
    time_us = int(status.get("sensor_time_us", 0))
    print(
        "[MESSAGE] iTFS::LITE status  "
        f"D# {status.get('idx', 0)}  "
        f"M {status.get('capture_mode', 0):3d}  "
        f"F# {status.get('capture_frame', 0):3d}  "
        f"t {time_us // 1000000}.{(time_us // 1000) % 1000:03d} sec"
    )
    print(
        "                             "
        f"TEMP {status.get('temp_rx_c', 0.0):6.2f} "
        f"{status.get('temp_tx_c', 0.0):6.2f} "
        f"{status.get('temp_core_c', 0.0):6.2f} C  "
        f"EXP {status.get('exposure_us', 0):3d} us"
    )
    power = status.get("power_level_v", (0.0, 0.0))
    print(
        "                             "
        f"VOLT  {status.get('usb_level_v', 0.0):5.2f}  "
        f"{power[0]:5.2f}  {power[1]:5.2f} V  "
        f"USE {status.get('cpu_usage_percent', 0.0):6.2f} "
        f"{status.get('ram_usage_percent', 0.0):6.2f} %"
    )
    print(
        "                             "
        f"DROP {status.get('frame_drop_count', 0):3d} "
        f"{status.get('udp_rx_drop_count', 0):3d} "
        f"{status.get('udp_tx_drop_count', 0):3d}             "
        f"WARN 0x{status.get('warning', 0):08X}"
    )


def print_lite_info(info):
    """Print an info_v3 snapshot in the same shape as the C++ info handler.

    `info` is the dictionary returned by `lite.read_info()` or
    `lite.latest_info()`. It includes the sensor identity, capture mode,
    data-output mode, timing, destination, DHCP, and firmware fields that the
    C++ LITE examples print.

    This function is intentionally read-only. Sensor configuration should stay
    in `ilidar_tool_lite.py`.
    """
    capture_mode = int(info.get("capture_mode", 0))
    data_output = int(info.get("data_output", 0))
    modes = split_data_output(data_output)

    print("[MESSAGE] iTFS::LITE info_v3 packet was received.")
    print(f"[MESSAGE] iTFS::LITE info_v3 {info.get('sensor_hw_id', '')[:24]} was detected.")
    print(
        "                             "
        f"SN# {info.get('sensor_sn', 0)}  "
        f"D# {info.get('idx', 0)}  "
        f"LOCK {yes_no(info.get('lock', 0))}(0x{info.get('lock', 0):02X})"
    )
    print(f"[MESSAGE] iTFS::LITE info_v3 CAPTURE_MODE: {capture_mode}(0x{capture_mode:02X})")
    print(f"                             FREQ: {capture_freq_name(capture_mode)}({(capture_mode >> 6) & 0x03})")
    print(f"                             EDGE FILTER: {yes_no((capture_mode >> 4) & 0x01)}({(capture_mode >> 4) & 0x01})")
    print(f"                             DUST FILTER: {yes_no((capture_mode >> 5) & 0x01)}({(capture_mode >> 5) & 0x01})")
    print(f"                             EXPOSURE: {capture_exposure_name(capture_mode)}({(capture_mode >> 2) & 0x03})")
    print(f"                             TOF: {capture_tof_name(capture_mode)}({capture_mode & 0x03})")
    print(f"[MESSAGE] iTFS::LITE info_v3 DATA_OUTPUT: {data_output}(0x{data_output:04X})")
    print(f"                             DEPTH: {depth_output_name(modes['depth'])}({modes['depth']})")
    print(f"                             AMPLITUDE: {stream_output_name(modes['amplitude'])}({modes['amplitude']})")
    print(f"                             INTENSITY: {stream_output_name(modes['intensity'])}({modes['intensity']})")
    print(f"                             CONFIDENCE: {confidence_output_name(modes['confidence'])}({modes['confidence']})")
    print("[MESSAGE] iTFS::LITE info_v3 OTHERS")
    print(f"                             PERIOD: {info.get('capture_period_ns', 0)} ns")
    shutter = info.get("capture_shutter", (0, 0, 0))
    print(f"                             EXPOSURE: [ {shutter[0]}, {shutter[1]}, {shutter[2]} ] us")
    print(f"                             IP:   {info.get('data_sensor_ip', '0.0.0.0')}")
    print(f"                             DEST: {info.get('data_dest_ip', '0.0.0.0')}:{info.get('data_port', 0)} (DHCP {yes_no(info.get('dhcp', 0))})")
    fw = info.get("sensor_fw_ver", (0, 0, 0))
    print(f"                             FWL V{fw[0]}.{fw[1]}.{fw[2]} -  {info.get('sensor_fw_time', '')} {info.get('sensor_fw_date', '')}")
