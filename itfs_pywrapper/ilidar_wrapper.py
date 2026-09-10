"""Core helpers for using the local iTFS Python wrapper scripts.

The example scripts import this module, and users can also import it from their
own scripts when they want the same local-build and receive-loop behavior.
It provides a small, focused layer around the native `ilidar` package:

* make the local pybind11 extension importable;
* build the extension in-place when it has not been built yet;
* open the C++ `iTFS::LiDAR` receive runtime with the same network arguments;
* receive native data/status/info callbacks through `Device`;
* copy image data into Python-owned `Img` objects;
* print status/info packets in the same format as the C++ iTFS examples.

This module does not configure the sensor. These helpers only read status,
info, and data already emitted by the C++ iTFS receive path.
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


def load_ilidar():
    """Import the local `ilidar` package, building the extension if needed.

    The wrapper is a native pybind11 extension, so the compiled file name is
    tied to the local Python ABI. This function adds `itfs_pywrapper` to
    `sys.path`, builds `_itfs` in-place when missing, and returns `ilidar`.
    """
    sys.path.insert(0, str(PYWRAPPER_DIR))
    if not extension_available("ilidar._itfs", "ilidar"):
        print("building local ilidar extension...")
        subprocess.check_call([sys.executable, "setup.py", "build_ext", "--inplace"], cwd=PYWRAPPER_DIR)
        if not extension_available("ilidar._itfs", "ilidar"):
            copy_built_extension()

    import ilidar

    return ilidar


def copy_built_extension():
    """Copy a built `_itfs*.so` from `build/` into `ilidar/`.

    `setup.py build_ext --inplace` normally performs this copy. This fallback
    handles platforms where setuptools leaves the extension under `build/`.
    """
    suffix = sysconfig.get_config_var("EXT_SUFFIX") or ".so"
    built_extensions = sorted(
        path for path in (PYWRAPPER_DIR / "build").rglob("_itfs*")
        if "ilidar" in path.parts and path.name.endswith(suffix) and not path.name.startswith("_itfs_lite")
    )
    if not built_extensions:
        built_extensions = sorted(
            path for path in (PYWRAPPER_DIR / "build").rglob("_itfs*")
            if "ilidar" in path.parts and not path.name.startswith("_itfs_lite")
        )
    if not built_extensions:
        raise ImportError("build finished but _itfs*.so was not found under build/")

    dst_dir = PYWRAPPER_DIR / "ilidar"
    dst_dir.mkdir(parents=True, exist_ok=True)
    dst = dst_dir / built_extensions[-1].name
    shutil.copy2(built_extensions[-1], dst)
    print(f"copied {built_extensions[-1]} -> {dst}")


def add_itfs_args(parser):
    """Add the common network/timeout options used by all iTFS examples.

    These options select the broadcast target, local UDP bind address, and
    frame wait timeout. They do not change the sensor output configuration.
    """
    parser.add_argument("--broadcast-ip", default=None)
    parser.add_argument("--listening-ip", default=None)
    parser.add_argument("--timeout", type=float, default=10.0)


def open_itfs(ilidar_module, args, queue_size=8):
    """Create an `ilidar.LiDAR` instance from parsed example arguments.

    The returned object owns the native C++ receive runtime. Attach
    `ItfsEventReceiver` to follow the C++ example's overwrite-and-flush data
    handling, and use the object as a context manager so it closes cleanly.
    """
    return ilidar_module.LiDAR(
        queue_size=queue_size,
        broadcast_ip=args.broadcast_ip,
        listening_ip=args.listening_ip,
    )


class ItfsEventReceiver:
    """Receive native iTFS events and copy image data only on data events.

    `Device` directly references the SDK-owned `device_t` and is valid only
    while the native callback is running. One Python-owned `Img` is kept
    per possible device and overwritten by `lidar_data_handler`, matching the C++ OpenCV
    example. Status and info events are converted to dictionaries and passed
    directly to their Python handlers without entering the image queue.
    """

    def __init__(
        self,
        ilidar_module,
        ilidar,
        status_handler=None,
        info_handler=None,
    ):
        """Allocate reusable per-device copies and register native handlers."""
        self._module = ilidar_module
        self.ilidar = ilidar
        self._status_handler = status_handler or print_itfs_status
        self._info_handler = info_handler or print_itfs_info
        self._info_printed = set()
        self._status_count = [
            0 for _ in range(int(self._module.max_device))
        ]
        self.lidar_cv = threading.Condition()
        self.lidar_img_data = [
            self._module.Img()
            for _ in range(int(self._module.max_device))
        ]
        self.lidar_q = deque()
        self._closed = False
        ilidar.set_event_handlers(
            on_data=self.lidar_data_handler,
            on_status=self.status_packet_handler,
            on_info=self.info_packet_handler,
        )

    def lidar_data_handler(self, device):
        """Deep-copy the received image and notify the main thread."""
        idx = device.idx
        img = self.lidar_img_data[idx]
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
            if info.get("version") in (1, 2):
                self._info_handler(info)
                self._info_printed.add(idx)

    def info_packet_handler(self, device):
        """Print the first info packet received from each sensor."""
        idx = device.idx
        if idx not in self._info_printed:
            # The native callback means a complete info/info_v2 packet was decoded.
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
                            "[WARNING] iTFS::LiDAR The main loop seems to be slower "
                            "than the LiDAR data reception handler."
                        )
                    if idx is None:
                        return self.lidar_img_data[received[-1]]
                    for recv_device_idx in reversed(received):
                        if recv_device_idx == idx:
                            return self.lidar_img_data[recv_device_idx]
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
        self.ilidar.set_event_handlers()


def wait_for_devices(ilidar, count=1, timeout=10.0):
    """Wait until at least `count` iTFS sensors have been discovered.

    `ilidar.ready` reports receive-runtime readiness, not sensor discovery. The
    returned list contains every device discovered before the timeout.
    """
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if ilidar.device_count >= count:
            return ilidar.devices()
        time.sleep(0.05)
    return ilidar.devices()


def find_device(ilidar, *, ip=None, sensor_sn=None):
    """Return a discovered iTFS device matching an IP or serial number.

    Device indexes follow discovery order and are stable only for the current
    runtime. Resolve a persistent IP address or serial number before selecting
    a device-specific stream.
    """
    if ip is None and sensor_sn is None:
        raise ValueError("ip or sensor_sn is required")
    for device in ilidar.devices():
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
    """Wait until a frame containing a depth image is received.

    Returns an `Img` from any sensor or from `idx`. Early discovery images
    without depth data are skipped until the timeout expires.
    """
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        img = source.read_frame(
            timeout=min(1.0, max(0.0, deadline - time.monotonic())),
            idx=idx,
        )
        if img is not None and img.depth is not None:
            return img
    return None


def drain_itfs_status_info(
    ilidar,
    idx=None,
    status_handler=None,
    info_handler=None,
):
    """Drain status/info for every sensor or one selected sensor."""
    status_handler = status_handler or print_itfs_status
    info_handler = info_handler or print_itfs_info
    while True:
        status = ilidar.read_status(timeout=0.0, idx=idx)
        if status is None:
            break
        status_handler(status)

    while True:
        info = ilidar.read_info(timeout=0.0, idx=idx)
        if info is None:
            break
        info_handler(info)


def frame_summary(img):
    """Return a compact one-line summary for warning/debug messages.

    Pixel arrays are intentionally omitted so this remains suitable for
    warnings such as "no point cloud from frame".
    """
    return (
        f"D# {img.idx} "
        f"{img.ip}:{img.port} "
        f"M {img.mode} "
        f"F# {img.frame} "
        f"STATUS {img.frame_status}"
    )


def print_itfs_status(status):
    """Print a status snapshot in the same shape as the C++ status handler.

    The dictionary is copied from the native status callback and is safe to
    keep after that callback returns. This function is read-only.
    """
    print(
        "[MESSAGE] iTFS::LiDAR status | "
        f"D#{status['idx']} "
        f"mode {status['capture_mode']} "
        f"frame {status['capture_frame']:2d} "
        f"time {status['sensor_time_us']} us "
        f"temp {status['temp_core_c']:.2f} "
        f"from {status['ip']}:{status['port']}"
    )
    if status.get("status_full_available"):
        full = status["status_full"]
        temps = ", ".join(f"{temp:.2f}" for temp in full["temp_c"])
        print(
            "\tstatus_full "
            f"frameStatus {full['sensor_frame_status']} "
            f"temps [ {temps} ] "
            f"vcsel {full['vcsel_level_v']:.2f}V "
            f"power {full['power_level_v']:.2f}V "
            f"warning {full['warning']}"
        )


def print_itfs_info(info):
    """Print an info/info_v2 snapshot like the C++ info handler.

    The output includes sensor identity, capture settings, network destination,
    synchronization settings, and firmware information. This function does not
    send commands or modify sensor configuration.
    """
    if info.get("version") == 0:
        print("[MESSAGE] iTFS::LiDAR info   | INVALID PACKET")
        return

    if info.get("version") == 2:
        print("[MESSAGE] iTFS::LiDAR info_v2 packet was received.")
        print(f"[MESSAGE] iTFS::LiDAR info_v2| D# {info['idx']}  lock {info['lock']}")
        print(
            f"\tSN #{info['sensor_sn']} mode {info['capture_mode']}, "
            f"rows {info['capture_row']}, period {info['capture_period_us']}"
        )
    else:
        print("[MESSAGE] iTFS::LiDAR info packet was received.")
        print(f"[MESSAGE] iTFS::LiDAR info   | D# {info['idx']}  lock {info['lock']}")
        print(
            f"\tSN #{info['sensor_sn']} mode {info['capture_mode']}, "
            f"rows {info['capture_row']}, period {info['capture_period']}"
        )

    print("\tshutter [ " + ", ".join(str(v) for v in info["capture_shutter"]) + " ]")
    print("\tlimit [ " + ", ".join(str(v) for v in info["capture_limit"]) + " ]")
    print(f"\tip   {info['data_sensor_ip']}")
    print(f"\tdest {info['data_dest_ip']}:{info['data_port']}")

    if info.get("version") == 2:
        print(
            f"\tsync {info['sync']}, syncBase {info['sync_trig_delay_us']} "
            f"autoReboot {info['arb']}, autoRebootTick {info['arb_timeout']}"
        )
    else:
        print(
            f"\tsync {info['sync']}, syncBase {info['sync_delay']} "
            f"autoReboot {info['arb']}, autoRebootTick {info['arb_timeout']}"
        )

    fw = info["sensor_fw_ver"]
    print(f"\tFW version: V{fw[0]}.{fw[1]}.{fw[2]} - {info['sensor_fw_time']} {info['sensor_fw_date']}")
    if info.get("version") == 2:
        fw0 = info["sensor_fw0_ver"]
        fw1 = info["sensor_fw1_ver"]
        fw2 = info["sensor_fw2_ver"]
        print(
            f"\tFW0: V{fw0[0]}.{fw0[1]}.{fw0[2]},  "
            f"FW1: V{fw1[0]}.{fw1[1]}.{fw1[2]},  "
            f"FW2: V{fw2[0]}.{fw2[1]}.{fw2[2]}"
        )
        if info["sensor_boot_mode"] == 0:
            print("\tSENSOR IS IN SAFE-MODE")
