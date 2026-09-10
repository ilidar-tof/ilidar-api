#!/usr/bin/env python3

import argparse
import os
import time
from pathlib import Path

from ilidar_wrapper import (
    ItfsEventReceiver,
    add_itfs_args,
    frame_summary,
    load_ilidar,
    open_itfs,
    print_itfs_info,
    print_itfs_status,
    wait_image_frame,
)


__version__ = "2.0.1"

HEIGHT_MIN_M = -1.5
HEIGHT_MAX_M = 1.5
DEVICE_SPACING_M = 1.0
# TODO: Replace these zeros with measured per-device extrinsic rotations.
YAW_RAD = (0.0,) * 8
PITCH_RAD = (0.0,) * 8
ROLL_RAD = (0.0,) * 8


def import_open3d_or_die():
    """Import Open3D and report an actionable error when it is unavailable."""
    if os.environ.get("XDG_SESSION_TYPE") == "wayland":
        os.environ["XDG_SESSION_TYPE"] = "x11"
    try:
        import open3d as o3d

        return o3d
    except Exception as exc:
        raise SystemExit(
            "Open3D import failed. Install it with:\n"
            "  python3 -m pip install open3d\n"
            f"Reason: {exc}"
        ) from None


def load_intrinsic(path):
    """Load the 240 x 320 x 3 direction table used by native conversion."""
    import numpy as np

    data = np.fromfile(path, dtype=np.float32)
    expected = 240 * 320 * 3
    if data.size != expected:
        raise SystemExit(f"invalid intrinsic file: {path}")
    return data.reshape((240, 320, 3))


def hsv_to_rgb(h):
    """Convert normalized HSV hue values to RGB colors without per-point loops."""
    import numpy as np

    h6 = h * 6.0
    i = np.floor(h6).astype(np.int32)
    f = h6 - i
    p = np.zeros_like(h)
    q = 1.0 - f
    t = f

    rgb = np.empty((h.shape[0], 3), dtype=np.float64)
    k = i % 6
    rgb[k == 0] = np.stack((np.ones_like(h[k == 0]), t[k == 0], p[k == 0]), axis=1)
    rgb[k == 1] = np.stack((q[k == 1], np.ones_like(h[k == 1]), p[k == 1]), axis=1)
    rgb[k == 2] = np.stack((p[k == 2], np.ones_like(h[k == 2]), t[k == 2]), axis=1)
    rgb[k == 3] = np.stack((p[k == 3], q[k == 3], np.ones_like(h[k == 3])), axis=1)
    rgb[k == 4] = np.stack((t[k == 4], p[k == 4], np.ones_like(h[k == 4])), axis=1)
    rgb[k == 5] = np.stack((np.ones_like(h[k == 5]), p[k == 5], q[k == 5]), axis=1)
    return rgb


def colorize_points(points):
    """Map point height to a blue-to-red HSV color range."""
    import numpy as np

    height = (points[:, 2] - HEIGHT_MIN_M) / (HEIGHT_MAX_M - HEIGHT_MIN_M)
    height = np.clip(height, 0.0, 1.0)
    hue = 0.66 * (1.0 - height)
    return hsv_to_rgb(hue)


def apply_device_transform(points, idx):
    """Apply placeholder extrinsics and separate devices along the Y axis."""
    import numpy as np

    yaw = YAW_RAD[idx]
    pitch = PITCH_RAD[idx]
    roll = ROLL_RAD[idx]
    cy, sy = np.cos(yaw), np.sin(yaw)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cr, sr = np.cos(roll), np.sin(roll)
    rotate_z = np.array(((cy, -sy, 0.0), (sy, cy, 0.0), (0.0, 0.0, 1.0)))
    rotate_y = np.array(((cp, 0.0, sp), (0.0, 1.0, 0.0), (-sp, 0.0, cp)))
    rotate_x = np.array(((1.0, 0.0, 0.0), (0.0, cr, -sr), (0.0, sr, cr)))
    transformed = points @ (rotate_z @ rotate_y @ rotate_x).T
    transformed[:, 1] += idx * DEVICE_SPACING_M
    return transformed


def create_grid(o3d, size_x=20, size_y=20, z=-1.0):
    """Create the fixed ground-reference grid shown by the viewer."""
    points = []
    lines = []
    for x in range(-size_x, size_x + 1):
        idx = len(points)
        points.append([x, -size_y, z])
        points.append([x, size_y, z])
        lines.append([idx, idx + 1])
    for y in range(-size_y, size_y + 1):
        idx = len(points)
        points.append([-size_x, y, z])
        points.append([size_x, y, z])
        lines.append([idx, idx + 1])
    grid = o3d.geometry.LineSet()
    grid.points = o3d.utility.Vector3dVector(points)
    grid.lines = o3d.utility.Vector2iVector(lines)
    grid.colors = o3d.utility.Vector3dVector([[0.3, 0.3, 0.3] for _ in lines])
    return grid


class Open3DPointCloudViewer:
    """Own one Open3D point cloud per discovered sensor."""

    def __init__(self, o3d):
        """Create the window, reference geometry, and keyboard callbacks."""
        self.o3d = o3d
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        if not self.vis.create_window("iTFS Open3D Point Cloud", width=960, height=720):
            raise RuntimeError(
                "Open3D failed to create a viewer window. Check that an OpenGL-capable "
                "desktop session is available."
            )
        self.vis.register_key_callback(ord("Q"), self.close)
        self.vis.register_key_callback(ord("S"), self.close)
        self.closed = False
        self.clouds = {}
        self.vis.add_geometry(create_grid(o3d))
        self.vis.add_geometry(o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0))
        opt = self.vis.get_render_option()
        if opt is not None:
            opt.background_color = [0.0, 0.0, 0.0]
            opt.point_size = 3.0

    def close(self, _vis=None):
        """Mark the viewer closed; Open3D key callbacks require a bool result."""
        self.closed = True
        return False

    def update(self, idx, points):
        """Replace one device cloud and render the latest scene."""
        import numpy as np

        if self.closed:
            return False
        if len(points) == 0:
            return self.poll()
        points = np.ascontiguousarray(points, dtype=np.float64)
        cloud = self.clouds.get(idx)
        if cloud is None:
            cloud = self.o3d.geometry.PointCloud()
            self.clouds[idx] = cloud
            cloud.points = self.o3d.utility.Vector3dVector(points)
            cloud.colors = self.o3d.utility.Vector3dVector(colorize_points(points))
            self.vis.add_geometry(cloud)
        else:
            cloud.points = self.o3d.utility.Vector3dVector(points)
            cloud.colors = self.o3d.utility.Vector3dVector(colorize_points(points))
            self.vis.update_geometry(cloud)
        return self.poll()

    def poll(self):
        """Process window events and return whether rendering should continue."""
        alive = self.vis.poll_events()
        self.vis.update_renderer()
        if not alive:
            self.closed = True
        return alive and not self.closed

    def destroy(self):
        """Release the native viewer window."""
        self.vis.destroy_window()


def status_packet_handler(status):
    """Print one status/status_full snapshot produced by the native callback."""
    # Corresponds to C++ status_packet_handler().
    # Use this for status/status_full snapshots only; configuration belongs to the C++ tool/API.
    print_itfs_status(status)


def info_packet_handler(info):
    """Print the first valid info/info_v2 snapshot for a device."""
    # Corresponds to C++ info_packet_handler().
    # Use this for info/info_v2 snapshots only; configuration belongs to the C++ tool/API.
    print_itfs_info(info)


def lidar_data_handler(viewer, img, intrinsic):
    """Convert one depth frame in native code and update its device cloud."""
    # Corresponds to C++ lidar_data_handler().
    # `img` is the per-device Img overwritten by the native data event.
    points = img.point_cloud(intrinsic)
    if points is None or len(points) == 0:
        print("no point cloud from frame", frame_summary(img))
        return True
    points = apply_device_transform(points, img.idx)
    return viewer.update(img.idx, points)


def main() -> None:
    """Run the Open3D receive and rendering loop."""
    parser = argparse.ArgumentParser()
    add_itfs_args(parser)
    parser.add_argument("--intrinsic", default=str(Path(__file__).resolve().with_name("iTFS-110.dat")))
    args = parser.parse_args()

    o3d = import_open3d_or_die()
    intrinsic = load_intrinsic(args.intrinsic)
    ilidar_module = load_ilidar()
    viewer = Open3DPointCloudViewer(o3d)

    with open_itfs(ilidar_module, args, queue_size=4) as ilidar:
        events = ItfsEventReceiver(
            ilidar_module,
            ilidar,
            status_handler=status_packet_handler,
            info_handler=info_packet_handler,
        )
        while not ilidar.ready:
            time.sleep(0.1)

        print("[MESSAGE] iTFS::LiDAR is ready.")
        print("close the viewer or press q, s, or Ctrl-C to stop")

        try:
            while True:
                if viewer.closed:
                    break

                img = wait_image_frame(events, args.timeout)
                if img is None:
                    print(f"no image frame within {args.timeout:.1f} seconds")
                    continue

                if not lidar_data_handler(viewer, img, intrinsic):
                    break
        except KeyboardInterrupt:
            print("stopping")
        finally:
            events.close()
            viewer.destroy()
            print("[MESSAGE] iTFS::LiDAR has been deleted.")


if __name__ == "__main__":
    main()
