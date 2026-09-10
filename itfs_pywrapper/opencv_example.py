#!/usr/bin/env python3

import argparse
import contextlib
import io
import site
import sys
import time

from ilidar_wrapper import (
    ItfsEventReceiver,
    add_itfs_args,
    load_ilidar,
    open_itfs,
    print_itfs_info,
    print_itfs_status,
    wait_image_frame,
)


__version__ = "2.0.1"

ILIDAR_SYNC = True
SYNC_PACKET_PERIOD_SECONDS = 10.0


def import_cv2_or_die():
    """Import system OpenCV while avoiding an incompatible user-site build."""
    usersite = site.getusersitepackages()
    original_path = list(sys.path)
    sys.path = [
        path for path in sys.path
        if path != usersite and not path.startswith(usersite + "/")
    ]
    try:
        with contextlib.redirect_stderr(io.StringIO()):
            import cv2

        return cv2
    except Exception as exc:
        raise SystemExit(
            "OpenCV import failed. Fix the local cv2/numpy install or run with --headless.\n"
            f"Reason: {exc}"
        ) from None
    finally:
        sys.path = original_path


def colorize(cv2, image):
    """Apply the example's JET palette to an 8-bit display image."""
    if image is None:
        return None
    return cv2.applyColorMap(image, cv2.COLORMAP_JET)


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


def lidar_data_handler(cv2, img):
    """Display depth and optional intensity views for one received device."""
    # Corresponds to C++ lidar_data_handler().
    # `img` is the per-device Img overwritten by the native data event.
    depth_u8 = img.depth_display
    if depth_u8 is None:
        print("[WARNING] iTFS::LiDAR There is no output depth image.")
        return None

    prefix = f"iTFS D{img.idx} {img.ip}:{img.port}"
    cv2.imshow(f"{prefix} DEPTH", colorize(cv2, depth_u8))

    intensity_u8 = img.intensity_display
    if intensity_u8 is not None:
        cv2.imshow(f"{prefix} INTENSITY", colorize(cv2, intensity_u8))

    return depth_u8


def main() -> None:
    """Run the interactive or headless OpenCV receive loop."""
    parser = argparse.ArgumentParser()
    add_itfs_args(parser)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--save-prefix", default=None)
    args = parser.parse_args()

    cv2 = None if args.headless else import_cv2_or_die()
    ilidar_module = load_ilidar()

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
        print("press s, q, ESC, or Ctrl-C to stop")

        previous_sync = time.monotonic()
        if ILIDAR_SYNC:
            ilidar.send_sync()
            print("[MESSAGE] iTFS::LiDAR cmd_sync packet was sent.")

        count = 0
        try:
            while True:
                img = wait_image_frame(events, args.timeout)
                if img is None:
                    print(f"no image frame within {args.timeout:.1f} seconds")
                    continue

                count += 1
                if ILIDAR_SYNC and time.monotonic() - previous_sync > SYNC_PACKET_PERIOD_SECONDS:
                    previous_sync = time.monotonic()
                    ilidar.send_sync()
                    print("[MESSAGE] iTFS::LiDAR cmd_sync packet was sent.")

                if cv2 is not None:
                    image_u8 = lidar_data_handler(cv2, img)
                    key = cv2.waitKey(10) & 0xFF
                    if key in (ord("s"), ord("q"), 27):
                        break
                else:
                    depth = img.depth
                    image_u8 = img.depth_display
                    center = int(depth[depth.shape[0] // 2, depth.shape[1] // 2])
                    print("                             DEPTH_CENTER", center)

                if args.save_prefix and count == 1:
                    if cv2 is not None:
                        cv2.imwrite(f"{args.save_prefix}_image.png", image_u8)
                    else:
                        import numpy as np

                        np.savetxt(f"{args.save_prefix}_depth.csv", img.depth, fmt="%d", delimiter=",")
        except KeyboardInterrupt:
            print("stopping")
        finally:
            events.close()
            if cv2 is not None:
                cv2.destroyAllWindows()
            print("[MESSAGE] iTFS::LiDAR has been deleted.")


if __name__ == "__main__":
    main()
