#!/usr/bin/env python3

import argparse
import contextlib
import io
import site
import sys
import time

from ilidar_lite_wrapper import (
    LiteEventReceiver,
    add_lite_args,
    load_ilidar_lite,
    open_lite,
    print_lite_info,
    print_lite_status,
    wait_image_frame,
)


__version__ = "2.0.1"

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
    """Print one status_v3 snapshot produced by the native callback."""
    # Corresponds to C++ status_packet_handler().
    # Use this for status_v3 snapshots only; sensor configuration belongs to ilidar_tool_lite.py.
    print_lite_status(status)


def info_packet_handler(info):
    """Print the first valid info_v3 snapshot for a device."""
    # Corresponds to C++ info_packet_handler().
    # Use this for info_v3 snapshots only; sensor configuration belongs to ilidar_tool_lite.py.
    print_lite_info(info)


def lidar_data_handler(cv2, img):
    """Display every enabled LITE image class for one received device."""
    # Corresponds to C++ lidar_data_handler().
    # `img` is the per-device LiteImgCpy overwritten by the native data event.
    depth_u8 = img.depth_display
    if depth_u8 is None:
        print("[WARNING] iTFS::LITE There is no output depth image.")
        return None

    prefix = f"iTFS-LITE D{img.idx} {img.ip}:{img.port}"
    cv2.imshow(f"{prefix} DEPTH", colorize(cv2, depth_u8))

    amplitude_u8 = img.amplitude_display
    if amplitude_u8 is not None:
        cv2.imshow(f"{prefix} AMPLITUDE", colorize(cv2, amplitude_u8))

    intensity_u8 = img.intensity_display
    if intensity_u8 is not None:
        cv2.imshow(f"{prefix} INTENSITY", colorize(cv2, intensity_u8))

    confidence_u8 = img.confidence_display
    if confidence_u8 is not None:
        confidence_image = (
            cv2.cvtColor(confidence_u8, cv2.COLOR_GRAY2BGR)
            if img.confidence_mask1
            else colorize(cv2, confidence_u8)
        )
        cv2.imshow(f"{prefix} CONFIDENCE", confidence_image)

    return depth_u8


def main() -> None:
    """Run the interactive or headless OpenCV receive loop."""
    parser = argparse.ArgumentParser()
    add_lite_args(parser)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--save-prefix", default=None)
    args = parser.parse_args()

    cv2 = None if args.headless else import_cv2_or_die()
    ilidar_lite = load_ilidar_lite()

    with open_lite(ilidar_lite, args, queue_size=4) as lite:
        events = LiteEventReceiver(
            ilidar_lite,
            lite,
            status_handler=status_packet_handler,
            info_handler=info_packet_handler,
        )
        while not lite.ready:
            time.sleep(0.1)

        print("[MESSAGE] iTFS::LITE is ready.")
        print("press q, ESC, or Ctrl-C to stop")

        count = 0
        try:
            while True:
                img = wait_image_frame(events, args.timeout)
                if img is None or img.depth is None:
                    print(f"no depth frame within {args.timeout:.1f} seconds")
                    continue

                count += 1
                depth = img.depth

                if cv2 is not None:
                    depth_u8 = lidar_data_handler(cv2, img)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord("q"), 27):
                        break
                else:
                    depth_u8 = img.depth_display
                    center = int(depth[depth.shape[0] // 2, depth.shape[1] // 2])
                    print("                             DEPTH_CENTER", center)

                if args.save_prefix and count == 1:
                    if cv2 is not None:
                        cv2.imwrite(f"{args.save_prefix}_depth.png", depth_u8)
                    else:
                        import numpy as np

                        np.savetxt(f"{args.save_prefix}_depth.csv", depth, fmt="%d", delimiter=",")
        except KeyboardInterrupt:
            print("stopping")
        finally:
            events.close()
            if cv2 is not None:
                cv2.destroyAllWindows()
            print("[MESSAGE] iTFS::LITE has been deleted.")


if __name__ == "__main__":
    main()
