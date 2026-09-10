#!/usr/bin/env python3

import argparse
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


def status_packet_handler(status):
    """Handle one copied status_v3 snapshot from the native callback."""
    # Corresponds to C++ status_packet_handler().
    # Use this for status_v3 snapshots only; sensor configuration belongs to ilidar_tool_lite.py.
    print_lite_status(status)


def info_packet_handler(info):
    """Handle the first valid info_v3 snapshot reported for a device."""
    # Corresponds to C++ info_packet_handler().
    # Use this for info_v3 snapshots only; sensor configuration belongs to ilidar_tool_lite.py.
    print_lite_info(info)


def lidar_data_handler(img):
    """Report the newest reusable image copy selected by the receive loop."""
    # Corresponds to C++ lidar_data_handler().
    # `img` is the per-device LiteImgCpy overwritten by the native data event.
    print(
        "[MESSAGE] iTFS::LITE image   | "
        f"D# {img.idx}  M {img.mode}  F# {img.frame:3d}  "
        f"{img.ip}:{img.port}"
    )


def main() -> None:
    """Build/load the binding and receive frames until the user interrupts."""
    parser = argparse.ArgumentParser()
    add_lite_args(parser)
    args = parser.parse_args()
    ilidar_lite = load_ilidar_lite()

    with open_lite(ilidar_lite, args, queue_size=16) as lite:
        events = LiteEventReceiver(
            ilidar_lite,
            lite,
            status_handler=status_packet_handler,
            info_handler=info_packet_handler,
        )
        while not lite.ready:
            time.sleep(0.1)

        print("[MESSAGE] iTFS::LITE is ready.")
        print("press Ctrl-C to stop")

        try:
            while True:
                img = wait_image_frame(events, args.timeout)
                if img is None:
                    print(f"no image frame within {args.timeout:.1f} seconds")
                    continue

                lidar_data_handler(img)
        except KeyboardInterrupt:
            print("stopping")
        finally:
            events.close()
            print("[MESSAGE] iTFS::LITE has been deleted.")


if __name__ == "__main__":
    main()
