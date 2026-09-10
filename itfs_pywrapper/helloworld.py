#!/usr/bin/env python3

import argparse
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


def status_packet_handler(status):
    """Handle one copied status/status_full snapshot from the native callback."""
    # Corresponds to C++ status_packet_handler().
    # Use this for status snapshots only; configuration belongs to the C++ tool/API.
    print_itfs_status(status)


def info_packet_handler(info):
    """Handle the first valid info/info_v2 snapshot reported for a device."""
    # Corresponds to C++ info_packet_handler().
    # Use this for info/info_v2 snapshots only; configuration belongs to the C++ tool/API.
    print_itfs_info(info)


def lidar_data_handler(img):
    """Report the newest reusable image copy selected by the receive loop."""
    # Corresponds to C++ lidar_data_handler().
    # `img` is the per-device Img overwritten by the native data event.
    print(
        "[MESSAGE] iTFS::LiDAR image  | "
        f"D# {img.idx}  M {img.mode}  F# {img.frame:2d}  "
        f"{img.ip}:{img.port}"
    )


def main() -> None:
    """Build/load the binding and receive frames until the user interrupts."""
    parser = argparse.ArgumentParser()
    add_itfs_args(parser)
    args = parser.parse_args()
    ilidar_module = load_ilidar()

    with open_itfs(ilidar_module, args, queue_size=16) as ilidar:
        events = ItfsEventReceiver(
            ilidar_module,
            ilidar,
            status_handler=status_packet_handler,
            info_handler=info_packet_handler,
        )
        while not ilidar.ready:
            time.sleep(0.1)

        print("[MESSAGE] iTFS::LiDAR is ready.")
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
            print("[MESSAGE] iTFS::LiDAR has been deleted.")


if __name__ == "__main__":
    main()
