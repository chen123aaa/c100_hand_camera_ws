#!/usr/bin/env python3
"""Preview and basic capture utility for the WHEELTEC C100 USB camera."""

from __future__ import annotations

import argparse
import datetime as dt
from pathlib import Path

import cv2


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Preview WHEELTEC C100 USB camera")
    parser.add_argument(
        "-d",
        "--device",
        default="0",
        help="Video device index or path, for example 0 or /dev/video0",
    )
    parser.add_argument("--width", type=int, default=640, help="Requested frame width")
    parser.add_argument("--height", type=int, default=480, help="Requested frame height")
    parser.add_argument("--fps", type=int, default=30, help="Requested FPS")
    parser.add_argument("--mjpeg", action="store_true", help="Request MJPEG format")
    parser.add_argument(
        "--save-dir",
        default=str(Path(__file__).resolve().parents[1] / "captures"),
        help="Directory for screenshots saved with the s key",
    )
    return parser.parse_args()


def open_capture(device: str, width: int, height: int, fps: int, mjpeg: bool) -> cv2.VideoCapture:
    source: int | str
    source = int(device) if device.isdigit() else device
    cap = cv2.VideoCapture(source, cv2.CAP_V4L2)

    if mjpeg:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    return cap


def describe_capture(cap: cv2.VideoCapture) -> str:
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    fourcc_value = int(cap.get(cv2.CAP_PROP_FOURCC))
    fourcc = "".join(chr((fourcc_value >> (8 * i)) & 0xFF) for i in range(4))
    return f"{width}x{height} @ {fps:.1f} fps, fourcc={fourcc!r}"


def main() -> int:
    args = parse_args()
    save_dir = Path(args.save_dir)
    save_dir.mkdir(parents=True, exist_ok=True)

    cap = open_capture(args.device, args.width, args.height, args.fps, args.mjpeg)
    if not cap.isOpened():
        print(f"ERROR: cannot open camera device {args.device!r}")
        print("Run ./scripts/list_video_devices.sh and check the correct /dev/videoX.")
        return 1

    print(f"Opened C100 candidate on device {args.device!r}: {describe_capture(cap)}")
    print("Press s to save a frame, q or Esc to quit.")

    window_name = "WHEELTEC C100 Preview"
    frame_count = 0

    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            print("ERROR: failed to read frame")
            break

        frame_count += 1
        overlay = f"{describe_capture(cap)} | frame={frame_count}"
        cv2.putText(
            frame,
            overlay,
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        cv2.imshow(window_name, frame)

        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord("q")):
            break
        if key == ord("s"):
            stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            path = save_dir / f"c100_{stamp}.png"
            cv2.imwrite(str(path), frame)
            print(f"Saved {path}")

    cap.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
