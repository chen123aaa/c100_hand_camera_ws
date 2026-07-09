#!/usr/bin/env python3
"""List video devices and identify likely USB camera capture nodes."""

from __future__ import annotations

import argparse
import glob
from pathlib import Path
import subprocess
from typing import Iterable

import cv2


def run_command(command: list[str]) -> str:
    try:
        completed = subprocess.run(
            command,
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )
    except FileNotFoundError:
        return ''
    return completed.stdout.strip()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Detect Linux video devices for C100 bring-up')
    parser.add_argument(
        '--probe',
        action='store_true',
        help='Try opening each capture device with OpenCV and reading one frame.',
    )
    return parser.parse_args()


def iter_video_nodes() -> Iterable[Path]:
    for path in sorted(glob.glob('/dev/video*')):
        yield Path(path)


def get_v4l2_info(device: Path) -> tuple[str, str, bool]:
    output = run_command(['v4l2-ctl', '-d', str(device), '--info'])
    name = 'unknown'
    bus = 'unknown'
    is_capture = False

    for raw_line in output.splitlines():
        line = raw_line.strip()
        if line.startswith('Card type'):
            name = line.split(':', 1)[1].strip()
        elif line.startswith('Bus info'):
            bus = line.split(':', 1)[1].strip()
        elif 'Video Capture' in line or 'Video Capture Multiplanar' in line:
            is_capture = True

    return name, bus, is_capture


def get_format_summary(device: Path) -> str:
    output = run_command(['v4l2-ctl', '-d', str(device), '--list-formats-ext'])
    if not output:
        return 'formats unavailable'

    interesting: list[str] = []
    current_format = ''
    for raw_line in output.splitlines():
        line = raw_line.strip()
        if line.startswith('[') and "'" in line:
            current_format = line
        elif line.startswith('Size: Discrete'):
            size = line.replace('Size: Discrete', '').strip()
            if current_format:
                interesting.append(f'{current_format} {size}')

    if not interesting:
        return 'formats found, no discrete sizes parsed'
    return '; '.join(interesting[:12])


def probe_opencv(device: Path) -> str:
    cap = cv2.VideoCapture(str(device), cv2.CAP_V4L2)
    if not cap.isOpened():
        return 'OpenCV: cannot open'

    ok, frame = cap.read()
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    cap.release()

    if not ok or frame is None:
        return f'OpenCV: opened, no frame, reported {width}x{height}@{fps:.1f}'
    return f'OpenCV: frame ok, frame={frame.shape[1]}x{frame.shape[0]}, reported {width}x{height}@{fps:.1f}'


def main() -> int:
    args = parse_args()
    devices = list(iter_video_nodes())
    if not devices:
        print('No /dev/video* nodes found.')
        return 1

    print('Video devices:')
    for device in devices:
        name, bus, is_capture = get_v4l2_info(device)
        marker = 'candidate' if is_capture else 'skip'
        print(f'\n{device} [{marker}]')
        print(f'  name: {name}')
        print(f'  bus:  {bus}')
        print(f'  formats: {get_format_summary(device)}')
        if args.probe and is_capture:
            print(f'  {probe_opencv(device)}')

    print('\nUse the selected hand-camera path explicitly, for example:')
    print('  ros2 launch c100_hand_camera c100_hand_camera.launch.py device:=/dev/video2')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
