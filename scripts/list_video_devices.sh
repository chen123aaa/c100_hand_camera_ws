#!/usr/bin/env bash
set -euo pipefail

echo "Video device nodes:"
ls -l /dev/video* 2>/dev/null || true

echo
echo "V4L2 devices:"
if command -v v4l2-ctl >/dev/null 2>&1; then
  v4l2-ctl --list-devices || true
else
  echo "v4l2-ctl is not installed. Install with: sudo apt install v4l-utils"
fi

echo
echo "Formats:"
if command -v v4l2-ctl >/dev/null 2>&1; then
  for dev in /dev/video*; do
    [ -e "$dev" ] || continue
    echo
    echo "== $dev =="
    v4l2-ctl -d "$dev" --list-formats-ext || true
  done
else
  echo "v4l2-ctl unavailable"
fi
