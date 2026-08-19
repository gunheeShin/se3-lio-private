#!/usr/bin/env bash
# Build the python-only image. It source-builds se3_lio with OpenCV, so the context
# is the MODULE ROOT (Dockerfile COPYs cpp/ + python/ + README.md).
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
docker build -f "$HERE/Dockerfile" -t se3_lio:py "$ROOT"
