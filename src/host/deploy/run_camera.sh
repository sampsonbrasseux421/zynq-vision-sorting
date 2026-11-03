#!/bin/sh
# run_camera.sh -- start the host application with the environment it expects.

export KMP_DUPLICATE_LIB_OK=TRUE
cd "$(dirname "$0")/.." || exit 1
exec python3 app/pc_yolo_send.py
