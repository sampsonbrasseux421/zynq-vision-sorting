#!/bin/sh
# run_gamepad.sh -- manual control with an Xbox gamepad.

export KMP_DUPLICATE_LIB_OK=TRUE
cd "$(dirname "$0")/.." || exit 1
exec python3 app/pad_arm.py
