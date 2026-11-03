#!/bin/sh
# run_all.sh -- compile and run every testbench, writing logs to results/
#
# Requires Icarus Verilog 11+  (sudo apt install iverilog)
# Usage:  sh run_all.sh          from the sim/ directory

set -e
cd "$(dirname "$0")"

RTL=../src/rtl
OUT=results
mkdir -p "$OUT"

run () {
    name=$1
    shift
    printf '%-28s' "$name"
    if ! iverilog -g2012 -o "$name.out" "$@" > "$OUT/$name.compile.txt" 2>&1; then
        echo "COMPILE FAILED  (see $OUT/$name.compile.txt)"
        return 0
    fi
    vvp "$name.out" > "$OUT/$name.log" 2>&1 || true
    rm -f "$name.out"
    if grep -q '\[FAIL\]\|TIMEOUT' "$OUT/$name.log"; then
        echo "FAIL"
    else
        echo "PASS"
    fi
}

run tb_zdt_frame_tx        tb_zdt_frame_tx.v        "$RTL/zdt_frame_tx.v"
run tb_zdt_tx_engine       tb_zdt_tx_engine.v       "$RTL/zdt_tx_engine.v" "$RTL/zdt_frame_tx.v"
run tb_zdt_engine3         tb_zdt_engine3.v         "$RTL/zdt_tx_engine.v" "$RTL/zdt_frame_tx.v"
run tb_zdt_sync            tb_zdt_sync.v            "$RTL/zdt_tx_engine.v" "$RTL/zdt_frame_tx.v"
run tb_zdt_axil            tb_zdt_axil.v            "$RTL/zdt_axil.v" "$RTL/zdt_tx_engine.v" "$RTL/zdt_frame_tx.v"
run tb_color_blob_detector tb_color_blob_detector.v "$RTL/color_blob_detector.v"
run tb_roi_check           tb_roi_check.v           "$RTL/color_blob_detector_roi.v" "$RTL/color_blob_detector.v"

echo
echo "Logs written to $OUT/"
