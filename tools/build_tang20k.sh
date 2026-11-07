#!/usr/bin/env bash
# ============================================================================
# Build / test / program bmp581_tang20k_top on the Sipeed Tang Nano 20K
# with the open-source Gowin flow (Yosys + nextpnr-himbaechel + apicula).
#
# Run from the repo root:
#   tools/build_tang20k.sh sim      simulate the top level (Icarus Verilog)
#   tools/build_tang20k.sh build    synthesize, place and route, make the bitstream
#   tools/build_tang20k.sh load     load it into the FPGA (gone after power-off)
#   tools/build_tang20k.sh flash    write it to flash (survives power-off)
#
# Works with either install:
#   OSS CAD Suite       -> yosys, nextpnr-himbaechel, gowin_pack, openFPGALoader
#   pip (yowasp)        -> yowasp-yosys, yowasp-nextpnr-himbaechel-gowin, gowin_pack
# ============================================================================
set -euo pipefail

RTL="src/rtl/bringup/bmp581_tang20k_top.sv src/rtl/bmp581_controller.sv src/rtl/common/spi_master.sv"
TB="src/tb/bringup/tb_bmp581_tang20k_top.sv src/tb/bmp581_model.sv"
CST="src/constraints/bmp581_tang20k.cst"
TOP="bmp581_tang20k_top"
OUT="build/tang20k"
DEVICE="GW2AR-LV18QN88C8/I7"
FAMILY="GW2A-18C"

pick() { for c in "$@"; do command -v "$c" >/dev/null 2>&1 && { echo "$c"; return; }; done; echo ""; }
YOSYS=$(pick yosys yowasp-yosys)
NEXTPNR=$(pick nextpnr-himbaechel yowasp-nextpnr-himbaechel-gowin)
PACK=$(pick gowin_pack)
LOADER=$(pick openFPGALoader openfpgaloader)

need() { [ -n "$1" ] || { echo "error: $2 not found on PATH ($3)"; exit 1; }; }

case "${1:-build}" in
  sim)
    need "$(pick iverilog)" iverilog "conda activate rtl, or source the OSS CAD Suite environment"
    mkdir -p "$OUT"
    iverilog -g2012 -o "$OUT/sim.vvp" $TB $RTL
    vvp -n "$OUT/sim.vvp"
    ;;
  build)
    need "$YOSYS" yosys "install OSS CAD Suite or: pip install yowasp-yosys"
    need "$NEXTPNR" nextpnr-himbaechel "install OSS CAD Suite or: pip install yowasp-nextpnr-himbaechel-gowin"
    need "$PACK" gowin_pack "install OSS CAD Suite or: pip install apycula"
    mkdir -p "$OUT"
    echo "== synthesis ($YOSYS)"
    "$YOSYS" -q -l "$OUT/yosys.log" -p "read_verilog -sv $RTL; synth_gowin -top $TOP -json $OUT/synth.json"
    echo "== place and route ($NEXTPNR)"
    "$NEXTPNR" --json "$OUT/synth.json" --write "$OUT/pnr.json" --device "$DEVICE" \
               --vopt family="$FAMILY" --vopt cst="$CST" --freq 27 -l "$OUT/nextpnr.log" >/dev/null 2>&1 \
      || { echo "place and route failed, see $OUT/nextpnr.log"; tail -20 "$OUT/nextpnr.log"; exit 1; }
    grep -E "Max frequency" "$OUT/nextpnr.log" | tail -1
    echo "== bitstream ($PACK)"
    "$PACK" -d "$FAMILY" -o "$OUT/$TOP.fs" "$OUT/pnr.json"
    echo "done: $OUT/$TOP.fs"
    ;;
  load)
    need "$LOADER" openFPGALoader "install OSS CAD Suite or: brew install openfpgaloader"
    "$LOADER" -b tangnano20k "$OUT/$TOP.fs"
    ;;
  flash)
    need "$LOADER" openFPGALoader "install OSS CAD Suite or: brew install openfpgaloader"
    "$LOADER" -b tangnano20k -f "$OUT/$TOP.fs"
    ;;
  *) echo "usage: $0 {sim|build|load|flash}"; exit 1 ;;
esac
