#!/bin/bash
# Regenerate the Verilog netlists in sim/rtl_gen/ from the VHDL CPU cores.
#
# Verilator cannot compile VHDL, so the three VHDL cores (V30, T80 Z80,
# mc8051 MCU) are converted to Verilog netlists with GHDL's yosys plugin.
# The generated netlists are checked in; this script is only needed when the
# VHDL sources change.
#
# Requires oss-cad-suite (yosys + ghdl + matched plugin):
#   https://github.com/YosysHQ/oss-cad-suite-build/releases
#   source ~/oss-cad-suite/environment && ./gen_vhdl_netlists.sh
set -euo pipefail

cd "$(dirname "$0")/.."
RTL=../rtl
OUT=rtl_gen
mkdir -p "$OUT"

if ! yosys -m ghdl -p "" >/dev/null 2>&1; then
    echo "error: yosys ghdl plugin not available - source the oss-cad-suite environment first" >&2
    exit 1
fi

echo "=== V30 (top: cpu) ==="
yosys -q -m ghdl -p "
  ghdl --std=08 \
    $RTL/v30/registerpackage.vhd \
    $RTL/v30/bus_savestates.vhd \
    $RTL/v30/reg_savestates.vhd \
    $RTL/v30/export.vhd \
    $RTL/v30/divider.vhd \
    $RTL/v30/cpu.vhd \
    -e cpu;
  write_verilog -norename $OUT/v30_cpu.v"

echo "=== T80 (top: T80s) ==="
yosys -q -m ghdl -p "
  ghdl --std=93c -fsynopsys -fexplicit \
    $RTL/T80/T80_Pack.vhd \
    $RTL/T80/T80_MCode.vhd \
    $RTL/T80/T80_ALU.vhd \
    $RTL/T80/T80_Reg.vhd \
    $RTL/T80/T80.vhd \
    $RTL/T80/T80s.vhd \
    -e T80s;
  write_verilog -norename $OUT/t80s.v"

echo "=== mc8051 (top: mc8051_core) ==="
# Analysis order: package, then leaf entity/arch/cfg triplets bottom-up
# (same order as rtl/8051/mc8051.qip), then the core.
yosys -q -m ghdl -p "
  ghdl --std=93c -fsynopsys -fexplicit \
    $RTL/8051/mc8051_p.vhd \
    $RTL/8051/control_fsm_.vhd $RTL/8051/control_fsm_rtl.vhd $RTL/8051/control_fsm_rtl_cfg.vhd \
    $RTL/8051/control_mem_.vhd $RTL/8051/control_mem_rtl.vhd $RTL/8051/control_mem_rtl_cfg.vhd \
    $RTL/8051/alumux_.vhd $RTL/8051/alumux_rtl.vhd $RTL/8051/alumux_rtl_cfg.vhd \
    $RTL/8051/alucore_.vhd $RTL/8051/alucore_rtl.vhd $RTL/8051/alucore_rtl_cfg.vhd \
    $RTL/8051/addsub_cy_.vhd $RTL/8051/addsub_cy_rtl.vhd $RTL/8051/addsub_cy_rtl_cfg.vhd \
    $RTL/8051/addsub_ovcy_.vhd $RTL/8051/addsub_ovcy_rtl.vhd $RTL/8051/addsub_ovcy_rtl_cfg.vhd \
    $RTL/8051/addsub_core_.vhd $RTL/8051/addsub_core_struc.vhd $RTL/8051/addsub_core_struc_cfg.vhd \
    $RTL/8051/comb_divider_.vhd $RTL/8051/comb_divider_rtl.vhd $RTL/8051/comb_divider_rtl_cfg.vhd \
    $RTL/8051/comb_mltplr_.vhd $RTL/8051/comb_mltplr_rtl.vhd $RTL/8051/comb_mltplr_rtl_cfg.vhd \
    $RTL/8051/dcml_adjust_.vhd $RTL/8051/dcml_adjust_rtl.vhd $RTL/8051/dcml_adjust_rtl_cfg.vhd \
    $RTL/8051/mc8051_siu_.vhd $RTL/8051/mc8051_siu_rtl.vhd $RTL/8051/mc8051_siu_rtl_cfg.vhd \
    $RTL/8051/mc8051_tmrctr_.vhd $RTL/8051/mc8051_tmrctr_rtl.vhd $RTL/8051/mc8051_tmrctr_rtl_cfg.vhd \
    $RTL/8051/mc8051_alu_.vhd $RTL/8051/mc8051_alu_struc.vhd $RTL/8051/mc8051_alu_struc_cfg.vhd \
    $RTL/8051/mc8051_control_.vhd $RTL/8051/mc8051_control_struc.vhd $RTL/8051/mc8051_control_struc_cfg.vhd \
    $RTL/8051/mc8051_core_.vhd $RTL/8051/mc8051_core_struc.vhd $RTL/8051/mc8051_core_struc_cfg.vhd \
    -e mc8051_core;
  write_verilog -norename $OUT/mc8051_core.v"

# GHDL lowercases identifiers; the RTL instantiates `T80s`. Restore the
# expected module name (case-only rename, safe with sed on the module header).
if grep -q '^module t80s(' "$OUT/t80s.v"; then
    sed -i '' 's/^module t80s(/module T80s(/' "$OUT/t80s.v"
fi

# sound.sv leaves WAIT_n and OUT0 unconnected, relying on the VHDL port
# defaults ('1' and '0'). The Verilog netlist loses those defaults and
# Verilator would tie the inputs to 0, permanently stalling the Z80 on
# WAIT_n. Convert them to internal constants.
sed -i '' \
    -e 's/^module T80s(\(.*\)WAIT_n, \(.*\))/module T80s(\1\2)/' \
    -e '/^  input WAIT_n;$/d' \
    -e 's/^  wire WAIT_n;$/  wire WAIT_n = 1'"'"'b1;/' \
    -e 's/^module T80s(\(.*\)OUT0, \(.*\))/module T80s(\1\2)/' \
    -e '/^  input OUT0;$/d' \
    -e 's/^  wire OUT0;$/  wire OUT0 = 1'"'"'b0;/' \
    "$OUT/t80s.v"
if grep -qE '^  (input|output).*(WAIT_n|OUT0)' "$OUT/t80s.v"; then
    echo "error: WAIT_n/OUT0 tie-off patch failed" >&2
    exit 1
fi

echo "=== checks ==="
grep -h '^module ' "$OUT"/*.v | sort | uniq -d > /tmp/dup_modules.txt || true
if [ -s /tmp/dup_modules.txt ]; then
    echo "error: duplicate module names across netlists:" >&2
    cat /tmp/dup_modules.txt >&2
    exit 1
fi
for top in cpu T80s mc8051_core; do
    if ! grep -qh "^module $top(" "$OUT"/*.v; then
        echo "error: expected top module '$top' not found" >&2
        exit 1
    fi
done
echo "OK: $(grep -hc '^module ' "$OUT"/*.v | paste -sd+ - | bc) modules generated"
wc -l "$OUT"/*.v
