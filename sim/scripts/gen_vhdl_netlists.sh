#!/bin/bash
# Regenerate the Verilog netlists in sim/rtl_gen/ from the VHDL CPU cores.
#
# Verilator cannot compile VHDL, so the remaining VHDL core (mc8051 MCU) is
# converted to a Verilog netlist with GHDL's yosys plugin. The V30 CPU is now
# native SystemVerilog (rtl/v30/*.sv) and needs no netlist.
# The generated netlist is checked in; this script is only needed when the
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

echo "=== checks ==="
grep -h '^module ' "$OUT"/*.v | sort | uniq -d > /tmp/dup_modules.txt || true
if [ -s /tmp/dup_modules.txt ]; then
    echo "error: duplicate module names across netlists:" >&2
    cat /tmp/dup_modules.txt >&2
    exit 1
fi
for top in mc8051_core; do
    if ! grep -qh "^module $top(" "$OUT"/*.v; then
        echo "error: expected top module '$top' not found" >&2
        exit 1
    fi
done
echo "OK: $(grep -hc '^module ' "$OUT"/*.v | paste -sd+ - | bc) modules generated"
wc -l "$OUT"/*.v
