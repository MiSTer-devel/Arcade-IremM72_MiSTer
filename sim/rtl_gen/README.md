# Generated Verilog netlists

These files are generated from the VHDL CPU cores by `../scripts/gen_vhdl_netlists.sh`
(GHDL yosys plugin from oss-cad-suite). They exist because Verilator cannot compile
VHDL. They are used **only by the simulator** — Quartus synthesis still uses the
original VHDL sources.

| File | Source | Top module |
|---|---|---|
| `v30_cpu.v` | `rtl/v30/*.vhd` | `cpu` (NEC V30 main CPU) |
| `t80s.v` | `rtl/T80/*.vhd` | `T80s` (Z80 sound CPU) |
| `mc8051_core.v` | `rtl/8051/*.vhd` | `mc8051_core` (i8751 MCU) |

Regenerate after changing any of the VHDL sources:

```sh
source ~/oss-cad-suite/environment   # https://github.com/YosysHQ/oss-cad-suite-build
make netlists                        # or scripts/gen_vhdl_netlists.sh
```

Notes:
- `t80s.v` is post-processed: `WAIT_n`/`OUT0` inputs (unconnected in `sound.sv`,
  relying on VHDL port defaults) are converted to internal constants.
- Netlists are checked in so the sim builds with only verilator + SDL2.
