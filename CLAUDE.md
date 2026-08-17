# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

An FPGA implementation (SystemVerilog/Verilog/VHDL) of Irem M72, M81, and M84 arcade
hardware for the **MiSTer** platform. It is a hardware core synthesized with Intel Quartus
for a Cyclone V, not a software program. Supported titles include R-Type, Ninja Spirit,
Image Fight, Hammerin' Harry, R-Type II, and others (see `Readme.md`).

## Building

The core is built with **Intel Quartus Prime** (targets Cyclone V `5CSEBA6U23I7`). There is
no CLI test/lint harness — "building" means synthesizing a `.rbf` bitstream.

- `Arcade-IremM72.qpf` / `Arcade-IremM72.qsf` — main project. Top-level entity is `sys_top`
  (from the `sys/` MiSTer framework); the core itself is the `emu` module in
  `Arcade-IremM72.sv`.
- `Arcade-IremM72-Fast.qsf` — alternate revision with faster/aggressive fitter settings.
- `files.qip` lists all RTL source files fed to synthesis. **When you add an RTL file you
  must register it in `files.qip`** (or `pll.qip` / a sub-`.qip`), or Quartus won't compile it.
- Built bitstreams are archived in `releases/`.

`sys/` is the vendored MiSTer framework (HPS I/O, video mixer, PLL reconfig, scaler) — treat
it as an external dependency; do not modify it when fixing core logic.

## Architecture

The signal flow mirrors the original PCB, and module/signal names deliberately track the
schematics (`docs/Irem_M72_schematics.pdf`, `docs/Irem_M84_schematics.pdf`) and the Nanao/KNA
custom-chip reverse engineering.

- **`Arcade-IremM72.sv` (`emu`)** — MiSTer wrapper: clocks/PLL, SDRAM arbitration, HPS I/O
  (OSD config string `CONF_STR`, DIP switches, controls), ROM download, hiscore, video output.
  Instantiates the core.
- **`rtl/m72.v` (`m72`)** — the actual arcade board. Wires together the CPU, video boards,
  sprite engine, sound, MCU, and interrupt controller. Start here to understand the core.
- **CPU**: microcode-ROM-driven NEC V30 ucore in `rtl/v30/`, imported from
  `nec_test`; `v30_bus.sv` adapts its multiplexed maximum-mode pins. `rtl/pal.sv`
  (`address_translator`) decodes the CPU address/IO space into region requests and control
  strobes — this is the memory map.
- **Video**: `rtl/kna70h015.sv` generates video timing (H/V counters, blanking, interrupts).
  `rtl/board_b_d.sv` (+ `board_b_d_layer.sv`, `board_b_d_sdram.sv`) is the tilemap/background
  "B-D board" (layers A/B + palette). `rtl/sprite.sv` is the sprite engine. `rtl/kna91h014.v`
  is the object/sprite palette; `rtl/kna6034201.v` another KNA custom. Final RGB is mixed at
  the bottom of `m72.v` (sprite over background priority, `CBLK`).
- **Sound**: `rtl/sound.sv` — Z80 (`rtl/tv80/`, Verilog tv80 core) driving YM2151 (`rtl/jt51/`, jotego's core) and
  a sample/DAC path (`rtl/sample_rom.sv`). M84 vs M72 sound differences are gated by `m84`.
- **MCU / protection**: `rtl/mcu.sv` + `rtl/mcu_emulator.sv` (8051 core in `rtl/8051/`) emulate
  the i8751 used by some games (Gallop, Daiku no Gensan). Communicates with the main CPU via
  `rtl/dualport_mailbox.sv`. `rtl/m72_pic.sv` is the UPD71059 interrupt controller.
- **Memory**: `rtl/sdram.sv` — 3-channel SDRAM controller (ch1 background, ch2 sprites, ch3
  CPU ROM fetches + ROM download). CPU work RAM is a 64Kx16 block RAM in `rtl/m72.v`
  (`work_ram`; the hiscore module reads it via port B). Region base addresses and the
  ROM-load layout live in `rtl/m72_pkg.sv` (`LOAD_REGIONS`, `region_t`, `board_cfg_t`).
  `rtl/rom.sv` (`rom_loader`) parses the downloaded ROM blob into SDRAM/BRAM regions per
  the MRA.
- **Savestates**: ssbus/DDR-streaming architecture from Arcade-IGSPGM (`rtl/savestates.sv`,
  `rtl/memory_stream.sv`, controller FSM + section map in `rtl/m72.v`/`rtl/m72_pkg.sv`).
  R-Type only for now; see `docs/savestates.md` for the design and known gaps.

### Board configuration
`board_cfg_t` (in `rtl/m72_pkg.sv`) selects behavior between M72/M81/M84 (`m84`, `memory_map`,
`main_mculatch`). It is set at ROM-load time from bytes in the MRA file, so a single bitstream
runs multiple board variants.

### MRA files
`docs/irem_m72_mra/`, `irem_m84_mra/`, etc. hold the `.mra` XML that tells MiSTer how to
assemble each game's ROM set (part order, region interleaving, `board_cfg` bytes, DIP
definitions). Adding/fixing game support usually means editing both RTL region handling and
the relevant `.mra`.

## Simulation & test ROMs (this `simulator` branch)

- **`sim/`** — a Verilator+ImGui/SDL2 simulator for the core (see `sim/README.md`).
  Build with `cd sim && make sim` (homebrew verilator + sdl2; capstone optional).
  Runs games from `roms/` zips via the loader-format MRAs in `releases/` (the MRAs under
  `docs/` are the old flat format and do NOT work with `rtl/rom.sv`). Headless JSON server
  mode (`./sim --server`) for scripted testing. The V30 and nu8051 cores are
  native SystemVerilog and are compiled directly by Verilator.
  Key sim-vs-hardware notes: `rtl/sdram.sv` channels use edge-detected req + 1-cycle rdy
  pulse (modeled in `sim/sim_sdram.h`); `ioctl_wr` must pulse one clk_sys cycle per byte.

- **`util/irem_emu`** — a custom **MAME** build (reference emulator) for the Irem hardware,
  used with `-debug` to compare behavior against the RTL.
- **`testroms/`** — homebrew test programs (C + asm) that run on the V30. The source and
  `Makefile` live on the `testrom` branch (`git show testrom:testroms/Makefile`); built
  artifacts land in `testroms/build/`. Toolchain: `ia16-elf-gcc` / `nasm` / `ia16-elf-objcopy`.
  Common targets (run from the `testroms` dir): `make` (build), `make run`, `make debug`,
  `make trace` (run under `irem_emu`), `make mister` (deploy to a MiSTer dev box),
  `make picorom` (flash a PicoROM cart). `split_rom.py`/`interleave.py` in `util/` split the
  built binary into the individual ROM chips the MRA expects.

## Conventions

- Match the surrounding RTL style: signal and module names follow the schematics and the
  original chip part numbers (`kna70h015`, `kna91h014`, `board_b_d`), and active-low signals
  are negated at the boundary (note the `~` on inputs/DIPs in `Arcade-IremM72.sv`). Preserve
  these names rather than "modernizing" them.
- `.editorconfig` governs formatting.
