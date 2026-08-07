# M72 Simulator

A Verilator-based simulator for the Irem M72/M84 core: run and test the core
without FPGA hardware.  Ported from the Arcade-IGSPGM simulator.

## Building

Requirements (homebrew): `verilator`, `sdl2`, and optionally `capstone` (V30
disassembly in the CPU window).

```sh
cd sim
make sim          # builds ./sim
```

The V30 ucore and nu8051 MCU are native SystemVerilog and are compiled
directly. The Z80 sound CPU is the native-Verilog tv80 (`rtl/tv80/`).

## Running

```sh
./sim                 # GUI, loads hharryu by default
./sim dbreed          # GUI, named game
./sim path/to.mra     # GUI, arbitrary (loader-format) MRA
./sim --server        # headless JSON server on stdin/stdout
```

Games are loaded from zips in `../roms` (override with `M72_ROM_DIR`).
Registered games: `dbreed`, `hharryu` (via `releases/*.mra`), `hharryb`,
`hharryb2` (hand-coded loaders), `testbed` (testroms build on the hharryu
board).  Note: `hharry` (World) is native M81 hardware which the core does not
implement; `hharryb2` is `MACHINE_NOT_WORKING` in MAME and renders imperfectly.

Only the loader-format MRAs in `releases/` work — the ones under `docs/` are
the older flat format.

GUI keys: arrows = joystick, `1` = start, `5` = coin, LShift/Z/X/C = buttons,
F12 = screenshot.

## Headless server

One JSON object per line on stdin, one response per line on stdout:

```sh
printf '%s\n' \
  '{"id":1,"method":"sim.initialize","params":{"headless":true}}' \
  '{"id":2,"method":"sim.load_game","params":{"name":"hharryu"}}' \
  '{"id":3,"method":"sim.reset","params":{"cycles":100}}' \
  '{"id":4,"method":"sim.run_frames","params":{"count":600}}' \
  '{"id":5,"method":"video.screenshot","params":{"path":"shot.png"}}' \
  | ./sim --server
```

Method families: `sim.*` (initialize/load_game/load_mra/reset/run_cycles/
run_frames/run_until/status/shutdown), `cpu.get_state`, `memory.*`
(read/write/list_regions), `signal.*` (read/list — VPI hierarchical names or
builtin aliases), `input.*` (set/clear/press/set_dipswitch/get_state),
`state.*` (save/load `.m72state` files via the core's savestate machinery,
see `../docs/savestates.md`; `nvram.*` still returns "unsupported"),
`trace.*` (FST), `audio_capture.*` (WAV),
`video.screenshot/set_flip`, `debug_link.*` (PicoROM emulation), `gui.*`
(TestROM GUI block).

`sim.run_until` takes a condition tree: `signal_equals/not_equals/less_than/
less_equal/greater_than/greater_equal` (params `signal`, `value`),
`cpu_pc_equals/cpu_pc_in_range/cpu_pc_out_of_range` (linear `(cs<<4)+ip`),
and `and`/`or`/`not` combinators.

## DebugLink (PicoROM emulation)

`debug_link.start` patches the PicoROM comms block ("PICO" magic) into the CPU
ROM at word address 0x1F800 (linear 0x3F000, matching `testroms/comms.c`) and
emulates the PicoROM byte protocol by watching CPU ROM reads.  Use
`debug_link.write` / `debug_link.read` (hex string payloads) to talk to a test
ROM built from `testroms/` (source on the `testrom` branch).  Load it with
`sim.load_game testbed` — CPU ROMs come from `../testroms/build/testbed/hharryu/`
by name, GFX/sound from the hharryu/hharry zips by CRC.

## Notes / current limitations

- Save states: fully wired for R-Type (`state.save`/`state.load`, GUI state
  window, `states/<game>/NNN.m72state`).  Other games' extra hardware (MCU,
  samples, mailbox) and the YM2151 are not saved yet — see
  `../docs/savestates.md` for the section map, debug signals
  (`ss_state_out`, `ss_stream_*`, ...) and `M72_STATE_*` env knobs.
- nvram/hiscore: not wired into the sim (hiscore module is MiSTer-side).
- The V30 CPU window shows CS:IP/opcode (+ capstone disasm); full register
  export isn't available from the netlist.
- Audio capture writes 50 kHz stereo WAV (`audio_capture.start/stop`).
- One sim tick = one CLK_32M cycle (three CLK_96M cycles); `sim.run_cycles`
  counts these ticks.
