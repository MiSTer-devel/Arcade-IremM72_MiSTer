# Savestates

Savestate support modeled on the Arcade-IGSPGM_MiSTer architecture: each
hardware block exposes its state as a numbered "section" on an internal
savestate bus (`ssbus_if`, see `rtl/savestates.sv`); `memory_stream.sv`
gathers the sections into a chunked stream in a DDR window (4 slots x 4MB at
`SS_DDR_BASE` = 0x3E000000, `rtl/m72_pkg.sv`), which the host then persists
(MiSTer OSD slots on FPGA, `.m72state` files in the simulator).

## How a save works (rtl/m72.v controller FSM)

1. `ss_do_save` -> the controller requests a pause. In addition to the normal
   pause conditions (bus idle, no SDRAM request in flight) a savestate pause
   requires the V30 BIU to be bus-quiet (`SS_BUS_QUIET`) and the sprite DMA
   idle (`TNSL`), so every RAM port the streamer hijacks is inert.
2. After a short settle, `save_state_data` enumerates all sections (query with
   timeout — sections that don't answer are skipped) and streams them into the
   DDR slot.
3. On restore the stream is scattered back; the V30's full architectural and
   micro state is written through its 202-entry SS register file
   (`rtl/v30/v30_bus.sv` slave), the Z80 through the generated
   `tv80_auto_ss.sv` (`auto_save_adaptor2`), and everything else through
   per-module slaves. A short drain satisfies the V30's write-staging
   contract, the bus adapter is reset to idle (`ss_restore_done`), and the
   core resumes when the beam reaches the saved `paused_v/paused_h` position.

## Section map

See `SSIDX_*` in `rtl/m72_pkg.sv`: work RAM, V30 regfile, Z80, sound RAM +
latches, sprite buffer/table/DMA state, both tilemap layers (VRAM + scroll
regs), palettes, CRTC counters, interrupt controller, and a build-version
stamp that round-trips through the file.

## Simulator

- `state save` / `state load` protocol methods (and the GUI state window)
  drive the same core handshake and write `states/<game>/NNN.m72state`.
- Debug: `signal.read` names `ss_state_out`, `ss_pause`, `ss_paused`,
  `ss_read`, `ss_write`, `ss_v30_quiet`, `ss_v30_err`, `ss_stream_*`.
- Env knobs: `M72_STATE_TIMEOUT_TICKS`, `M72_STATE_PROGRESS_TICKS`,
  `M72_STATE_SECTION_TRACE=1` (logs each section during save/restore).

## FPGA (Arcade-IremM72.sv)

OSD: savestate slot / autoincrement options, `R[43]` save / `R[44]` restore,
hotkeys Alt-F1 (save) and F1 (restore) via `savestate_ui`. The DDR pins are
shared between the savestate streamer (priority, via `acquire`) and the
screen-rotation framebuffer through `ddr_mux`.

## Known gaps / scope

- **Only R-Type is validated** (memory map 0, no MCU/samples). State the
  other games need — MCU internal/external RAM, the CPU/MCU mailbox, the
  sample player, M84 specifics — is intentionally not saved yet; other games
  will not restore correctly.
- **jt51 (YM2151) state is not saved.** After a restore, music/SFX are wrong
  or silent until the sound driver reprograms the chip; if the driver relies
  on the YM timer IRQ for its tick, sound may stay silent until then.
  Future work: generate a `jt51_auto_ss` with `util/state_module.py`.
- **FPGA hardware untested.** The OSD wiring is in place but has not been
  validated on a DE10-Nano yet.
- **`M72_DEBUG` builds have no savestates** — the DDR pins belong to
  `ddr_debug` (96MHz domain) there.
- Audio filter (IIR) state, sprite line buffers and the pause-replay scroll
  table are transient and intentionally not saved (they regenerate within a
  frame; the scroll replay is gated during restore).
- Work RAM moved from SDRAM to block RAM as part of this feature (the
  hiscore path now reads the BRAM directly); CPU RAM accesses no longer
  stall the CE train, which slightly shifts instruction-level timing vs
  earlier builds.
