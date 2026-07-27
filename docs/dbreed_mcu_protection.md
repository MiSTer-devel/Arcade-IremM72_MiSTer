# dbreed (Dragon Breed) i8751 protection handshake — problem & debug notes

> **RESOLVED (2026-07-26).** dbreed now clears the protection handshake and boots
> into the game. The fix was in the **V30 core**, re-imported from nec_test
> `650dc97` (commit `c1b88e3`): the "R6c" fix in `v30_biu.sv` — a far-flush
> **direct-commit `ST_T1` was being swallowed by an unconditional idle (`ST_TI`)
> entry** in the same `always_ff` edge, corrupting instruction sequencing on the
> V30's self-modifying trampoline code in the mailbox. With that fixed the
> rendezvous "+1 chain" advances cleanly (`…FD → 0x04 → 0x0E …`) and the V30
> leaves the mailbox into normal ROM execution. (The MCU/V30 lockstep-CE change,
> commit `69700b7`, helped the handshake along but the CPU fix is what closed it.)
> The notes below are kept as a record of the mechanism and the MAME/simulator
> trace-capture recipes, which remain useful for future MCU work.
>
> Earlier framing (now historical): both the old Oregano `mc8051` and the new
> `nu8051` core failed identically, which correctly ruled out the *8051* core and
> pointed at the shared V30-side / timing path — the culprit turned out to be a
> V30 BIU sequencing bug that only this cooperative self-modifying-code handshake
> exercised.

Original problem writeup follows.

---

## 1. The problem in one paragraph

dbreed boots into a protection handshake that is a **self-modifying-code trampoline**
in the dual-port mailbox (V30 sees it at `0xB0000`, MCU at XDATA `0xC000`; the part
is an MB8421, see `docs`/datasheet). The MCU injects a small header at `0xB0000`
(`inc byte cs:[0x7FE]` = rendezvous +1, then `jmp $` spin), and **releases** the V30
by briefly pulsing the jump-displacement byte `C006` to steer the spin into a
freshly-injected POST routine (a 0x55AA RAM write/verify test at `B0068..B00D0`),
which runs and `br aw` back to `B0000` for the next transaction. The two CPUs
synchronize through a polled "+1 chain" on byte `0xC7FE` (= dpram word `0x3FF` =
V30 `0xB07FE`). Our core completes a couple of transactions (rendezvous crawls
`0xFA → 0xFB → 0xFC → 0xFD`) then **wedges**: the MCU spins at ROM `0x3CE` waiting
for the next `+1`, and the V30 is stuck inside an injected POST routine that never
returns. It is a delicate real-time coupling; MAME only works because
`mcu_data_w` calls `scheduler().synchronize()` on **every** MCU dpram write.

**Ruled out** (investigated and disproven): injected-data corruption (the MCU
writes byte-correct code — verified `addr_r=b → din_r=golden[b]`); a byte-lane
swap (only a sim read-tool artifact, see §5); MB8421 BUSY arbitration (there is
**zero** write-write contention — V30 and MCU writes are time-disjoint).

**Attempts that helped but did not fix it:** a static `ce_mcu` phase offset (all of
0/1/2/3 stall at the same point) and tying `ce_mcu` to the V30 CE train (commit
`69700b7` — advances a couple more transactions, still desyncs in a POST routine).

Key addresses / values:
- Rendezvous byte: MCU `0xC7FE` = dpram word `0x3FF` = V30 `0xB07FE`.
- MCU ROM: `0x03C1..0x03D3` rendezvous poll (spins while `[C7FE]==r1`, accepts on
  exactly `+1` at `0x3D4`); `0x033A`/`0x040C` code inject; `0x0443` one-time 4KB
  fill; release pulse writes `C006 = 0x09+IRAM[0x58]` at MCU PC `~0x361` then
  restores `C006=0xFE` at `~0x364`. Inject source table at MCU ROM `0x0622`.
- V30 ROM: `0x200..0x23C` boot cooperative RAM test (sweep pattern → spin-wait for
  the MCU complement → `inc` all bytes = the first `+1` → `ljmp 0xB000:0`).
- Injected header (golden, from MCU ROM `0x0622`): `2E FE 06 FE 07` =
  `inc byte cs:[0x7FE]`, then `EB FE` = `jmp $` at `0xB0005`.
- Boot doorbell (separate, INT0): V30 writes top word `0xB0FFE` (word `0x7FF`,
  command `0x0D`/`0x0E`) → MCU INT0 vector `0x0003→0x04EB`. Not used for the
  steady-state rendezvous (that is polled; INT0/EX0 is disabled during it).

Full mechanism + history is also in the assistant memory file
`dbreed-protection-handshake`.

---

## 2. Reference emulator: MAME (`util/irem_emu`)

`util/irem_emu` is stock **MAME 0.285** built for this hardware. Use romset
**`dbreedjm72`** (the set our `releases/Dragon Breed (Japan, M72 hardware).mra`
targets) — it runs the **real i8751** (`:mcu`, mcs51 core), *not* an HLE. Do **not**
use `dbreed`/`dbreedm72` (M81 / MCU `nodump`).

### Crash-safe headless invocation (macOS: avoid crash-reporter popups)

Always run headless, self-exiting, redirected, `</dev/null`; never leave it
interactive. Never let MAME abort (Lua errors abort it):

```sh
util/irem_emu dbreedjm72 \
  -rompath /Users/akawaka/Source/Arcade-IremM72_MiSTer/roms \
  -debug -debugscript ds.txt \
  -seconds_to_run 2 -video none -sound none -nothrottle -window -nomaximize \
  >run.log 2>&1 </dev/null
```

For Lua taps instead of `-debug`, use `-autoboot_script trace.lua` with the same
`-seconds_to_run N -video none -sound none -nothrottle -window -nomaximize`.

### Debugger trace scripts (`-debugscript`)

MCU instruction trace to the first code inject:
```
ds.txt:
  trace inject.tr,:mcu
  bpset 0x33a
  go
```

V30 execution *inside the mailbox* (the trampoline):
```
ds_v30.txt:
  bpset 0xB0000
  go
  bpclear
  trace v30mb.tr,:maincpu
  go
```

### Lua tap notes / gotchas

- MAME **aborts on any Lua error** — keep scripts error-free.
- Time is `manager.machine.time:as_double()` (a property; not `machine:time()`).
- V30 registers are NEC-named: **`PS` = segment (x86 CS)**, **`PC` = offset (IP)**,
  **`GENPC` = linear**.
- **Do not** put a V30 program-space watch across the whole `0xB0000-0xB0FFF` — the
  injected POST loops fetch millions of times and it wall-hangs. Watch the **MCU
  write side** (`mem.xdata`/dpram writes) or a narrow spin address instead.
- Mailbox writes call `scheduler().synchronize()` in the driver — this is the
  serialization that makes the handshake work in MAME.
- Disassemble the MCU ROM with:
  `/Users/akawaka/Source/mame/unidasm db_c-pr-.ic1 -arch i8051`

---

## 3. Simulator (Verilator, `sim/`)

Build: `cd sim && make sim` (binary is `sim`, **not** `M72`). Headless JSON server:
`./sim --server`, one JSON object per line on stdin, one response per line on
stdout. See `sim/README.md`.

### Minimal driver (boot dbreed, sample the rendezvous byte + V30 PC)

```sh
printf '%s\n' \
 '{"id":1,"method":"sim.initialize","params":{"headless":true}}' \
 '{"id":2,"method":"sim.load_game","params":{"name":"dbreed"}}' \
 '{"id":3,"method":"sim.reset","params":{"cycles":100}}' \
 '{"id":4,"method":"sim.run_frames","params":{"count":150}}' \
 '{"id":5,"method":"memory.read","params":{"region":"MCU_SHARED_RAM","address":2046,"size":2}}' \
 '{"id":6,"method":"cpu.get_state","params":{}}' \
 | ./sim --server
```

Useful methods: `sim.initialize/load_game/reset/run_frames/run_until`,
`cpu.get_state` (`pc` = linear `(cs<<4)+ip`; `registers[]` = AW,BW,CW,DW,SP,BP,IX,IY,
DS0,PS,SS,DS1,PC,PSW-ish), `memory.list_regions`, `memory.read {region,address,size}`
(returns `data_hex`), `trace.start/stop`, `state.save/load`.

Regions: `CPU_ROM WORK_RAM MCU_ROM MCU_RAM MCU_SHARED_RAM SPRITE_* BG_* SOUND_ROM
SAMPLE_ROM VRAM_A VRAM_B`. dbreed uses `board_cfg.memory_map = 2` (work RAM at
`0x80000-0x9FFFF`; DS/SS = `0x9000`). dbreed wedges by **~frame 40**.

### Gotchas (each cost real time — read these)

- **`state.save`/`state.load` use field `filename`, not `path`.** Pass a bare name
  (e.g. `"d40.m72state"`); the saver prepends `states/<game>/`.
- **`trace.start` uses `filename` + `depth`.** The filename buffer is `char[64]`, so
  use a short name in the sim cwd (e.g. `"hs.fst"`) and move it afterwards; a long
  absolute path silently fails to open. `depth: 99` captures the full hierarchy.
- **`MCU_SHARED_RAM` `memory.read` is byte-lane-swapped** vs the V30's fetch order
  (the sim's `Memory16b` interleaves `ram_1`/`ram_0` opposite to how the V30 reads
  words). A raw dump of the injected code looks byte-pair-swapped even though the
  bytes are correct in the RAM. When comparing to golden, account for this or trace
  the actual `mcu_shared_ram` write signals.
- **`signal.read` only exposes VPI-public nets** (mostly memories) — internal regs
  return `invalid_signal`. To poll an internal reg, add `/* verilator public_flat */`
  to its declaration and rebuild. Otherwise capture it in an FST.
- The FST time axis is not wall-clock; treat timestamps as monotonic ordering.
- SDRAM-stall catch-up makes the V30 CE bursty (`ce_steady` in `rtl/m72.v`); relevant
  because MCU/V30 relative phase drifts. `ce_mcu` is now tied to the V30 CE (commit
  `69700b7`).

### FST + pywellen extraction recipe

Trace, then read named signals with `pywellen` (`uv run --with pywellen python3 ...`):

```python
import pywellen
w = pywellen.Waveform('hs.fst')
byname = {v.full_name: v for v in w.all_vars()}          # Var.full_name is a property
def changes(name):                                        # -> list of (time, value)
    ev = []
    w.stream_changes(lambda t, sigid, val: ev.append((t, val)), [byname[name]])
    return ev
```

Key signal names (all under `TOP.sim_top.m72_inst.`):
- Mailbox: `mcu_shared_ram.int_r_rq / .int_r_ack / .int_l_rq / .int_l_ack`,
  `.cs_r .we_r .addr_r .dout_r .din_r` (MCU/right port),
  `.cs_l .we_l .addr_l .din_l` (V30/left port).
- MCU PC: `mcu_dbg_rom_addr`.   MCU INT0: `mcu.nu8051.INT0_N`.
- V30 bus address: `v30.cpu_addr`.

Typical analyses used during the investigation (rebuild as needed):
- **rendezvous history** — every write/read of byte `0x7FE` (word `0x3FF`) with value
  and originating PC/segment; shows the `+1` chain crawling then freezing.
- **coherency check** — build a shadow memory from all `we_r`/`we_l` writes and verify
  every `dout_r` read equals the last write (mailbox was fully coherent).
- **contention check** — for each MCU access, look for a V30 access to the same word
  within ±1 CLK (only V30-read-during-MCU-write occurs; zero write-write).
- **release-pulse check** — MCU writes to byte `0xB0006` (`C006`) and whether the V30
  fetches `0xB0004/0xB0006` inside the pulse window.

---

## 4. What to try next

The timing experiments confirm the axis but no single change fixes every sync point.
Candidate directions:
- Trace the injected **POST RAM-test** routine (`B0068..B00D0`) in both MAME and the
  sim and find exactly where our V30 sticks in it (it tests RAM the MCU is
  concurrently touching — a second, independent V30↔MCU sync point).
- Reproduce MAME's `synchronize()`-on-every-MCU-write behavior more faithfully in RTL
  (guarantee each MCU mailbox write is visible to the V30 for ≥1 fetch, and/or
  serialize contended shared-RAM accesses).

## 5. Files / artifacts

- RTL: `rtl/mcu.sv` (nu8051 wrapper + emulator mux), `rtl/nu8051/` (vendored core),
  `rtl/dualport_mailbox.sv` (MB8421 mailbox), `rtl/m72.v` (`ce_mcu` tie, mailbox +
  MCU instances), `rtl/sample_rom.sv`, `rtl/mcu_emulator.sv` (HLE fallback).
- Commits: `5cb0fa2` (nu8051 swap + MCU savestates), `69700b7` (MCU/V30 lockstep CE).
- MB8421 datasheet: `~/Downloads/MB8421.PDF` (interrupt + BUSY arbitration).
