# R-Type sound commands (M72)

How the main CPU asks for sound, what the command bytes mean, and how the
simulator's **Sound Commands** window decodes them.

## The protocol

The V30 has no path to the YM2151.  All it can do is write one byte to an I/O
port; the Z80 sound board does the rest.

| write | decode | RTL | effect |
|-------|--------|-----|--------|
| I/O port `0x00` (byte, low lane) | `A[7:1] == 0x00 && ~M_IO && wr && bytesel[0]` | `snd_latch1_wr` in `rtl/pal.sv` | latched into `snd_latch1`, sets `snd_latch1_ready`, pulls the Z80 `INT` |
| I/O port `0xC0` (byte, low lane) | `A[7:1] == 0x60` (M72 only) | `snd_latch2_wr` | latched into `snd_latch2`, pulls the Z80 `NMI` — **R-Type never uses it** |

**Commands take no parameters.**  One byte in, nothing else: no length prefix,
no follow-up bytes, no handshake beyond the latch-ready flag.  Everything the
byte selects — which song, how many FM channels, what priority — lives in
tables inside the Z80's own program.

The Z80 runs with `IM 0`, and `rtl/sound.sv` synthesises the interrupt-acknowledge
byte from the two request sources:

```verilog
z80_din = {2'b11, ~snd_latch1_ready, SIRQ_N, 4'b1111};   // during M1 & IORQ
```

That is an `RST` opcode (`11 ttt 111`), so the vector taken depends on who is
asking:

| condition | byte | vector | handler |
|-----------|------|--------|---------|
| command latch full, no YM IRQ | `0xDF` | `RST 18h` | `0x0068` — read the command |
| YM2151 timer IRQ, latch empty | `0xEF` | `RST 28h` | `0x0076` — timer tick |
| both | `0xCF` | `RST 08h` | `0x0076` — timer tick |
| neither | `0xFF` | `RST 38h` | `0x0066` — `EI / RET` |

The command handler at `0x0068` is the whole main-CPU-facing protocol:

```asm
0068: D3 06        OUT (06h),A     ; clear snd_latch1_ready
006A: F5           PUSH AF
006B: DB 02        IN  A,(02h)     ; read the command byte
006D: E6 7F        AND 7Fh         ; bit 7 is discarded
006F: 32 00 F8     LD  (0F800h),A  ; hand it to the main loop
0072: F1           POP AF
0073: FB           EI
0074: ED 4D        RETI
```

So the usable command space is `0x00`–`0x7F`; **bit 7 is masked off** and
`0xFF` is the driver's own "no command pending" marker at `0xF800`.

## Where the Z80 program comes from

R-Type's ROM set has no sound ROM (`releases/R-Type (World).mra` declares only
CPU/sprite/tile regions).  The Z80's 64 KB address space is the shared RAM in
`rtl/sound.sv`, and the V30 uploads the driver into it at boot through the
`SDBEN`/`BRQ` window.

The image lives at offset `0x20000` of the interleaved CPU ROM — i.e. the first
32 KB of `rt_r-l1-b.3c` and `rt_r-h1-b.1c`, which hold **the same bytes** so the
V30 can read the program a byte at a time from either lane.  Taking every other
byte of the interleaved image from `0x20000` to `0x30000` yields the 32 KB Z80
program verbatim; it matches a dump of MAME's `:soundcpu` program space after
boot.

## Command dispatch

The main loop at `0x00E7`:

```asm
00E7: LD   HL,0F800h
00EA: LD   A,(HL)
00EB: AND  A
00EC: JP   m,02E8h        ; 0FFh -> nothing pending, go run the sequencer
00EF: LD   (HL),0FFh      ; consume
00F1: JP   z,0DC4h        ; command 00 -> reset driver, silence everything
00F4: LD   (0F8A1h),A
00F7: LD   E,A / SLA E / LD D,0
00FC: LD   HL,1000h
00FF: ADD  HL,DE          ; 16-bit entry pointer at 1000h + cmd*2
0100: LD   E,(HL) / INC HL / LD D,(HL) / EX DE,HL
0104: LD   A,(HL)         ; entry header
0109: CP   30h
010B: JP   nc,02E8h       ; header >= 30h -> unused slot, ignore
```

`0x1000` is a 128-entry pointer table.  Each entry starts with a header byte:

| header | meaning |
|--------|---------|
| `0x0n` | **play** — start `n+1` tracks unconditionally |
| `0x1n` | **conditional** — one ref byte follows; act only if that sound is currently playing, and reuse its channel allocation.  Used to *stop* a song. |
| `0x2n` | **modifier** — `[ref][vol][pitch]`; attach an extra voice to the sound `ref` that is already playing |
| `>= 0x30` | unused slot (all of them point at one shared `0xF0` byte at `0x1100`) |

For play/conditional entries, `n+1` little-endian track pointers follow; the
first byte at each track pointer is that track's priority/channel-group byte.

Driver state worth watching:

* `0xF800` — pending command (`0xFF` = idle)
* `0xFE00 + id` — per-command "is playing" slot (`0xFF` = not playing)
* `0xFF00`–`0xFFFF` — 16 channel-control blocks, 16 bytes each

## Command table

Music ids come in triplets — `N` starts a song, `N+1` stops it, `N+2` attaches a
modifier voice.  The **seen** column is the number of times each command was
observed across two full automated MAME playthroughs (attract mode, coin-up,
all eight stages, ending, game over, continue countdown).  Blank means the
command exists in the table but was never issued in those runs.

| cmd | entry | tracks (addr / priority) | seen | meaning |
|-----|-------|--------------------------|------|---------|
| `00` | special | - | 49 | Reset driver / stop all sound |
| `01` | play | 238A/01 2545/01 26BA/01 2803/01 | 3 | BGM: Stage 1 (restart) |
| `02` | conditional ref `01` | 3368/01 3368/01 3368/01 3368/01 | 1 | BGM stop: Stage 1 (restart) |
| `03` | modifier ref `01` | vol 00, pitch 70 | 1 | BGM modifier: Stage 1 (restart) |
| `04` | play | 2993/01 2A55/01 2A64/01 2BB5/01 | 2 | BGM: Stage 2 |
| `05` | conditional ref `04` | 3368/01 3368/01 3368/01 3368/01 | 2 | BGM stop: Stage 2 |
| `06` | modifier ref `04` | vol 00, pitch 70 | 2 | BGM modifier: Stage 2 |
| `07` | play | 4157/01 430D/01 448E/01 455E/01 | 2 | BGM: Stage 3 |
| `08` | conditional ref `07` | 3368/01 3368/01 3368/01 3368/01 | 6 | BGM stop: Stage 3 |
| `09` | modifier ref `07` | vol 00, pitch 70 | 4 | BGM modifier: Stage 3 |
| `0A` | play | 2C87/01 2D22/01 2DD7/01 2E7A/01 | 4 | BGM: Stage 4 |
| `0B` | conditional ref `0A` | 3368/01 3368/01 3368/01 3368/01 | 2 | BGM stop: Stage 4 |
| `0C` | modifier ref `0A` | vol 00, pitch 70 | 2 | BGM modifier: Stage 4 |
| `0D` | play | 35CE/01 379A/01 39BB/01 3BDA/01 | 2 | BGM: Stage 6 |
| `0E` | conditional ref `0D` | 3368/01 3368/01 3368/01 3368/01 | 2 | BGM stop: Stage 6 |
| `0F` | modifier ref `0D` | vol 00, pitch 70 | 2 | BGM modifier: Stage 6 |
| `10` | play | 336A/01 343A/01 34B9/01 3520/01 | 2 | BGM: Stage 5 |
| `11` | conditional ref `10` | 3368/01 3368/01 3368/01 3368/01 | 2 | BGM stop: Stage 5 |
| `12` | modifier ref `10` | vol 00, pitch 70 | 2 | BGM modifier: Stage 5 |
| `13` | play | 4711/01 4872/01 496B/01 4A42/01 | 2 | BGM: Stage 7 |
| `14` | conditional ref `13` | 3368/01 3368/01 3368/01 3368/01 | 2 | BGM stop: Stage 7 |
| `15` | modifier ref `13` | vol 00, pitch 70 | 2 | BGM modifier: Stage 7 |
| `16` | play | 4B2C/01 4B5D/01 4B73/01 4BC1/01 | 2 | BGM: Stage 8 |
| `17` | conditional ref `16` | 3368/01 3368/01 3368/01 3368/01 | 2 | BGM stop: Stage 8 |
| `18` | modifier ref `16` | vol 00, pitch 70 | 2 | BGM modifier: Stage 8 |
| `19` | play | 3DE6/01 3EE7/01 3EF8/01 3FCB/01 | 16 | BGM: Boss |
| `1A` | conditional ref `19` | 3368/01 3368/01 3368/01 3368/01 | 16 | BGM stop: Boss |
| `1B` | modifier ref `19` | vol 00, pitch 70 | 140 | BGM modifier: Boss |
| `1C` | play | 4079/01 40C9/01 40DA/01 4115/01 | 14 | BGM: Stage clear |
| `1D` | conditional ref `1C` | 3368/01 3368/01 3368/01 3368/01 | - | BGM stop: Stage clear |
| `1E` | modifier ref `1C` | vol 00, pitch 70 | - | BGM modifier: Stage clear |
| `1F` | play | 2F0C/01 2FF8/01 313D/01 327A/01 | 2 | BGM: Stage 1 (game start) |
| `20` | conditional ref `1F` | 3368/01 3368/01 3368/01 3368/01 | 1 | BGM stop: Stage 1 (game start) |
| `21` | modifier ref `1F` | vol 00, pitch 70 | 1 | BGM modifier: Stage 1 (game start) |
| `22` | play | 4BD6/01 4C1D/01 4C58/01 4C93/01 | 1 | BGM: Game over |
| `23` | conditional ref `22` | 3368/01 3368/01 3368/01 3368/01 | - | BGM stop: Game over |
| `24` | unused | - | - | - |
| `25` | play | 4F88/01 4FC9/01 4FE2/01 4FFB/01 | 6 | BGM: Attract mode |
| `26` | conditional ref `25` | 3368/01 3368/01 3368/01 3368/01 | 2 | BGM stop: Attract mode |
| `27` | modifier ref `25` | vol 00, pitch 70 | 19 | BGM modifier: Attract mode |
| `28` | play | 4CCE/01 4D94/01 4E45/01 4F07/01 | - | BGM: unidentified |
| `29` | conditional ref `28` | 3368/01 3368/01 3368/01 3368/01 | - | BGM stop: unidentified |
| `2A` | modifier ref `28` | vol 00, pitch 70 | - | BGM modifier: unidentified |
| `2B` | play | 540C/01 5499/01 5555/01 55DE/01 | 2 | BGM: Ending |
| `2C` | conditional ref `2B` | 3368/01 3368/01 3368/01 3368/01 | - | BGM stop: Ending |
| `2D` | modifier ref `2B` | vol 00, pitch 70 | - | BGM modifier: Ending |
| `2E` | unused | - | - | - |
| `2F` | unused | - | - | - |
| `30` | play | 53B6/50 | 13776 | SFX: Player shot |
| `31` | play | 1A0A/40 | 199 | SFX: Charged beam fired |
| `32` | play | 1A4F/03 | 392 | SFX: Beam charging (loop) |
| `33` | conditional ref `32` | 1A96/01 | 392 | SFX stop: sound 32 |
| `34` | play | 1A98/40 | 2092 | SFX: unidentified |
| `35` | play | 52E2/0E 535B/0F | 21 | SFX: Player death |
| `36` | play | 1AF2/40 | 15 | SFX: unidentified |
| `37` | play | 1B16/40 | 20 | SFX: unidentified |
| `38` | play | 1B4B/05 1B76/06 | 5 | SFX: unidentified |
| `39` | unused | - | - | - |
| `3A` | play | 1BA0/10 | 62 | SFX: unidentified |
| `3B` | play | 1C64/40 | 6240 | SFX: unidentified |
| `3C` | play | 1C74/40 1CB7/41 | 102 | SFX: unidentified |
| `3D` | play | 1CFD/40 1D49/41 | 31 | SFX: unidentified |
| `3E` | unused | - | - | - |
| `3F` | play | 1D49/41 | 2613 | SFX: unidentified |
| `40` | play | 2283/10 | - | SFX: unidentified |
| `41` | play | 2298/10 | - | SFX: unidentified |
| `42` | unused | - | - | - |
| `43` | unused | - | - | - |
| `44` | unused | - | - | - |
| `45` | unused | - | - | - |
| `46` | unused | - | - | - |
| `47` | unused | - | - | - |
| `48` | unused | - | - | - |
| `49` | unused | - | - | - |
| `4A` | unused | - | - | - |
| `4B` | unused | - | - | - |
| `4C` | play | 540C/01 | - | Ending music, track 1 alone |
| `4D` | play | 5499/01 | - | Ending music, track 2 alone |
| `4E` | play | 5555/01 | - | Ending music, track 3 alone |
| `4F` | play | 55DE/01 | - | Ending music, track 4 alone |
| `50` | play | 1C1A/30 | 1785 | SFX: Hit / explosion |
| `51` | play | 1D99/30 | 1256 | SFX: unidentified |
| `52` | play | 1BD5/30 | 1459 | SFX: unidentified |
| `53` | play | 1E9D/30 | 411 | SFX: unidentified |
| `54` | play | 1DF3/30 | 169 | SFX: unidentified |
| `55` | play | 1E33/10 | 624 | SFX: unidentified |
| `56` | play | 53F2/50 | 3229 | SFX: unidentified |
| `57` | play | 1E68/50 | 3565 | SFX: unidentified |
| `58` | unused | - | - | - |
| `59` | play | 1F20/20 | 616 | SFX: unidentified |
| `5A` | play | 1F5E/10 1FBE/11 | 21 | SFX: unidentified |
| `5B` | conditional ref `5A` | 2025/10 2025/10 | 6 | SFX stop: sound 5A |
| `5C` | unused | - | - | - |
| `5D` | play | 2027/20 | 1211 | SFX: unidentified |
| `5E` | play | 2073/50 | 3006 | SFX: unidentified |
| `5F` | play | 2099/30 | 708 | SFX: unidentified |
| `60` | unused | - | - | - |
| `61` | play | 20DE/10 | 41 | SFX: unidentified |
| `62` | play | 2121/50 | 120 | SFX: unidentified |
| `63` | play | 2152/02 21A7/03 | 2 | SFX: Coin inserted |
| `64` | play | 2202/20 2266/21 | 11 | SFX: unidentified |
| `65` | play | 22C4/50 | 28 | SFX: unidentified |
| `66` | play | 2312/30 | 6 | SFX: unidentified |
| `67` | play | 22EC/50 | 18 | SFX: unidentified |
| `68` | play | 237A/20 | 32 | SFX: unidentified |
| `69` | play | 573F/10 | - | SFX: unidentified |
| `6A` | play | 56BE/20 5722/21 | - | SFX: unidentified |
| `6B` | unused | - | - | - |
| `6C` | unused | - | - | - |
| `6D` | unused | - | - | - |
| `6E` | unused | - | - | - |
| `6F` | unused | - | - | - |
| `70` | play | 501C/30 5033/31 5043/32 5053/33 | 1 | SFX: Continue countdown 9 |
| `71` | play | 5063/30 507A/31 508A/32 509A/33 | 1 | SFX: Continue countdown 8 |
| `72` | play | 50AA/30 50C1/31 50D1/32 50E1/33 | 1 | SFX: Continue countdown 7 |
| `73` | play | 50F1/30 5108/31 5118/32 5128/33 | 1 | SFX: Continue countdown 6 |
| `74` | play | 5138/30 514F/31 515F/32 516F/33 | 1 | SFX: Continue countdown 5 |
| `75` | play | 517F/30 5196/31 51A6/32 51B6/33 | 1 | SFX: Continue countdown 4 |
| `76` | play | 51C6/30 51DD/31 51ED/32 51FD/33 | 1 | SFX: Continue countdown 3 |
| `77` | play | 520D/30 5224/31 5234/32 5244/33 | 1 | SFX: Continue countdown 2 |
| `78` | play | 5254/30 526B/31 527B/32 528B/33 | 1 | SFX: Continue countdown 1 |
| `79` | play | 529B/30 52B2/31 52C2/32 52D2/33 | 1 | SFX: Continue countdown 0 |
| `7A` | unused | - | - | - |
| `7B` | unused | - | - | - |
| `7C` | unused | - | - | - |
| `7D` | unused | - | - | - |
| `7E` | unused | - | - | - |
| `7F` | unused | - | - | - |

### How the labels were established

* **Stage BGM.** An automated MAME run (`-autoboot_script` Lua tap on the io
  write to `0x00`, invulnerability DIP set through a cfg override, autofire,
  ship parked mid-screen) played through all eight stages twice.  Each stage
  transition is a clean `N+2` modifier → `N+1` stop → `0x19` boss → `0x1B` →
  `0x1A` → `0x1C` stage-clear sequence, which pins every stage's id.
* **`0x1F` vs `0x01`.** `0x1F` is issued once, at the very start of a game.
  After a death, and for stage 1 on the second loop, the game issues `0x01`
  instead — so `0x1F` is the stage-1 theme with its opening and `0x01` is the
  plain restart.  Verified by turning invulnerability off mid-stage-2 and
  watching the respawn issue `0x04` (that stage's own id).
* **`0x30`–`0x33`, `0x50`.** Single-tap vs. held-fire runs: a tap gives exactly
  `0x30`; holding gives `0x30`, then `0x32` ~0.13 s later, and on release
  `0x33` (stop the charge loop) + `0x31` (charged shot).  `0x50` follows about
  half a second later, when the beam reaches something - so it is a hit or
  explosion, though which one of the several explosion-shaped ids it is has
  not been pinned down further.
* **`0x63` / `0x26`.** Issued in the same frame as the coin input, together
  with the conditional stop of the attract song.
* **`0x70`–`0x79`.** Ten commands 1.127 s apart on the CONTINUE screen,
  counting 9 down to 0.
* **`0x2B`.** Issued right after the final boss dies, on both loops.

Roughly two dozen SFX ids are still unlabelled.  They are all single- or
double-track `play` entries and the structural decode covers them; the sim
window is the tool for finishing the job — play, watch the log, name what you
hear.

## Simulator support

`sim/sim_sound_ui.cpp` adds a **Sound Commands** window:

* every latch write is captured on the rising edge of `snd_latch1_wr` /
  `snd_latch2_wr` (the taps are made visible to the harness by `public`
  directives in `sim/verilator.vlt`, no synthesis RTL is touched);
* each byte is decoded against the command table **read live out of the sound
  RAM**, so the window also proves the V30 uploaded the driver correctly;
* the header shows the pending command at `0xF800` and the set of sounds the
  driver currently has playing (`0xFE00`);
* consecutive identical commands collapse into a repeat count.

The driver is recognised by fingerprint (reset vector `F3 ED 46 C3 A1 00` plus
`table[0] == 0x1100`), not by game name, so it works however R-Type was loaded.
Other games still get raw byte logging.

Set `M72_SOUND_LOG=1` to mirror the same log to stderr for headless/server runs.

## Auditioning the unlabelled SFX

The quickest way to finish the naming is to hear each command in isolation.
With the reference MAME build (`util/irem_emu`), a Lua tap can write straight
into the sound latch and `-wavwrite` captures the result:

```lua
-- step.lua: play one command per second, starting at 0x30
GT = {}
local mac = manager.machine
local mio = mac.devices[":maincpu"].spaces["io"]
GN, GLAST = 0x30, -1
emu.register_frame_done(function()
  local t = math.floor(mac.time:as_double())
  if t > 12 and t ~= GLAST then
    GLAST = t
    if GN <= 0x7f then print(string.format("PLAY %02X at t=%d", GN, t)) end
    mio:write_u8(0x00, GN)   -- write_u8, not write_word: the latter is nil in 0.285
    GN = GN + 1
  end
end)
```

```sh
util/irem_emu rtype -rompath roms -autoboot_script step.lua -autoboot_delay 0 \
  -video none -nothrottle -skip_gameinfo -seconds_to_run 220 -wavwrite sfx.wav
```

The simulator can do the same thing natively: the **Sound Commands** window has
a hex field and a *Send* button that pokes the byte into `snd_latch1` exactly as
the SND strobe would, and the headless server exposes it as
`{"method":"sound.send","params":{"value":52}}`.  Combine it with
`audio_capture.start` / `stop` for a WAV, or with `M72_YM_LOG=1` to trace the
YM2151 register writes the driver makes in response - diffing that trace against
the same trace from `irem_emu` is the quickest way to tell a driver/timing
problem from a jt51 problem.

