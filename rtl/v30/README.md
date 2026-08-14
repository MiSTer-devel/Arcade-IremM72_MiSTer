# ucore — the ROM-driven V30 core (hardware twin of `sim/`)

## GOVERNANCE RULE (read before changing anything in this directory)

**SUPERSEDED 2026-08-04 by the user directive in `CLAUDE.md` — the
silicon-match phase.** The rule below governed U0-U5 and is kept, struck
through in effect, because every finding those stages booked was classified
under it and a reader of that ledger needs the rule it was written against.

### The rule NOW

**SILICON MATCH is the only correctness bar.** *"Matching the model is no longer
acceptable."*

1. **RTL-vs-silicon divergence is a WORK ITEM. Always.** Whether or not the
   model shares it. There is no longer a class of divergence that is closed by
   pointing at `sim/`.
2. **The C++ sim is an INSTRUMENT, not the reference.** It keeps every job it
   was good at — lockstep (`ulockstep`), attribution, the family census, telling
   a rendering bug apart from a spec bug — and loses exactly one: it does not
   decide what is correct. Silicon does.
3. **Model-shared is an ATTRIBUTION, not an acceptance.** "Both engines miss it"
   still routes the fix to `sim/` first where the mechanism is the model's, and
   the ucore still regenerates from it — the dependency direction is unchanged
   and it is still the cheap way to fix a shared mechanism once. What changed is
   that the item stays OPEN on the ucore's books until silicon matches, instead
   of being booked as inherited residue and closed.
4. **A defective rig is fixed and RE-CAPTURED.** Goldens invalidated by a rig
   defect are DISCARDED from all gate sets — archived by rename, with an entry
   in the invalidation ledger (`ucore_provenance.md` §58.3). Raw captures are
   retained; nothing gates on them.

Reproducing a known-imperfect sim behaviour is **no longer a pass condition.**

### The rule U0-U5 was built under (HISTORICAL — do not apply)

> **The correctness target at every gate is "identical to `sim/`
> clock-for-clock", including the simulator's own registered non-exactnesses
> versus silicon** (the 907-case REP w0 family, Q2's unmeasured raise, V5's
> 154/188). Concretely:
>
> 1. **RTL-vs-sim divergence is a bug in the RTL.** Fix the RTL.
> 2. **RTL-vs-silicon divergence that the sim does NOT share is a bug in the
>    RTL.** Fix the RTL.
> 3. **RTL-vs-silicon divergence that the sim DOES share is a ledger finding**,
>    and it is booked in `docs/notes/ucore_provenance.md` under the inherited
>    taxonomy. It is *never* patched locally in the RTL: the sim is the spec, so
>    the fix lands in `sim/` first, is re-gated there, and only then is
>    regenerated/re-implemented here. **No ucore landing without the sim landing
>    first.**
>
> Reproducing a known-imperfect sim behaviour is the *pass* condition, not a
> defect. Do not "improve" ucore past the model.

**What the change reclassifies, itemised**: `ucore_provenance.md` §58.2.

## Hard constraints inherited from the closed campaigns

- **SIMPLICITY** (standing user directive): this is 80's-era hardware; nothing
  on the die is wasted. A large fitted table, a many-cased rule, or a per-opcode
  special case is a signal of misunderstanding, not a deliverable.
- **"grep for one" stays true**: no per-opcode timing exceptions in the RTL.
- Standing refutations that must not be re-introduced: §24.8's 2-clock-grid slot
  reading; the `d*` straight line; `LC8` / `pf_drain`.
- The `ready_prev` flop is the ONLY wait mechanism (eval-instant spine).
- `R-STALL` is explicitly NOT implemented.

## Generated tables (do not hand-edit)

Everything in this directory that is not `README.md` is emitted by
`sw/gen_ucore_tables.py` from the in-repo dumps, and is proved byte-identical to
the reference model by **gate G0**, `python3 sw/check_ucore_tables.py`.

| file | shape | addressed by | source |
|---|---|---|---|
| `ucrom.hex` | 1028 × 29b | `{bank[8:0], row[1:0]}` | `docs/V20BITS.TXT` |
| `ucdecode.hex` | 8192 × 10b = `{valid, bank[8:0]}` | `{page[2:0], opc[7:0], rowgrp[1:0]}` | `docs/V20BITS.TXT` activation patterns |
| `pla3_tables.svh` | 3 × 256 × 14b | `{mode[1:0], opcode[7:0]}` | `docs/pla3_outputs.txt` |
| `ucrom_census.json` | — | — | the provenance census |

The `.hex` files are **`$readmemh` word lists** (one hex word per line, address 0
first) — *not* Intel HEX. They carry no comments, deliberately: the same files
are read by both Verilator and Quartus. If Quartus 17.1 needs `.mif`, convert at
build time; do not edit the emitted data.

### The micro-address is 15 bits, not 13

`upc = {page[2:0], opc[7:0], rowgrp[1:0], row[1:0]}`. The 257 activation
patterns are matched against the **low 13 bits** `{page, opc, rowgrp}` only; the
winning bank's four rows are then indexed by `row`. `sim/exec_impl.h:796` is the
authority:

```
bank = rom_.bank_of(upc.page, upc.opc, upc.rowgrp());
op   = rom_.op(bank * 4 + upc.row());
```

That is why the flattening is **two** direct-addressed tables (`ucdecode` then
`ucrom`) and not one. All 257 match patterns are still fully resolved at build
time — the RTL carries **zero** match/priority logic. See finding **F1** in
`docs/notes/ucore_provenance.md` for the resource arithmetic.

### The one ambiguous micro-address

`111.00000010.00` (`0x1C08`, the interrupt-acknowledge vector fetch, ucsim
ledger R4) is matched by two patterns and the 13 dumped bits cannot separate
them. `ucdecode.hex` carries the **native** resolution — the SECOND match, bank
120, rows `01E0..01E3` — byte-identical to `sim/ucrom.h::bank_of(emu=false)`.
The 8080-emulation alternative (bank 119, rows `01DC..01DF`) is recorded in
`ucrom_census.json` and deliberately **not** emitted: ucore has no BRKEM path,
and a second table would be dead silicon.

## Status

- **U0** — generated tables only (gate G0 green).
- **U1** — `v30u_biu.sv` (the mechanism BIU), `v30_core.sv` (the top),
  `v30u_eu.sv` (a tied-off placeholder) and `v30u_ss_pkg.sv` (94 BIU
  addresses, `SS_VERSION 0x80`).  Gate U1: `sw/ulockstep.py --suite --waits
  0,1,2,3` = **32/32 scenarios lockstep with `sim/`, every clock**.  The boot
  leg is booked as finding F3 (the reset sequence is a microcode march, not a
  BIU-only stream) and is carried to U2.  No EU: the sequencer is U2.

Select the engine with `sw/check_core.py --core ucore` / `--build --core
ucore`; `fsm` remains the default so every FSM baseline gate is re-runnable
unchanged.
