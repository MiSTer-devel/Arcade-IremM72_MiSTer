//============================================================================
// Imported from nu8051 (machine-cycle-accurate Intel 8051/8052 core, SystemVerilog)
// Source: rtl/nu8051_seq.sv, commit e2c97de0fabb3b3baa1eb98a15284bc74ef92f4e
// Do not hand-edit; re-import from upstream. The M72 integration wrapper is
// rtl/mcu.sv (distilled from nu8051 rtl/m72/mcu.sv), NOT a raw re-import.
//============================================================================
//============================================================================
//
//  nu8051_seq - phase/cycle sequencer, fetch pipeline, micro-schedule
//               (core_design.md §2, §5.1, §7)
//
//  SCOPE (PLAN §4 P3 + C4.1 .. C4.8): every shape - A / B / C1 / C2 / C3 /
//               D / E / F -
//  the Phase-3 pilot (NOP, 0xA5 and the MOV register/direct/
//  immediate family), the rest of the data-movement family: MOV DPTR,#data16,
//  MOVC A,@A+DPTR / A,@A+PC, XCH / XCHD, PUSH / POP and SWAP A, (C4.2) the
//  arithmetic family: ADD / ADDC / SUBB in all four source forms, INC / DEC on
//  A / Rn / direct / @Ri and the 16-bit INC DPTR, (C4.3) MUL AB, DIV AB
//  and DA A, (C4.4) the byte logic family: ANL / ORL / XRL in all six forms,
//  CLR A, CPL A and the four rotates RL / RLC / RR / RRC, and (C4.5) the bit
//  family: SETB / CLR / CPL on C and on `bit`, MOV C,bit / MOV bit,C and
//  ANL / ORL C,±bit, and (C4.6) the branch family: SJMP / AJMP x8 / LJMP /
//  JMP @A+DPTR, JZ / JNZ / JC / JNC, JB / JNB / JBC, CJNE in all four forms
//  and DJNZ Rn / DJNZ direct, and (C4.7) the call family: ACALL x8 / LCALL /
//  RET / RETI, and (C4.8) MOVX A,@DPTR / A,@Ri and @DPTR,A / @Ri,A - the
//  external data bus.  With C4.8 the supported set is ALL 256 opcodes: the
//  PHASE4-TODO trap at the foot of this file (an unsupported opcode reaching
//  its commit edge decodes and fetches correctly but commits nothing) is now
//  unreachable and stays only as the guard it always was.  (C5.3) adds the
//  one shape that is not an opcode: the injected LCALL, shape I.
//
//  ------------------------------------------------------------------
//  Canonical tick schedule implemented here (core_design §2.2):
//
//   end S1P1  slot-B byte arrival (IR / op2 / discard) + pc+=1 if consumed;
//             slot-A address launch at pc; ALE-A rises; slot-B PSEN# ends
//   end S1P2  (IRAM read #1 address presented during S1P2)
//   end S2P1  ALE-A falls; IRAM/SFR read #1 data -> rd1_data (F-1)
//   end S2P2  IRAM read #2 data -> rd2_data (@Ri dereference); PSEN#-A asserts
//   end S3P1  timer apply tick (exported)
//   end S4P1  slot-A byte arrival (op1 / discard) + pc+=1 if consumed;
//             slot-B address launch at pc; ALE-B rises; slot-A PSEN# ends
//   end S5P1  ALE-B falls; port pin sample; LATE direct-operand read -> rd2
//   end S5P2  peripheral sampling tick; RESET sample; PSEN#-B asserts
//   end S6P1  (stack write slot 1 - Phase 4)
//   end S6P2  ARCHITECTURAL COMMIT (TQ6); mcyc/seqstate advance
//
//  ------------------------------------------------------------------
//  Read-slot assignment (core_design §5.1 + the one mechanical extension
//  §5.1 does not name, recorded as QUESTION-P3-3):
//
//    R1  (addr during S1P2, data end S2P1)  -> rd1_data
//        C1: the @Ri pointer register, or the Rn source byte
//        C2: the `direct` source byte of a 2-cycle shape -> captured into
//            rd2_data so the C1 pointer in rd1_data survives
//    R2  (addr during S2P1, data end S2P2)  -> rd2_data
//        C1: the @Ri dereference (address = the R1 byte, live on the RAM
//            output during S2P1 only - INV-IRD)
//    R3  (addr during S4P2, data end S5P1)  -> rd2_data      [extension]
//        C1 of a 2-byte/1-cycle shape whose source is `direct`: the operand
//        address only arrives at the end of S4P1, so no earlier slot can
//        exist.  MOV A,direct (E5H) is the pilot member; XCH A,direct (C5H)
//        joins it in C4.1.
//
//  C4.1 additions to the read slots:
//    R1 of C1 also serves the stack POP read (address = SP; §5.1 "stack pops:
//    R1/R2 slots of C1" - SP is a flop, so it is available immediately), and
//    R1 of C2 already served the `direct` source of the 2-cycle shapes, which
//    is what PUSH direct needs.
//
//  C4.2 (arithmetic) adds NO read slot: every arithmetic source is one of the
//  operand kinds the pilot already schedules (Rn -> R1, @Ri -> R1+R2,
//  `direct` of a shape-B form -> R3, #data -> op1).  What it does add is the
//  ALU A-side mux: the whole ADD/ADDC/SUBB family operates on ACC, but
//  INC/DEC operate on their OWN operand (INC R3 has nothing to do with ACC),
//  so the ALU `a` port takes `src_val` for those and `acc_q` otherwise.  Both
//  values are already held in the staging flops at the S6P2 commit, so this is
//  a mux, not a new schedule.  INC DPTR (A3H, shape C1, 2 cycles) is the only
//  16-bit arithmetic op and bypasses the 8-bit datapath entirely: it commits
//  `dptr_q + 1` through the dedicated DPTR write port (§5.4), touching neither
//  IRAM, the SFR bus, nor any flag.
//
//  C4.3 (MUL / DIV / DA A) adds NO read slot either - all three operate on the
//  ACC and B *registers*, which are direct taps (§5.4), never bus reads:
//
//    * DA A (shape A) is pure combinational ALU work (§4.3) reading ACC plus
//      the live PSW.CY / PSW.AC; the only new wiring is the supported-set rule.
//    * MUL AB / DIV AB are the ISA's only shape-F (1-byte / 4-cycle) opcodes.
//      Their fetch cadence needs nothing new: `d_len == 1` and `is_last` at
//      mcyc == 3 already make C1..C3 run pure discarded slots at `pc` and C4's
//      slot B the next-opcode consume, which is exactly the §2.5 shape-F row.
//      The sequencer's whole contribution is the START TICK - the edge ending
//      S2P1(C1), per §4.2 - and routing the ALU's two result bytes to the ACC
//      and B write ports at the S6P2(C4) commit.  The 8 serial iterations then
//      run at ticks S2P2(C1)..S6P1(C1), 8 of the 48 CE ticks in the budget,
//      and the result sits in the serial register file for the remaining three
//      machine cycles.  Nothing in the fetch pipeline interacts with it.
//
//  C4.4 (byte logic) adds ONE read slot and two mux arms:
//
//    * ANL/ORL/XRL A,{Rn,@Ri,direct,#data} and the unary accumulator ops
//      (CLR A, CPL A, SWAP A, RL/RLC/RR/RRC A) reuse the existing schedule
//      verbatim - accumulator destination, byte source alphabet, shapes A/B.
//    * ANL/ORL/XRL direct,A (shape B) needs nothing either: the generated
//      src/dst derivation puts the `direct` byte in BOTH d_src and d_dst
//      (its reads list is {dir, A} and `dir` outranks `A` in the generator's
//      src priority), so the R3 late slot already reads it and it arrives on
//      the ALU's B port as `src_val`.  ANL/ORL/XRL are commutative, so ACC on
//      the A port computes the same result - no A-side arm needed.
//    * ANL/ORL/XRL direct,#data (shape C3) is the one new schedule: d_src is
//      `imm` there, so nothing reads the DESTINATION.  d_rmw is exactly the
//      timing §5 "reads its own destination" marker, so the read is armed
//      structurally from {d_rmw, d_dst, d_src} and takes the R1 slot of C2 -
//      op1 (the direct address) has been stable since the end of S4P1(C1).
//      Its value lands in rd2_data and feeds the ALU's A port.
//    * RLC A / RRC A carry d_src == d_dst == OPK_C (their reads/writes lists
//      hold both A and C, and `C` outranks `A` in both generator priority
//      tables), so the accumulator write is implicit in the ALU op - the same
//      shape as MUL/DIV, whose d_dst == OPK_B yet also write A.
//
//  C4.4 also QUALIFIES the RMW port-read (timing §5): the list is by mnemonic
//  ("ANL, ORL, XRL, JBC, CPL, INC, DEC, DJNZ, MOV, CLR, SETB") and the same
//  sentence qualifies it with "every addressing form of these mnemonics whose
//  DESTINATION is a port direct address or port bit".  d_rmw carries the
//  mnemonic membership (a per-opcode fact, so it stays in the generator); the
//  destination qualification is a decode-field test and belongs here.  `ANL
//  P1,A` reads the latch, `ANL A,P1` reads the pin - directed g_ports
//  rmw-anl-52 vs nonrmw-a-55 are the two witnesses.
//
//  C4.5 (bit ops) adds NO read slot and NO flop - only the §5.3 bit-address
//  unit (pure address math over op1) and two operand-mux arms:
//
//    * The containing byte of a bit operand is read through the SAME slot the
//      byte forms of that shape use: shape B (SETB/CLR/CPL bit, MOV C,bit)
//      takes the R3 late slot, shape C2 (MOV bit,C, ANL/ORL C,±bit) takes the
//      R1 slot of C2.  Both already land in rd2_data, so `bit_val` and the
//      ALU's A port read one staging flop.  The arming test is structural -
//      "d_src or d_dst is OPK_BIT" - and no bit opcode carries an @Ri, Rn or
//      `direct` operand, so no slot can collide.
//    * Bit writes are byte read-modify-writes (§5.3): the ALU replaces one bit
//      of `a` (the byte just read) using the MASK on `b`, and the whole byte
//      commits at S6P2 through the ordinary destination path with the
//      containing byte as its address.  Nothing else in the byte survives by
//      accident - it survives because it was read back.
//    * Carry-destination forms present {7'b0, source bit} on `b` instead, so
//      one ALU arm per mnemonic serves both halves of the family (§4.1).
//      `ANL C,/bit` / `ORL C,/bit` complement that source bit and write
//      nothing back; the complement is the generated d_bitn field, because
//      their ten other decode fields are identical to the un-complemented
//      twins' (generator contract point 4 / QUESTION-P45-1).
//    * The PQ5 unimplemented-SFR case needs no code here: an unowned
//      containing byte reads NU8051_CFG_UNIMPL_RD through the core's SFR mux
//      (so every bit of it reads 1) and its write finds no owner and is
//      dropped - byte-direct and bit accesses share that mechanism exactly.
//
//  C4.6 (branches) adds NO read slot, NO flop and NO save-state map entry -
//  the whole family is combinational math over operands the existing schedule
//  already stages, applied at ONE new instant:
//
//    * the PC LOAD at the edge ending S4P1 of the final cycle (§2.5).  That
//      edge is also the slot-B launch, so `sb_addr` takes the target and the
//      next opcode is fetched from it with no dead slot.  Taken and not-taken
//      are the SAME 2-cycle instruction (instr §2): identical slot plan,
//      identical ALE/PSEN cadence, identical cycle count - the streams differ
//      only in the address of B(final) and therefore in the loaded PC.
//    * the TARGET forms come from the generated `d_tgt` (QUESTION-P46-1);
//      without it SJMP rel and AJMP addr11 are indistinguishable.  `d_tgt`
//      also fixes the operand-byte LAYOUT: `rel` is always the last operand
//      byte, so `CJNE A,#data,rel` keeps its immediate in op1 (where the
//      non-branch 3-byte forms keep a `direct`) and `DJNZ direct,rel` is not
//      mistaken for the two-direct-byte MOV direct,direct.
//    * the CONDITIONS come from `d_cond`, whose branch semantics this chunk
//      owns.  JB/JNB/JBC reuse the C4.5 bit-address unit verbatim, with the
//      containing-byte read extended from shape C2 to shape C3 (same R1 slot
//      of C2, 5 ticks of margin before the S4P1 decision).
//    * CJNE compares `d_src2` (the compared operand: A / Rn / @Ri) against
//      `d_src` (#data / direct) through the existing ALU_CJNE arm, writing C
//      only - neither operand is modified.  DJNZ is ALU_DEC, so the C4.2
//      is_incdec A-side arm already gives it its own operand, and its
//      destination write is the ordinary one.
//    * JBC is the ISA's ONLY conditional destination write: `d_alu_op` is
//      ALU_BITCLR (the generator applies the bit-op override to any row whose
//      DESTINATION is a bit, so the §5.3 read-modify-write path serves it
//      unchanged) and `dst_we_ok` suppresses the commit on the not-taken path.
//      It is on the timing §5 RMW list, so its containing-byte read takes the
//      port LATCH while JB/JNB (destination: none) take the S5P1 pin sample.
//
//  C4.7 (call / return) adds NO decode-field test on an opcode byte, NO new
//  flop and NO save-state map append - but it is the first chunk to use the
//  S6P1 write slot and the first to schedule TWO stack reads:
//
//    * TARGETS are C4.6's math verbatim: ACALL is AJMP's 2K-page form (d_tgt
//      == TGT_A11, off `pc_adv` = the address of the NEXT instruction, so an
//      ACALL at 07FEH pages off 0800H) and LCALL is LJMP's absolute pair
//      (TGT_A16).  RET/RETI carry no target operand at all (TGT_NONE): their
//      target is the popped byte pair.  The PC-load instant is unchanged -
//      the edge ending S4P1 of the final cycle, which §2.5 states as one rule
//      for "branch/call/RET/RETI/JMP @A+DPTR/interrupt vector".
//    * PUSHES (ACALL/LCALL): the return address is `pc` after all operand
//      consumes (§2.5) - precisely the value the PC load overwrites at that
//      S4P1 edge - so it is staged into op2_r/op1_r AT that edge and written
//      out low-then-high (instr §4) as IRAM[SP+1] at S6P1 and IRAM[SP+2] at
//      S6P2, with SP+2 committed at S6P2 (§2.6).  This is D1.2 F-5's pop
//      staging convention run in the other direction (QUESTION-P47-2): op1/op2
//      are dead by then in both shapes, and slot A of a call's final cycle is
//      a discarded fetch, so nothing can collide with the staging write.
//    * POPS (RET/RETI): the R1 and R2 slots of C1 read SP and SP-1 (§5.1),
//      landing PCH in op2_r and PCL in op1_r (F-5) with 9 ticks of margin
//      before the S4P1(C2) PC load; SP-2 commits at S6P2.  ACALL/LCALL carry
//      `d_src == OPK_STACK` too (their `reads` list names SP) but read no
//      stack byte, so the read arming excludes them explicitly.
//    * STACK EDGES fall out of the 8-bit SP arithmetic: FFH+1 wraps to 00H,
//      and §5.2's indirect rule then drops 8051-mode accesses above 7FH (PQ5)
//      per BYTE - a push pair straddling 7FH/80H writes one byte and drops the
//      other, and each pop half is judged by its own address.  `push-sp7f` /
//      `push-spff` and their acall/lcall/ret/reti siblings in `g_stack` are
//      the witnesses; the T2.2d `push-sp-itself` vector (PUSH SP pushes the
//      POST-increment value) is untouched - it is a C4.1 PUSH, and this chunk
//      adds arms rather than changing the ones it walks past.
//    * RETI differs from RET in exactly one generated bit, `d_reti`
//      (QUESTION-P47-1): it pulses `irq_reti` at its S6P2 commit so the irq
//      unit can clear the in-progress flip-flop of the level being returned
//      from (§6.3).  Those flip-flops are C5.3 work and do not exist yet, so
//      the pulse is architecturally inert TODAY - the wire exists because the
//      clear INSTANT is a sequencer fact, and Phase 5 should find it placed.
//
//  C4.8 (MOVX) is the ISA's last chunk and the external data bus's only
//  user.  It adds NO ALU arm, NO write port and NO decode-field test on an
//  opcode byte - what it adds is a BUS SCHEDULE (§1.3 MOVX table, timing §4)
//  the two staging registers D1.2 already reserved for it (`movx_dout` /
//  `movx_din`, map 0x010/0x011 after edit E-1, F-2), so there is no
//  save-state append here either:
//
//    * SLOT B OF C1 CARRIES THE ADDRESS, NOT A FETCH.  At the edge ending
//      S4P1(C1) the slot-B launch is replaced by the xdata address launch:
//      MEM_ADDR takes DPH:DPL (@DPTR) or {P2 LATCH, Ri} (@Ri - TQ10, the
//      byte M72 pages 0xCxxx with), ALE-B still pulses over S4P2-S5P1 to
//      latch it (timing §4), and `fp_valid` stays 0 - which is what
//      suppresses the S5P2 PSEN#-B assert, i.e. the first TQ3 window
//      (S6P1(C1)-S1P1(C2)).  The suppression is structural: no fetch in
//      flight, no PSEN#.
//    * SLOT A OF C2 DOES NOT EXIST.  The S1P1(C2) edge launches nothing, so
//      there is no ALE-A pulse (the ONE skipped ALE, timing §2) and the
//      S2P2 PSEN#-A assert reads `fp_valid == 0` - the second TQ3 window
//      (S3P1(C2)-S4P1(C2)).  Both windows therefore fall out of the same
//      one-bit fact rather than being open-coded phase tests.
//    * STROBES: RD#/WR# assert at the edge ending S6P2(C1) and deassert at
//      the edge ending S3P2(C2) - low for the 6 CE ticks S1P1..S3P2 (§1.3).
//      The read data is sampled AT the deassert edge (TQ1: "just before the
//      read strobe is deactivated" = the RD# rising edge, the latest instant
//      the text allows) into `movx_din`, held 6 ticks to the S6P2(C2) ACC
//      commit (F-2).  `movx_dout` is loaded at the edge ending S6P1(C1) so
//      MEM_DOUT is valid throughout S6P2(C1)..S4P1(C2) (§1.2 row).
//    * THE ADDRESS NEEDS NO STAGING REGISTER: MEM_ADDR is itself the output
//      register the launch edges load, and the next launch after the MOVX
//      address is the C2 slot-B fetch at the edge ending S4P1(C2) - exactly
//      where §1.2 ends the window.  So the address is stable across the
//      whole strobe window (CAD-8) by construction.  QUESTION-P48-1
//      RESOLVED (savestate_design §3.3 edit E-1): D1.2's `movx_addr` slot
//      is deleted from the map; SSA_S_MEM_ADDR is the address holder.
//    * TQ4: `bus_seen` is set in BOTH MOVX cycles (at the C1 address launch
//      and at the S1P1 edge of C2), so the P0 latch takes FFH at the S6P2
//      edge of each - "any machine cycle containing external bus activity",
//      whatever EA# is doing.  The P2 latch is never touched (timing §4).
//    * The commit is the ordinary one: `d_dst == OPK_A` with ALU_PASS over
//      `src_val`, whose OPK_XRAM_* arm is `movx_din`.  A MOVX write commits
//      nothing at all (its destination is the xdata space, which no write
//      port in this module can address), which is why the write forms need
//      no `dst` arm.
//
//  C5.3 (the interrupt system) adds the ISA's LAST shape - shape I, the
//  injected LCALL (§2.3) - and it is the first cycle role that executes no
//  opcode at all.  What it does NOT add is a decode-field test on an opcode
//  byte, a new flop beyond the two D1.2 already reserved
//  (SSA_S_TAKE_SRC/_TAKE_PRIO, 0x012/0x013), or an edit to the peripheral
//  tick gate - C5.2 restructured that into the exclusion form
//  `!prime && !rst_hold` for exactly this moment ([D-10]), so ILCALL1/2 tick
//  the timers the instant they exist:
//
//    * THE TAKE DECISION is applied at the S6P2 edge, the one instant an
//      instruction boundary exists at (§2.2's commit row).  Its four terms
//      are the manual's three blocking conditions plus the request itself:
//      `irq_req` (already resolved against the in-progress flip-flops INSIDE
//      the irq unit - rule 1), `exec && is_last` (rule 2) and `!irq_block3`
//      (rule 3).  Rule 3 needs no "one more instruction" flop: a denied poll
//      simply re-runs during the next instruction, whose final cycle allows
//      it.  Its RETI half is the generated `d_reti` bit (QUESTION-P47-1) and
//      its IE/IP half a DESTINATION test over the decode fields, so every
//      addressing form is covered by construction - `MOV IE,#d`, `POP IP`,
//      `SETB EA`, `CPL ET0` alike.
//    * ILCALL1/ILCALL2 are two ordinary machine cycles with the ordinary
//      fetch cadence, all four of their arrivals discarded at `npc` (§2.5
//      shape I).  The FIRST of those - the byte the completing instruction's
//      slot B already had in flight - is the one place a CONSUMED slot is
//      dropped without a pc increment (`ilcall_drop`), which is what makes
//      the interrupted flow re-fetch its next opcode after the RETI.
//    * THE PUSH IS THE C4.7 PUSH.  `push_pair` widens the call's two write
//      slots to ILCALL2: `{PCH,PCL}` staged into `{op2,op1}` at the S4P1
//      edge that loads the vector (P47-2's convention, and `pc_adv == pc`
//      here because slot A of ILCALL2 is a discarded fetch), written out
//      low-then-high at S6P1/S6P2, SP+2 committed with them.  Only the PC is
//      pushed - never the PSW (timing §6).
//    * ILCALL2 IS A FINAL CYCLE for blocking rule 2, which is not an
//      implementation convenience but the manual's Figure-24 sentence: a
//      higher-priority request latched at S5P2 of the first LCALL cycle "will
//      be vectored to during C5 and C6 without any instruction of the
//      lower-priority routine executing".  `tests/directed_rtl/c5_3`'s
//      `p-fig24-*` pair brackets that window at one S5P2 edge.
//
//  ------------------------------------------------------------------
//  MOVC (shape E, [D-01]): the table read occupies SLOT B of C1 - address
//  A+DPTR / A+PC launched at the edge ending S4P1(C1), byte latched at the
//  edge ending S1P1(C2) and routed into `op1` (D1.2 F-4: routing only, no
//  dedicated capture register).  That arrival is a CONSUMED fetch that must
//  NOT advance the PC (§2.5: pc counts consumed *program* bytes), which is the
//  one place the fetch pipeline's consume bit and the pc-increment enable
//  differ.  Both are derived from state that already exists (seqstate/mcyc/ir)
//  so no flop - and therefore no save-state map append - is needed.
//
//============================================================================

`timescale 1ns/1ps

module nu8051_seq #(
    parameter bit P_8052 = 1'b1,
    parameter int ROM_AW = 13
)(
    input  logic        CLK,
    input  logic        CE,
    input  logic        RESET,
    input  logic        EA_N,

    // external / internal code bus
    output logic [15:0] MEM_ADDR,
    output logic  [7:0] MEM_DOUT,
    input  logic  [7:0] MEM_DIN,
    output logic        MEM_ALE,
    output logic        MEM_PSEN_N,
    output logic        MEM_RD_N,
    output logic        MEM_WR_N,
    output logic [ROM_AW-1:0] ROM_ADDR,
    input  logic  [7:0] ROM_DATA,

    // IRAM port (§5.1)
    output logic  [7:0] iram_addr,
    input  logic  [7:0] iram_rdata,
    output logic        iram_we,
    output logic  [7:0] iram_wdata,

    // SFR bus (§5.4)
    output logic  [7:0] sfr_addr,
    output logic        sfr_rmw,
    input  logic  [7:0] sfr_rdata,
    output logic        sfr_we,
    output logic  [7:0] sfr_wdata,

    // direct taps + dedicated register writes
    input  logic  [7:0] acc_q,
    input  logic  [7:0] b_q,
    input  logic  [7:0] psw_q,
    input  logic  [7:0] sp_q,
    input  logic [15:0] dptr_q,
    // C4.8 / TQ10: the P2 LATCH is the high address byte of a MOVX @Ri (it is
    // never disturbed by bus activity, timing §4), so the seq taps it exactly
    // like ACC/PSW/SP/DPTR rather than spending an SFR read slot on it.
    input  logic  [7:0] p2_q,
    output logic        acc_we,
    output logic  [7:0] acc_wdata,
    output logic        b_we,
    output logic  [7:0] b_wdata,
    output logic        sp_we,
    output logic  [7:0] sp_wdata,
    output logic        dptr_we,
    output logic [15:0] dptr_wdata,
    output logic        flag_we_c,
    output logic        flag_c,
    output logic        flag_we_ac,
    output logic        flag_ac,
    output logic        flag_we_ov,
    output logic        flag_ov,

    // tick strobes for the peripherals (§6.1)
    output logic  [3:0] ph_o,
    output logic        tk_s2p2,         // C5.5: TF2's set instant (TQ8)
    output logic        tk_s3p1,
    output logic        tk_s5p1,
    output logic        tk_s5p2,
    output logic        tk_s6p1,
    output logic        tk_s6p2,
    output logic        bus_active,      // TQ4: external bus used this cycle
    output logic        prime_cycle,     // §2.8 boundary priming cycle

    // interrupt unit (§6.3).  The snapshot flops live in nu8051_irq (D1.2
    // F-3); the seq consumes them as WIRES for this cycle's take decision and
    // hands back the vectoring acknowledge.
    input  logic        irq_req,
    input  logic  [2:0] irq_src,
    input  logic        irq_prio,
    // C4.7: RETI's "re-arm the level just serviced" pulse (§6.3, periph §6.6),
    // asserted at the RETI instruction's S6P2 commit edge.  The CLEAR INSTANT
    // is a sequencer fact; the flip-flops it clears live in the irq unit.
    output logic        irq_reti,
    // C5.3: one pulse at the S6P2 edge of ILCALL1 [D-08] - sets the level's
    // in-progress FF and clears the source flag per the periph §6.4 table.
    output logic        irq_ack,
    output logic  [2:0] irq_ack_src,
    output logic        irq_ack_prio,

    // C5.6: PCON power control (periph §7, core_design §6.5).  The two held
    // bits come in from nu8051_sfr; `pcon_idl_clr` is the hardware clear the
    // terminating interrupt performs, and `pd_freeze` is the CE gate the top
    // level applies to every peripheral - the CE-domain reading of "the
    // oscillator is stopped" (QUESTION-P56-3).
    input  logic        pcon_idl,
    input  logic        pcon_pd,
    output logic        pcon_idl_clr,
    output logic        pd_freeze,
    output logic        cpu_idle,        // §1.9 observation: IDLE seqstate

    // observation taps (§1.9 debug signals live in the top)
    output logic  [1:0] mcyc_o,
    output logic  [2:0] seqstate_o,
    output logic [15:0] pc_o,
    output logic        rst_hold_o,
    output logic        retire_o,
    output logic [31:0] hash_o,

    // save state (savestate_design §5.1/§5.2, map 0x001-0x01A).  The staged
    // command is passed straight through to nu8051_alu (0x040-0x042), which
    // is instantiated here; its read word comes back out as `ss_rdata_alu`.
    input  logic  [9:0] ss_addr,
    input  logic [15:0] ss_wdata,
    input  logic        ss_we,
    output logic [15:0] ss_rdata,
    output logic [15:0] ss_rdata_alu

`ifdef NU8051_BACKDOOR
    ,
    input  logic        bkd_load,
    input  logic [15:0] bkd_pc
`endif
);

    import nu8051_ss_pkg::*;

`include "nu8051_defs.svh"
// The generated header carries an include guard, but Verilator shares
// `define state across the compilation unit; undef it around the include
// so every module that needs NU8051_CFG_* actually gets it.
/* verilator lint_off UNUSEDPARAM */
`ifdef NU8051_CFG_SVH
  `undef NU8051_CFG_SVH
`endif
`include "nu8051_cfg.svh"
`undef NU8051_CFG_SVH
/* verilator lint_on UNUSEDPARAM */

    // ------------------------------------------------------------------
    // decode (generated, core_design §3)
    // ------------------------------------------------------------------
    logic [1:0] d_len;
    logic [2:0] d_cycles;
    logic [2:0] d_shape;
    logic [3:0] d_class;
    logic [4:0] d_alu_op;
    logic [3:0] d_src, d_dst, d_src2;
    logic       d_rmw;
    logic [3:0] d_cond;
    logic [2:0] d_flagw;
    logic       d_bitn;
    logic [2:0] d_tgt;
    logic       d_reti;

    logic [7:0] ir_r, op1_r, op2_r;
    logic [7:0] rd1_data, rd2_data;

    // fetch-slot pipeline (§2.4): exactly one slot is ever in flight
    localparam logic [1:0] FPD_IR  = SS_FPDST_IR;
    localparam logic [1:0] FPD_OP1 = SS_FPDST_OP1;
    localparam logic [1:0] FPD_OP2 = SS_FPDST_OP2;

    logic        fp_valid, fp_ext, fp_consume;
    logic  [1:0] fp_dst;

    // C4.8 bus staging (D1.2 map SSA_S_MOVX_DOUT/SSA_S_MOVX_DIN, reserved for
    // this chunk - so no append and no version bump): the MEM_DOUT backing
    // register and the F-2 read-data hold.  There is NO movx_addr register:
    // the xdata address is launched straight into the MEM_ADDR output
    // register (SSA_S_MEM_ADDR) at the end-S4P1(C1) launch edge and is
    // stable there for the whole window, so a second copy would be redundant
    // state - and a mux of the two would put the decode of `ir` in MEM_ADDR's
    // cone, which closes a (false) combinational loop through any memory
    // model that returns data combinationally from the address.
    // QUESTION-P48-1 RESOLVED: the map slot is deleted and the seq region
    // compacted (savestate_design §3.3 edit E-1, pre-A0 so renumber legal).
    logic  [7:0] movx_dout, movx_din;

    logic  [3:0] ph;
    logic  [1:0] mcyc;
    logic  [2:0] seqstate;
    logic [15:0] pc;
    logic        ea_latched;
    logic        rst_s5p2;
    logic        bus_seen;

    // C5.3: the take decision's latched winner (D1.2 map SSA_S_TAKE_SRC /
    // SSA_S_TAKE_PRIO, 0x012/0x013 - reserved for this chunk, so no append
    // and no version bump).  Loaded at the S6P2 edge that starts ILCALL1 and
    // read by both injected cycles: `take_src` selects the vector address,
    // `take_prio` names the level whose in-progress FF the acknowledge sets.
    logic  [2:0] take_src;
    logic        take_prio;

    wire [7:0] fetch_byte  = fp_ext ? MEM_DIN : ROM_DATA;
    wire       ir_arriving = (ph == PH_S1P1) && fp_valid && fp_consume
                             && (fp_dst == FPD_IR);
    // decode input: at the opcode-arrival edge the byte is still on the fetch
    // bus, so this edge's slot-A plan must already see the NEW opcode.
    wire [7:0] ir = ir_arriving ? fetch_byte : ir_r;

`include "nu8051_decode.svh"

    // ------------------------------------------------------------------
    // sequencer roles
    // ------------------------------------------------------------------
    wire rst_hold = (seqstate == SS_SEQST_RESET_HOLD);
    wire prime    = (seqstate == SS_SEQST_PRIME);
    wire exec     = (seqstate == SS_SEQST_EXEC);
    // C5.3: the two injected-LCALL cycles (shape I, §2.3).  They are ORDINARY
    // machine cycles - the peripheral tick gate below is written as an
    // exclusion of PRIME/RESET_HOLD precisely so these states tick with no
    // edit ([D-10]) - but they execute no opcode, so every decode-driven
    // commit term stays gated on `exec`.
    wire ilcall1  = (seqstate == SS_SEQST_ILCALL1);
    wire ilcall2  = (seqstate == SS_SEQST_ILCALL2);
    // C5.6: idle (periph §7).  "The internal clock signal is gated off to the
    // CPU, but not to the interrupt, timer, and serial port functions": the
    // phase counter free-runs, the peripheral strobes below are gated on
    // PRIME/RESET_HOLD only and therefore keep ticking, and the only thing
    // this state does is poll (core_design §6.5).
    wire idle     = (seqstate == SS_SEQST_IDLE);
    assign cpu_idle = idle;

    // ------------------------------------------------------------------
    // C5.6: PCON.IDL / PCON.PD (periph §7, core_design §6.5)
    //
    // Both are LEVEL conditions on the stored bit, not edge conditions on the
    // write, because that is what the hardware is: a latch that gates a
    // clock.  The commit edge that writes PCON is the same edge the entry
    // decision is taken at (TQ6), so the value tested is the POST-commit one -
    // the sequencer's own `sfr_we`/`sfr_wdata`, falling back to the held bit
    // when this edge writes something else.  The level reading is what a
    // Phase-6 restore needs: an injected PCON.0 must halt the core at the next
    // instruction boundary, with no write anywhere.  It is also why the
    // hardware clear below rides ANY take rather than only one that terminates
    // an idle - see QUESTION-P56-2 and the `pcon_idl_clr` comment.
    //
    // "If 1s are written to PD and IDL at the same time, PD takes precedence"
    // (periph §7) - hence the `!pd` term on the idle condition, applied to the
    // post-commit value so a single `MOV PCON,#03H` powers down.
    // ------------------------------------------------------------------
    wire       pcon_wr_now  = sfr_we && (sfr_addr == SFR_PCON);
    wire       pcon_idl_eff = pcon_wr_now ? sfr_wdata[0] : pcon_idl;
    wire       pcon_pd_eff  = pcon_wr_now ? sfr_wdata[1] : pcon_pd;
    wire       idle_want    = pcon_idl_eff && !pcon_pd_eff;

    // The freeze itself: the HELD PD bit (a write cannot take effect before
    // its own edge), released by RESET_HOLD - power down's only exit is a
    // hardware reset, and the reset it starts has to reach nu8051_sfr to clear
    // the very bit that is holding the gate shut.  The top level uses this as
    // the peripheral CE gate; the clocked block below uses it as its own.
    assign pd_freeze = pcon_pd && !rst_hold;

    wire       is_last = ({1'b0, mcyc} + 3'd1 == d_cycles);
    wire [3:0] ph_next = (ph == PH_S6P2) ? 4'd0 : (ph + 4'd1);

    // Supported set (pilot + C4.1): a STRUCTURAL rule over generated decode
    // fields (class + alu op + shape + operand kinds), never an opcode list -
    // core_design §3 contract point 4.
    wire src_ok = (d_src == OPK_NONE) || (d_src == OPK_A) || (d_src == OPK_RN)
               || (d_src == OPK_RI)   || (d_src == OPK_DIR) || (d_src == OPK_IMM);
    wire dst_ok = (d_dst == OPK_NONE) || (d_dst == OPK_A) || (d_dst == OPK_RN)
               || (d_dst == OPK_RI)   || (d_dst == OPK_DIR);
    wire shape_ok = (d_shape == SH_A) || (d_shape == SH_B)
                 || (d_shape == SH_C2) || (d_shape == SH_C3);
    // byte moves: the pilot family (shapes A/B/C2/C3, byte operand alphabet)
    wire mov_byte_ok = (d_alu_op == ALU_PASS) && shape_ok && src_ok && dst_ok;
    // MOV DPTR,#data16 (shape C3, imm16 -> DPTR)
    wire mov_d16_ok  = (d_alu_op == ALU_PASS) && (d_shape == SH_C3)
                    && (d_src == OPK_IMM16) && (d_dst == OPK_DPTR);
    // XCH / XCHD: the operand is both source and destination, A is implicit
    wire xch_ok      = ((d_alu_op == ALU_XCH) || (d_alu_op == ALU_XCHD))
                    && (d_src == d_dst)
                    && ((d_src == OPK_RN) || (d_src == OPK_RI)
                        || (d_src == OPK_DIR));
    wire push_ok     = (d_alu_op == ALU_PUSH) && (d_shape == SH_C2)
                    && (d_src == OPK_DIR) && (d_dst == OPK_STACK);
    wire pop_ok      = (d_alu_op == ALU_POP) && (d_shape == SH_C2)
                    && (d_src == OPK_STACK) && (d_dst == OPK_DIR);
    // MOVC A,@A+DPTR / A,@A+PC (shape E, slot-B table read [D-01])
    wire movc_ok     = (d_shape == SH_E) && (d_dst == OPK_A)
                    && ((d_src == OPK_CODE_DPTR) || (d_src == OPK_CODE_PC));
    // SWAP A came in with C4.1 and is a member of the C4.4 unary rule below.

    // ---- C4.2 arithmetic ----------------------------------------------
    // ADD / ADDC / SUBB: accumulator destination, byte source alphabet; the
    // C input of ADDC/SUBB is PSW.CY, read live by the ALU (not an operand
    // kind), so the decode `src` is the memory-side operand in every form.
    wire is_addsub   = (d_alu_op == ALU_ADD) || (d_alu_op == ALU_ADDC)
                    || (d_alu_op == ALU_SUBB);
    wire addsub_ok   = is_addsub && (d_dst == OPK_A)
                    && ((d_shape == SH_A) || (d_shape == SH_B))
                    && ((d_src == OPK_RN) || (d_src == OPK_RI)
                        || (d_src == OPK_DIR) || (d_src == OPK_IMM));
    // INC / DEC: read-modify-write of ONE operand, so src == dst by
    // construction (the generated rows carry the same kind in both fields).
    wire incdec_ok   = ((d_alu_op == ALU_INC) || (d_alu_op == ALU_DEC))
                    && (d_src == d_dst)
                    && ((d_shape == SH_A) || (d_shape == SH_B))
                    && ((d_src == OPK_A) || (d_src == OPK_RN)
                        || (d_src == OPK_RI) || (d_src == OPK_DIR));
    // INC DPTR (A3H): the ISA's only 16-bit increment, shape C1 / 2 cycles
    wire incdptr_ok  = (d_alu_op == ALU_INC) && (d_shape == SH_C1)
                    && (d_src == OPK_DPTR) && (d_dst == OPK_DPTR);

    // ---- C4.3 MUL / DIV / DA A -----------------------------------------
    // MUL AB / DIV AB: the shape-F pair - the {A,B} register pair is both
    // source and destination, and the 4-cycle budget is what carries the §4.2
    // serial unit.  DA A: shape A, accumulator in and out, C read AND written
    // (set-only, §4.3) with AC read but never written.
    wire is_muldiv   = (d_alu_op == ALU_MUL) || (d_alu_op == ALU_DIV);
    wire muldiv_ok   = is_muldiv && (d_shape == SH_F)
                    && (d_src == OPK_B) && (d_dst == OPK_B);
    wire da_ok       = (d_alu_op == ALU_DA) && (d_shape == SH_A)
                    && (d_src == OPK_A) && (d_dst == OPK_A);

    // ---- C4.4 byte logic ------------------------------------------------
    // Three families, all keyed on the ALU op + shape + operand kinds:
    //   1. accumulator destination : ANL/ORL/XRL A,{Rn,@Ri,direct,#data}
    //   2. direct destination (RMW): ANL/ORL/XRL direct,{A,#data} - the `,A`
    //      forms resolve to d_src == dir (generator src priority), the
    //      `,#data` forms to d_src == imm, which is also what tells the two
    //      apart everywhere below
    //   3. unary accumulator ops   : CLR A / CPL A / SWAP A / RL A / RR A,
    //      plus the two rotate-through-carry forms whose src/dst both resolve
    //      to OPK_C (see the header note)
    wire is_logic_op = (d_alu_op == ALU_ANL) || (d_alu_op == ALU_ORL)
                    || (d_alu_op == ALU_XRL);
    wire is_rot_c    = (d_alu_op == ALU_RLC) || (d_alu_op == ALU_RRC);
    wire logic_a_ok  = is_logic_op && (d_dst == OPK_A)
                    && ((d_shape == SH_A) || (d_shape == SH_B))
                    && ((d_src == OPK_RN) || (d_src == OPK_RI)
                        || (d_src == OPK_DIR) || (d_src == OPK_IMM));
    wire logic_dir_ok = is_logic_op && (d_dst == OPK_DIR)
                    && (((d_shape == SH_B)  && (d_src == OPK_DIR))
                     || ((d_shape == SH_C3) && (d_src == OPK_IMM)));
    wire unary_a_ok  = ((d_alu_op == ALU_CLRA) || (d_alu_op == ALU_CPLA)
                        || (d_alu_op == ALU_SWAP) || (d_alu_op == ALU_RL)
                        || (d_alu_op == ALU_RR))
                    && (d_shape == SH_A) && (d_dst == OPK_A)
                    && ((d_src == OPK_A) || (d_src == OPK_NONE));
    wire rot_c_ok    = is_rot_c && (d_shape == SH_A)
                    && (d_src == OPK_C) && (d_dst == OPK_C);

    // ---- C4.5 bit family -------------------------------------------------
    // Three sub-families, keyed on the bit ALU ops + shape + operand kinds.
    // What separates them is WHERE the bit lives, which is exactly the pair
    // (d_src, d_dst) - the carry is an operand kind (OPK_C) like any other:
    //   1. carry-only  : SETB/CLR/CPL C          (shape A, no bit operand)
    //   2. bit destination (RMW on the containing byte): SETB/CLR/CPL bit
    //      (shape B) and MOV bit,C (shape C2)
    //   3. bit source, carry destination: MOV C,bit (shape B) and the four
    //      ANL/ORL C,±bit forms (shape C2); the `/bit` complement is the
    //      generated d_bitn field, not an opcode test (QUESTION-P45-1)
    wire is_bit_alu  = (d_alu_op == ALU_BITSET) || (d_alu_op == ALU_BITCLR)
                    || (d_alu_op == ALU_BITCPL) || (d_alu_op == ALU_BITMOV)
                    || (d_alu_op == ALU_BITANL) || (d_alu_op == ALU_BITORL);
    wire bit_c_ok    = is_bit_alu && (d_shape == SH_A) && (d_dst == OPK_C)
                    && ((d_src == OPK_NONE) || (d_src == OPK_C));
    wire bit_dst_ok  = is_bit_alu && (d_dst == OPK_BIT)
                    && (((d_shape == SH_B)
                         && ((d_src == OPK_NONE) || (d_src == OPK_BIT)))
                     || ((d_shape == SH_C2) && (d_src == OPK_C)));
    wire bit_src_ok  = is_bit_alu && (d_src == OPK_BIT) && (d_dst == OPK_C)
                    && ((d_shape == SH_B) || (d_shape == SH_C2));

    // ---- C4.6 branch family ---------------------------------------------
    // Five sub-families, keyed on (d_cond, d_tgt, d_shape, operand kinds).
    // `d_tgt` (QUESTION-P46-1) is what makes the rule structural at all: it
    // names HOW the target is computed, which is the only thing separating
    // SJMP rel (80H) from AJMP addr11 (01H..E1H) - identical in every other
    // generated field.  `d_cond` (this chunk owns its branch semantics) names
    // WHETHER the transfer happens.
    //   1. unconditional: SJMP / AJMP / LJMP / JMP @A+DPTR - no destination,
    //      no datapath operand except JMP's DPTR tap
    //   2. accumulator / carry conditionals: JZ/JNZ (src = A), JC/JNC (src = C)
    //   3. bit conditionals: JB/JNB read a bit and write nothing; JBC also
    //      clears it, but ONLY on the taken path (instr §3) - the one
    //      conditional destination write in the ISA
    //   4. CJNE: compares d_src2 (A/Rn/@Ri) against d_src (#data/direct) and
    //      writes C only; neither operand is modified
    //   5. DJNZ: decrements its own operand (ALU_DEC, src == dst) and branches
    //      on a non-zero result
    wire is_branch  = (d_class == CLS_BRANCH);
    wire jmp_ok     = is_branch && (d_cond == CND_ALWAYS) && (d_dst == OPK_NONE)
                   && (((d_tgt == TGT_REL) && (d_shape == SH_C2)
                        && (d_src == OPK_NONE))
                    || ((d_tgt == TGT_A11) && (d_shape == SH_C2)
                        && (d_src == OPK_NONE))
                    || ((d_tgt == TGT_A16) && (d_shape == SH_C3)
                        && (d_src == OPK_NONE))
                    || ((d_tgt == TGT_ADPTR) && (d_shape == SH_C1)
                        && (d_src == OPK_DPTR)));
    wire jcc_ok     = is_branch && (d_tgt == TGT_REL) && (d_shape == SH_C2)
                   && (d_dst == OPK_NONE)
                   && ((((d_cond == CND_Z) || (d_cond == CND_NZ))
                        && (d_src == OPK_A))
                    || (((d_cond == CND_C) || (d_cond == CND_NC))
                        && (d_src == OPK_C)));
    wire jbit_ok    = is_branch && (d_tgt == TGT_REL) && (d_shape == SH_C3)
                   && (d_src == OPK_BIT)
                   && ((((d_cond == CND_BIT) || (d_cond == CND_NBIT))
                        && (d_dst == OPK_NONE))
                    || ((d_cond == CND_JBC) && (d_dst == OPK_BIT)
                        && (d_alu_op == ALU_BITCLR)));
    wire cjne_ok    = is_branch && (d_cond == CND_CJNE) && (d_alu_op == ALU_CJNE)
                   && (d_tgt == TGT_REL) && (d_shape == SH_C3)
                   && (d_dst == OPK_C)
                   && ((d_src == OPK_IMM) || (d_src == OPK_DIR))
                   && ((d_src2 == OPK_A) || (d_src2 == OPK_RN)
                       || (d_src2 == OPK_RI));
    wire djnz_ok    = is_branch && (d_cond == CND_DJNZ) && (d_alu_op == ALU_DEC)
                   && (d_tgt == TGT_REL) && (d_src == d_dst)
                   && (((d_src == OPK_RN)  && (d_shape == SH_C2))
                    || ((d_src == OPK_DIR) && (d_shape == SH_C3)));

    // ---- C4.7 call / return family ---------------------------------------
    // Eleven opcodes, three sub-families, told apart by (d_tgt, d_shape) - the
    // same two fields C4.6 reads, which is why the family needs no extension
    // of its own except `d_reti` (QUESTION-P47-1).  The generator's call
    // closure asserts the partition: every call-class row is an unconditional
    // ALU_PASS stack->stack transfer whose (tgt, shape) is one of a11/C2,
    // a16/C3, none/C1, so these three wires cover all eleven with no overlap.
    //   1. ACALL addr11 (11/31/.../F1H): AJMP's page math + a return-address
    //      push pair
    //   2. LCALL addr16 (12H): LJMP's absolute target + the same push pair
    //   3. RET / RETI   (22H/32H): the pop pair; RETI additionally re-arms the
    //      interrupt level (d_reti, §6.3)
    wire is_call_cls = (d_class == CLS_CALL);
    wire is_acall    = is_call_cls && (d_tgt == TGT_A11) && (d_shape == SH_C2);
    wire is_lcall    = is_call_cls && (d_tgt == TGT_A16) && (d_shape == SH_C3);
    wire is_call     = is_acall || is_lcall;
    wire is_ret      = is_call_cls && (d_tgt == TGT_NONE) && (d_shape == SH_C1);
    wire call_ok     = (is_call || is_ret) && (d_alu_op == ALU_PASS)
                    && (d_cond == CND_ALWAYS)
                    && (d_src == OPK_STACK) && (d_dst == OPK_STACK);

    // ---- C4.8 MOVX family -------------------------------------------------
    // Six opcodes, one shape (SH_D), two directions.  The generated operand
    // kinds OPK_XRAM_DPTR / OPK_XRAM_RI are what make the rule structural:
    // they name the xdata SPACE and the address SOURCE in one field, so the
    // direction is just "which side of the move the xdata operand is on" and
    // the accumulator half is `d_dst == OPK_A` (read) / `d_src2 == OPK_A`
    // (write - the generator's src priority gives the address operand the
    // `src` slot on the write forms).
    wire movx_shape = (d_shape == SH_D);
    wire movx_is_rd = movx_shape && (d_dst == OPK_A)
                   && ((d_src == OPK_XRAM_DPTR) || (d_src == OPK_XRAM_RI));
    wire movx_is_wr = movx_shape && (d_src2 == OPK_A)
                   && ((d_dst == OPK_XRAM_DPTR) || (d_dst == OPK_XRAM_RI));
    wire movx_ok    = (d_alu_op == ALU_PASS) && (movx_is_rd || movx_is_wr);
    // the @Ri half of the family, in either direction: 8-bit address on the
    // bus low byte, P2 latch on the high byte (TQ10)
    wire movx_ri    = (d_src == OPK_XRAM_RI) || (d_dst == OPK_XRAM_RI);

    wire supported = (d_class == CLS_MISC)
                  || ((d_class == CLS_MOV) && (mov_byte_ok || mov_d16_ok
                                               || xch_ok || push_ok || pop_ok))
                  || ((d_class == CLS_MOVC) && movc_ok)
                  || ((d_class == CLS_LOGIC) && (logic_a_ok || logic_dir_ok
                                                 || unary_a_ok || rot_c_ok))
                  || ((d_class == CLS_ARITH) && (addsub_ok || incdec_ok
                                                 || incdptr_ok || muldiv_ok
                                                 || da_ok))
                  || ((d_class == CLS_BIT) && (bit_c_ok || bit_dst_ok
                                               || bit_src_ok))
                  || ((d_class == CLS_BRANCH) && (jmp_ok || jcc_ok || jbit_ok
                                                  || cjne_ok || djnz_ok))
                  || ((d_class == CLS_CALL) && call_ok)
                  || ((d_class == CLS_MOVX) && movx_ok);

    // ------------------------------------------------------------------
    // internal / external fetch decision, per slot address (§1.4)
    // ------------------------------------------------------------------
    function automatic logic slot_ext(input logic [15:0] a);
        slot_ext = (!ea_latched) || (a >= 16'(1 << ROM_AW));
    endfunction

    // The MOVC table byte arrives at end S1P1(C2) into `op1` (F-4).  It is a
    // consumed fetch (the byte is captured) but NOT a program byte, so `pc`
    // must not advance ([D-01] + §2.5).  Derived, never a pipeline flop.
    wire        movc_shape   = (d_shape == SH_E);
    wire        movc_arrival = exec && (mcyc == 2'd1) && movc_shape;

    // C4.8: the two MOVX cycles.  C1 replaces its slot-B FETCH with the xdata
    // address launch and C2 has no slot-A fetch at all (§1.3 / §2.5 shape-D
    // row); everything else in the schedule is the ordinary one.
    wire        movx    = exec && movx_shape && supported;
    wire        movx_c1 = movx && (mcyc == 2'd0);
    wire        movx_c2 = movx && (mcyc == 2'd1);

    // C5.3: the slot-B byte already in flight when the ILCALL is injected was
    // fetched at `npc` by the completing instruction's final cycle.  Shape I
    // (§2.5 note) DISCARDS it - no capture, no pc increment - because the
    // interrupted flow's next opcode must be re-fetched after the RETI.  This
    // is the only place an arriving CONSUMED slot is dropped, and it is a
    // seqstate test, not an opcode one.
    wire        ilcall_drop = ilcall1 && (ph == PH_S1P1);

    // pc after the arrival edge currently being processed
    wire consume_now = fp_valid && fp_consume && !ilcall_drop;
    wire pc_inc_now  = consume_now && !((ph == PH_S1P1) && movc_arrival);
    wire [15:0] pc_adv = pc_inc_now ? (pc + 16'd1) : pc;

    // ---- slot plans ---------------------------------------------------
    // slot A (launched end S1P1): consumed only as operand byte 2, in C1
    wire        sa_consume = exec && (mcyc == 2'd0) && (d_len >= 2'd2);
    // slot B (launched end S4P1).  C5.3: ILCALL2's slot B is the ISR's opcode
    // fetch at the vector address - the one consumed slot of shape I.
    wire        sb_last    = prime || (exec && is_last) || ilcall2;
    wire        sb_op2     = exec && (mcyc == 2'd0) && (d_len == 2'd3);
    // slot B of MOVC C1 carries the code-table read instead of a program byte
    wire        sb_movc    = exec && (mcyc == 2'd0) && movc_shape;
    wire        sb_consume = sb_last || sb_op2 || sb_movc;
    wire [1:0]  sb_dst     = sb_last ? FPD_IR : (sb_op2 ? FPD_OP2 : FPD_OP1);
    // the slot-B ADDRESS is formed further down (`sb_addr`): it is `pc` except
    // on a MOVC C1 (the code-table read, [D-01]) and on the final cycle of a
    // taken transfer, where it is the target - C4.6's whole bus-level effect.

    // ------------------------------------------------------------------
    // operand addressing
    // ------------------------------------------------------------------
    wire [7:0] bank_base = {3'b000, psw_q[PSW_RS1], psw_q[PSW_RS0], 3'b000};
    wire [7:0] rn_addr   = bank_base | {5'b00000, ir_r[2:0]};
    wire [7:0] ri_addr   = bank_base | {7'b0000000, ir_r[0]};

    // C4.6: a relative-target form spends its LAST operand byte on the offset
    // (`rel` is the last operand token of every d_tgt == REL row - the
    // generator asserts that closure), so op1 carries whatever else the
    // instruction needs and op2 is the offset.  That is what separates
    // `CJNE A,#data,rel` (op1 = #data) from `MOV direct,#data` (op2 = #data)
    // and `DJNZ direct,rel` (op1 = direct) from `MOV direct,direct`.
    wire has_rel = (d_tgt == TGT_REL);
    // MOV direct,direct (85H) is the only opcode carrying TWO direct operand
    // bytes: its encoding is opcode, SOURCE addr, DEST addr (instr §4), so the
    // destination lives in op2 and everything else keeps it in op1.  The
    // discriminator is the operand-byte count, not `src==dst==dir`: XCH
    // A,direct (C5H) is also dir->dir but carries ONE direct byte, and
    // DJNZ direct,rel (D5H) is a 3-byte dir->dir whose second byte is the
    // offset.
    wire both_dir      = (d_src == OPK_DIR) && (d_dst == OPK_DIR)
                      && (d_len == 2'd3) && !has_rel;
    wire [7:0] dir_src = op1_r;
    wire [7:0] dir_dst = both_dir ? op2_r : op1_r;
    wire [7:0] imm_val = ((d_len == 2'd3) && !has_rel) ? op2_r : op1_r;
    wire [7:0] rel_val = (d_len == 2'd3) ? op2_r : op1_r;

    // ---- C4.5 bit-address unit (§5.3) ---------------------------------
    // Pure address math over the bit operand, which is always op1 (no bit
    // opcode carries two operand bytes in this chunk):
    //   b[7]==0  IRAM  : byte 20H + b[6:3], bit b[2:0]   (bit space 20-2FH)
    //   b[7]==1  SFR   : byte b & F8H,      bit b[2:0]   (periph §2.3, PQ17)
    // A bit address whose containing byte has no SFR owner is PQ5 territory
    // and needs nothing here: the core's SFR read mux substitutes
    // NU8051_CFG_UNIMPL_RD for the containing byte and the write finds no
    // owner, so it is dropped - the same mechanism the byte-direct forms use.
    wire       bit_is_sfr = op1_r[7];
    wire [7:0] bit_byte   = bit_is_sfr ? {op1_r[7:3], 3'b000}
                                       : {4'b0010, op1_r[6:3]};
    wire [2:0] bit_idx    = op1_r[2:0];
    wire [7:0] bit_mask   = 8'h01 << bit_idx;
    wire       bit_dst    = (d_dst == OPK_BIT);
    // the containing byte is staged in rd2_data by the slots below; the
    // `/bit` complement (d_bitn) is applied here, once, on the read side -
    // "the source bit itself is not affected" (instr §4) falls out of the
    // fact that nothing writes back on a carry-destination form
    wire       bit_val    = rd2_data[bit_idx] ^ d_bitn;
    // ALU B port for the bit family: the containing-byte mask when the
    // destination is a bit, the source bit when it is the carry (§4.1 row)
    wire [7:0] bit_b      = bit_dst ? bit_mask : {7'b0, bit_val};

    // C4.6: CJNE's compared operand is the SECOND source - the src priority
    // necessarily names the `#data` / `direct` half (QUESTION-P46-1) - so the
    // first half is armed off d_src2.  Keyed on the ALU op too, so no other
    // shape's d_src2 (ADDC's carry, MOVX @Ri's pointer) can claim a slot.
    wire cmp_rn   = (d_alu_op == ALU_CJNE) && (d_src2 == OPK_RN);
    wire cmp_ri   = (d_alu_op == ALU_CJNE) && (d_src2 == OPK_RI);

    wire need_ptr = (d_src == OPK_RI) || (d_dst == OPK_RI) || cmp_ri;
    wire need_rn  = (d_src == OPK_RN) || cmp_rn;
    // the `direct` source read cycle: shape B has to wait for op1 (arrives end
    // S4P1 of C1) and uses the late slot; the 2-cycle shapes read in C2.
    wire dir_rd_late = exec && (mcyc == 2'd0) && (d_src == OPK_DIR)
                       && (d_shape == SH_B);
    wire dir_rd_c2   = exec && (mcyc == 2'd1) && (d_src == OPK_DIR)
                       && ((d_shape == SH_C2) || (d_shape == SH_C3));
    // C4.4: a direct-DESTINATION read-modify-write whose source is NOT that
    // same direct byte still has to read its destination.  d_rmw is exactly
    // the timing §5 "reads its own destination" marker, so this is a decode
    // field test, not an opcode list; the members are the three ANL/ORL/XRL
    // direct,#data forms (shape C3, d_src == imm), which take the R1 slot of
    // C2 - op1 has carried the direct address since the end of S4P1(C1).
    wire dst_rmw_rd  = d_rmw && (d_dst == OPK_DIR) && (d_src != OPK_DIR);
    wire dst_rd_c2   = exec && (mcyc == 2'd1) && (d_shape == SH_C3)
                       && dst_rmw_rd;
    // C4.5: EVERY bit operand needs its containing byte on the bus - as the
    // source of a carry-destination form and as the read half of a bit
    // destination's read-modify-write alike, so the arming test is just
    // "this opcode has a bit operand".  The slot is the same one the byte
    // forms of the same shape use: shape B waits for op1 and takes the R3
    // late slot, the 2-cycle shapes take the R1 slot of C2 (op1 has been
    // stable since the end of S4P1(C1)).  Nothing else can claim either slot
    // in these opcodes - no bit form has an @Ri, Rn or `direct` operand.
    // C4.6 joins JB/JNB/JBC (shape C3) to the C2 arm: their bit address is in
    // op1 from the end of S4P1(C1), the byte lands in rd2_data at the end of
    // S2P1(C2), and the condition needs it by the end of S4P1(C2) - 5 ticks of
    // margin.  No collision: a bit-operand opcode never carries an @Ri, Rn or
    // `direct` operand, in either shape.
    wire bit_rd      = (d_dst == OPK_BIT) || (d_src == OPK_BIT);
    wire bit_rd_late = exec && (mcyc == 2'd0) && (d_shape == SH_B)  && bit_rd;
    wire bit_rd_c2   = exec && (mcyc == 2'd1) && bit_rd
                       && ((d_shape == SH_C2) || (d_shape == SH_C3));
    // the C2 read slot serves the `direct` SOURCE, that destination, or a
    // bit operand's containing byte; the late slot the shape-B pair of those
    wire        rd_c2_en   = dir_rd_c2 || dst_rd_c2 || bit_rd_c2;
    wire  [7:0] rd_c2_addr = bit_rd_c2 ? bit_byte
                           : (dst_rd_c2 ? dir_dst : dir_src);
    wire        rd_late_en   = dir_rd_late || bit_rd_late;
    wire  [7:0] rd_late_addr = bit_rd_late ? bit_byte : dir_src;

    // C4.4: the port read-modify-write qualification (timing §5).  d_rmw is
    // the mnemonic membership; the manual's own qualification is "every
    // addressing form of these mnemonics whose DESTINATION is a port direct
    // address or port bit", which is a decode-field test - so `ANL P1,A`
    // (destination direct) reads the latch and `ANL A,P1` (destination A)
    // reads the S5P1 pin sample.  The OPK_BIT arm is the bit-destination half
    // of the same sentence, live from C4.5: `SETB P1.3` / `CPL P1.3` /
    // `MOV P1.3,C` read the LATCH (destination = a port bit) while
    // `MOV C,P1.3` and `ANL C,P1.3` read the PIN (destination = the carry).
    // All nine bit opcodes with an operand carry d_rmw (their mnemonics are on
    // the timing §5 list), so d_dst alone discriminates; the g_ports bit-form
    // vectors are the two witnesses.
    wire rmw_read = d_rmw && ((d_dst == OPK_DIR) || (d_dst == OPK_BIT));
    // POP: the stack read uses the R1 slot of C1 (§5.1) - SP is a flop, so its
    // value is available from the first tick of the instruction.
    // C4.7: RET / RETI pop TWO bytes through the R1 and R2 slots of C1, at SP
    // and SP-1 (§5.1 "stack pops (RET/RETI): R1/R2 slots of C1 at SP, SP-1").
    // ACALL / LCALL carry `d_src == OPK_STACK` as well (their `reads` list
    // names SP) but read no stack byte at all - their traffic is the two
    // WRITES at S6P1/S6P2 - so the read arming excludes them.
    wire stk_rd      = exec && (mcyc == 2'd0) && (d_src == OPK_STACK)
                       && !is_call;
    wire ret_pop     = exec && (mcyc == 2'd0) && is_ret;

    // C4.8: a MOVX @Ri reads its POINTER register through the same R1 slot of
    // C1 the @Ri byte forms use - the value has to be on `movx_xaddr` by the
    // end-S4P1(C1) launch edge, and R1 lands it in rd1_data at the end
    // of S2P1, 8 ticks earlier.  The address is `ri_addr` (bank | ir[0]), NOT
    // `rn_addr`: the write forms carry d_src == OPK_RN (the generator's src
    // priority names the pointer register), so `MOVX @R0,A` (F2H) would
    // otherwise read R2 out of ir[2:0].  The override is on the movx wire, so
    // no other shape's R1 plan moves.
    wire movx_ptr_rd = movx_c1 && movx_ri;
    wire r1_en = exec && (mcyc == 2'd0) && (need_ptr || need_rn || stk_rd
                                            || movx_ptr_rd);
    wire [7:0] r1_addr = movx_ptr_rd ? ri_addr
                       : (stk_rd ? sp_q : (need_ptr ? ri_addr : rn_addr));
    // R2 serves the @Ri dereference, CJNE's @Ri operand, or the SECOND stack
    // pop; the three cannot coexist (no opcode carries an @Ri and the stack).
    wire r2_en = exec && (mcyc == 2'd0) && ((d_src == OPK_RI) || cmp_ri
                                            || ret_pop);

    // ------------------------------------------------------------------
    // operand values (C4.6 moved this section above the destination logic:
    // the branch unit's conditions are computed from the same staged operands
    // and have to be formed before the slot-B address they steer)
    // ------------------------------------------------------------------
    // stack pointer arithmetic (§2.6): PUSH pre-increments, POP post-decrements
    // C4.7 adds the two-byte pair: a CALL writes SP+1 then SP+2 and commits
    // SP+2; a RET reads SP then SP-1 and commits SP-2.  All four wires wrap
    // modulo 256 by construction, which IS the 8051's stack wrap (FFH -> 00H);
    // §5.2's indirect rule then drops the 8051-mode writes above 7FH.
    wire [7:0] sp_inc  = sp_q + 8'd1;
    wire [7:0] sp_dec  = sp_q - 8'd1;
    wire [7:0] sp_inc2 = sp_q + 8'd2;
    wire [7:0] sp_dec2 = sp_q - 8'd2;
    wire       is_push = (d_alu_op == ALU_PUSH);
    wire       is_pop  = (d_alu_op == ALU_POP);

    // PUSH SP (C0 81H): the manual's operation box is `(SP)<-(SP)+1` THEN
    // `((SP))<-(direct)`, so a PUSH whose source is SP itself pushes the
    // already-incremented value (instr §4; g_stack `push-sp-itself`).
    wire push_sp_bypass = is_push && (dir_src == SFR_SP);

    logic [7:0] src_val;
    always_comb begin
        unique case (d_src)
            OPK_A:   src_val = acc_q;
            // MUL/DIV: the B register is a direct tap (§5.4), never a bus read
            OPK_B:   src_val = b_q;
            OPK_RN:  src_val = rd1_data;
            OPK_RI:  src_val = (!P_8052 && rd1_data[7]) ? NU8051_CFG_UNIMPL_RD
                                                        : rd2_data;
            OPK_DIR: src_val = push_sp_bypass ? sp_inc : rd2_data;
            OPK_IMM: src_val = imm_val;
            // MOV DPTR,#data16: byte2 = DPH, byte3 = DPL (instr §4).  The byte
            // datapath carries the low half; the 16-bit write is below.
            OPK_IMM16: src_val = op2_r;
            // POP: the stack byte captured in the C1 R1 slot (PQ5 already
            // applied at the arrival edge for 8051-mode upper-128 reads)
            OPK_STACK: src_val = rd1_data;
            // MOVC: the table byte staged in op1 (D1.2 F-4)
            OPK_CODE_DPTR, OPK_CODE_PC: src_val = op1_r;
            // C4.8 MOVX read: the xdata byte sampled at the RD# rising edge
            // and held in `movx_din` (TQ1 / D1.2 F-2).  Both address forms
            // land here - the address decides where the byte came from, not
            // how it commits, and ALU_PASS -> ACC is the ordinary path.
            OPK_XRAM_DPTR, OPK_XRAM_RI: src_val = movx_din;
            default: src_val = 8'h00;
        endcase
    end

    // C4.6: the value of the SECOND source kind - CJNE's first (compared)
    // operand.  Only the compare rule reads it, and the generator's closure
    // check pins d_src2 to {A, Rn, @Ri} on every CND_CJNE row, so the three
    // arms below are exhaustive for its only consumer.  The 8051-mode PQ5
    // substitution mirrors `src_val`'s @Ri arm: `CJNE @R0,#data` with
    // R0 >= 80H compares the FFH constant.
    logic [7:0] src2_val;
    always_comb begin
        src2_val = acc_q;
        if (d_src2 == OPK_RN)
            src2_val = rd1_data;
        else if (d_src2 == OPK_RI)
            src2_val = (!P_8052 && rd1_data[7]) ? NU8051_CFG_UNIMPL_RD
                                                : rd2_data;
    end

    // ------------------------------------------------------------------
    // C4.6 branch unit (core_design §2.3 branch shapes, §2.5 PC rules)
    //
    // TARGET (`d_tgt`, QUESTION-P46-1).  All four forms are pure combinational
    // math over state that is already stable when the final cycle reaches its
    // S4P1 edge - the PC-load instant (§2.5) - so the whole family needs NO
    // new flop, no new read slot and no save-state map append:
    //
    //   REL    pc + sign_extend(rel).  `pc` here is the address of the NEXT
    //          instruction: the offset is relative to the first byte after the
    //          branch (instr §4), and by §2.5 the last operand byte was
    //          consumed at end S4P1(C1) (2-byte forms) or end S1P1(C2)
    //          (3-byte forms), so `pc_adv` at this edge IS that address.
    //   A11    {pc_adv[15:11], ir[7:5], op1} - the 2K page of the next
    //          instruction's first byte, NOT of the AJMP (instr §4).  Only
    //          `pc_adv` can produce it: an AJMP at 07FEH pages off 0800H.
    //   A16    {op1, op2} - LJMP's absolute pair, high byte first.
    //   ADPTR  DPTR + A, zero-extended (JMP @A+DPTR); no PC term at all.
    //
    // CONDITION (`d_cond`, whose branch semantics this chunk owns).  Every
    // input is a live tap or a staging flop written no later than the end of
    // S2P2 of the final cycle:
    //   Z/NZ   ACC tap;  C/NC  PSW.CY tap
    //   BIT / NBIT / JBC   `bit_val` = the containing byte read into rd2_data
    //          by the C2 read slot (§5.3), which C4.6 extends to shape C3
    //   CJNE   src2_val (A / Rn / @Ri) != src_val (#data / direct)
    //   DJNZ   the decremented operand is non-zero.  `djnz_res` is the same
    //          value ALU_DEC commits (its A port is `src_val` through the
    //          is_incdec arm), recomputed here only because the condition is
    //          needed 5 ticks before the ALU result is committed.
    //
    // TIMING (§2.5, instr §2): a conditional branch is a FIXED 2 cycles taken
    // or not.  Nothing below changes `d_cycles`, the slot plan or the fetch
    // cadence - taken and not-taken differ ONLY in the address slot B of the
    // final cycle launches (and therefore in the PC that is loaded), which is
    // exactly what the §2.5 per-slot table prescribes.
    // ------------------------------------------------------------------
    wire [15:0] tgt_rel   = pc_adv + {{8{rel_val[7]}}, rel_val};
    wire [15:0] tgt_a11   = {pc_adv[15:11], ir_r[7:5], op1_r};
    wire [15:0] tgt_a16   = {op1_r, op2_r};
    wire [15:0] tgt_adptr = dptr_q + {8'h00, acc_q};
    // C4.7: RET / RETI carry no target OPERAND - their target is the popped
    // pair, staged high-then-low in op2/op1 by the C1 read slots (D1.2 F-5).
    wire [15:0] tgt_ret   = {op2_r, op1_r};

    logic [15:0] br_target;
    always_comb begin
        unique case (d_tgt)
            TGT_A11:   br_target = tgt_a11;
            TGT_A16:   br_target = tgt_a16;
            TGT_ADPTR: br_target = tgt_adptr;
            TGT_REL:   br_target = tgt_rel;
            // TGT_NONE on a transfer means RET/RETI (the generator's call
            // closure pins that); on anything else br_target is never read.
            default:   br_target = tgt_ret;
        endcase
    end

    wire [7:0] djnz_res = src_val - 8'd1;

    logic cond_true;
    always_comb begin
        unique case (d_cond)
            CND_ALWAYS:        cond_true = 1'b1;
            CND_Z:             cond_true = (acc_q == 8'h00);
            CND_NZ:            cond_true = (acc_q != 8'h00);
            CND_C:             cond_true = psw_q[PSW_CY];
            CND_NC:            cond_true = ~psw_q[PSW_CY];
            CND_BIT, CND_JBC:  cond_true = bit_val;
            CND_NBIT:          cond_true = ~bit_val;
            CND_CJNE:          cond_true = (src2_val != src_val);
            CND_DJNZ:          cond_true = (djnz_res != 8'h00);
            default:           cond_true = 1'b0;          // CND_NONE
        endcase
    end

    // C5.3: the injected LCALL's target - the vector of the source latched at
    // the take decision (periph §6.1 table; the addresses are spaced 8 bytes
    // from 0003H, 1-7 / PDF 13).  Read only in ILCALL2.
    logic [15:0] ivec_addr;
    always_comb begin
        unique case (take_src)
            3'd0:    ivec_addr = 16'h0003;      // IE0
            3'd1:    ivec_addr = 16'h000B;      // TF0
            3'd2:    ivec_addr = 16'h0013;      // IE1
            3'd3:    ivec_addr = 16'h001B;      // TF1
            3'd4:    ivec_addr = 16'h0023;      // RI + TI
            default: ivec_addr = 16'h002B;      // TF2 + EXF2 (8052, C5.5)
        endcase
    end

    // The transfer itself: applied at the edge ending S4P1 of the final cycle
    // (§2.5).  C4.7 joins the CALL class to the same instant - §2.5 lists
    // "branch/call/RET/RETI/JMP @A+DPTR/interrupt vector" as one rule, and all
    // eleven call opcodes are `d_cond == CND_ALWAYS`, so the condition term
    // needs no arm of its own.  C5.3 joins the last member of that same §2.5
    // list: ILCALL2 loads the vector at ITS S4P1, which is also its slot-B
    // launch, so the ISR's first opcode fetch goes to the vector with no dead
    // slot - the identical shape a taken LCALL has.
    wire xfer_now = (exec && is_last && supported && cond_true
                     && (is_branch || is_call_cls)) || ilcall2;
    wire [15:0] xfer_target = ilcall2 ? ivec_addr : br_target;

    // C4.7: the return address is `pc` after all operand consumes (§2.5) -
    // exactly the value the PC LOAD overwrites at this edge - so the two push
    // bytes are staged into op2_r/op1_r AT that edge: the D1.2 F-5 pop staging
    // convention run in the other direction (QUESTION-P47-2).  Both registers
    // are dead by then in both call shapes (ACALL's last operand byte arrived
    // at end S4P1(C1), LCALL's at end S1P1(C2)) and slot A of the final cycle
    // is a discarded fetch, so no arrival can collide with the staging write.
    // No new flop, and therefore no save-state map append.
    //
    // C5.3: the injected LCALL pushes the interrupted flow's `pc` UNMODIFIED
    // (timing §6: only the PC, never the PSW) and its S4P1 edge destroys it
    // exactly the same way, so it stages through the same two registers.
    // `pc_adv == pc` in ILCALL2 (its slot A is a discarded fetch), so one
    // expression serves both.
    wire push_pair  = (exec && is_last && supported && is_call) || ilcall2;
    wire call_stage = (xfer_now && is_call) || ilcall2;

    // MOVC table address: A + DPTR, or A + PC where PC is the address of the
    // following instruction = pc after the opcode consume (instr §4).  At the
    // slot-B launch edge (end S4P1 of C1) `pc_adv` is exactly that value.
    wire [15:0] movc_addr = ((d_src == OPK_CODE_DPTR) ? dptr_q : pc_adv)
                          + {8'h00, acc_q};
    // slot-B address: the branch target on a taken transfer's final cycle, the
    // MOVC table address in C1 of a MOVC, the running `pc` otherwise.  The two
    // substitutions cannot collide (a MOVC is not a transfer).
    wire [15:0] sb_addr   = sb_movc    ? movc_addr
                          : xfer_now   ? xfer_target : pc_adv;

    // ------------------------------------------------------------------
    // IRAM / SFR bus muxing by phase
    // ------------------------------------------------------------------
    // The IRAM address is presented during the phase BEFORE the capture edge;
    // the SFR read mux is combinational and is addressed during the capture
    // phase itself.
    logic [7:0] dst_addr;                 // resolved IRAM write address
    logic       dst_is_sfr, dst_drop;

    // C5.6: `!pd_freeze` is what keeps the frozen decode from committing again.
    // Power down leaves `seqstate`/`mcyc`/`ir_r` standing at the completing
    // instruction while `ph` free-runs (§7.1), so without this term every
    // twelfth CE tick would re-run the S6P2 of the instruction that powered the
    // core down - harmless while the peripherals have no clock, but not while
    // RESET is opening their gate.
    wire commit = exec && is_last && supported && (ph == PH_S6P2) && !pd_freeze;

    // C4.6: JBC clears the tested bit ONLY on the taken path (instr §3) - the
    // ISA's one conditional destination write.  Every other branch-class
    // destination write (DJNZ's decrement) is unconditional, and every
    // conditional branch other than JBC writes no destination at all, so this
    // one `d_cond` test is the whole rule.
    wire dst_we_ok = !((d_cond == CND_JBC) && !cond_true);

    always_comb begin
        dst_addr   = 8'h00;
        dst_is_sfr = 1'b0;
        dst_drop   = 1'b0;
        unique case (d_dst)
            OPK_RN:  dst_addr = rn_addr;
            OPK_RI:  begin
                dst_addr = rd1_data;
                // 8051: indirect 80H-FFH is unimplemented, writes dropped (PQ5)
                dst_drop = !P_8052 && rd1_data[7];
            end
            OPK_DIR: begin
                dst_addr   = dir_dst;
                dst_is_sfr = dir_dst[7];
            end
            // C4.5: a bit destination writes back its CONTAINING BYTE (§5.3);
            // the IRAM half of bit space is 20-2FH, so it is never dropped.
            OPK_BIT: begin
                dst_addr   = bit_byte;
                dst_is_sfr = bit_is_sfr;
            end
            // PUSH: the stack byte goes to IRAM[SP+1] by INDIRECT addressing
            // (§5.2), so on the 8051 a push above 7FH is dropped (PQ5).
            // C4.7: a CALL's S6P2 half is the PCH byte at SP+2 (its PCL half
            // went out at SP+1 in the S6P1 slot); RET/RETI share the operand
            // kind but write no byte at all (`stk_wr` below).
            OPK_STACK: begin
                dst_addr = is_call ? sp_inc2 : sp_inc;
                dst_drop = !P_8052 && dst_addr[7];
            end
            default: ;
        endcase
    end

    // C4.7: the call pair is the ISA's only DUAL-write instruction (§5.1) -
    // PCL into the S6P1 write slot at SP+1, PCH into the ordinary S6P2 commit
    // slot at SP+2.  Low byte first (instr §4 / §2.6).  RET/RETI write no
    // stack byte, so `stk_wr` (not the operand kind) arms the memory write.
    // C5.3 widens the two write slots from `call_push` to `push_pair`: an
    // ILCALL2 writes the same two bytes at the same two edges through the
    // same addresses.  Its S6P2 half cannot ride the decode-driven `commit`
    // arm (no opcode is executing), so it gets its own term below.
    wire call_push   = exec && is_last && supported && is_call;
    wire push_lo_now = push_pair && (ph == PH_S6P1) && !pd_freeze;
    wire push_hi_now = ilcall2   && (ph == PH_S6P2);
    wire push_lo_drop = !P_8052 && sp_inc[7];
    wire push_hi_drop = !P_8052 && sp_inc2[7];
    wire stk_wr      = is_push || is_call;

    always_comb begin
        case (ph)
            // read #1 address: C1 = Ri pointer / Rn; C2 = the `direct` source
            // (or, C4.4, the direct RMW destination)
            PH_S1P2: iram_addr = rd_c2_en ? rd_c2_addr
                               : (r1_en ? r1_addr : 8'h00);
            // read #2 address = the @Ri pointer, live on the RAM output
            // during S2P1 only (INV-IRD) - or, for RET/RETI, the second pop
            // at SP-1 (§5.1); no opcode carries both
            PH_S2P1: iram_addr = ret_pop ? sp_dec
                               : (r2_en ? iram_rdata : 8'h00);
            // late read (shape B, `direct` source or bit operand): op1 has
            // just arrived
            PH_S4P2: iram_addr = rd_late_en ? rd_late_addr : 8'h00;
            // C4.7 write slot 1 (§5.1 "dual-write (CALL pushes) at S6P1+S6P2")
            PH_S6P1: iram_addr = push_pair ? sp_inc : 8'h00;
            PH_S6P2: iram_addr = ilcall2 ? sp_inc2 : dst_addr;
            default: iram_addr = 8'h00;
        endcase
    end

    assign iram_we    = (commit && dst_we_ok
                                && (d_dst == OPK_RN
                                   || (d_dst == OPK_RI && !dst_drop)
                                   || (d_dst == OPK_DIR && !dst_is_sfr)
                                   || (d_dst == OPK_BIT && !dst_is_sfr)
                                   || (d_dst == OPK_STACK && stk_wr
                                       && !dst_drop)))
                     || (push_lo_now && !push_lo_drop)
                     || (push_hi_now && !push_hi_drop);
    // SFR bus: read address during the capture phase, write address at commit
    always_comb begin
        sfr_addr = 8'h00;
        sfr_rmw  = rmw_read;
        case (ph)
            PH_S2P1: sfr_addr = rd_c2_en ? rd_c2_addr : r1_addr;
            PH_S5P1: sfr_addr = rd_late_en ? rd_late_addr : 8'h00;
            // only a `direct` or `bit` destination reaches the SFR bus; a stack
            // write is always indirect (§5.2) and must not present an SFR
            // address here
            PH_S6P2: sfr_addr = ((d_dst == OPK_DIR) || (d_dst == OPK_BIT))
                                ? dst_addr : 8'h00;
            default: sfr_addr = 8'h00;
        endcase
    end
    assign sfr_we = commit && dst_we_ok
                    && ((d_dst == OPK_DIR) || (d_dst == OPK_BIT))
                    && dst_is_sfr;

    // ---- ALU (core_design §4.1) ---------------------------------------
    // The A-side result of every supported op comes out of the shared ALU:
    // ALU_PASS/XCH/PUSH/POP -> y = b (the operand), ALU_XCHD -> the nibble
    // merge, ALU_SWAP -> the accumulator nibble swap, ALU_ADD/ADDC/SUBB ->
    // a (op) b with the §4.1 C/AC/OV candidates, ALU_INC/DEC -> a +- 1.
    //
    // A-side mux (C4.2): §4.1 defines INC/DEC as `a +- 1`, and their operand
    // is the one they read *and* write - `INC R3` must not see ACC.  Every
    // other op in the supported set is accumulator-rooted.  `src_val` already
    // carries the PQ5 substitution for 8051-mode indirect reads above 7FH, so
    // `INC @R0` with R0 >= 80H increments the FFH constant (and its write is
    // dropped by `dst_drop`), exactly as the reference model does.  The rule is
    // keyed on the ALU op alone, not the class, so DJNZ (ALU_DEC, class
    // branch) inherits it correctly when C4.5 lands.
    logic [7:0] alu_y, alu_b_out;
    logic       alu_c, alu_ac, alu_ov;
    logic [15:0] md_acc_w;
    logic  [3:0] md_cnt_w;
    logic        md_busy_w;

    // C4.4 extends the same rule to the direct-destination RMW logic forms:
    // `ANL P1,#3Ch` computes on its DESTINATION's old value (staged into
    // rd2_data by the C2 read slot), not on ACC.  The `direct,A` forms need no
    // arm - the generator's src priority already delivers the direct byte on
    // the B port, and ANL/ORL/XRL are commutative.
    //
    // C4.5 puts a bit DESTINATION on the same arm: its containing byte (also
    // staged in rd2_data) is what the ALU replaces one bit of.  `bit_dst`
    // therefore joins `dst_rmw_rd` here rather than being a third case.
    //
    // C4.6 adds one A-side arm and widens the B-side test: CJNE's A port is
    // its FIRST operand (§4.1 "1 iff op1 < op2 unsigned"), which is d_src2's
    // value, and JB/JNB/JBC are bit-operand opcodes outside CLS_BIT, so the
    // mask / source-bit substitution keys on the bit operand itself.  DJNZ
    // needs neither: it is ALU_DEC, so the is_incdec arm already puts its own
    // operand on the A port, exactly as the header note predicted in C4.2.
    wire is_incdec = (d_alu_op == ALU_INC) || (d_alu_op == ALU_DEC);
    wire is_cjne   = (d_alu_op == ALU_CJNE);
    wire [7:0] alu_a = (dst_rmw_rd || bit_dst) ? rd2_data
                     : is_cjne                ? src2_val
                     : is_incdec              ? src_val : acc_q;
    // B port: the bit family substitutes the mask / source bit (§5.3), every
    // other op passes its byte source operand.
    wire [7:0] alu_b = ((d_class == CLS_BIT) || bit_rd) ? bit_b : src_val;

    // C4.3: the serial unit's start tick (§4.2) - the edge ending S2P1 of C1,
    // where both operand taps (ACC on the `a` port, B on `b` via src_val) have
    // been stable since the opcode landed in IR.  `md_flush` drops an iteration
    // in flight on a reset hold or a bkd_load boundary force (§2.8), the two
    // events that can strand the engine mid-count.
    wire md_start = exec && (mcyc == 2'd0) && (ph == PH_S2P1)
                    && is_muldiv && supported;
`ifdef NU8051_BACKDOOR
    wire md_flush = rst_hold || bkd_load;
    // QUESTION-P6-3 (savestate_design §6, hazard class C-1): the ALU's
    // PARKED flush arm - the one outside its `if (ce)` - may fire only on the
    // sim-only boundary force, never on `rst_hold`.  A level-driven parked
    // clear is a free-running clear: while the core sits in RESET_HOLD with
    // CE parked it would re-zero the MUL/DIV registers on every CLK edge and
    // so undo an SS restore of 0x040-0x042, which the §6 rule ("while CE == 0
    // core state changes only via SS writes") forbids.  Found dynamically by
    // the G4 width sweep, whose seq leg leaves seqstate = 0 = RESET_HOLD.
    wire md_park_flush = bkd_load;
`else
    wire md_flush = rst_hold;
    wire md_park_flush = 1'b0;
`endif

    nu8051_alu u_alu (
        .CLK      (CLK),
        .CE       (CE),
        .md_flush (md_flush),
        .md_park_flush(md_park_flush),
        .md_start (md_start),
        .op       (d_alu_op),
        .a        (alu_a),
        .b        (alu_b),
        .cin      (psw_q[PSW_CY]),
        .acin     (psw_q[PSW_AC]),
        .y        (alu_y),
        .b_out    (alu_b_out),
        .c_out    (alu_c),
        .ac_out   (alu_ac),
        .ov_out   (alu_ov),
        .md_acc_o (md_acc_w),
        .md_cnt_o (md_cnt_w),
        .md_busy_o(md_busy_w),
        .ss_addr  (ss_addr),
        .ss_wdata (ss_wdata),
        .ss_we    (ss_we),
        .ss_rdata (ss_rdata_alu)
    );

    // The operand-side value of an exchange (the half the ALU's single result
    // port cannot carry): XCH writes the old ACC back, XCHD swaps only the low
    // nibbles (§4.1 "XCHD A,@Ri: read Ri, read @Ri, write @Ri").
    wire is_xch  = (d_alu_op == ALU_XCH);
    wire is_xchd = (d_alu_op == ALU_XCHD);
    wire [7:0] mem_wval = is_xch  ? acc_q
                        : is_xchd ? {src_val[7:4], acc_q[3:0]}
                                  : alu_y;

    // C4.7: the two call pushes carry the staged return address instead of an
    // ALU result - the low byte in the S6P1 slot, the high byte at the commit.
    // The SFR bus never sees them: a stack write is indirect by definition
    // (§5.2), so `sfr_wdata` keeps the ALU value unconditionally.
    assign iram_wdata = push_pair ? ((ph == PH_S6P1) ? op1_r : op2_r)
                                  : mem_wval;
    assign sfr_wdata  = mem_wval;
    // an exchange always writes A as well, whatever its `dst` operand kind is;
    // MUL/DIV write the {B,A} PAIR, so their `dst == OPK_B` row still commits A;
    // RLC/RRC resolve d_dst to OPK_C (their writes list is {A,C} and C outranks
    // A in the generator's dst priority), so their accumulator write is
    // likewise implicit in the ALU op (C4.4)
    assign acc_we     = commit && ((d_dst == OPK_A) || is_xch || is_xchd
                                   || is_muldiv || is_rot_c);
    assign acc_wdata  = alu_y;

    // B register (§5.4 dedicated write port): MUL/DIV only.  On DIV by zero the
    // ALU echoes the operands back (IQ2 "A/B unchanged"), so the write arm needs
    // no special case - it stores what is already there.
    assign b_we       = commit && is_muldiv;
    assign b_wdata    = alu_b_out;

    // Stack pointer: PUSH commits SP+1, POP commits SP-1 - except POP SP,
    // where the manual decrements FIRST and then loads the popped value
    // (p 2-64 example: "the Stack Pointer was decremented to 2FH before being
    // loaded with the value popped"; instr §4 "POP SP takes effect after the
    // decrement").  Expressed as destination-write priority, exactly the [D-03]
    // rule for PSW: suppress the side-write and let the explicit direct write
    // through.  QUESTION-P41-1 records the corpus divergence here.
    //
    // C4.7: a CALL commits SP+2 and a RET/RETI commits SP-2 - both by the same
    // rule, and neither has a destination that could contend for SP (their
    // `dst` is the stack itself, never a `direct` byte).
    wire pop_to_sp = is_pop && (dir_dst == SFR_SP);
    assign sp_we    = (commit && (is_push || is_call || is_ret
                                  || (is_pop && !pop_to_sp)))
                    || push_hi_now;
    assign sp_wdata = ilcall2 ? sp_inc2
                    : is_push ? sp_inc
                    : is_call ? sp_inc2
                    : is_ret  ? sp_dec2 : sp_dec;

    // C4.7: RETI re-arms the interrupt level it is returning from - the clear
    // lands at its S6P2 commit (§6.3 "RETI clears the current-level in-progress
    // FF at its S6P2 commit; RET does not", periph §6.6).  `d_reti` is the
    // generated field that tells the two apart (QUESTION-P47-1); the flip-flops
    // themselves are C5.3, so the irq module sinks this pulse for now.
    assign irq_reti = commit && d_reti;

    // ------------------------------------------------------------------
    // C5.3: the interrupt TAKE decision (§6.3, timing §6)
    //
    // Evaluated during every machine cycle and applied at its S6P2 edge.
    // Take iff all four hold:
    //
    //   1. `irq_req` - the S5P2 snapshot of the PREVIOUS cycle, already
    //      resolved by the irq unit against the two in-progress flip-flops
    //      (blocking rule 1, applied there because the flops are there).
    //   2. this edge ends the FINAL cycle of the thing in progress (blocking
    //      rule 2 - "the current polling cycle is not the final cycle in the
    //      execution of the instruction in progress").  An injected LCALL is
    //      a two-cycle shape and obeys the same rule: only ILCALL2 qualifies,
    //      which is exactly what makes Figure 24's nested window land "during
    //      C5 and C6 without any instruction of the lower-priority routine
    //      executing" (timing §6).
    //   3. blocking rule 3 - the instruction completing here is not RETI and
    //      writes neither IE nor IP.  NO "one more instruction" flop is
    //      needed: the denied poll simply re-runs during the next
    //      instruction, whose final cycle allows it (§6.3 point 3, D1.2 §2.8).
    //      RETI is carried by the generated `d_reti` bit, never an opcode
    //      test (QUESTION-P47-1); the IE/IP term is a DESTINATION test over
    //      the decode fields, so it covers every path the manual's "any write
    //      to the IE or IP registers" names - `MOV IE,#d` / `MOV IE,A` /
    //      `ANL IE,#d` / `POP IE` / `XCH A,IP` (direct destination) and
    //      `SETB EA` / `CLR ET0` / `MOV IP.1,C` / `CPL EA` (bit destination,
    //      whose containing byte the §5.3 bit-address unit has already
    //      resolved to A8H / B8H).  It is a decode fact, not a taken-path
    //      one: a JBC on an IE bit is an IE-write instruction whether or not
    //      its condition holds.
    //   4. (rule 1 again, see 1.)
    //
    // ILCALL1/2 carry the PREVIOUS instruction's decode in `ir_r`, so the
    // rule-3 term must be masked out of the ILCALL2 arm - it is a statement
    // about an executing opcode and there is none.
    // ------------------------------------------------------------------
    wire dst_is_ie_ip = ((d_dst == OPK_DIR)
                         && ((dir_dst == SFR_IE) || (dir_dst == SFR_IP)))
                     || ((d_dst == OPK_BIT)
                         && ((bit_byte == SFR_IE) || (bit_byte == SFR_IP)));
    wire irq_block3   = d_reti || dst_is_ie_ip;
    // C5.6: an IDLE cycle is a third eligible polling cycle.  Blocking rule 2
    // ("not the final cycle of the instruction in progress") is vacuous there -
    // no instruction is in progress - and rule 3 is a statement about an
    // executing opcode, so neither applies; rule 1 still does, because it lives
    // in the irq unit's in-progress flip-flops and `irq_req` is already
    // filtered by them.  That is what makes idle exit possible AND keeps an
    // interrupt an ISR has masked out from waking the core (QUESTION-P56-1).
    wire irq_take     = irq_req && ((exec && is_last && !irq_block3) || ilcall2
                                    || idle);

    // periph §7: "any enabled interrupt ... will cause PCON.0 to be cleared
    // by hardware, terminating idle mode".  The clear rides the TAKE, at its
    // S6P2 application edge, and it is NOT conditioned on already being idle
    // (QUESTION-P56-2).  That is the cheap implementation, it is MAME's
    // (`manage_idle_on_interrupt` runs on any taken interrupt), and it is the
    // only one that settles the corner the manual does not mention - a write
    // of IDL on the very edge that vectors - without the pathology the
    // alternative produces: a surviving IDL would halt the core again at the
    // S6P2 of the ISR's FIRST instruction, i.e. inside the routine that was
    // supposed to be servicing the interrupt.
    assign pcon_idl_clr = irq_take && (ph == PH_S6P2);

    // The vectoring acknowledge: one pulse at the S6P2 edge of ILCALL1
    // [D-08] - the instant the source flag clears and the level's
    // in-progress flip-flop sets (§6.3, periph §6.4).
    assign irq_ack      = ilcall1 && (ph == PH_S6P2);
    assign irq_ack_src  = take_src;
    assign irq_ack_prio = take_prio;

    // DPTR destinations (§5.4 dedicated 16-bit write port):
    //   MOV DPTR,#data16 (90H, shape C3): op1 = DPH, op2 = DPL (instr §4)
    //   INC DPTR         (A3H, shape C1): the ISA's only 16-bit increment.
    //     Carry propagates DPL -> DPH by construction; DPH alone is never
    //     touched, and no flag is affected (§4.1 "INC/DEC never touch C").
    assign dptr_we    = commit && (d_dst == OPK_DPTR);
    assign dptr_wdata = (d_alu_op == ALU_INC) ? (dptr_q + 16'd1)
                                              : {op1_r, op2_r};

    // Flag side-writes, masked by the generated d_flagw (§4.1).  Live from
    // C4.2: ADD/ADDC/SUBB carry d_flagw = 3'b111 (C|AC|OV) in the generated
    // ROM, INC/DEC carry 3'b000, so the mask alone decides - no opcode fact is
    // open-coded here.  When a flag-writing instruction ALSO writes PSW
    // directly the destination write wins; that arbitration lives in
    // nu8051_sfr's psw_nxt ([D-03]), not here.
    assign flag_we_c  = commit && d_flagw[0];
    assign flag_c     = alu_c;
    assign flag_we_ac = commit && d_flagw[1];
    assign flag_ac    = alu_ac;
    assign flag_we_ov = commit && d_flagw[2];
    assign flag_ov    = alu_ov;

    // ---- read capture value -------------------------------------------
    // arrival-edge capture (D1.2 F-1 / INV-IRD): the RAM output register is
    // referenced ONLY here, at the arrival edge.
    // the stack read is INDIRECT (§5.2): never the SFR bus, and 8051-mode
    // reads above 7FH return the PQ5 constant
    wire [7:0] r1_cap = stk_rd ? ((!P_8052 && sp_q[7]) ? NU8051_CFG_UNIMPL_RD
                                                       : iram_rdata)
                      : (r1_addr[7] ? sfr_rdata : iram_rdata);
    // the C2 / late slots serve a `direct` operand or a bit operand's
    // containing byte; both resolve space by bit 7 of the resolved BYTE
    // address, which the bit-address unit has already produced (§5.3)
    wire [7:0] rd_c2_cap   = rd_c2_addr[7]   ? sfr_rdata : iram_rdata;
    wire [7:0] rd_late_cap = rd_late_addr[7] ? sfr_rdata : iram_rdata;
    wire [7:0] deref_cap = (!P_8052 && rd1_data[7]) ? NU8051_CFG_UNIMPL_RD
                                                    : iram_rdata;
    // C4.7: the second pop (SP-1) is indirect like the first and takes the same
    // PQ5 substitution, judged by ITS OWN address - a `RET` with SP == 80H on
    // the 8051 reads the constant for PCH (80H) and IRAM[7FH] for PCL.  The
    // g_stack SP=7Fh/FFh rows and the seq corpus' wrap cases check both halves.
    wire [7:0] pop_lo_cap = (!P_8052 && sp_dec[7]) ? NU8051_CFG_UNIMPL_RD
                                                   : iram_rdata;

    // ------------------------------------------------------------------
    // tick strobes + observation
    // ------------------------------------------------------------------
    assign ph_o       = ph;
    assign mcyc_o     = mcyc;
    assign seqstate_o = seqstate;
    assign pc_o       = pc;
    assign rst_hold_o = rst_hold;
    // The PRIME cycle is a fetch-pipeline priming cycle, not an executed
    // machine cycle: at a real instruction boundary the slot-B opcode fetch
    // was launched by the PREVIOUS instruction's final cycle, whose peripheral
    // tick was already counted.  Ticking the timers again during PRIME would
    // double-count one machine cycle against every injected vector (and
    // against the reset priming cycle R1), so the timer apply tick - the only
    // strobe that advances architectural peripheral state - is suppressed in
    // PRIME and RESET_HOLD and runs in EVERY OTHER seqstate.  That exclusion
    // form, not an `exec` test, is what core_design §6.1 [D-10] requires: the
    // interrupt-LCALL cycles (ILCALL1/2) are ordinary machine cycles and IDLE
    // explicitly keeps the timer/serial/interrupt clocks running (periph §7),
    // so those states must tick the moment C5.3/C5.5 introduce them - with no
    // edit here.  Today the three live states make this identical to `exec`.
    // The sampling strobes DO run in PRIME so the pin-sample registers hold
    // the boundary's pin levels (core_design §2.8 "no phantom edge").
    // C5.5 / TQ8: the 8052's TF2 is the one flag the manual sets at S2P2
    // ("the Timer 2 flag TF2 is set at S2P2 and is polled in the same cycle in
    // which the timer overflows", 3-25).  It is an APPLY-class strobe like
    // S3P1 - the flag it transfers is produced by a count - so it carries the
    // same PRIME suppression.
    // C5.6: the six strobes need NO idle term - "the internal clock signal is
    // gated off to the CPU, but not to the interrupt, timer, and serial port
    // functions" (periph §7) is exactly the exclusion form above, which is
    // what [D-10] anticipated.  POWER DOWN is the opposite case ("the
    // oscillator is stopped ... all functions are stopped"): `ph` is frozen
    // there, so a strobe would otherwise stick high on whatever phase the
    // freeze caught.  The gate is the HELD PD bit, not `pd_freeze`, so no tick
    // escapes in the tick between RESET arriving and RESET_HOLD applying.
    assign tk_s2p2    = (ph == PH_S2P2) && !prime && !rst_hold && !pcon_pd;
    assign tk_s3p1    = (ph == PH_S3P1) && !prime && !rst_hold && !pcon_pd;
    assign tk_s5p1    = (ph == PH_S5P1) && !rst_hold && !pcon_pd;
    assign tk_s5p2    = (ph == PH_S5P2) && !rst_hold && !pcon_pd;
    assign tk_s6p1    = (ph == PH_S6P1) && !rst_hold && !pcon_pd;
    assign tk_s6p2    = (ph == PH_S6P2) && !rst_hold && !pcon_pd;
    assign bus_active  = bus_seen;
    assign prime_cycle = prime;
    assign retire_o   = exec && is_last && (ph == PH_S6P2) && !pd_freeze;

    // state fold for the CE-low watchdog / CE-duty sweep (§1.9): a pure
    // combinational function of CE-GATED state only.  Deliberately excludes
    // every input-driven or park-clobbered value (MEM_DIN, ROM_DATA, pins,
    // the IRAM output register, bkd_rdata).
    assign hash_o = {pc, ir_r, 4'b0, ph}
                  ^ {op1_r, op2_r, rd1_data, rd2_data}
                  ^ {MEM_ADDR, 8'b0, MEM_ALE, MEM_PSEN_N, MEM_RD_N, MEM_WR_N,
                     fp_valid, fp_ext, fp_consume, bus_seen}
                  ^ {14'b0, fp_dst, 16'(ROM_ADDR)}
                  ^ {26'b0, ea_latched, rst_s5p2, mcyc, 2'b0}
                  ^ {29'b0, seqstate}
                  // C4.3: the serial MUL/DIV unit is the first multi-cycle
                  // internal engine, so its iteration state has to be inside
                  // the CE-low watchdog's digest (§1.9 / CAD-12) - a park
                  // between two iteration ticks must be invisible.
                  ^ {md_acc_w, 11'b0, md_busy_w, md_cnt_w}
                  // C4.8: the three MOVX staging registers are CE-gated
                  // state that lives across up to 9 ticks of a machine cycle
                  // (movx_din from S3P2 to the S6P2 commit), so a park inside
                  // the MOVX window has to be invisible in the digest too
                  // (§1.9 / CAD-12).  They are registers, not bus inputs -
                  // MEM_DIN itself stays out of the fold.
                  ^ {16'b0, movx_dout, movx_din}
                  // C5.3: the take latch is CE-gated state that lives across
                  // both injected cycles, so a park inside an ILCALL has to
                  // be invisible in the digest too (§1.9 / CAD-12).  The irq
                  // unit's own flops join the core-level fold (§1.9).
                  ^ {28'b0, take_prio, take_src};

    // ------------------------------------------------------------------
    // C4.8: the external address/data bus outputs (§1.2 rows)
    // ------------------------------------------------------------------
    // MEM_ADDR carries the xdata address by being LOADED with it at the
    // end-S4P1(C1) launch edge, in the same output register and by the same
    // one-address-per-slot rule the fetch slots use.  It then holds - nothing
    // else launches until the end-S4P1(C2) slot-B fetch, which is exactly
    // where the window ends (§1.2 "valid from the edge ending S4P1(C1)
    // through the edge ending S4P1(C2)") - so the address is stable across
    // the whole RD#/WR# window by construction (CAD-8) with no second copy.
    // MEM_DOUT: the write-data backing register, loaded at the edge ending
    // S6P1(C1) so the byte is valid throughout S6P2(C1)..S4P1(C2) ("just
    // before WR# is activated ... until after WR# is deactivated", timing
    // §4); don't-care outside that window, so it simply holds (§1.2).
    assign MEM_DOUT = movx_dout;

    // the xdata address itself: DPH:DPL, or {P2 latch, Ri} for the @Ri forms
    // (TQ10 - the P2 LATCH, not the pin sample and not P2_OUT).  `rd1_data`
    // holds the pointer register from the R1 slot of C1.
    wire [15:0] movx_xaddr = movx_ri ? {p2_q, rd1_data} : dptr_q;

    // ------------------------------------------------------------------
    // save-state read mux (registered, savestate_design §5.1)
    //
    // 26 arms = the whole SEQ region.  Explicit zero-extension everywhere so
    // a widened register fires a Verilator width warning (the §4 width-drift
    // lint).  ROM_ADDR is mapped at the ROM_AW superset width 13: on a
    // ROM_AW = 12 build bit 12 reads 0 and is ignored on write, which is what
    // `16'(ROM_ADDR)` / `ss_wdata[ROM_AW-1:0]` give for free.
    // ------------------------------------------------------------------
    always_ff @(posedge CLK) begin
        case (ss_addr)
            SSA_S_PH:          ss_rdata <= {12'b0, ph};
            SSA_S_SEQSTATE:    ss_rdata <= {13'b0, seqstate};
            SSA_S_MCYC:        ss_rdata <= {14'b0, mcyc};
            SSA_S_PC:          ss_rdata <= pc;
            SSA_S_IR:          ss_rdata <= {8'b0, ir_r};
            SSA_S_OP1:         ss_rdata <= {8'b0, op1_r};
            SSA_S_OP2:         ss_rdata <= {8'b0, op2_r};
            SSA_S_EA_LATCHED:  ss_rdata <= {15'b0, ea_latched};
            SSA_S_RST_S5P2:    ss_rdata <= {15'b0, rst_s5p2};
            SSA_S_FP_VALID:    ss_rdata <= {15'b0, fp_valid};
            SSA_S_FP_EXT:      ss_rdata <= {15'b0, fp_ext};
            SSA_S_FP_CONSUME:  ss_rdata <= {15'b0, fp_consume};
            SSA_S_FP_DST:      ss_rdata <= {14'b0, fp_dst};
            SSA_S_RD1_DATA:    ss_rdata <= {8'b0, rd1_data};
            SSA_S_RD2_DATA:    ss_rdata <= {8'b0, rd2_data};
            SSA_S_MOVX_DOUT:   ss_rdata <= {8'b0, movx_dout};
            SSA_S_MOVX_DIN:    ss_rdata <= {8'b0, movx_din};
            SSA_S_TAKE_SRC:    ss_rdata <= {13'b0, take_src};
            SSA_S_TAKE_PRIO:   ss_rdata <= {15'b0, take_prio};
            SSA_S_MEM_ADDR:    ss_rdata <= MEM_ADDR;
            SSA_S_ROM_ADDR:    ss_rdata <= 16'(ROM_ADDR);
            SSA_S_ALE:         ss_rdata <= {15'b0, MEM_ALE};
            SSA_S_PSEN_N:      ss_rdata <= {15'b0, MEM_PSEN_N};
            SSA_S_RD_N:        ss_rdata <= {15'b0, MEM_RD_N};
            SSA_S_WR_N:        ss_rdata <= {15'b0, MEM_WR_N};
            SSA_S_BUS_SEEN:    ss_rdata <= {15'b0, bus_seen};
            default:           ss_rdata <= 16'h0000;
        endcase
    end

    // ------------------------------------------------------------------
    // main clocked block - every flop CE-gated (§2.7)
    // ------------------------------------------------------------------
    always_ff @(posedge CLK) begin
        // ---- save-state write decode (§5.2): the restore-priority arm, at
        // the top of the block and above every reset/CE arm.  `SS_WE && CE`
        // and `SS_WE && RESET` are both illegal (A-SS1/A-SS2), so the arms
        // this swallows are no-ops by contract; the priority exists to make
        // a restore deterministic in simulation.
        if (ss_we) begin
            case (ss_addr)
                SSA_S_PH:         ph         <= ss_wdata[3:0];
                SSA_S_SEQSTATE:   seqstate   <= ss_wdata[2:0];
                SSA_S_MCYC:       mcyc       <= ss_wdata[1:0];
                SSA_S_PC:         pc         <= ss_wdata;
                SSA_S_IR:         ir_r       <= ss_wdata[7:0];
                SSA_S_OP1:        op1_r      <= ss_wdata[7:0];
                SSA_S_OP2:        op2_r      <= ss_wdata[7:0];
                SSA_S_EA_LATCHED: ea_latched <= ss_wdata[0];
                SSA_S_RST_S5P2:   rst_s5p2   <= ss_wdata[0];
                SSA_S_FP_VALID:   fp_valid   <= ss_wdata[0];
                SSA_S_FP_EXT:     fp_ext     <= ss_wdata[0];
                SSA_S_FP_CONSUME: fp_consume <= ss_wdata[0];
                SSA_S_FP_DST:     fp_dst     <= ss_wdata[1:0];
                SSA_S_RD1_DATA:   rd1_data   <= ss_wdata[7:0];
                SSA_S_RD2_DATA:   rd2_data   <= ss_wdata[7:0];
                SSA_S_MOVX_DOUT:  movx_dout  <= ss_wdata[7:0];
                SSA_S_MOVX_DIN:   movx_din   <= ss_wdata[7:0];
                SSA_S_TAKE_SRC:   take_src   <= ss_wdata[2:0];
                SSA_S_TAKE_PRIO:  take_prio  <= ss_wdata[0];
                SSA_S_MEM_ADDR:   MEM_ADDR   <= ss_wdata;
                SSA_S_ROM_ADDR:   ROM_ADDR   <= ss_wdata[ROM_AW-1:0];
                SSA_S_ALE:        MEM_ALE    <= ss_wdata[0];
                SSA_S_PSEN_N:     MEM_PSEN_N <= ss_wdata[0];
                SSA_S_RD_N:       MEM_RD_N   <= ss_wdata[0];
                SSA_S_WR_N:       MEM_WR_N   <= ss_wdata[0];
                SSA_S_BUS_SEEN:   bus_seen   <= ss_wdata[0];
                default: ;
            endcase
        end else if (CE && pd_freeze) begin
            // ---- §7 power down ---------------------------------------------
            // "The oscillator is stopped ... all functions are stopped, the
            // contents of the on-chip RAM and the Special Function Registers
            // are maintained ... the only exit ... is a hardware reset."
            //
            // The CE-domain reading (QUESTION-P56-3): every FUNCTIONAL flop
            // freezes - this block does nothing, and the top level takes CE
            // away from every peripheral - while `ph` keeps counting, because
            // §7.1 [D-09] already says the phase counter "free-runs under CE
            // at all times, including during reset hold - this gives RESET its
            // architectural sampling cadence".  Power down is the second state
            // that needs exactly that: RESET is recognised on its ordinary
            // S5P2 sample here, with no rule of its own, and the tick strobes
            // are gated on `pcon_pd` so a free-running `ph` moves nothing.
            ph <= ph_next;
            if (ph == PH_S5P2) begin
                rst_s5p2 <= RESET;
                if (RESET) seqstate <= SS_SEQST_RESET_HOLD;
            end
        end else if (CE) begin
            ph <= ph_next;

            if (rst_hold) begin
                // ---- §7.1 reset hold: ph free-runs, bus idle -----------
                mcyc       <= 2'd0;
                pc         <= 16'h0000;
                ir_r       <= 8'h00;
                op1_r      <= 8'h00;
                op2_r      <= 8'h00;
                rd1_data   <= 8'h00;
                rd2_data   <= 8'h00;
                fp_valid   <= 1'b0;
                fp_ext     <= 1'b0;
                fp_consume <= 1'b0;
                fp_dst     <= FPD_IR;
                bus_seen   <= 1'b0;
                movx_dout  <= 8'h00;      // §2.8: MOVX staging clears
                movx_din   <= 8'h00;
                take_src   <= 3'd0;       // C5.3: no vectoring in flight
                take_prio  <= 1'b0;
                MEM_ALE    <= 1'b1;       // §1.2 forced 1 during reset hold
                MEM_PSEN_N <= 1'b1;
                MEM_RD_N   <= 1'b1;
                MEM_WR_N   <= 1'b1;
                if (ph == PH_S5P2) begin
                    rst_s5p2   <= RESET;
                    ea_latched <= EA_N;   // TQ7: last capture wins
                end
                if (ph == PH_S6P2) begin
                    // release: the tick after this edge is S1P1 of R1, the
                    // priming cycle (TQ5 [D-09]: exactly one priming cycle)
                    if (!rst_s5p2 && !RESET) begin
                        seqstate <= SS_SEQST_PRIME;
                        MEM_ALE  <= 1'b0;
                    end
                end
            end else if (idle) begin
                // ---- §7 idle: the CPU clock is gated off ------------------
                // Nothing fetches, nothing executes, `pc` holds the address of
                // the instruction FOLLOWING the one that set IDL - which is
                // therefore what the terminating interrupt's LCALL pushes, so
                // "after RETI execution resumes with the instruction following
                // the one that put the device into idle" is a property of the
                // existing ILCALL machinery rather than a special case.
                //
                // The peripheral strobes are untouched (they are gated on
                // PRIME/RESET_HOLD/PD only), so timers, the serial port and
                // the interrupt sampler keep running exactly as §7 requires -
                // that is the whole point of the state.
                if (ph == PH_S1P1) begin
                    // "ALE and PSEN# hold at logic high levels" (periph §7).
                    // The slot-B PSEN# window of the entering instruction ends
                    // where it always would; ALE is then DRIVEN high and left
                    // there, since no slot ever launches to lower it again.
                    MEM_PSEN_N <= 1'b1;
                    MEM_ALE    <= 1'b1;
                    // the opcode the entering instruction's slot B already had
                    // in flight is dropped, with no pc increment - the same
                    // drop an ILCALL performs, and for the same reason: on
                    // resume the next instruction is re-fetched from `pc`
                    fp_valid   <= 1'b0;
                end
                if (ph == PH_S5P2) begin
                    // exit 2: hardware reset.  Idle keeps the phase counter,
                    // so RESET is recognised on its ordinary S5P2 sample here
                    // (unlike power down - QUESTION-P56-3)
                    rst_s5p2 <= RESET;
                    if (RESET) seqstate <= SS_SEQST_RESET_HOLD;
                end
                if (ph == PH_S6P2) begin
                    // exit 1: any interrupt the poll admits.  `irq_take`
                    // carries the idle arm; the vectoring itself is the
                    // ordinary two-cycle injected LCALL.
                    if (irq_take) begin
                        seqstate  <= SS_SEQST_ILCALL1;
                        mcyc      <= 2'd0;
                        take_src  <= irq_src;
                        take_prio <= irq_prio;
                        // ALE was DRIVEN high for the duration of idle; the
                        // clock is about to restart, so it returns to its
                        // ordinary waveform here - low through the S1P1 of the
                        // resuming cycle, high S1P2..S2P1 with the slot-A
                        // launch (timing §2 / CAD-1/2).  The manual fixes the
                        // level during idle and says nothing about the exit
                        // edge; this is the only choice that leaves the
                        // cadence law intact (QUESTION-P56-4).
                        MEM_ALE   <= 1'b0;
                    end
                end
            end else begin
                case (ph)
                    // ---- edge ending S1P1 --------------------------------
                    PH_S1P1: begin
                        // C5.3: `consume_now` (not the raw pipeline bits)
                        // drops the byte in flight at ILCALL entry
                        if (consume_now) begin
                            unique case (fp_dst)
                                FPD_IR:  ir_r  <= fetch_byte;
                                FPD_OP2: op2_r <= fetch_byte;
                                default: op1_r <= fetch_byte;
                            endcase
                        end
                        pc <= pc_adv;
                        // C4.8: MOVX C2 has NO slot A (§1.3): no launch, so
                        // no ALE-A pulse (the one skipped ALE, timing §2) and
                        // nothing for the S2P2 edge to open a PSEN# window
                        // with (the second TQ3 window).  The cycle still
                        // drives the bus - RD#/WR# went low at the previous
                        // edge - so TQ4's P0 clobber is armed here.
                        if (movx_c2) begin
                            fp_valid <= 1'b0;
                            bus_seen <= 1'b1;
                        end else begin
                            // slot-A launch (address = pc after this consume)
                            fp_valid   <= 1'b1;
                            fp_ext     <= slot_ext(pc_adv);
                            fp_consume <= sa_consume;
                            fp_dst     <= FPD_OP1;
                            if (slot_ext(pc_adv)) begin
                                MEM_ADDR <= pc_adv;
                                bus_seen   <= 1'b1;
                            end else
                                ROM_ADDR <= pc_adv[ROM_AW-1:0];
                            MEM_ALE <= 1'b1;     // ALE-A high S1P2..S2P1
                        end
                        MEM_PSEN_N <= 1'b1;      // slot-B PSEN window ends
                    end
                    // ---- edge ending S1P2 -------------------------------
                    PH_S1P2: ;                   // read #1 address presented
                    // ---- edge ending S2P1 -------------------------------
                    PH_S2P1: begin
                        MEM_ALE <= 1'b0;
                        // C4.7: RET/RETI stage their popped bytes in op2/op1
                        // (D1.2 F-5) - PCH here, PCL at the next edge - so the
                        // ordinary staging flops are not disturbed.  op2/op1
                        // are unused by these 1-byte shapes: no fetch arrival
                        // in either of their cycles targets an operand
                        // register.
                        if (r1_en) begin
                            if (ret_pop) op2_r    <= r1_cap;
                            else         rd1_data <= r1_cap;
                        end
                        if (rd_c2_en) rd2_data <= rd_c2_cap;
                    end
                    // ---- edge ending S2P2 -------------------------------
                    PH_S2P2: begin
                        if (r2_en) begin
                            if (ret_pop) op1_r    <= pop_lo_cap;
                            else         rd2_data <= deref_cap;
                        end
                        MEM_PSEN_N <= ~(fp_valid && fp_ext);   // PSEN#-A
                    end
                    PH_S3P1: ;
                    // ---- edge ending S3P2 -------------------------------
                    // C4.8: the MOVX strobe deasserts here (6 CE ticks low,
                    // S1P1..S3P2, §1.3) and the read data is sampled AT this
                    // edge - TQ1's "just before the read strobe is
                    // deactivated" = the RD# rising edge, the latest instant
                    // the text allows and the one that maximises external
                    // settle time.  `movx_din` holds it to the S6P2 commit
                    // (F-2); the write forms sample nothing.
                    PH_S3P2: begin
                        if (movx_c2) begin
                            MEM_RD_N <= 1'b1;
                            MEM_WR_N <= 1'b1;
                            if (movx_is_rd) movx_din <= MEM_DIN;
                        end
                    end
                    // ---- edge ending S4P1 -------------------------------
                    PH_S4P1: begin
                        if (consume_now) begin
                            unique case (fp_dst)
                                FPD_IR:  ir_r  <= fetch_byte;
                                FPD_OP2: op2_r <= fetch_byte;
                                default: op1_r <= fetch_byte;
                            endcase
                        end
                        // C4.6: a control transfer loads the PC at THIS edge -
                        // the S4P1 of its final cycle (§2.5), which is also the
                        // slot-B launch below, so the next opcode fetch goes to
                        // the target with no dead slot.  Not taken: the PC is
                        // simply not loaded and the cycle count is unchanged.
                        pc <= xfer_now ? xfer_target : pc_adv;
                        // C4.7: a CALL stages the return address it is about to
                        // lose into op2/op1 for the S6P1/S6P2 pushes.  Mutually
                        // exclusive with the arrival capture above (slot A of a
                        // call's final cycle is a discarded fetch).
                        if (call_stage) begin
                            op1_r <= pc_adv[7:0];       // pushed at S6P1
                            op2_r <= pc_adv[15:8];      // pushed at S6P2
                        end
                        MEM_PSEN_N <= 1'b1;      // slot-A PSEN window ends
                        // slot-B launch (MOVC C1: the code-table address
                        // A+DPTR / A+PC instead of pc - [D-01]; the base for
                        // @A+PC is the address of the NEXT instruction, i.e.
                        // pc after the opcode consume, instr §4)
                        // C4.8: in MOVX C1 this slot carries the xdata
                        // ADDRESS instead of a fetch (§1.3).  `fp_valid` 0 is
                        // what suppresses the S5P2 PSEN#-B assert - the first
                        // TQ3 window (S6P1(C1)-S1P1(C2)) - and the xdata bus
                        // is external whatever EA# says, so `bus_seen` (TQ4)
                        // is unconditional here.  ALE-B still pulses: it is
                        // the pulse that latches the address (timing §4).
                        if (movx_c1) begin
                            fp_valid  <= 1'b0;
                            MEM_ADDR  <= movx_xaddr;
                            bus_seen  <= 1'b1;
                        end else begin
                            fp_valid   <= 1'b1;
                            fp_ext     <= slot_ext(sb_addr);
                            fp_consume <= sb_consume;
                            fp_dst     <= sb_dst;
                            if (slot_ext(sb_addr)) begin
                                MEM_ADDR <= sb_addr;
                                bus_seen   <= 1'b1;
                            end else
                                ROM_ADDR <= sb_addr[ROM_AW-1:0];
                        end
                        MEM_ALE    <= 1'b1;      // ALE-B high S4P2..S5P1
                    end
                    PH_S4P2: ;                   // late read address presented
                    // ---- edge ending S5P1 -------------------------------
                    PH_S5P1: begin
                        MEM_ALE <= 1'b0;
                        if (rd_late_en) rd2_data <= rd_late_cap;
                    end
                    // ---- edge ending S5P2 -------------------------------
                    PH_S5P2: begin
                        rst_s5p2   <= RESET;
                        MEM_PSEN_N <= ~(fp_valid && fp_ext);   // PSEN#-B
                        if (RESET) seqstate <= SS_SEQST_RESET_HOLD;
                    end
                    // ---- edge ending S6P1 -------------------------------
                    // C4.8: the write byte reaches MEM_DOUT here so it is
                    // valid throughout S6P2(C1) - "just before WR# is
                    // activated" (timing §4) - and stays until after the
                    // strobe rises.  ACC is the source in every write form
                    // (d_src2 == OPK_A) and nothing has committed to it this
                    // cycle, so the live tap is the architectural value.
                    PH_S6P1: begin
                        if (movx_c1 && movx_is_wr) movx_dout <= acc_q;
                    end
                    // ---- edge ending S6P2: architectural commit (TQ6) ----
                    PH_S6P2: begin
                        bus_seen <= 1'b0;
                        // C4.8: the strobes assert at the C1->C2 boundary
                        // edge and stay low for S1P1..S3P2 of C2 (§1.3).
                        if (movx_c1) begin
                            if (movx_is_rd) MEM_RD_N <= 1'b0;
                            else            MEM_WR_N <= 1'b0;
                        end
                        // C5.3: the interrupt-take decision is applied at
                        // this edge (§2.2 S6P2 row, §6.3).  It outranks the
                        // ordinary advance because taking means the NEXT two
                        // cycles are the injected LCALL whatever was about to
                        // happen - including out of ILCALL2 itself, which is
                        // the Figure-24 nested-vectoring case.
                        if (irq_take) begin
                            seqstate  <= SS_SEQST_ILCALL1;
                            mcyc      <= 2'd0;
                            take_src  <= irq_src;
                            take_prio <= irq_prio;
                        // C5.6 / periph §7: the power-control latches, tested
                        // on the POST-commit value at the edge that writes
                        // them, and only where an instruction actually
                        // completes.  PD outranks IDL (`idle_want` carries the
                        // `!pd` term), and both outrank the ordinary advance
                        // because the instruction that sets them is "the last
                        // instruction executed before entering" the mode.
                        end else if (exec && is_last && pcon_pd_eff) begin
                            // "ALE and PSEN# output lows" (periph §7); the
                            // freeze itself engages at the next tick, in the
                            // `pcon_pd` branch at the top of this block.
                            MEM_ALE    <= 1'b0;
                            MEM_PSEN_N <= 1'b0;
                            MEM_RD_N   <= 1'b1;
                            MEM_WR_N   <= 1'b1;
                            mcyc       <= 2'd0;
                        end else if (exec && is_last && idle_want) begin
                            seqstate <= SS_SEQST_IDLE;
                            mcyc     <= 2'd0;
                        end else if (ilcall1) begin
                            seqstate <= SS_SEQST_ILCALL2;
                            mcyc     <= 2'd1;
                        end else if (ilcall2) begin
                            seqstate <= SS_SEQST_EXEC;
                            mcyc     <= 2'd0;
                        end else if (prime) begin
                            seqstate <= SS_SEQST_EXEC;
                            mcyc     <= 2'd0;
                        end else if (is_last)
                            mcyc <= 2'd0;
                        else
                            mcyc <= mcyc + 2'd1;
                    end
                    default: ;                   // S3P1/S4P2
                endcase
            end
        end
`ifdef NU8051_BACKDOOR
        else if (bkd_load) begin
            // canonical instruction-boundary state (§2.8)
            ph         <= 4'd0;                 // S1P1 of a PRIME cycle
            seqstate   <= SS_SEQST_PRIME;
            mcyc       <= 2'd0;
            pc         <= bkd_pc;
            ir_r       <= 8'h00;
            op1_r      <= 8'h00;
            op2_r      <= 8'h00;
            rd1_data   <= 8'h00;
            rd2_data   <= 8'h00;
            fp_valid   <= 1'b0;
            fp_ext     <= 1'b0;
            fp_consume <= 1'b0;
            fp_dst     <= FPD_IR;
            bus_seen   <= 1'b0;
            movx_dout  <= 8'h00;
            movx_din   <= 8'h00;
            take_src   <= 3'd0;          // §2.8 / D1.2 §3.2: SSA_S_TAKE_* = 0
            take_prio  <= 1'b0;
            rst_s5p2   <= 1'b0;
            ea_latched <= EA_N;
            MEM_ALE    <= 1'b0;
            MEM_PSEN_N <= 1'b1;
            MEM_RD_N   <= 1'b1;
            MEM_WR_N   <= 1'b1;
        end
`endif
    end

    // ------------------------------------------------------------------
    // PHASE4-TODO trap (sim only): an opcode outside the Phase-3 supported
    // set reached its commit edge.  Its fetch cadence and cycle count are
    // correct; its architectural effects are suppressed (deterministic
    // NOP-like behaviour), so a stray fetch cannot corrupt state.  Reported
    // ONCE PER OPCODE so the vector tail's phantom instruction (which the TB
    // deliberately starts past the last retire, QUESTION-T22-1) cannot flood
    // the log.
    // ------------------------------------------------------------------
`ifndef SYNTHESIS
    // verilator lint_off UNUSEDSIGNAL
    logic todo_seen [0:255];
    initial for (int i = 0; i < 256; i++) todo_seen[i] = 1'b0;
    always_ff @(posedge CLK) begin
        if (CE && exec && is_last && (ph == PH_S6P2) && !supported) begin
            if (!todo_seen[ir_r]) begin
                todo_seen[ir_r] <= 1'b1;
                // NOTE: $display, not $error.  Under Verilator's mandatory
                // --assert a runtime $error is an assertion failure and stops
                // the simulation, and the TB records ONE machine cycle past
                // every vector's last retire (QUESTION-T22-1), which always
                // starts a phantom instruction whose opcode is whatever byte
                // follows in the code window - normally a non-pilot opcode.
                // An aborting trap would therefore kill every legitimate run.
                $display("PHASE4-TODO: opcode %02Xh (shape %0d class %0d) decoded but not executed by the Phase-3 sequencer; architectural commit suppressed", ir_r, d_shape, d_class);
            end
        end
    end
    // verilator lint_on UNUSEDSIGNAL
`endif

    initial begin
        take_src = 3'd0; take_prio = 1'b0;
        ph = 4'd0; mcyc = 2'd0; seqstate = SS_SEQST_RESET_HOLD;
        pc = 16'h0000; ir_r = 8'h00; op1_r = 8'h00; op2_r = 8'h00;
        rd1_data = 8'h00; rd2_data = 8'h00;
        fp_valid = 1'b0; fp_ext = 1'b0; fp_consume = 1'b0; fp_dst = FPD_IR;
        ea_latched = 1'b0; rst_s5p2 = 1'b1; bus_seen = 1'b0;
        movx_dout = 8'h00; movx_din = 8'h00;
        MEM_ADDR = 16'h0000; ROM_ADDR = '0;
        MEM_ALE = 1'b1; MEM_PSEN_N = 1'b1; MEM_RD_N = 1'b1; MEM_WR_N = 1'b1;
        ss_rdata = 16'h0000;
    end

endmodule
