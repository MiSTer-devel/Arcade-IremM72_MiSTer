//============================================================================
// Imported from nec_test (https://github.com/.../nec_test)
// Source: hdl/rtl/core, commit 595e2c4bbf7e474e3240eb25ff94515385c807ca
// Cycle-accurate NEC V30 (uPD70116) max-mode core, verified vs golden traces.
// Do not hand-edit; re-import from upstream. No license header upstream (same author).
//============================================================================

//============================================================================
//
//  v30_eu - V30 (uPD70116) execution unit, Campaign 3 opcode tranche
//
//  Microsequenced FSM whose per-state timing is written directly against
//  the golden-trace event schedules (tests/v30/v0.1, summarized by
//  sw/trace_stats.py). Cycle offsets below are relative to the
//  instruction's first-byte queue pop (F @ 0), prefetched (saturated)
//  variants:
//
//   pops:    modrm @ 1; disp8 @ 3; disp16 lo @ 2, hi @ 4; imm16 @ 2,3
//   EU bus requests become visible to the BIU commit evaluator at the
//   end of the cycle in which eu_req/eu_ready are asserted:
//     loads  (incl. RMW read):  d0/d1: ready @ 4     d2: req @ 4, ready @ 5
//     stores (MOV 88/89):       d0: ready @ 4   d1: req @ 4, ready @ 5
//                               d2: req @ 5-6, ready @ 7
//     PUSH: ready @ 3    POP: ready @ 2
//   post-access:
//     MOV/ALU load: writeback 2 cycles after the access's final T4, F +1
//     store/PUSH/POP: next F directly after the final T4
//     RMW writeback ready: ALU done+3, INC/DEC (FE) done+4, shift (D0)
//     done+6
//     MULU8 exec: EX at cycle 23 (reg) / done+23 (mem)
//     DIVU16:     EX at cycle 27 (reg) / done+27 (mem)
//     IDIV (F6.7/F7.7, mission I; dly relative to the first wait cycle,
//     mem forms +1, s = 3 extra cycles when the dividend is negative):
//       early trap (den=0 or |num_hi| >= |den|): IVT ready @ dly 21+s
//       (byte AND word); late trap (|q| > 2^(n-1)-1, symmetric): byte
//       36+s, word 44+s; non-trap EX: byte 37+s, word 44+s. Divisor and
//       quotient signs cost nothing - only the dividend negate does.
//     DIVU16 trap: IVT read ready at cycle 14 (reg) / done+14 (mem);
//     IVT offset/segment words read back-to-back; PSW push ready
//     IVTdone+3; PS push ready PSWdone+2; the cycle after PSdone raises
//     the queue flush (QS=E) together with the PC push request; the
//     handler prefetch then wins the slot after the PC push by itself.
//
//  Implemented opcodes (PARTIAL snapshot - the op_* decode wires in the
//  decode section below are the authoritative implemented set; the core
//  now covers 300+ forms and this list is no longer exhaustive):
//  00-3B (all 32 ALU r/m byte/word/direction forms), 40-4F,
//  50-57, 58-5F, 86/87 (XCHG), 88/89/8A/8B, 8C/8E (sreg MOV), 8D
//  (LDEA), 90, 98/99 (CVTBW/CVTWL), 9B (POLL), 9D (POP PSW), A0-A3
//  (acc moffs), A4/A5/AA/AB/AC/AD (MOVBK/STM/LDM singles), B8-BF,
//  D0/4, D7 (TRANS), E4/E5/EC/ED (IN), F4 (HALT), F6/4, F6/7 F7/7
//  (IDIV), F7/6, FA/FB (DI/EI), FE/0, the prefixes 26/2E/36/3E
//  (segment override) and F3 (REP), the V30 0F forms 0F18 (TEST1
//  rm8,imm3), 0F20 (ADD4S), 0F28 (ROL4 rm8), 0F 31/33/39/3B
//  (INS/EXT bit-field, mod3; laws at the op_insext decode comment),
//  and control flow EB/E9
//  (BR), 70-7F (full Bcc set), E2 (DBNZ), E8 (CALL near), C3/C2 (RET).
//  Unknown opcodes park the sequencer (S_HALT). 0F forms pop the
//  second byte at F+2 and the modrm at F+3; standard EA machinery.
//
//  Block 4 (missions L/M/N): INT/NMI recognition at instruction
//  boundaries, the INTA pair + trap-chain vectoring, HALT entry/wake,
//  POLL, IN, EI/DI/POP-PSW IE laws, REP interruption - all timing per
//  docs/facts/interrupt_model.md (fitted against the 15 interrupt
//  tranches + 4 IN tranches, tests/v30/v0.1).
//
//  Mission J laws (per-form goldens, 500 cases each, all cycle-exact):
//   - prefixes retire as their own instruction (own F pop, 2 cycles);
//     the override/REP latch lives until the prefixed instruction
//     retires. A segment override absorbs into the EA machinery
//     (ea_seg_sel mux) at no extra cost - measurements.md law.
//   - moffs forms (A0-A3): address pops @2/3, reservation during the
//     hi pop (like disp pops), access ready hi+1, loads retire at
//     done, stores at done (store data = acc pair).
//   - sreg MOVs: reg forms retire in 2 cycles (S_DEC); 8C mem follows
//     the READER reservation+ready schedule (d0/d1 @4, d2 @5); 8E
//     writeback at done+1 (one faster than 8B).
//   - LDEA: no bus access or reservation; mod0 retires at cycle 2,
//     disp forms at their final disp pop.
//   - TRANS: BX+AL byte read ready @3, retire at done (9 cycles).
//   - string singles: access ready @3 (LDM/STM), retire at done;
//     MOVBK queues its WRITE while the read is in flight - it commits
//     at the read's own T3 edge with the data forwarded inside the
//     BIU (eu_fwd), retire at write done (13 cycles).
//   - REP: first access 2 cycles later than the single form (the
//     extra wait does not reserve the bus); iterations chain via
//     requests raised during the running access (slopes 8/4); CW=0
//     early-out retires at pop+11; REP STM CW=1 retires at the pop+12
//     slot (or completion if later), all other REP retires done+1.
//
//  Control-flow timing (mission E; see docs/facts/biu_model.md for the
//  unified flush law): internal flush X = lastpop+3 (EB/E9), pop+4
//  (Jcc taken), pop+6 (DBNZ taken), hi+3 (CALL, one cycle before its
//  push request; push ready hi+4), done+1 (RET/RET-pop; RET-pop read
//  ready hi+1). Not-taken: Jcc retires end of pop+1, DBNZ pop+2.
//  Reservation starts: EB/E8/C2 at their final pop, E9 pop+1, Jcc/E2
//  pop+2; RET reserves from decode through the stack read.
//
//  Undefined-flag behavior per docs/facts/undefined_flags.md, with laws
//  discovered from the golden traces (each verified on all 500 cases):
//   - DIVU's entire flag residue = the flags of its 16-bit overflow
//     pre-check compare SUB(DW, divisor); the trap condition is that
//     compare's !borrow, and the trapped and non-trapped paths leave
//     exactly those compare flags (pushed PSW included).
//   - Byte RMW memory ops compute across the full 16-bit internal pair
//     ({sibling, byte}, source pair likewise) and drive the whole
//     16-bit result on the bus; carries/shifts propagate into the
//     driven (unwritten) sibling lane. Flags come from the byte op.
//   - XCHG mem timing = the ALU RMW path (write ready done+3); the
//     write drives the pre-exchange register pair, no flags.
//   - ROL4 rotates the WHOLE AL: rm <- {rm[3:0], AL[3:0]},
//     AL <- {AL[3:0], rm[7:4]} (manual implies AL[7:4] preserved; it
//     is not). Mem form drives {AL_new, mem_new}; write ready done+11;
//     reg form takes 11 wait cycles. No flags.
//   - ADD4S is a nibble-serial BCD adder: see bcd_add8 below for the
//     carry-rail quirk, pre-adjust Z, and the driven-sibling law.
//     Flags: CY=S=AC=carry, Z=P=zacc, V=0. Loop: src(DS0:IX+k) rd,
//     dst(DS1:IY+k) rd, wr per byte; ready offsets src=pop2+5 /
//     wrdone+1, dst=srcdone+2, wr=dstdone+4; the EU holds a bus
//     reservation from the first src request through retire; retire
//     at last wrdone+4 (carry) / +5 (no carry).
//
//============================================================================


module v30_eu (
    input             clk,
    input             ce,           // clock-enable: advance state this clk
    input             srst,

    // queue side
    input       [7:0] q_byte,
    input             q_avail,
    input             q_avail2,
    input             q_fresh,    // head byte became poppable this cycle
    input             q_any,
    output            q_pop,
    output            q_first,
    output            q_flush,
    output     [15:0] flush_cs,
    output     [15:0] flush_ip,

    // BIU access side
    output reg        eu_req,
    output            eu_hold,     // blocks prefetch without counting as
                                   // request history (trap-chain gaps)
    output reg        eu_ready,
    output reg        eu_soon,     // ready will assert next cycle (held resv)
    output            eu_soon_ea,  // eu_soon specifically from an S_EA2 reg-EA
                                   // reader/sreg-store (idle-window early
                                   // commit; excludes the S_WAITX/INT eu_soon)
    output            eu_soon_ivt, // hardware-interrupt (NMI/INT) IVT-read
                                   // idle-window early-commit lead: the last
                                   // pre-IVT wait cycle (S_WAITX dly==1 ->
                                   // S_TRAP_IVT1, irq_disp) arms defer_idle so
                                   // the IVT read commits one cycle earlier in
                                   // a pure idle window (reg-EA analogue)
    output            eu_soon_strio, // Family-7 (task #24): strio-single idle-
                                   // window early-commit lead. High at S_RSV of a
                                   // non-REP INS/OUTS single, where eu_ready is
                                   // guaranteed next cycle (S_REQ). Arms the BIU
                                   // defer_idle (pure-idle qlen6 plain forms).
                                   // Pure comb, zero flops (no savestate change).
    output            flush_fast,  // far flush commits redirect mid-cycle
    output            eu_defer_wr, // RMW mem write (S_WREQ): must NOT take a
                                   // waited-cycle deferred eval (eval_ext)
                                   // commit - the chip commits it at the next
                                   // PLAIN idle do_commit, not at a post-read
                                   // prefetch's deferred eval (measured, w1)
    output            eu_mem_acc,  // eu_req is a real memory data access (load/
                                   // store/EA), NOT a branch/jump reservation -
                                   // lets the BIU distinguish a starved-load
                                   // reservation from a branch-flush reservation
                                   // (both q_cnt=0/K_MEM/eu_wr=0)
    output            eu_rsv_dhi,   // Phase 3 reservation-CLASS hint (measured
                                    // Phase 2k, chip ground truth): the CURRENT
                                    // coincident reservation is the S_DHI
                                    // reader/RMW-read final-displacement-pop
                                    // class. This class OWNS the bus slot vs a
                                    // coincident deferred-eval prefetch (chip
                                    // IDLEs at q_cnt=1); the BIU vetoes
                                    // pf_late_rsv on it. Pure Moore of state -
                                    // meaningful only while eu_req is up.
    output            eu_rsv_push_calc, // Phase 3: same, the S_PUSH_CALC push
                                    // class - owns the slot only at q_cnt>=2
                                    // (the BIU applies the q_cnt gate).
    output            eu_rsv_lead,   // eu_req=0 onset fix (session 019f663c,
                                    // Codex staged GO): the chip's mem-access
                                    // reservation LEADS the model's eu_req by
                                    // one EU-state. High in the state one BEFORE
                                    // eu_req rises for the measured family(ies)
                                    // (disp16 store @ S_DHI, eu_req @ S_RSV). The
                                    // BIU turns this into an eval_ext-gated
                                    // prefetch suppression -> w0-NEUTRAL (never
                                    // consulted at w0; the chip's post-EA
                                    // prefetch legitimately commits there).
    output            eu_rsv_strio, // Family-5 (task #24): strio-single uline-1
                                    // request lead. High at the opcode pop
                                    // (S_FIRST) and dispatch (S_DEC) of a non-REP
                                    // INS/OUTS single. The BIU vetoes ONLY the
                                    // T3-eval fetch-completion grant (decision
                                    // instant >= pop+1); TI grants exempt. Pure
                                    // comb, zero flops (no savestate change).
    output reg        eu_wr,
    output            eu_fwd,     // write data = the BIU's last read data
                                  // (string-op read->write forwarding)
    output reg        eu_word,
    output reg [19:0] eu_addr,
    output reg  [1:0] eu_seg,
    output reg [15:0] eu_wdata,
    output reg  [1:0] eu_kind,
    output            eu_wrap,     // access offset == FFFFh: the split
                                   // second byte wraps to offset 0 of the
                                   // same segment (16-bit offset math;
                                   // IO wraps in port space)    // 0=mem 1=io 2=inta 3=halt
    input             eu_started,
    input             bus_phase,   // BIU 2-cycle grid parity (T1=0)
    input             grid_phase,  // rebuild Stage 1: TRUE stretched-grid
                                   // parity (== bus_phase at w0). Available
                                   // but not yet consumed (Stage 2 re-points
                                   // the bus_phase gates onto it).
    output            eu_lock,     // LOCK (F0) prefix active for the locked
                                   // instruction now executing (Stage 6): the
                                   // BIU drives BUSLOCK_N from this, releasing
                                   // it at the locked write's T4.
    input             bus_t4,      // BIU cycle is a T4
    input             bus_tw,      // BIU is inserting a wait cycle (0 at w0);
                                   // gate a dly with !bus_tw to count bus cyc
    input       [2:0] bus_ts,      // BIU T-state (0=Ti 1..4=T1-T4 5=cTi)
    input             eu_done,
    input             eu_wdone,   // early write completion (trap chain law)
    input             eu_rdone,   // early read completion (mirror of eu_wdone,
                                  // == eu_done at w0; read data via eu_rd_now)
    input             eu_t1,      // pulse: first T1 of the current EU access
    input      [15:0] eu_rdata,
    input             eu_rd_now,   // early strobe: read data edge (end T3)
    input      [15:0] eu_rdata_now,

    output            psw_ie,
    output reg        halt_disp,   // HALT decoded: BIU shows the HALT
                                   // pseudo-cycle at its display law

    // external event pins (synchronized here)
    input             pin_int,
    input             pin_nmi,
    input             pin_poll_n,

    // TB backdoor (verification only): load/observe architectural state
    input             bkd_load,
    input     [223:0] bkd_regs,   // {psw,ip,ds,ss,cs,es,di,si,bp,sp,bx,dx,cx,ax}
    output    [223:0] dbg_regs,
    output            dbg_first_pop,
    output            dbg_pend,      // ghost load still in flight

    input [v30_ss_pkg::SS_ADDR_W-1:0] ss_addr,
    input      [15:0] ss_wdata,
    input             ss_we,
    output reg [15:0] ss_rdata
);

import v30_ss_pkg::*;

always_ff @(posedge clk) begin
    case (ss_addr)
        SSA_E_RF0: ss_rdata <= rf[0];
        SSA_E_RF1: ss_rdata <= rf[1];
        SSA_E_RF2: ss_rdata <= rf[2];
        SSA_E_RF3: ss_rdata <= rf[3];
        SSA_E_RF4: ss_rdata <= rf[4];
        SSA_E_RF5: ss_rdata <= rf[5];
        SSA_E_RF6: ss_rdata <= rf[6];
        SSA_E_RF7: ss_rdata <= rf[7];
        SSA_E_SR0: ss_rdata <= sr[0];
        SSA_E_SR1: ss_rdata <= sr[1];
        SSA_E_SR2: ss_rdata <= sr[2];
        SSA_E_SR3: ss_rdata <= sr[3];
        SSA_E_PSW: ss_rdata <= psw;
        SSA_E_PC: ss_rdata <= pc;
        SSA_E_ARCH_IP: ss_rdata <= arch_ip;
        SSA_E_STATE: ss_rdata <= {9'b0, state};
        SSA_E_WNEXT: ss_rdata <= {9'b0, wnext};
        SSA_E_DRET: ss_rdata <= {9'b0, dret};
        SSA_E_DLY: ss_rdata <= {10'b0, dly};
        SSA_E_DIV_REM_LO: ss_rdata <= div_rem[15:0];
        SSA_E_DIV_REM_HI: ss_rdata <= {15'b0, div_rem[16]};
        SSA_E_DIV_QUO: ss_rdata <= div_quo;
        SSA_E_DIV_DEN: ss_rdata <= div_den;
        SSA_E_DIV_CNT: ss_rdata <= {10'b0, div_cnt};
        SSA_E_DIV_BUSY: ss_rdata <= {15'b0, div_busy};
        SSA_E_DIV_WORD: ss_rdata <= {15'b0, div_word};
        SSA_E_DIV_SIGNED: ss_rdata <= {15'b0, div_signed};
        SSA_E_DIV_NSIGN: ss_rdata <= {15'b0, div_nsign};
        SSA_E_DIV_DSIGN: ss_rdata <= {15'b0, div_dsign};
        SSA_E_DIV_PEND: ss_rdata <= {15'b0, div_pend};
        SSA_E_DIV_LATE: ss_rdata <= {15'b0, div_late};
        SSA_E_SH_R: ss_rdata <= sh_r;
        SSA_E_SH_X: ss_rdata <= {8'b0, sh_x};
        SSA_E_SH_OTH: ss_rdata <= {8'b0, sh_oth};
        SSA_E_SH_CY: ss_rdata <= {15'b0, sh_cy};
        SSA_E_SH_OP: ss_rdata <= {13'b0, sh_op};
        SSA_E_SH_WF: ss_rdata <= {15'b0, sh_wf};
        SSA_E_SH_N: ss_rdata <= {8'b0, sh_n};
        SSA_E_SH_BUSY: ss_rdata <= {15'b0, sh_busy};
        SSA_E_SH_FBASE: ss_rdata <= sh_fbase;
        SSA_E_SH_RES: ss_rdata <= sh_res;
        SSA_E_SH_FL: ss_rdata <= sh_fl;
        SSA_E_OPC: ss_rdata <= {8'b0, opc};
        SSA_E_OPC2: ss_rdata <= {8'b0, opc2};
        SSA_E_MRM: ss_rdata <= {8'b0, mrm};
        SSA_E_IMMB: ss_rdata <= {8'b0, immb};
        SSA_E_DISP: ss_rdata <= disp;
        SSA_E_A4_CNT: ss_rdata <= {8'b0, a4_cnt};
        SSA_E_A4_K: ss_rdata <= {8'b0, a4_k};
        SSA_E_A4_SRC: ss_rdata <= a4_src;
        SSA_E_A4_CARRY: ss_rdata <= {15'b0, a4_carry};
        SSA_E_A4_Z: ss_rdata <= {15'b0, a4_z};
        SSA_E_MEM_OP: ss_rdata <= mem_op;
        SSA_E_IVT_OFF: ss_rdata <= ivt_off;
        SSA_E_IVT_SEG: ss_rdata <= ivt_seg;
        SSA_E_TRAP_PSW: ss_rdata <= trap_psw;
        SSA_E_SEG_OVR_EN: ss_rdata <= {15'b0, seg_ovr_en};
        SSA_E_SEG_OVR: ss_rdata <= {14'b0, seg_ovr};
        SSA_E_LOCK_EN: ss_rdata <= {15'b0, lock_en};
        SSA_E_REP_EN: ss_rdata <= {15'b0, rep_en};
        SSA_E_REP_KIND: ss_rdata <= {14'b0, rep_kind};
        SSA_E_FLUSH_NOW: ss_rdata <= {15'b0, flush_now};
        SSA_E_STR_WR: ss_rdata <= {15'b0, str_wr};
        SSA_E_RSLOT: ss_rdata <= {10'b0, rslot};
        SSA_E_REP1_ABORT: ss_rdata <= {15'b0, rep1_abort};
        SSA_E_STR_DONE: ss_rdata <= {15'b0, str_done};
        SSA_E_CMP1: ss_rdata <= cmp1;
        SSA_E_CMP_R2S: ss_rdata <= {15'b0, cmp_r2s};
        SSA_E_FL_CS: ss_rdata <= fl_cs;
        SSA_E_FL_IP: ss_rdata <= fl_ip;
        SSA_E_EA_SAVE_LO: ss_rdata <= ea_save[15:0];
        SSA_E_EA_SAVE_HI: ss_rdata <= {12'b0, ea_save[19:16]};
        SSA_E_EA_SAVE_SEG: ss_rdata <= {14'b0, ea_save_seg};
        SSA_E_LDP2: ss_rdata <= {15'b0, ldp2};
        SSA_E_FRET_PH: ss_rdata <= {14'b0, fret_ph};
        SSA_E_FACC: ss_rdata <= {14'b0, facc};
        SSA_E_IRET_PW: ss_rdata <= {15'b0, iret_pw};
        SSA_E_POPR_PEND: ss_rdata <= {15'b0, popr_pend};
        SSA_E_PREP_ACC: ss_rdata <= {15'b0, prep_acc};
        SSA_E_PRACC: ss_rdata <= {13'b0, pracc};
        SSA_E_W4SKIP: ss_rdata <= {15'b0, w4skip};
        SSA_E_PREP_BPD: ss_rdata <= {15'b0, prep_bpd};
        SSA_E_SHW: ss_rdata <= {7'b0, shw};
        SSA_E_POPM_HOLD: ss_rdata <= {10'b0, popm_hold};
        SSA_E_IE_OFF: ss_rdata <= {12'b0, ie_off};
        SSA_E_IE_LEN: ss_rdata <= {11'b0, ie_len};
        SSA_E_IE_FLD: ss_rdata <= ie_fld;
        SSA_E_IE_W0: ss_rdata <= ie_w0;
        SSA_E_IE_MODE: ss_rdata <= {14'b0, ie_mode};
        SSA_E_IE_PH2: ss_rdata <= {15'b0, ie_ph2};
        SSA_E_IE_DLY: ss_rdata <= {4'b0, ie_dly};
        SSA_E_IE_CHAIN: ss_rdata <= {15'b0, ie_chain};
        SSA_E_IE_RDYHOLD: ss_rdata <= {15'b0, ie_rdyhold};
        SSA_E_IE_LGOT: ss_rdata <= {15'b0, ie_lgot};
        SSA_E_INT_P: ss_rdata <= {12'b0, int_p};
        SSA_E_NMI_P: ss_rdata <= {11'b0, nmi_p};
        SSA_E_NMI_LATCH: ss_rdata <= {15'b0, nmi_latch};
        SSA_E_POLL_S1: ss_rdata <= {15'b0, poll_s1};
        SSA_E_SHADOW: ss_rdata <= {15'b0, shadow};
        SSA_E_IE_PEND: ss_rdata <= {15'b0, ie_pend};
        SSA_E_IE_VAL: ss_rdata <= {15'b0, ie_val};
        SSA_E_PSW_OLD: ss_rdata <= psw_old;
        SSA_E_POP_PEND: ss_rdata <= {15'b0, pop_pend};
        SSA_E_IE_P: ss_rdata <= {12'b0, ie_p};
        SSA_E_WAITS_SEEN: ss_rdata <= {15'b0, waits_seen};
        SSA_E_POST_FLUSH: ss_rdata <= {15'b0, post_flush};
        SSA_E_INSN_IP: ss_rdata <= insn_ip;
        SSA_E_IVT_VEC: ss_rdata <= {8'b0, ivt_vec};
        SSA_E_HWAKE_IE0: ss_rdata <= {15'b0, hwake_ie0};
        SSA_E_IRQ_DISP: ss_rdata <= {15'b0, irq_disp};
        SSA_E_IRQ_NMI_IVT: ss_rdata <= {15'b0, irq_nmi_ivt};
        SSA_E_EU_WR: ss_rdata <= {15'b0, eu_wr};
        SSA_E_EU_WORD: ss_rdata <= {15'b0, eu_word};
        SSA_E_EU_ADDR_LO: ss_rdata <= eu_addr[15:0];
        SSA_E_EU_ADDR_HI: ss_rdata <= {12'b0, eu_addr[19:16]};
        SSA_E_EU_SEG: ss_rdata <= {14'b0, eu_seg};
        SSA_E_EU_WDATA: ss_rdata <= eu_wdata;
        SSA_E_EU_KIND: ss_rdata <= {14'b0, eu_kind};
        SSA_E_HALT_DISP: ss_rdata <= {15'b0, halt_disp};
        default: ss_rdata <= 16'h0000;
    endcase
end

// PS1:0 segment-status codes (= AD17:16 during T2-T4)
localparam bit [1:0] SEG_ES = 2'd0;   // DS1
localparam bit [1:0] SEG_SS = 2'd1;
localparam bit [1:0] SEG_CS = 2'd2;   // PS
localparam bit [1:0] SEG_DS = 2'd3;   // DS0

// PSW bits
localparam int FB_CY = 0, FB_P = 2, FB_AC = 4, FB_Z = 6, FB_S = 7,
               FB_V = 11;

// BIU access kinds
localparam bit [1:0] K_MEM  = 2'd0;
localparam bit [1:0] K_IO   = 2'd1;
localparam bit [1:0] K_INTA = 2'd2;

//----------------------------------------------------------------------------
// architectural state
//----------------------------------------------------------------------------
reg [15:0] rf [0:7];    // AW CW DW BW SP BP IX IY
reg [15:0] sr [0:3];    // DS1(ES) SS PS(CS) DS0(DS)
reg [15:0] psw;
reg [15:0] pc;          // running fetch-stream offset (past popped bytes)
reg [15:0] arch_ip;     // instruction-boundary IP (retire-updated)

assign psw_ie = psw[9];

//----------------------------------------------------------------------------
// microsequencer state
//----------------------------------------------------------------------------
typedef enum logic [6:0] {
    S_HALT, S_FIRST, S_DEC,
    S_IMM_LO, S_IMM_HI, S_IMM8, S_NOP,
    S_AI_I8, S_AI_I16, S_AIGAP, S_TESTGAP, S_BCD_IMM, S_SHWAIT,
    S_EA1, S_EA2, S_DISP8, S_DLO, S_DGAP, S_DHI, S_DSTALL,
    S_RSV, S_REQ, S_BUSW, S_FRETW, S_POPMW, S_POPR,
    S_61G, S_61W,
    S_PUSH_CALC,
    S_LD_W1, S_LD_W2,
    S_WAITX, S_EX,
    S_RMWX, S_WREQ, S_WBUSW,
    S_0F, S_DEC2, S_T1GAP, S_IMM3,
    S_IE_SET, S_IE_WAIT, S_IE_R1, S_IE_R1W, S_IE_R2, S_IE_R2W,
    S_IE_WR, S_IE_WRW, S_IE_IMM,
    S_A4_SETUP, S_A4_SRC, S_A4_SRCW, S_A4_G1, S_A4_DST, S_A4_DSTW,
    S_A4_G2, S_A4_WR, S_A4_WRW, S_A4_END,
    S_JDISP, S_JDLO, S_JDHI, S_JWAIT, S_JNT, S_JFLUSH,
    S_JSLO, S_JSHI, S_MLO, S_MHI, S_RESET,
    S_FCALLFL, S_FCALLP1, S_FCALLP2, S_INTV,
    S_PREP_L, S_PREP_W2, S_PREP_RD, S_PREP_RDGO, S_PREP_W3A,
    S_PREP_PW2, S_PREP_W3, S_PREP_W4,
    S_STRW, S_STRR, S_STRS, S_STRE,
    S_OUTS_W, S_OUTS_R, S_INS_W, S_INS_R,
    S_CMPW1, S_CMPW2, S_CMPNXT, S_SCASW, S_SCASNXT,
    S_CALLFL, S_CALLPUSH, S_CALLW, S_RETF, S_FCFL2,
    S_TRAP_IVT1, S_TRAP_IVT2, S_TRAP_IVT2W,
    S_TRAP_W1, S_TRAP_PSW, S_TRAP_PSWW,
    S_TRAP_W2, S_TRAP_PS, S_TRAP_PSW2W,
    S_TRAP_FLUSH, S_TRAP_PC, S_TRAP_PCW,
    S_IRQ_D, S_INT_W0, S_INT_A1, S_INT_A1W, S_INT_G, S_INT_A2, S_INT_A2W,
    S_IRQ_REPW, S_IRQ_REPFL,
    S_HALTED, S_HWAIT,
    S_POLL_WAIT, S_POLL_X,
    S_IN_PORT
} state_e;

state_e     state;
reg  [5:0]  dly;         // countdown for S_WAITX / S_RMWX / S_RSV / S_TRAP_W*
state_e     wnext;       // state entered when S_WAITX expires
state_e     dret;        // disp-pop state resumed after a queue-dry stall

// ---- shared iterative (bit-serial) divider ------------------------------
// Campaign 4 synthesis: replaces the 9 combinational lpm_divide instances
// (4 of them 32-bit) with ONE restoring shift-subtract unit. The DIV
// microsequence already spends its measured wait window idle; the unit is
// loaded at dispatch (reg form in S_DEC, mem form in S_BUSW) and stepped
// one iteration per clock through that window (16 iters word / 8 byte -
// both comfortably inside the smallest non-trap window), landing the
// quotient/remainder in mem_op/disp (and, for IDIV, psw/late-trap) before
// the S_EX retirement reads them. Cycle counts and results are unchanged.
// The early trap (den==0 / high-half overflow) stays a cheap combinational
// pre-check at dispatch and gates whether the unit runs at all. AAM's 8/8
// divide (S_EX) stays a small combinational divide.
reg  [16:0] div_rem;     // working remainder (17-bit for the shift headroom)
reg  [15:0] div_quo;     // quotient shift reg (dividend low shifts through)
reg  [15:0] div_den;     // divisor magnitude
reg  [5:0]  div_cnt;     // iterations remaining (16 word / 8 byte)
reg         div_busy;    // unit stepping
reg         div_word;    // 1 = 16-bit (word) form, 0 = 8-bit (byte) form
reg         div_signed;  // IDIV: apply sign fixup, late-trap and quot flags
reg         div_nsign;   // dividend sign (remainder + quotient fixup)
reg         div_dsign;   // divisor sign (quotient fixup)
reg         div_pend;    // a divide is heading to retire via S_WAITX
reg         div_late;    // latched late-trap (signed quotient magnitude ovf)

// ---- shared iterative shift/rotate unit -------------------------------
// Campaign 4 synthesis: replaces the single 255-deep combinational
// `shrot` unroll (D0-D3/C0/C1, all 8 sub-ops) with ONE shift stage that
// steps exactly n single-bit shifts (n = the FULL 8-bit count 0-255, NO
// masking preserved), one per clock through the shift micro-op's already
// idle wait window (the same window S_SHWAIT/S_WAITX already burn). The
// unit is loaded at each dispatch site (reg forms in S_DEC, C0/C1 at the
// count-imm pop, mem forms at read-done) and assembles the architectural
// result + flags into sh_res/sh_fl before the S_EX / S_RMWX retirement
// reads them. Bit- and cycle-identical to the old combinational path.
reg  [15:0] sh_r;        // word working value
reg  [7:0]  sh_x;        // byte working value (active lane, x_hi=0)
reg  [7:0]  sh_oth;      // byte sibling lane (shift register)
reg         sh_cy;       // working carry
reg  [2:0]  sh_op;       // sub-op (mrm_reg): 0..7
reg         sh_wf;       // word form
reg  [7:0]  sh_n;        // shifts remaining (full 8-bit count)
reg         sh_busy;     // unit stepping
reg  [15:0] sh_fbase;    // base flags (psw at dispatch; count=0 preserves)
reg  [15:0] sh_res;      // assembled result value (byte = {oth, x})
reg  [15:0] sh_fl;       // assembled new flags

reg  [7:0]  opc;
reg  [7:0]  opc2;        // 0F-prefixed second byte
reg  [7:0]  mrm;
reg  [7:0]  immb;        // TEST1 imm3 byte
reg [15:0]  disp;
// ADD4S loop state
reg  [7:0]  a4_cnt;      // bytes remaining
reg  [7:0]  a4_k;        // byte index (address offset)
reg [15:0]  a4_src;      // latched source read ({other lane, byte})
reg         a4_carry;    // BCD carry chain
reg         a4_z;        // accumulated zero flag (pre-adjust bytes)
reg [15:0]  mem_op;      // operand as read ({sibling, byte} for byte ops)
reg [15:0]  ivt_off, ivt_seg;
reg [15:0]  trap_psw;
// prefix latches (segment override / REP), cleared when the prefixed
// instruction retires
reg         seg_ovr_en;
reg  [1:0]  seg_ovr;
reg         lock_en;     // LOCK (F0) prefix latch: lives until the locked
                         // instruction retires (Stage 6). Drives eu_lock.
reg         rep_en;
reg  [1:0]  rep_kind;    // 0=REP/REPE(F3) 1=REPNE(F2) 2=REPC(65) 3=REPNC(64)
reg         flush_now;   // registered flush (trap path)
reg         str_wr;      // MOVBK phase: write half of the element
reg  [5:0]  rslot;       // REP retire slot: counts from the opcode pop
reg         rep1_abort;  // REP boundary-1 abort decision (latched pop+7)
reg         str_done;    // final string access completed (S_STRE)
reg [15:0]  cmp1;        // CMPBK: first (DS:IX) operand
reg         cmp_r2s;     // CMPBK: second read accepted
reg [19:0]  ea_save;     // POP-mem: EA write target (stack read first)
reg  [1:0]  ea_save_seg;
reg         ldp2;        // LES/LDS: second (segment) word in flight
reg  [1:0]  fret_ph;     // RETI stack-read phase (completed reads)
reg  [1:0]  facc;        // RETF/RETI stack reads accepted
reg         iret_pw;     // RETI: PSW stack read still in flight post-flush
reg         popr_pend;   // 8F.0 reg: stack read in flight post-retire
reg         prep_acc;    // PREPARE: BP push accepted
reg  [2:0]  pracc;       // POP R: reads accepted
reg         w4skip;      // PREPARE: swallow the last copy's eu_done
reg         prep_bpd;    // PREPARE: BP push done (latched)
reg  [8:0]  shw;         // shift/rotate full-count burn counter
// INS/EXT (0F 31/33/39/3B) bit-field state
reg  [3:0]  ie_off;      // effective bit offset (offset reg & 15)
reg  [4:0]  ie_len;      // effective field length 1..16
reg  [15:0] ie_fld;      // INS: source field / EXT: extracted result
reg  [15:0] ie_w0;       // EXT s>16: first-word latch
reg  [1:0]  ie_mode;     // 0=normal 1=alias-raw(off0/len16) 2=runaway
reg         ie_ph2;      // INS: second-word (carry-out) phase
reg  [11:0] ie_dly;      // burn counter (runaway needs 256*len)
reg         ie_chain;    // INS split W1: word-1 read chained in-flight
reg         ie_rdyhold;  // EXT late-pop: reserve one cycle before ready
reg         ie_lgot;     // INS imm form: length byte has been popped
reg [15:0]  fl_cs, fl_ip; // flush redirect target
// external-event machinery (block 4). The recognition sample is the
// boundary-decision cycle reading a 4-deep pin pipeline (fitted: the
// latest catching INT assert is boundary-4, NMI edge boundary-5).
reg  [3:0]  int_p;
reg  [4:0]  nmi_p;
reg         nmi_latch;           // NMI is edge-triggered and latched
reg         poll_s1;             // POLL_N synchronizer
reg         shadow;              // recognition shadow (sreg loads)
reg         ie_pend, ie_val;     // deferred EI/DI IE write (dry queue)
reg  [15:0] psw_old;             // pre-POP-PSW value (see pop_pend)
reg         pop_pend;            // POP PSW committed early; an interrupt
                                 // at its boundary pushes the NEW value
                                 // but the LIVE psw reverts to psw_old
                                 // before the IE/BRK clear (measured)
reg  [3:0]  ie_p;                // IE history: the boundary decision
                                 // uses IE@B-3 (this delay IS the EI /
                                 // POP-PSW enable shadow - measured); bit 3
                                 // is the post-flush (flush-3) tap
reg         waits_seen;          // latched once any Tw occurs = "waits are
                                 // configured"; 0 at w0 (w0-neutral gate for the
                                 // Jcc doomed-prefetch flush law)
reg         post_flush;          // 1-cycle pulse at the first S_FIRST after
                                 // a taken-branch S_JFLUSH: recognition taps
                                 // the pin/IE at flush-3 (int_p[3]/ie_p[3])

// POP-PSW boundary-race law (closure block): when INT recognition
// lands on POP PSW's own boundary, the LIVE flags commit either the
// popped value (class A) or the pre-pop value (class B), IE/BRK
// cleared either way; the pushed PSW is the popped value in both.
// DETERMINISTIC in the two flag words' data (proved by bench
// factorial at fixed timing), but algebraically dense (ANF ~2000
// terms, asymmetric, high-order) - implemented as the exhaustively
// measured 2^14 truth table (one bit per (pre,pop) flag-bit pair;
// provenance docs/facts/int9d_race_table.json.gz). Address =
// {pre,pop} x {V,DIR,S,Z,AC,P,CY}. Cells where the silicon also
// leaves the INT-pending latch set (ghost re-dispatch once IE=1;
// arch state = class A) are stored as A; the ghost latch itself is
// out of scope for the golden windows (see interrupt_model.md).
// The 2^14 race table (race_rom) is replaced by a combinational predicate
// race_law: per-(pre.DIR,pop.DIR)-quadrant staircase rank(pre6) >= thr(pop6),
// the pre7==pop7 diagonal committed as class A, and 68 measured resonance/
// ghost-repair exception cells (XOR). hdl/rtl/core/race_law.svh is GENERATED
// bit-exact vs int9d_race.hex (the measured contract, retained) by
// sw/gen_race_law.py; sw/check_race_law.py is the standing gate that verifies
// the checked-in .svh exhaustively (2^14) against the hex plus regenerate/byte-
// compare + header-digest. No $readmemh, no ROM (race_law stays LOGIC in
// Quartus -- Info 276004 "inappropriate RAM size", verified R0b spike), no flop.
`include "race_law.svh"
wire [6:0] r9d_pre = {psw_old[11], psw_old[10], psw_old[7],
                      psw_old[6], psw_old[4], psw_old[2], psw_old[0]};
wire [6:0] r9d_pop = {psw[11], psw[10], psw[7],
                      psw[6], psw[4], psw[2], psw[0]};
wire race_B = race_law(r9d_pre, r9d_pop);
reg [15:0]  insn_ip;             // first byte of the current instruction
                                 // INCLUDING prefixes (REP resume point)
reg  [7:0]  ivt_vec;             // vector for the S_TRAP_IVT1 chain
reg         hwake_ie0;           // HALT released by masked INT: resume
reg         irq_disp;            // current WAITX is an interrupt dispatch
reg         irq_nmi_ivt;         // the current S_WAITX->S_TRAP_IVT1 is the
                                 // NMI running-boundary IVT wait (direct from
                                 // S_FIRST, NOT the INT INTA2->IVT gap): gates
                                 // the eu_soon_ivt idle-window early commit

// mrm_reg selects the group sub-op. Two undoc aliases (v20-verified) fold in
// here so every mem-form dispatch / exec / length site sees the canonical
// sub-op: F6/F7 /1 -> /0 (TEST rm,imm); FF /7 -> /6 (PUSH rm16). Safe because
// for these opcodes mrm_reg is only a sub-op selector, never a register index
// (the operand is mrm_rm), and only these exact opc/reg pairs are remapped.
wire [2:0] mrm_reg =
    ((opc == 8'hF6 || opc == 8'hF7) && mrm[5:3] == 3'd1) ? 3'd0 :
    (opc == 8'hFF && mrm[5:3] == 3'd7)                   ? 3'd6 : mrm[5:3];
wire [2:0] mrm_rm  = mrm[2:0];
wire [1:0] mrm_mod = mrm[7:6];

//----------------------------------------------------------------------------
// decode
//----------------------------------------------------------------------------
// ALU r/m forms: all 32 encodings (8 ops x {rm8,r8 / rm16,r16 / r8,rm8 /
// r16,rm16}). opc[0]=w (byte/word), opc[1]=d (0: dest=rm, 1: dest=reg).
// Mission S implemented the word/direction forms (they were previously
// parked at S_HALT, and at that time the golden suite only covered the
// rm8,r8 representative; Mission G later emitted the full 32-form matrix).
wire op_alu    = (opc & 8'hC4) == 8'h00;
wire op_movs8  = opc == 8'h88;
wire op_movs16 = opc == 8'h89;
wire op_movl8  = opc == 8'h8A;
wire op_movl16 = opc == 8'h8B;
wire op_movri  = opc == 8'hC6 || opc == 8'hC7;      // MOV r/m, imm (grp /0)
wire op_grpf6  = opc == 8'hF6;   // group-3 byte: /reg = TEST/-/NOT/NEG/MULU/MUL/DIVU/DIV (/1 undoc, parked)
wire op_grpf7  = opc == 8'hF7;   // group-3 word: same /reg map as F6 (/1 undoc, parked)
wire op_grpd0  = opc == 8'hD0;   // group-2 byte shift/rotate by 1: all 8 /reg ops (/4 SHL fast path, rest via shrot)
wire op_grpfe  = opc == 8'hFE;   // group-4 byte: /0 INC, /1 DEC (/2-/7 undefined, parked)
wire op_xchg8  = opc == 8'h86;
wire op_xchg16 = opc == 8'h87;
wire op_jcc    = opc[7:4] == 4'h7;               // full Jcc set 70-7F
wire op_loopf  = opc == 8'hE0 || opc == 8'hE1 ||  // DBNZNE/DBNZE/DBNZ/BCWZ
                 opc == 8'hE2 || opc == 8'hE3;    // (LOOP/LOOPE/LOOPNE/JCXZ)
wire op_ret    = opc == 8'hC3 || opc == 8'hC2;
wire op_moff   = opc == 8'hA0 || opc == 8'hA1;   // MOV AL/AW, moffs16
wire op_moffw  = opc == 8'hA2 || opc == 8'hA3;   // MOV moffs16, AL/AW
wire op_lea    = opc == 8'h8D;                   // LDEA
wire op_srst   = opc == 8'h8C;                   // MOV rm16, sreg
wire op_srld   = opc == 8'h8E;                   // MOV sreg, rm16
wire op_xlat   = opc == 8'hD7 || opc == 8'hD6;   // TRANS (D6 = undoc alias of D7 XLAT; v20-verified: AL<-[BX+AL])
wire op_movstr = opc == 8'hA4 || opc == 8'hA5;   // MOVBK
wire op_stostr = opc == 8'hAA || opc == 8'hAB;   // STM
wire op_lodstr = opc == 8'hAC || opc == 8'hAD;   // LDM
wire op_cmpstr = opc == 8'hA6 || opc == 8'hA7;   // CMPBK
wire op_scastr = opc == 8'hAE || opc == 8'hAF;   // CMPM
wire op_outstr = opc == 8'h6E || opc == 8'h6F;   // OUTS (mem DS:IX -> port DW)
wire op_instr  = opc == 8'h6C || opc == 8'h6D;   // INS  (port DW -> mem ES:IY)
wire op_str    = op_movstr | op_stostr | op_lodstr | op_cmpstr | op_scastr |
                 op_outstr | op_instr;
wire op_segp   = opc == 8'h26 || opc == 8'h2E ||
                 opc == 8'h36 || opc == 8'h3E;   // segment override
wire op_repp   = opc == 8'hF3 || opc == 8'hF2 ||     // REP/REPE, REPNE
                 opc == 8'h65 || opc == 8'h64;       // REPC, REPNC
wire op_lockp  = opc == 8'hF0;                       // LOCK prefix (Stage 6)
// eu_lock: the LOCK latch (set at the F0 prefix's decode, held until the
// locked instruction retires). The BIU turns this into the BUSLOCK_N pulse,
// which asserts from the latch (measured: LOCK goes low at the locked op's
// execute start, ~one cycle after the F0 decode - the latch edge) and releases
// at the locked write's T4.
assign eu_lock = lock_en;
// full x86 condition matrix over the low opcode nibble
wire jcc_base =
    (opc[3:1] == 3'd0) ? psw[FB_V] :
    (opc[3:1] == 3'd1) ? psw[FB_CY] :
    (opc[3:1] == 3'd2) ? psw[FB_Z] :
    (opc[3:1] == 3'd3) ? (psw[FB_CY] | psw[FB_Z]) :
    (opc[3:1] == 3'd4) ? psw[FB_S] :
    (opc[3:1] == 3'd5) ? psw[FB_P] :
    (opc[3:1] == 3'd6) ? (psw[FB_S] ^ psw[FB_V]) :
                         ((psw[FB_S] ^ psw[FB_V]) | psw[FB_Z]);
wire jcc_taken = jcc_base ^ opc[0];
wire op_in     = opc == 8'hE4 || opc == 8'hE5 ||
                 opc == 8'hEC || opc == 8'hED;   // IN acc,imm8 / acc,DW
wire op_out    = opc == 8'hE6 || opc == 8'hE7 ||
                 opc == 8'hEE || opc == 8'hEF;   // OUT imm8,acc / DW,acc
wire op_0f     = opc == 8'h0F;                       // two-byte forms
wire op_test1  = op_0f && opc2 == 8'h18;             // TEST1 rm8,imm3
// INS/EXT bit-field forms (0F 31/39 = INS, 0F 33/3B = EXT). Measured
// laws (fitted on all 4 x 500 goldens, see docs/facts below + git log):
//  off = offreg(rm reg8) & 15, len = (lenval & 15) + 1, s = off+len;
//  the offset reg is written s&15 BEFORE the AW source read (INS) -
//  an AL/AH offset reg inserts the UPDATED AW field. EXT with offset
//  reg AL, or AH with len=16, degenerates to off=0/len=16 (AW <- raw
//  word at DS0:IX, IX+=2); EXT offset reg AH with len<16 is a runaway
//  internal loop: AW <- 0, IX/offreg untouched, 256*len cycle burn.
//  Flags: AC=V=0; INS: Z=(s==15), CY=S=(s>15), P=par8(s-16);
//  EXT: Z=(s==16), CY=S=(s>16), P=par8(s-17); runaway: only P=par8(-len).
//  INS dest is ES:IY (no override); EXT src DS0:IX (override applies).
wire op_insext = op_0f && (opc2 == 8'h31 || opc2 == 8'h33 ||
                           opc2 == 8'h39 || opc2 == 8'h3B);
wire ie_ins    = !opc2[1];                           // 31/39 vs 33/3B
wire ie_immf   = opc2[3];                            // 39/3B imm4 len
wire op_rol4   = op_0f && opc2 == 8'h28;             // ROL4 rm8
wire op_ror4   = op_0f && opc2 == 8'h2A;             // ROR4 rm8
wire op_bit1   = op_0f && opc2[7:4] == 4'h1;         // 0F 10-1F bit ops
wire b1_imm    = opc2[3];                            // imm4 vs CL index
wire op_accimm = (opc & 8'hC6) == 8'h04;             // ALU acc, imm
wire op_testai = opc == 8'hA8 || opc == 8'hA9;       // TEST acc, imm
wire op_test   = opc == 8'h84 || opc == 8'h85;       // TEST rm, reg
wire op_xchga  = opc[7:3] == 5'b10010 &&
                 opc[2:0] != 3'd0;                   // XCH AW, reg
wire op_pushsr = (opc & 8'hE7) == 8'h06;             // PUSH sreg
wire op_popsr  = (opc & 8'hE7) == 8'h07 &&
                 opc != 8'h0F;                       // POP sreg (not 0F)
wire op_pushi  = opc == 8'h68 || opc == 8'h6A;       // PUSH imm16/simm8
wire op_popm   = opc == 8'h8F;                       // POP mem (/0)
wire op_grpff  = opc == 8'hFF;                       // INC/DEC/PUSH/... rm16
wire op_imuli  = opc == 8'h69 || opc == 8'h6B;       // MUL reg,rm,imm
wire op_ldptr  = opc == 8'hC4 || opc == 8'hC5;       // LES/LDS (mem only)
wire op_fpo    = (opc & 8'hF8) == 8'hD8 ||
                 opc == 8'h66 || opc == 8'h67 ||     // FPO1 / FPO2 (ESC)
                 opc == 8'h63;                       // 0x63: undoc modrm word
                                                     // no-op (decode EA + dummy
                                                     // read, no writeback/flags).
                                                     // v20-verified: IP-only. This
                                                     // is the 63C0 preload preamble
                                                     // the internal core lacked.
wire op_chk    = opc == 8'h62;                       // CHKIND (mem only)
wire op_prep   = opc == 8'hC8;                       // PREPARE
wire op_disp   = opc == 8'hC9;                       // DISPOSE
wire op_retf   = opc == 8'hCB || opc == 8'hCA;       // RETF / RETF pop
wire op_iret   = opc == 8'hCF;                       // RETI
wire op_grp80  = opc == 8'h80 || opc == 8'h82;       // ALU rm8, imm8 (82 = undoc alias of 80; v20-verified)
wire op_grp81  = opc == 8'h81;                       // ALU rm16, imm16
wire op_grp83  = opc == 8'h83;                       // ALU rm16, simm8
wire op_alui   = op_grp80 | op_grp81 | op_grp83;
wire op_grpd1  = opc == 8'hD1;                       // shrot rm16, 1
wire op_grpd2  = opc == 8'hD2;                       // shrot rm8, CL
wire op_grpd3  = opc == 8'hD3;                       // shrot rm16, CL
wire op_grpc0  = opc == 8'hC0;                       // shrot rm8, imm8
wire op_grpc1  = opc == 8'hC1;                       // shrot rm16, imm8
wire op_shift  = op_grpd0 | op_grpd1 | op_grpd2 | op_grpd3 |
                 op_grpc0 | op_grpc1;
wire op_shimm  = op_grpc0 | op_grpc1;
// D0 group 4 keeps its original fitted path (shl8_1); everything else
// in the shifter family runs through shrot
wire op_shrot  = op_shift && !(op_grpd0 && mrm_reg == 3'd4);
wire op_modrm  = op_alu | op_movs8 | op_movs16 | op_movl8 | op_movl16 |
                 op_grpf6 | op_grpf7 | op_grpd0 | op_grpfe | op_alui |
                 op_grpd1 | op_grpd2 | op_grpd3 | op_grpc0 | op_grpc1 |
                 op_test | op_popm | op_grpff | op_imuli | op_ldptr |
                 op_fpo | op_chk | op_movri |
                 op_xchg8 | op_xchg16 | op_lea | op_srst | op_srld;

wire is_store  = op_movs8 | op_movs16 | op_srst;     // write-only mem access
wire is_load   = op_movl8 | op_movl16 | op_srld;
wire is_word_t = op_movs16 | op_movl16 | op_grpf7 |  // word transfer
                 (op_alu & opc[0]) |
                 op_grp81 | op_grp83 | op_grpd1 | op_grpd3 | op_grpc1 |
                 (op_test && opc[0]) | op_popm | op_grpff | op_imuli |
                 op_ldptr | (op_bit1 && opc2[0]) | op_fpo | op_chk |
                 (op_movri && opc[0]) |
                 op_xchg16 | op_srst | op_srld;
wire is_reader = !is_store;                          // mem forms read first

//----------------------------------------------------------------------------
// effective address
//----------------------------------------------------------------------------
wire [15:0] ea_base =
    (mrm_rm == 3'd0) ? rf[3] + rf[6] :
    (mrm_rm == 3'd1) ? rf[3] + rf[7] :
    (mrm_rm == 3'd2) ? rf[5] + rf[6] :
    (mrm_rm == 3'd3) ? rf[5] + rf[7] :
    (mrm_rm == 3'd4) ? rf[6] :
    (mrm_rm == 3'd5) ? rf[7] :
    (mrm_rm == 3'd6) ? ((mrm_mod == 2'd0) ? 16'd0 : rf[5]) :
                       rf[3];

wire [1:0] ea_seg_def =
    (mrm_rm == 3'd2 || mrm_rm == 3'd3 ||
     (mrm_rm == 3'd6 && mrm_mod != 2'd0)) ? SEG_SS : SEG_DS;
// a segment-override prefix absorbs the default (measurements.md law)
wire [1:0] ea_seg_sel = seg_ovr_en ? seg_ovr : ea_seg_def;

// modrm sreg field (0=ES 1=CS 2=SS 3=DS) -> sr[] index
function automatic [1:0] srmap(input [1:0] f);
    srmap = (f == 2'd1) ? SEG_CS : (f == 2'd2) ? SEG_SS : f;
endfunction

//----------------------------------------------------------------------------
// register-operand helpers
//----------------------------------------------------------------------------
function automatic [7:0] reg8_get(input [2:0] r);
    reg8_get = r[2] ? rf[{1'b0, r[1:0]}][15:8] : rf[{1'b0, r[1:0]}][7:0];
endfunction

// 16-bit register pair holding a byte register, arranged {sibling, byte}
function automatic [15:0] reg8_pair(input [2:0] r);
    logic [15:0] w;
    w = rf[{1'b0, r[1:0]}];
    reg8_pair = r[2] ? {w[7:0], w[15:8]} : w;
endfunction

task automatic wr_reg8(input [2:0] r, input [7:0] v);
    if (r[2]) rf[{1'b0, r[1:0]}][15:8] <= v;
    else      rf[{1'b0, r[1:0]}][7:0]  <= v;
endtask

// Load the iterative shift/rotate unit at dispatch. `a` is the operand
// pair ({sibling, byte} for byte forms, x_hi=0 so x=a[7:0], oth=a[15:8]);
// `cnt` is the FULL 8-bit shift count (0-255, no masking). count=0
// preserves both the value and every flag (result = a, flags = psw).
task automatic sh_load(input [2:0] op, input wf,
                       input [15:0] a, input [7:0] cnt);
    sh_op    <= op;
    sh_wf    <= wf;
    sh_cy    <= psw[FB_CY];
    sh_fbase <= psw;
    sh_n     <= cnt;
    if (wf) sh_r <= a;
    else begin sh_x <= a[7:0]; sh_oth <= a[15:8]; end
    if (cnt == 8'd0) begin
        sh_busy <= 1'b0;
        sh_res  <= a;
        sh_fl   <= psw;
    end else
        sh_busy <= 1'b1;
endtask

//----------------------------------------------------------------------------
// flag/ALU helpers. Return {new_psw[15:0], result}.
//----------------------------------------------------------------------------
function automatic [23:0] alu8(input [2:0] op, input [7:0] a, input [7:0] b,
                               input [15:0] f);
    logic [8:0]  t;
    logic [4:0]  tn;
    logic [7:0]  r;
    logic [15:0] nf;
    logic        logic_op;
    logic_op = (op == 3'd1) || (op == 3'd4) || (op == 3'd6);
    tn = '0;
    unique case (op)
        3'd0: begin t = {1'b0,a} + {1'b0,b};
                    tn = {1'b0,a[3:0]} + {1'b0,b[3:0]}; end
        3'd2: begin t = {1'b0,a} + {1'b0,b} + {8'd0, f[FB_CY]};
                    tn = {1'b0,a[3:0]} + {1'b0,b[3:0]} + {4'd0, f[FB_CY]}; end
        3'd3: begin t = {1'b0,a} - {1'b0,b} - {8'd0, f[FB_CY]};
                    tn = {1'b0,a[3:0]} - {1'b0,b[3:0]} - {4'd0, f[FB_CY]}; end
        3'd5, 3'd7:
              begin t = {1'b0,a} - {1'b0,b};
                    tn = {1'b0,a[3:0]} - {1'b0,b[3:0]}; end
        3'd1: t = {1'b0, a | b};
        3'd4: t = {1'b0, a & b};
        3'd6: t = {1'b0, a ^ b};
        default: t = '0;
    endcase
    r = t[7:0];
    nf = f;
    if (logic_op) begin
        nf[FB_CY] = 1'b0;
        nf[FB_AC] = 1'b0;
        nf[FB_V]  = 1'b0;
    end else begin
        nf[FB_CY] = t[8];
        nf[FB_AC] = tn[4];
        if (op == 3'd0 || op == 3'd2)
            nf[FB_V] = (~(a[7] ^ b[7])) & (a[7] ^ r[7]);
        else
            nf[FB_V] = (a[7] ^ b[7]) & (a[7] ^ r[7]);
    end
    nf[FB_S] = r[7];
    nf[FB_Z] = r == 8'd0;
    nf[FB_P] = ~^r;
    alu8 = {nf, r};
endfunction

// 16-bit twin of alu8: {new_psw, result}; P from the LOW byte only
function automatic [31:0] alu16(input [2:0] op, input [15:0] a,
                                input [15:0] b, input [15:0] f);
    logic [16:0] t;
    logic [4:0]  tn;
    logic [15:0] r;
    logic [15:0] nf;
    logic        logic_op;
    logic_op = (op == 3'd1) || (op == 3'd4) || (op == 3'd6);
    tn = '0;
    unique case (op)
        3'd0: begin t = {1'b0,a} + {1'b0,b};
                    tn = {1'b0,a[3:0]} + {1'b0,b[3:0]}; end
        3'd2: begin t = {1'b0,a} + {1'b0,b} + {16'd0, f[FB_CY]};
                    tn = {1'b0,a[3:0]} + {1'b0,b[3:0]} + {4'd0, f[FB_CY]}; end
        3'd3: begin t = {1'b0,a} - {1'b0,b} - {16'd0, f[FB_CY]};
                    tn = {1'b0,a[3:0]} - {1'b0,b[3:0]} - {4'd0, f[FB_CY]}; end
        3'd5, 3'd7:
              begin t = {1'b0,a} - {1'b0,b};
                    tn = {1'b0,a[3:0]} - {1'b0,b[3:0]}; end
        3'd1: t = {1'b0, a | b};
        3'd4: t = {1'b0, a & b};
        3'd6: t = {1'b0, a ^ b};
        default: t = '0;
    endcase
    r = t[15:0];
    nf = f;
    if (logic_op) begin
        nf[FB_CY] = 1'b0;
        nf[FB_AC] = 1'b0;
        nf[FB_V]  = 1'b0;
    end else begin
        nf[FB_CY] = t[16];
        nf[FB_AC] = tn[4];
        if (op == 3'd0 || op == 3'd2)
            nf[FB_V] = (~(a[15] ^ b[15])) & (a[15] ^ r[15]);
        else
            nf[FB_V] = (a[15] ^ b[15]) & (a[15] ^ r[15]);
    end
    nf[FB_S] = r[15];
    nf[FB_Z] = r == 16'd0;
    nf[FB_P] = ~^r[7:0];
    alu16 = {nf, r};
endfunction

function automatic [23:0] incdec8(input dec, input [7:0] a,
                                  input [15:0] f);
    logic [7:0] r;
    logic [15:0] nf;
    r = dec ? a - 8'd1 : a + 8'd1;
    nf = f;
    nf[FB_AC] = dec ? (a[3:0] == 4'h0) : (a[3:0] == 4'hF);
    nf[FB_V]  = dec ? (a == 8'h80) : (r == 8'h80);
    nf[FB_S]  = r[7];
    nf[FB_Z]  = r == 8'd0;
    nf[FB_P]  = ~^r;
    incdec8 = {nf, r};
endfunction

function automatic [23:0] shl8_1(input [7:0] a, input [15:0] f);
    logic [7:0] r;
    logic [15:0] nf;
    r = {a[6:0], 1'b0};
    nf = f;
    nf[FB_CY] = a[7];
    nf[FB_V]  = r[7] ^ a[7];
    nf[FB_AC] = 1'b0;
    nf[FB_S]  = r[7];
    nf[FB_Z]  = r == 8'd0;
    nf[FB_P]  = ~^r;
    shl8_1 = {nf, r};
endfunction

function automatic [31:0] incdec16(input dec, input [15:0] a,
                                   input [15:0] f);
    logic [15:0] r;
    logic [15:0] nf;
    r = dec ? a - 16'd1 : a + 16'd1;
    nf = f;
    nf[FB_AC] = dec ? (a[3:0] == 4'h0) : (a[3:0] == 4'hF);
    nf[FB_V]  = dec ? (a == 16'h8000) : (r == 16'h8000);
    nf[FB_S]  = r[15];
    nf[FB_Z]  = r == 16'd0;
    nf[FB_P]  = ~^r[7:0];
    incdec16 = {nf, r};
endfunction

// 32/16 unsigned divide: {trap, quotient, remainder}. The trap condition
// is "no borrow" from the overflow pre-check compare (below).
// SUPERSEDED and unused: the divide path now runs through the shared
// iterative div_* unit (commit c2beb6a). This combinational function and
// divu16_8 below are retained but no longer called by any active path.
function automatic [32:0] divu32(input [31:0] num, input [15:0] den);
    logic [31:0] q32, r32;
    if (den == 16'd0 || num[31:16] >= den) begin
        divu32 = {1'b1, 32'd0};
    end else begin
        q32 = num / {16'd0, den};
        r32 = num % {16'd0, den};
        divu32 = {1'b0, q32[15:0], r32[15:0]};
    end
endfunction

// DIVU8: AX / den8 -> {trap, q8, r8} (structural mirror of divu32).
// SUPERSEDED and unused - see the note on divu32 above; F6.6 DIVU8 now
// runs on the iterative unit and passes its goldens 500/500.
function automatic [16:0] divu16_8(input [15:0] num, input [7:0] den);
    logic [15:0] q16, r16;
    if (den == 8'd0 || {8'd0, num[15:8]} >= {8'd0, den}) begin
        divu16_8 = {1'b1, 16'd0};
    end else begin
        q16 = num / {8'd0, den};
        r16 = num % {8'd0, den};
        divu16_8 = {1'b0, q16[7:0], r16[7:0]};
    end
endfunction

// 8-bit compare residue for the DIVU8 pre-check (mirrors psw_sub16)
function automatic [15:0] psw_sub8f(input [7:0] a, input [7:0] b,
                                    input [15:0] f);
    logic [7:0] r;
    logic [15:0] nf;
    r = a - b;
    nf = f;
    nf[FB_CY] = b > a;
    nf[FB_AC] = b[3:0] > a[3:0];
    nf[FB_V]  = ((a[7] ^ b[7]) & (a[7] ^ r[7]));
    nf[FB_S]  = r[7];
    nf[FB_Z]  = r == 8'd0;
    nf[FB_P]  = ~^r;
    psw_sub8f = nf;
endfunction

// Signed divides (IDIV): magnitude divide + sign fixups, per the laws in
// docs/facts/undefined_flags.md (mission F) and the mission-I timing fit.
// Returns {early, late, flags, quotient, remainder}:
//  - early trap = divisor 0 or magnitude pre-check |num_high| >=
//    |divisor|; late trap = unsigned quotient magnitude exceeds
//    2^(n-1)-1 (SYMMETRIC: quotient -2^(n-1) traps too).
//  - flags: early trap leaves the residue of the magnitude pre-check
//    compare SUB(|num_high|, |divisor|) at operand width; late-trap and
//    non-trap paths leave S/Z/P of the UNSIGNED quotient magnitude with
//    CY=AC=V=0 (the sign-fixup micro-ops never touch flags).
//  - quotient truncates toward zero, remainder sign follows the
//    dividend.
// The laws above still govern IDIV, but idiv32/idiv16 themselves are
// SUPERSEDED and unused: the active IDIV path is the iterative div_*
// unit plus the inline early-trap pre-check (commit c2beb6a). These two
// combinational reference functions are retained but never called.
function automatic [49:0] idiv32(input [31:0] num, input [15:0] den,
                                 input [15:0] f);
    logic [31:0] an, q32, r32;
    logic [15:0] ad, q, r, nf;
    logic early, late;
    an = num[31] ? (~num + 32'd1) : num;
    ad = den[15] ? (~den + 16'd1) : den;
    early = (den == 16'd0) || (an[31:16] >= ad);
    if (early) begin
        q32 = '0; r32 = '0;
    end else begin
        q32 = an / {16'd0, ad};
        r32 = an % {16'd0, ad};
    end
    late = !early && (q32 > 32'd32767);
    if (early)
        nf = psw_sub16(an[31:16], ad, f);
    else begin
        nf = f;
        nf[FB_S]  = q32[15];
        nf[FB_Z]  = q32[15:0] == 16'd0;
        nf[FB_P]  = ~^q32[7:0];
        nf[FB_CY] = 1'b0;
        nf[FB_AC] = 1'b0;
        nf[FB_V]  = 1'b0;
    end
    q = (num[31] ^ den[15]) ? (~q32[15:0] + 16'd1) : q32[15:0];
    r = num[31] ? (~r32[15:0] + 16'd1) : r32[15:0];
    idiv32 = {early, late, nf, q, r};
endfunction

function automatic [33:0] idiv16(input [15:0] num, input [7:0] den,
                                 input [15:0] f);
    logic [15:0] an, q16, r16, nf;
    logic  [7:0] ad, q, r;
    logic [23:0] t;
    logic early, late;
    an = num[15] ? (~num + 16'd1) : num;
    ad = den[7] ? (~den + 8'd1) : den;
    early = (den == 8'd0) || (an[15:8] >= ad);
    if (early) begin
        q16 = '0; r16 = '0;
    end else begin
        q16 = an / {8'd0, ad};
        r16 = an % {8'd0, ad};
    end
    late = !early && (q16 > 16'd127);
    if (early) begin
        t = alu8(3'd5, an[15:8], ad, f);    // SUB8 compare residue
        nf = t[23:8];
    end else begin
        nf = f;
        nf[FB_S]  = q16[7];
        nf[FB_Z]  = q16[7:0] == 8'd0;
        nf[FB_P]  = ~^q16[7:0];
        nf[FB_CY] = 1'b0;
        nf[FB_AC] = 1'b0;
        nf[FB_V]  = 1'b0;
    end
    q = (num[15] ^ den[7]) ? (~q16[7:0] + 8'd1) : q16[7:0];
    r = num[15] ? (~r16[7:0] + 8'd1) : r16[7:0];
    idiv16 = {early, late, nf, q, r};
endfunction

// ADD4S packed-BCD byte add, nibble-serial as the silicon does it
// (fitted bit-exact on all 1020 golden byte iterations):
//  - low digit: binary nibble add; +6 adjust if it carried (c1) or
//    exceeded 9 (c2);
//  - high digit SUM takes c1+c2 as two carries, but the high ADJUST
//    DECISION sees only c1|c2 (single carry rail): fire = ahi+bhi+
//    (c1|c2) > 9. CY out = fire.
//  - sibx marks the extra +1 the driven sibling lane picks up when the
//    high sum carried (c3) and its pre-adjust digit exceeds 9.
//  - prez: the PRE-adjust byte was zero (Z accumulates on this, not on
//    the adjusted result).
// Returns {zq, pad, second-rail-wrap, fire, sibx, prez, result}.
function automatic [13:0] bcd_add8(input [7:0] a, input [7:0] b, input cin);
    logic [4:0] lo, hi;
    logic       c1, c2, c3, fire, sibx, prez, zq;
    logic [3:0] dlo0, dlo, dhi0, dhi;
    lo = {1'b0, a[3:0]} + {1'b0, b[3:0]} + {4'd0, cin};
    c1 = lo[4];
    dlo0 = lo[3:0];
    c2 = dlo0 > 4'd9;
    dlo = (c1 || c2) ? dlo0 + 4'd6 : dlo0;
    hi = {1'b0, a[7:4]} + {1'b0, b[7:4]} + {4'd0, c1} + {4'd0, c2};
    c3 = hi[4];
    dhi0 = hi[3:0];
    fire = ({1'b0, a[7:4]} + {1'b0, b[7:4]} + {4'd0, c1 | c2}) > 5'd9;
    dhi = fire ? dhi0 + 4'd6 : dhi0;
    sibx = c3 && (dhi0 > 4'd9);
    prez = (dhi0 == 4'd0) && (dlo0 == 4'd0);
    // F2 unified-law Z term (ADD direction). U per-byte tests the mu-line-2
    // digit-serial intermediate {ripple_pending, hi_raw_c1, dlo} == 0 with
    // ripple_pending == c2; the conjunction collapses algebraically:
    // c2=0 && dlo==0 => dlo0==0 => c1=0 (else dlo=6) => hi_raw_c1==0 =>
    // a_hi+b_hi==0. Net: zq <=> the RAW 9-bit byte sum (a + b + cin) == 0 --
    // the chip never calls an add byte zero if anything happened. zq lands
    // at the SAME s[13] as bcd_sub8's (the [12:11] pad has no badj/braw on
    // the add rail). See docs/notes/v03_tail_family_law_design.md.
    zq = ({1'b0, a} + {1'b0, b} + {8'd0, cin}) == 9'd0;
    bcd_add8 = {zq, 1'b0,
                ({2'b00, a[7:4]} + {2'b00, b[7:4]} +
                 {5'd0, c1} + {5'd0, c2}) > 6'd31,
                fire, sibx, prez, dhi, dlo};
endfunction

// SUB4S/CMP4S nibble-serial subtract, fitted on the 0F22 goldens
// (499/500; one anomalous sample documented in the checkpoint):
// the low -6 adjust itself BORROWS when it wraps (dlo0 < 6), that
// wrap-borrow joins c1 in the HIGH SUM, while the high ADJUST
// DECISION runs on a single-borrow rail (c1 only) and fires on
// borrow OR >9 - the mirror image of ADD4S's one-carry-rail quirk.
function automatic [13:0] bcd_sub8(input [7:0] a, input [7:0] b, input bin);
    logic [4:0] lo, hi, dec;
    logic       c1, c2, fl, wrapb, fire, sibx, prez, zq;
    logic [3:0] dlo0, dlo, dhi0, dhi;
    logic [8:0] rawx, adjx;
    lo = {1'b0, a[3:0]} - {1'b0, b[3:0]} - {4'd0, bin};
    c1 = lo[4];
    dlo0 = lo[3:0];
    c2 = dlo0 > 4'd9;
    fl = c1 || c2;
    dlo = fl ? dlo0 - 4'd6 : dlo0;
    wrapb = fl && (dlo0 < 4'd6);
    hi = {1'b0, a[7:4]} - {1'b0, b[7:4]} - {4'd0, c1} - {4'd0, wrapb};
    dhi0 = hi[3:0];
    dec = {1'b0, a[7:4]} - {1'b0, b[7:4]} - {4'd0, c1};
    // closure block: the high-adjust decision also fires at dec==9
    // with an invalid low nibble - but ONLY when the low compare did
    // not borrow (the pre-adjust >0x99 DAS threshold gated by !c1;
    // fitted exactly on all 1000 0F22/0F26 goldens)
    fire = dec[4] || (dec > 5'd9) ||
           (dec[3:0] == 4'd9 && c2 && !c1);
    dhi = fire ? dhi0 - 4'd6 : dhi0;
    sibx = hi[4] && (dhi0 > 4'd9);
    prez = (dhi0 == 4'd0) && (dlo0 == 4'd0);
    // byte-boundary borrows for the driven sibling lane (closure
    // block; sib = dst_o - src_o - braw - badj + 1, exact on all
    // 1020 golden SUB4S writes): braw = the raw byte subtract's
    // borrow, badj = the -6/-60h adjust step's borrow
    rawx = {1'b0, a} - {1'b0, b} - {8'd0, bin};
    adjx = {1'b0, rawx[7:0]} - (fl ? 9'd6 : 9'd0) -
           (fire ? 9'h60 : 9'h0);
    // F2 unified-law Z term (SUB/CMP direction): the mu-line-2 ADJD/W
    // digit-serial intermediate {ripple_pending, hi_raw_c1, dlo} tested
    // == 0. For SUB/CMP ripple_pending == wrapb, and wrapb=1 => dlo =
    // dlo0+10 != 0, so the rail bit is IDENTICALLY vacuous over the whole
    // domain -> zq collapses to {dec[3:0], dlo} == 0. dec[3:0] is the raw
    // high nibble (raw low borrow c1 only, BEFORE wrapb and the -6 fire
    // adjust) -- it is the mu-intermediate. Do NOT "fix" this to dhi0
    // (dhi0 includes wrapb): they coincide on Z-relevant cases but
    // dec[3:0] is the correct rail. All 15 wrap divergents have c1
    // activity, so a naive c1-activity gate is dead on sub -- it is the
    // RIPPLE that gates, structurally 0 whenever the digits read zero.
    // See docs/notes/v03_tail_family_law_design.md.
    zq = (dec[3:0] == 4'd0) && (dlo == 4'd0);
    bcd_sub8 = {zq, adjx[8], rawx[8], fire, sibx, prez, dhi, dlo};
endfunction

// DIVU leaves the flags of its 16-bit overflow pre-check compare
// SUB(DW, divisor) - verified bit-exact on all 500 golden cases, both
// the trap residue (pushed PSW) and the non-trap final flags. The
// "forced constant" pattern in undefined_flags.md is the special case
// dx < den of this law.
function automatic [15:0] psw_sub16(input [15:0] a, input [15:0] b,
                                    input [15:0] f);
    logic [15:0] r;
    logic [15:0] nf;
    r = a - b;
    nf = f;
    nf[FB_CY] = b > a;
    nf[FB_AC] = b[3:0] > a[3:0];
    nf[FB_V]  = ((a[15] ^ b[15]) & (a[15] ^ r[15]));
    nf[FB_S]  = r[15];
    nf[FB_Z]  = r == 16'd0;
    nf[FB_P]  = ~^r[7:0];
    psw_sub16 = nf;
endfunction

//----------------------------------------------------------------------------
// execute-stage combinational results (operand a = rm, b = reg)
//----------------------------------------------------------------------------
wire [7:0]  rm_byte = (mrm_mod == 2'd3) ? reg8_get(mrm_rm) : mem_op[7:0];
wire [15:0] rm_word = (mrm_mod == 2'd3) ? rf[mrm_rm] : mem_op;
// ALU r/m operand order by the direction bit (opc[1]): d=0 dest=rm
// (a=rm, b=reg), d=1 dest=reg (a=reg, b=rm). SUB/SBB/CMP are
// non-commutative so the order sets the flags too.
wire [7:0]  alu_a8  = opc[1] ? reg8_get(mrm_reg) : rm_byte;
wire [7:0]  alu_b8  = opc[1] ? rm_byte : reg8_get(mrm_reg);
wire [15:0] alu_a16 = opc[1] ? rf[mrm_reg] : rm_word;
wire [15:0] alu_b16 = opc[1] ? rm_word : rf[mrm_reg];
wire [23:0] ex_alu  = alu8(opc[5:3], alu_a8, alu_b8, psw);
wire [31:0] ex_alu16 = alu16(opc[5:3], alu_a16, alu_b16, psw);
wire [23:0] ex_inc  = incdec8(mrm_reg == 3'd1, rm_byte, psw);
wire [23:0] ex_shl  = shl8_1(rm_byte, psw);

// ALU rm,imm groups (80/81/83): imm collected in disp; op = mrm_reg
wire [15:0] ai_imm  = op_grp83 ? {{8{disp[7]}}, disp[7:0]} : disp;
wire [23:0] ex_ai8  = alu8(mrm_reg, rm_byte, disp[7:0], psw);
wire [31:0] ex_ai16 = alu16(mrm_reg, (mrm_mod == 2'd3) ? rf[mrm_rm]
                                                       : mem_op,
                            ai_imm, psw);
// shifter count: the FULL 8-bit count, NO masking (V30 does not mask to
// 5 bits; 1 for the by-1 forms, CL for D2/D3, imm8 in disp for C0/C1)
wire sh_word = op_grpd1 | op_grpd3 | op_grpc1;
wire [7:0] sh_cnt = (op_grpd0 | op_grpd1) ? 8'd1 :
                    (op_grpd2 | op_grpd3) ? rf[1][7:0] : disp[7:0];
// operand pair fed to the iterative shift unit at dispatch: byte forms
// pass {sibling, byte} (mem read pair, or {0, reg}); word forms the value.
// (mrm is latched at the mem/C0-C1 dispatch sites that use this wire.)
wire [15:0] sh_operand =
    sh_word ? ((mrm_mod == 2'd3) ? rf[mrm_rm] : mem_op)
            : ((mrm_mod == 2'd3) ? {8'h00, reg8_get(mrm_rm)} : mem_op);
// byte-form mem write: 16-bit pair arithmetic with the imm byte
// SIGN-EXTENDED onto the sibling lane (measured: 80.0 ADD imm=08
// sibling +carry only; 80.1 OR imm=f0 sibling -> FF; 80.2 ADC borrow)
wire [15:0] ai_pair = op_grp80 ? {{8{disp[7]}}, disp[7:0]} : ai_imm;
wire [15:0] ai_wide =
    (mrm_reg == 3'd0) ? mem_op + ai_pair :
    (mrm_reg == 3'd2) ? mem_op + ai_pair + {15'd0, psw[FB_CY]} :
    (mrm_reg == 3'd3) ? mem_op - ai_pair - {15'd0, psw[FB_CY]} :
    (mrm_reg == 3'd5) ? mem_op - ai_pair :
    (mrm_reg == 3'd1) ? (mem_op | ai_pair) :
    (mrm_reg == 3'd4) ? (mem_op & ai_pair) :
                        (mem_op ^ ai_pair);   // 6=XOR (7 never writes)

// Byte RMW mem ops run their operation across the full 16-bit internal
// pair ({sibling, byte}) and drive the whole result onto the bus; only
// the active lane commits to memory but carries/shifts propagate into
// the driven sibling byte (measured). Flags still come from the byte op.
wire [15:0] src_pair = reg8_pair(mrm_reg);
wire [15:0] rmw_wide =
    op_grpfe ? (mrm_reg == 3'd1 ? mem_op - 16'd1 : mem_op + 16'd1) :
    op_grpd0 ? {mem_op[14:0], 1'b0} :
    (opc[5:3] == 3'd0) ? mem_op + src_pair :
    (opc[5:3] == 3'd2) ? mem_op + src_pair + {15'd0, psw[FB_CY]} :
    (opc[5:3] == 3'd3) ? mem_op - src_pair - {15'd0, psw[FB_CY]} :
    (opc[5:3] == 3'd5) ? mem_op - src_pair :
    (opc[5:3] == 3'd1) ? (mem_op | src_pair) :
    (opc[5:3] == 3'd4) ? (mem_op & src_pair) :
                         (mem_op ^ src_pair);

//----------------------------------------------------------------------------
// interrupt recognition (measured laws: docs/facts/interrupt_model.md)
// - sampled once per instruction at its boundary (S_FIRST)
// - blocked for one boundary by sreg loads and EI (shadow), and always
//   between a prefix and its instruction (live prefix latches)
// - NMI is edge-latched and ignores IE; INT is a level gated by IE
//----------------------------------------------------------------------------
wire irq_int  = post_flush ? (int_p[3] && ie_p[3]) : (int_p[2] && ie_p[2]);
wire irq_any  = nmi_latch || irq_int;
wire irq_take = irq_any && !shadow && !rep_en && !seg_ovr_en && !lock_en;
// REP iteration-boundary sampling runs one stage deeper (fitted)
wire irq_rep  = nmi_latch || (int_p[3] && psw[9]);

//----------------------------------------------------------------------------
// queue pop control
//----------------------------------------------------------------------------
wire pop_want = (state == S_FIRST && !irq_take) ||
                (state == S_DEC && op_modrm) ||
                (state == S_0F) || (state == S_DEC2) || (state == S_IMM3) ||
                (state == S_IE_IMM) ||
                (state == S_IMM_LO) || (state == S_IMM_HI) ||
                (state == S_IMM8) ||
                (state == S_AI_I8) || (state == S_AI_I16) ||
                (state == S_BCD_IMM) ||
                (state == S_IN_PORT) ||
                // the FINAL displacement pop (disp8 / disp16-high) defers
                // one cycle when the head byte became poppable THIS cycle
                // (head was dry last cycle) AND the pop lands on an
                // in-flight fetch's T2 - i.e. a freshly-landed word whose
                // first-availability cycle collides with the next
                // back-to-back fetch's T2. Measured, Campaign 4 disp-phase
                // matrix (96 cells): blocked cells all fresh+T2; chip pops
                // normally on T2 when the head byte was already available,
                // and the disp16 LOW pop is never blocked (chip popped a
                // fresh byte on T2). The old "2-cycle dry-retry grain" was
                // an aliased fit of this block + availability.
                ((state == S_DISP8 || state == S_DHI) &&
                 !(bus_ts == 3'd2 && q_fresh)) ||
                (state == S_DLO) ||
                (state == S_JDISP) || (state == S_JDLO) ||
                (state == S_JDHI) || (state == S_JSLO) ||
                (state == S_JSHI) || (state == S_MLO) ||
                (state == S_MHI) || (state == S_INTV) ||
                (state == S_PREP_L && dly == 6'd0);

// split-access segment wrap (measured, fz494): a word access at offset
// FFFFh reads/writes its second byte at offset 0 of the SAME segment -
// 16-bit offset arithmetic, not 20-bit linear increment. IO ports wrap
// in 16-bit port space.
assign eu_wrap = (eu_kind == K_IO)
               ? (eu_addr[15:0] == 16'hFFFF)
               : ((eu_addr[15:0] - {sr[eu_seg][11:0], 4'h0}) == 16'hFFFF);

// F4a: sequential MULTI-WORD operands step their 16-bit OFFSET mod 64K.
function automatic [19:0] ea_step2(input [19:0] a, input [1:0] sg);
    logic [15:0] off;
    off = a[15:0] - {sr[sg][11:0], 4'h0};
    ea_step2 = {sr[sg], 4'h0} + {4'h0, off + 16'd2};
endfunction

assign q_pop   = pop_want && q_avail;
assign q_first = state == S_FIRST;
// flush: registered for the trap path; combinational for branch flush
// cycles (S_JFLUSH/S_RETF) and CALL's push-status cycle
assign q_flush = flush_now || (state == S_JFLUSH) || (state == S_RETF) ||
                 (state == S_CALLFL) || (state == S_FCALLFL) ||
                 (state == S_FCFL2) || (state == S_IRQ_REPFL);
assign flush_cs = fl_cs;
// EA far jump: the flush cycle commits the redirected prefetch
// mid-cycle (measured; near flushes commit at the cycle end)
// fast (mid-cycle-commit) flush: EA and the FF rm REG branches; the
// FF mem branches flush with the normal end-of-cycle commit (measured)
assign flush_fast = state == S_JFLUSH &&
                    (opc == 8'hEA || (op_grpff && mrm_mod == 2'd3));
assign flush_ip = fl_ip;
assign dbg_first_pop = q_pop && q_first;

//----------------------------------------------------------------------------
// EU request outputs (combinational Moore per state)
//----------------------------------------------------------------------------
always_comb begin
    eu_req   = 1'b0;
    eu_ready = 1'b0;
    eu_soon  = 1'b0;
    unique case (state)
        S_RSV:  begin
            eu_req = 1'b1;
            // Family-7 contingent arm (architect-ratified): a strio single at
            // S_RSV has eu_ready GUARANTEED next cycle (S_REQ) -- the eu_soon
            // contract, honored unconditionally (stronger than the S_EA2
            // reader precedent). The general eu_soon feeds exactly one BIU
            // consumer, the fetch-T3 defer_t4 arm, which catches the prefix-
            // qlen5 bridging-fetch element read (missed T3-eval pickup) and
            // commits it mid-T4 -- the chip's element-on-the-fetch-T4 signature.
            // Bus-state exclusive with the defer_idle main arm (that needs
            // ST_TI at S_RSV; this needs a fetch-T3).
            eu_soon = (op_instr || op_outstr) && !rep_en;
        end
        // reader reservations (measured on cold-start traces): no-disp
        // forms reserve through the EA-compute cycles; disp forms only
        // in the cycle their final displacement byte actually pops
        // the sreg store (8C) follows the READER reservation + ready
        // schedule throughout (measured); LEA reserves nothing
        // POP mem starts its reservation one cycle AFTER the disp pop
        // (a prefetch may commit at the pop-cycle end; measured) - its
        // S_WAITX wait below carries the reservation instead
        // POP mem (8F.0) does NOT reserve during the EA compute: a
        // queue-dry mod0 pop's in-flight fetch chain proceeds and the
        // read commits at the next fetch's T3 edge (closure block)
        S_EA1: eu_req = (is_reader || op_srst) && !op_lea && !op_popm &&
                        !op_movri && mrm_mod == 2'd0;
        S_EA2: begin
            eu_req = !op_lea && !op_popm && !op_movri;
            // Readers mark eu_soon so a fetch-T3 eval coinciding with S_EA2
            // defers to T4 and the read commits back-to-back (Mission S:
            // reg-EA readers whose ready lands on a prefetch T4 read two
            // cycles late otherwise). Stores do NOT set eu_soon: the bare
            // eu_req reservation (asserted from S_EA1) blocks the post-EA
            // prefetch so the write commits at the following idle eval
            // instead of slipping past a second fetch (Campaign 5: reg-EA
            // stores whose S_REQ ready lands on a fetch T4 wrote 2 cycles
            // late at phases 2/8 - the reservation must lead the request).
            eu_soon = eu_req && (is_reader || op_srst);
        end
        // POP mem reserves at its disp pop only on phase-1 pops
        // (T2/T4-aligned; phase-0 pops let the pop-end commit pass)
        S_DISP8, S_DHI: eu_req = (is_reader || op_srst) && !op_lea &&
                                 !op_movri &&
                                 q_pop && (!op_popm || bus_phase);
        // moffs forms (A0-A3) reserve during their final address-byte
        // pop, exactly like the disp pops (measured: cold A2 blocks the
        // prefetch commit at the in-flight fetch's T3 edge); IN's port
        // pop reserves identically
        S_MHI: eu_req = q_pop;
        S_IN_PORT: eu_req = q_pop;
        // SET1/NOT1 imm-mem reserve (req without ready) from pop+1
        // through the RMW write: an idle-end eval AT the pop still
        // goes to the prefetcher, an in-flight fetch's T3 eval a
        // cycle later is blocked (measured on 0F1C-1F).
        // Every mem RMW write additionally reserves the LAST compute
        // cycle (dly==1, the cycle before S_WREQ): the write's request
        // becomes ready in S_WREQ one cycle after a completing prefetch's
        // T4, so without a lead reservation the coincident fetch-T3 eval
        // lets a fresh prefetch win the slot and the write commits ~2
        // cycles late (chip blocks the prefetch and commits the write in
        // the following idle - the store analogue of the reg-EA S_EA2
        // reservation; measured fz80300/80560/81065/81117, f6/f7/fe RMW).
        S_RMWX: eu_req = (op_bit1 && opc2[2] && mrm_mod != 2'd3) ||
                        (mrm_mod != 2'd3 && dly == 6'd1);

        // POP r16 / RET reserve the bus already during decode (measured:
        // cold-start POP suppresses the prefetch commit at cycle 1)
        S_DEC:  eu_req = !op_modrm && (opc[7:3] == 5'b01011 ||
                                       opc == 8'hC3 || opc == 8'h9D ||
                                       opc == 8'hF4 || op_popsr ||
                                       opc == 8'hEC || opc == 8'hED ||
                                       opc == 8'hEE || opc == 8'hEF ||
                                       opc == 8'hC9 || opc == 8'hCB ||
                                       opc == 8'hCC || opc == 8'hCF ||
                                       opc == 8'h61);
        // reservation start (measured per opcode from old-stream commits
        // inside the resolution window): EB/E8 at the final pop cycle,
        // E9 at pop+1, Jcc/E2 at pop+2
        // software INT (BRK3/BRK/BRKV): the pre-IVT wait holds the bus
        // (measured: no prefetch commit between the pop and the IVT read).
        // BRKV joins the reservation only for its last three wait cycles
        // (the V-check lead-in leaves the bus free; measured: a prefetch
        // commits at dly==3 on the CE tranche). eu_soon marks the final
        // wait cycle so a completing fetch's T3 eval defers into T4.
        S_WAITX: begin
            eu_req = (wnext == S_TRAP_IVT1 &&
                      (opc == 8'hCC ||
                       ((opc == 8'hCD || opc == 8'hCE) && dly <= 6'd2))) ||
                     // CALL far imm holds the bus from the seg-hi pop
                     // to its PS push (measured: no prefetch commit);
                     // POP mem holds from disp-pop+1 to its stack read
                     (wnext == S_REQ && (opc == 8'h9A || op_popm)) ||
                     // PREPARE level>=2: the copy-read reservation is
                     // up only in the last wait cycle (with eu_soon) -
                     // earlier fetch commits proceed (closure block)
                     (dly == 6'd1 && wnext == S_PREP_RDGO) ||
                     // PUSHA (0x60) first write: reserve the LAST wait
                     // cycle before S_REQ (dly==1), exactly like PUSH
                     // r16's S_PUSH_CALC. The first stack write's request
                     // goes ready in S_REQ one cycle after a completing
                     // prefetch's T4; without the lead reservation the
                     // coincident fetch-T3 eval lets a fresh prefetch win
                     // the slot and the write commits ~2 cycles late
                     // (chip blocks it and commits the write in the
                     // following idle; measured fz80256/80282, 16 seeds).
                     (dly == 6'd1 && wnext == S_REQ && opc == 8'h60);
            eu_soon = eu_req && dly == 6'd1 && wnext == S_TRAP_IVT1;
        end
        S_JDISP: eu_req = q_pop && opc == 8'hEB;
        S_JDHI:  eu_req = q_pop && (opc == 8'hE8 || opc == 8'hC2 ||
                                    opc == 8'hCA);
        // EA reserves at the last seg-byte pop (measured: no prefetch
        // commit after the pop); 9A allows a chained fetch there and
        // reserves only from pop+1 (S_WAITX arm)
        S_JSHI:  eu_req = q_pop && opc == 8'hEA;
        // PUSH imm reserves at its final imm pop (measured, 68/6A).
        // MOV r/m,imm store reserves at its final imm pop too, so the
        // write slot is held against a coincident prefetch T3 eval (fitted
        // vs the C6/C7 goldens): C6 (byte) final pop is S_AI_I8, C7 (word)
        // S_AI_I16. Guarded on the mem form (mod != 3).
        S_AI_I8:  eu_req = q_pop && opc == 8'h6A;
        S_AI_I16: eu_req = q_pop && (opc == 8'h68 || op_prep ||
                                     (op_movri && mrm_mod != 2'd3));
        // PUSH forms reserve the bus one cycle early: the write is ready
        // the next cycle (S_REQ), and a prefetch T3 completion eval that
        // coincides with S_PUSH_CALC must not steal the slot ahead of it
        // (Mission S: PUSH r16 whose calc cycle lands on a prefetch T3 let
        // the prefetch commit, pushing the write two cycles late). req
        // without ready = reservation; the actual commit still fires in
        // S_REQ where eu_ready is high.
        S_PUSH_CALC: eu_req = 1'b1;
        // Loop family (E0-E3, taken) doomed-prefetch law (sweep_loop.py, the
        // E0/E1/E2/E3 x prefetch-phase x body matrix, chip-vs-TB): the
        // resolution wait HARD-reserves the bus (no prefetch of any kind)
        // ONLY in the last 3 cycles before the flush (dly<=3); at dly>=4 the
        // prefetcher runs FREELY. A prefetch (idle-start OR an in-flight
        // fetch's back-to-back successor) committed at dly>=4 survives as the
        // one doomed fetch the flush later discards (its T4 becomes the
        // redirect commit point); a commit whose eval would land at dly<=3 is
        // blocked. The measured cutoff is exactly dly=4 free / dly=3 blocked
        // for BOTH commit kinds (idle-start: E0 body2 ph3/9/15/21 at dly=4;
        // back-to-back: E2 ph2/8/14 at dly=4 vs ph0/4/10 at dly=3). The old
        // per-opcode reservation-start (E2 reserved from dly=4, E0/E1 had no
        // reservation) was a golden-phase alias of this. EB/E9/Jcc are not in
        // the doomed class and keep their own hard reservation above.
        // Flush front (Stage 3): op_jcc joins the doomed-prefetch law (free at
        // dly>=4, hard-reserve only dly<=3) - measured (sw/exp_flush.py, chip-
        // vs-TB): under waits the chip runs ONE doomed fall-through prefetch
        // during Jcc resolution before the flush (Jcc DIVERGES at the flush,
        // EB/E9/LOOP/CALL MATCH). The old Jcc "reserve except dly==3" hard-
        // reservation blocked it. EB/E9 (not op_jcc) keep their hard reserve.
        S_JWAIT: eu_req = (!((op_jcc && waits_seen) || op_loopf) ||
                           (wnext == S_JFLUSH &&
                            dly <= ((op_jcc && waits_seen) ? 6'd1 : 6'd3))) &&
                          // w0 Jcc keeps its single-cycle dly==3 prefetch gap
                          // (waits_seen==0); under waits Jcc reserves only the
                          // final cycle (dly<=1) so the chip's doomed
                          // fall-through prefetch commits (measured exp_flush)
                          !(op_jcc && !waits_seen && dly == 6'd3) &&
                          // CALL rm reg: no reservation (measured: a
                          // prefetch commits inside the wait)
                          !(op_grpff && mrm_reg == 3'd2 &&
                            mrm_mod == 2'd3);
        // CALL: the flush cycle keeps the reservation so the push (ready
        // next cycle) wins the first slot ahead of the redirected prefetch
        S_CALLFL: eu_req = 1'b1;
        // RET holds its reservation through the stack read (measured: no
        // prefetch commit at the read's T3 edge; plain POP r16 allows it)
        S_BUSW: eu_req = op_ret || op_retf || op_iret ||
                         (opc == 8'h9A && eu_wr);
        // RETF/RETI chain their stack pops back-to-back: the next read's
        // request+address are up during the current read (measured: the
        // second/third MEMR begins at the previous read's T4)
        S_FRETW: begin
            eu_req   = 1'b1;
            eu_ready = facc < (op_iret ? 2'd3 : 2'd2);
        end

        // reset countdown: the bus stays quiet until the reset flush
        S_RESET: eu_req = 1'b1;
        // INS/EXT accesses; the INS carry-path word-1 read gap holds a
        // reservation (fitted: R2 T1 = W1 T1 + 8, no prefetch steal)
        S_IE_WAIT: eu_req = ie_ph2 && wnext == S_IE_R2;
        // split word-0 write: the word-1 read request rides in-flight
        // (accepted at the write's second-sub T3 edge, T1 back-to-back)
        S_IE_WRW: begin
            eu_req   = ie_chain;
            eu_ready = ie_chain;
        end
        S_IE_R1: begin
            eu_req   = 1'b1;
            eu_ready = !ie_rdyhold;
        end
        S_IE_R2, S_IE_WR,
        S_REQ, S_WREQ,
        S_A4_SRC, S_A4_DST, S_A4_WR,
        S_CALLPUSH, S_FCALLP1, S_FCALLP2, S_FCFL2,
        S_PREP_RD, S_PREP_PW2, S_PREP_W3,
        S_STRW, S_STRR, S_STRS,
        S_OUTS_W, S_OUTS_R, S_INS_W, S_INS_R,
        S_TRAP_IVT1, S_TRAP_IVT2,
        S_TRAP_PSW, S_TRAP_PS, S_TRAP_FLUSH, S_TRAP_PC,
        S_INT_A1, S_INT_A2: begin
            eu_req   = 1'b1;
            eu_ready = 1'b1;
        end
        S_POPMW: begin              // drop once the write is accepted
            eu_req   = facc == 2'd0;
            eu_ready = facc == 2'd0;
        end
        // POP R: seven chained stack reads (the saved-SP slot is
        // skipped), request pipelined during the current read
        S_61G: eu_req = 1'b1;
        S_61W: begin
            eu_req   = pracc < 3'd7;
            eu_ready = pracc < 3'd7;
        end
        // CMPBK: the ES:IY read request stays up until accepted while
        // the DS:IX data is still in flight (structural; fit pending)
        S_CMPW1: begin
            eu_req   = !cmp_r2s;
            eu_ready = !cmp_r2s;
        end
        // PREPARE: the BP push request rides through the level-pop wait
        S_PREP_L: begin
            eu_req   = !prep_acc;
            eu_ready = !prep_acc;
        end
        // one-cycle reservation before the frame push; the first copy
        // read is ready in S_PREP_RDGO itself (address set up in the
        // preceding wait cycle - closure block)
        S_PREP_W3A: eu_req = 1'b1;
        S_PREP_RDGO: begin
            eu_req   = 1'b1;
            eu_ready = 1'b1;
        end
        // INTA sequence holds the bus between its cycles (measured: no
        // prefetch in the inter-INTA gap); the wake-wait states hold too
        S_IRQ_D, S_INT_W0, S_INT_A1W, S_INT_G, S_INT_A2W: eu_req = 1'b1;
        // ADD4S holds the bus reservation between its accesses through
        // retire (measured: no prefetch commit inside the loop)
        S_A4_SRCW, S_A4_G1, S_A4_DSTW, S_A4_G2, S_A4_WRW, S_A4_END:
            eu_req = 1'b1;
        default: ;
    endcase
    // 8F.0 reg form: the stack read request stays up across retire
    // (the register loads in flight; measured: retire at pop+3 with
    // the MEMR completing after the next instruction begins).
    // Closure block: the RESERVATION is up from pop+1 (a queue-dry
    // pop's in-flight fetch is blocked at its T3 edge) while ready
    // matures at the POP-mem pop-phase slot (pop+2 from a T1/T2 pop,
    // pop+3 otherwise) - fitted on all four golden geometries
    if (popr_pend) begin
        eu_req   = 1'b1;
        eu_ready = !(state == S_WAITX &&
                     (dly == 6'd2 ||
                      (dly == 6'd1 && popm_hold == 6'd3)));
    end
end

// eu_soon raised specifically by an S_EA2 reg-EA reader / sreg-store: the
// idle-window early-commit signal (a whole-EA-compute-in-bus-idle read
// commits one cycle earlier on the chip, directly in the idle window,
// where the fetch-T4 defer_t4 path has no in-flight fetch to land on).
// Qualified to S_EA2 so the S_WAITX/INT eu_soon (deferred swint) is
// untouched.
assign eu_soon_ea = (state == S_EA2) && eu_soon;

// Phase 3 (measured Phase 2k, chip ground truth; Codex GO): reservation-CLASS
// hints for the BIU's coincident-reservation arbitration veto. At a coincident
// (age-0) pending reservation - exactly the pf_late_rsv window (eu_req high,
// eu_req_p1==0) - the chip's reserve-vs-prefetch decision is a function of the
// reservation's OWN source state. Only the S_DHI reader/RMW-read final-disp-pop
// class and the S_PUSH_CALC push class own the bus slot; every other source
// (S_RSV/S_MHI/S_JWAIT/S_DEC/...) keeps the baseline yield-to-CODE. Exported as
// two clean 1-bit class hints (NOT the raw 7-bit state - avoid brittle EU->BIU
// coupling). Pure Moore of the current state; the BIU only consults them inside
// pf_late_rsv, which already gates on eu_req && !eu_req_p1.
assign eu_rsv_dhi       = (state == S_DHI);
assign eu_rsv_push_calc = (state == S_PUSH_CALC);

// eu_req=0 onset fix (session 019f663c, chip ground truth: eureq0_char census -
// 7/7 class-1 doomed-prefetch cases show the model's eu_req rising exactly one
// EU-state AFTER the eval where the chip has already reserved the bus). The
// chip's reservation LEADS eu_req by one state; at w0 the model matches (its
// post-EA prefetch legitimately commits at that cycle - PROVEN by the 169000
// golden showing CODE/T1 there), so the lead only bites under waits. The BIU
// consults this ONLY inside the eval_ext (waited) window -> w0-NEUTRAL.
//   disp16 store (88/89): reserves at S_DHI, model eu_req rises at S_RSV.
// q_pop-gated: the reservation intent forms when the disp-high byte pops and
// the effective address computes (a stalled S_DHI has no address yet). Staged
// per Codex: store first; moffs (S_MLO, op_moff) next; POP r16 (S_FIRST)
// deferred (opcode not yet registered there).
//   MOFFS load (A0/A1) at S_MLO: the opportunity census (moffs_optcensus.py,
// chip ground truth, 20 aligned eval_ext+S_MLO cells) found a PARITY/WIDTH
// discriminator - unlike the store, the moffs load does NOT reserve blanket:
//   A1 word load, EVEN addr (aligned, 1 bus cycle): chip PREFETCHES (12/12) -
//     must NOT veto (a blanket veto would wrongly suppress a legal prefetch).
//   A1 word load, ODD addr (split, 2 bus cycles): chip RESERVES (4/4).
//   A0 byte load: chip RESERVES (4/4).
// => reserve unless it is an ALIGNED WORD load. At S_MLO the low-address byte
// is popping (q_byte = addr[7:0]), so addr LSB = q_byte[0] and width = opc[0]
// (A1=word). Aligned word = opc[0] && !q_byte[0]; veto = !(aligned word).
assign eu_rsv_lead      = ((state == S_DHI) && (op_movs8 || op_movs16) && q_pop) ||
                          ((state == S_MLO) && op_moff && q_pop &&
                           (!opc[0] || q_byte[0]));

// strio-single uline-1 request lead (V20 ucode 0294/02A0: INS/OUTS singles issue
// their bus request on the routine's FIRST uline; MOVS/LODS/STOS/SCAS on uline 2 -
// measured cold A4 vs 6E, 5000/5000 each). Consumed by the BIU's T3-eval fetch-
// completion grant ONLY. REP (3-uline preamble 0298/02A4) and classic strings do
// NOT assert. Pure comb (Moore state + q_byte peek), zero flops.
assign eu_rsv_strio     = ((state == S_FIRST) && q_pop && !rep_en &&
                           (q_byte[7:2] == 6'b011011)) ||      // 6C-6F, pop cycle
                          ((state == S_DEC) && !rep_en &&
                           (op_instr || op_outstr));            // dispatch cycle

// Family-7 strio-single idle-window lead (main arm). At S_RSV of a non-REP
// INS/OUTS single, eu_req is up and eu_ready is GUARANTEED next cycle (S_REQ) -
// the documented eu_soon contract. Consumed by the BIU defer_idle arm ONLY
// (gated q_aged==0 there), which brings the pure-idle (qlen6 plain) element
// commit one cycle forward to the chip's S_REQ instant. Pure comb, zero flops.
assign eu_soon_strio    = (state == S_RSV) && (op_instr || op_outstr) && !rep_en;

// RMW mem write, ALU-imm forms ONLY (op80/81/83; op_alui): apply the
// stricter deferred-eval qualifier. Measured: only the ALU-imm RMW defers
// its write under waits (sweep_rmw.py: chip 12,14,14,16,18,20 - the w1==w2
// quantization). The immediate pop AFTER the operand read (S_AI_I8/I16)
// pushes the ALU-imm write-ready one slot later so it coincides with the
// post-read prefetch's T4 at w1; the chip then waits for the next plain
// idle. The imm-less RMW forms (NOT/NEG/INC/DEC mem, 0F CLR1/SET1/NOT1 mem,
// XCHG mem) have their write-ready one slot EARLIER (before that T4) and
// commit at the deferred eval via rule A - clean +2/wait, NO quantization
// (sweep NOT1 byte[mem] = 10,12,14,16). Gating on op_alui keeps those on
// the fitted rule A (fz84001 0F NOT1 regressed when all S_WREQ deferred).
// Exclude it from the waited-cycle deferred eval
// (eval_ext) commit. Measured (sweep_rmw.py, ADD word[mem],imm w0-w5): the
// chip commits the RMW write at read-T1 + 12 + 2*W_effective, landing on a
// PLAIN idle do_commit AFTER the post-read prefetch - never at that
// prefetch's deferred eval. The TB's rule-A/B (the S_RMWX lead reservation
// registered across S_RMWX+S_WREQ) otherwise commits it 2 cycles early
// whenever the write-ready coincides with the post-read prefetch's T4 (the
// w1 phase). The eu_req reservation still blocks prefetch through the gap,
// so the write simply commits at the next idle. S_WREQ is RMW-write-only
// (the fitted 88/89 stores use S_REQ), so no golden w0/w1/w3 form is
// affected. Reader (S_REQ read) and store (S_REQ write) paths untouched.
assign eu_defer_wr = (state == S_WREQ) && op_alui;
// eu_mem_acc: eu_req is a real memory data access, not a branch/jump/flush
// reservation. Excludes the S_J* branch-resolution states (whose eu_req is a
// bus hold, not a data access) so the BIU can extend the starved-prefetch
// override to LOAD reservations (indistinguishable from branch reservations by
// eu_kind/eu_wr alone).
assign eu_mem_acc = eu_req &&
                    !(state == S_JWAIT  || state == S_JDISP || state == S_JDLO ||
                      state == S_JDHI   || state == S_JSLO  || state == S_JSHI ||
                      state == S_JNT    || state == S_JFLUSH|| state == S_CALLFL);

// Hardware-interrupt (NMI/INT) IVT-read idle-window early commit: on the
// last pre-IVT wait cycle (S_WAITX dly==1 with wnext==S_TRAP_IVT1 and the
// interrupt-dispatch flag), lead the BIU's defer_idle so the IVT read
// commits directly in a pure idle window one cycle earlier - matching the
// chip in random contexts where the vectoring request goes ready with no
// in-flight prefetch to ride (the golden NOP-sled path rides a fetch, so
// the BIU only arms this in the idle ST_TI branch and stays untouched).
// Excludes divide-trap/software-INT (irq_disp low there).
// Phase-gated: the chip commits the IVT-read display on the bus-grid
// boundary (the next bus_phase==0 cycle). This arming cycle is E-1 (the
// S_TRAP_IVT1 entry is E); only when E-1 is bus_phase==1 does E land on
// the boundary, giving the one-cycle-earlier commit. When E-1 is
// bus_phase==0 (E off-boundary, e.g. the saturated NOP sled) the normal
// do_commit path already lands the display on the next boundary (E+1) -
// arming there would commit a cycle too early (would break the golden
// INT/NMI tranches), so it is excluded.
assign eu_soon_ivt = (state == S_WAITX) && (dly == 6'd1) &&
                     (wnext == S_TRAP_IVT1) && irq_nmi_ivt && bus_phase;

//----------------------------------------------------------------------------
// INS/EXT combinational helpers (laws in the decode-section comment)
//----------------------------------------------------------------------------
wire [5:0]  ie_s      = {2'd0, ie_off} + {1'd0, ie_len};
wire [31:0] ie_mask32 = ((32'd1 << ie_len) - 32'd1) << ie_off;
wire [31:0] ie_fsh    = {16'd0, ie_fld} << ie_off;
wire [7:0]  ie_pv_ins = {2'd0, ie_s} - 8'd16;
wire [7:0]  ie_pv_ext = {2'd0, ie_s} - 8'd17;
wire [7:0]  ie_pv_run = 8'd0 - {3'd0, ie_len};
wire [15:0] ie_psw_b  = (psw & 16'h0700) | 16'hF002;
wire [15:0] ie_psw_ins = ie_psw_b
    | (ie_s > 6'd15 ? 16'h0081 : 16'h0000)      // CY + S = carry
    | (ie_s == 6'd15 ? 16'h0040 : 16'h0000)     // Z
    | {13'd0, ~^ie_pv_ins, 2'd0};               // P
wire [15:0] ie_psw_ext = ie_psw_b
    | (ie_s > 6'd16 ? 16'h0081 : 16'h0000)
    | (ie_s == 6'd16 ? 16'h0040 : 16'h0000)
    | {13'd0, ~^ie_pv_ext, 2'd0};
wire [15:0] ie_psw_run = ie_psw_b | {13'd0, ~^ie_pv_run, 2'd0};

// fitted burn constants (cycles; see the schedule laws in the header)
localparam int unsigned IE_R1D   = 1;   // operand pop -> read request
localparam int unsigned IE_R1D0  = 15;  // INS off=0 s<16 pre-read burn
localparam int unsigned IE_G1    = 14;  // INS re-read burn base (+off)
localparam int unsigned IE_GW    = 5;   // INS write base (+3len+2(off-1))
localparam int unsigned IE_GW0   = 5;   // INS off=0 write base (+3len)
localparam int unsigned IE_GW16  = 34;  // INS s>=16 word-0 write slot
localparam int unsigned IE_W2R2  = 1;   // INS write-0 -> word-1 read
localparam int unsigned IE_R2G   = 2;   // EXT word-1 read gap
localparam int unsigned IE_TAIL  = 27;  // EXT retire base (+off; +256len)
localparam int unsigned IE_TAIL2 = 9;   // EXT s>16 retire base (+off)
localparam int unsigned IEI_IMM0 = 3;   // 0F39 off=0: mrm -> imm pop
localparam int unsigned IEI_R1   = 10;   // 0F39 off=0: imm -> read
localparam int unsigned IEI_W16  = 30;  // 0F39 off=0 len=16: imm -> write
localparam int unsigned IE_W16R  = 35;  // 0F31 off=0 len=16: mrm -> write
localparam int unsigned IEI_R2   = 10;   // 0F39 off>0 s<16: imm -> re-read
localparam int unsigned IEI_W1   = 30;  // 0F39 s>=16: imm -> write (-off)

//----------------------------------------------------------------------------
// edge-time helper tasks
//----------------------------------------------------------------------------
task automatic retire();
    arch_ip <= pc;
    state   <= S_FIRST;
    seg_ovr_en <= 1'b0;      // prefix latches end with their instruction
    rep_en     <= 1'b0;
    lock_en    <= 1'b0;
    shadow     <= 1'b0;      // shadowing instructions re-set it after
endtask

// POP mem (8F.0): the stack read matures at pop+2 from a T1/T2 pop,
// pop+3 otherwise (measured over all 500 golden mem cases)
wire [5:0] popm_rdy =
    (bus_ts == 3'd1 || bus_ts == 3'd2) ? 6'd2 : 6'd3;
reg  [5:0] popm_hold;   // popm_rdy latched at the last queue pop

// latch memory-operand access parameters (EA paths); off = 16-bit offset
task automatic setup_access(input [15:0] off);
    if (op_popm) begin
        // POP mem: the stack read goes first; the EA write follows
        // with the popped data (ea target saved here)
        ea_save <= {sr[ea_seg_sel], 4'h0} + {4'h0, off};
        ea_save_seg <= ea_seg_sel;
        eu_addr <= {sr[SEG_SS], 4'h0} + {4'h0, rf[4]};
        eu_seg  <= SEG_SS;
        eu_word <= 1'b1;
        eu_wr   <= 1'b0;
    end else begin
        eu_addr <= {sr[ea_seg_sel], 4'h0} + {4'h0, off};
        eu_seg  <= ea_seg_sel;
        eu_word <= is_word_t;
        eu_wr   <= is_store;
    end
    if (op_movs8)  eu_wdata <= reg8_pair(mrm_reg);
    if (op_movs16) eu_wdata <= rf[mrm_reg];
    if (op_srst)   eu_wdata <= sr[srmap(mrm_reg[1:0])];
endtask

// string-op element step (DF = PSW bit 10)
wire [15:0] str_step = opc[0] ? (psw[10] ? 16'hFFFE : 16'd2)
                              : (psw[10] ? 16'hFFFF : 16'd1);

// MOVBK's write takes its data from the BIU's read latch (forwarded at
// the commit edge - the write commits at the read's own T3 edge)
assign eu_fwd = state == S_STRW || state == S_POPMW ||
                state == S_PREP_PW2 || state == S_OUTS_W ||
                state == S_INS_W;

// stack push at SP-2 (also decrements SP)
task automatic issue_push(input [15:0] wdata);
    eu_addr  <= {sr[SEG_SS], 4'h0} + {4'h0, rf[4] - 16'd2};
    eu_seg   <= SEG_SS;
    eu_wr    <= 1'b1;
    eu_word  <= 1'b1;
    eu_wdata <= wdata;
    rf[4]    <= rf[4] - 16'd2;
endtask

//----------------------------------------------------------------------------
// main FSM
//----------------------------------------------------------------------------
always_ff @(posedge clk) begin
    if (ss_we) begin
        case (ss_addr)
            SSA_E_RF0: rf[0] <= ss_wdata;
            SSA_E_RF1: rf[1] <= ss_wdata;
            SSA_E_RF2: rf[2] <= ss_wdata;
            SSA_E_RF3: rf[3] <= ss_wdata;
            SSA_E_RF4: rf[4] <= ss_wdata;
            SSA_E_RF5: rf[5] <= ss_wdata;
            SSA_E_RF6: rf[6] <= ss_wdata;
            SSA_E_RF7: rf[7] <= ss_wdata;
            SSA_E_SR0: sr[0] <= ss_wdata;
            SSA_E_SR1: sr[1] <= ss_wdata;
            SSA_E_SR2: sr[2] <= ss_wdata;
            SSA_E_SR3: sr[3] <= ss_wdata;
            SSA_E_PSW: psw <= ss_wdata;
            SSA_E_PC: pc <= ss_wdata;
            SSA_E_ARCH_IP: arch_ip <= ss_wdata;
            SSA_E_STATE: state <= state_e'(ss_wdata[6:0]);
            SSA_E_WNEXT: wnext <= state_e'(ss_wdata[6:0]);
            SSA_E_DRET: dret <= state_e'(ss_wdata[6:0]);
            SSA_E_DLY: dly <= ss_wdata[5:0];
            SSA_E_DIV_REM_LO: div_rem[15:0] <= ss_wdata;
            SSA_E_DIV_REM_HI: div_rem[16] <= ss_wdata[0];
            SSA_E_DIV_QUO: div_quo <= ss_wdata;
            SSA_E_DIV_DEN: div_den <= ss_wdata;
            SSA_E_DIV_CNT: div_cnt <= ss_wdata[5:0];
            SSA_E_DIV_BUSY: div_busy <= ss_wdata[0];
            SSA_E_DIV_WORD: div_word <= ss_wdata[0];
            SSA_E_DIV_SIGNED: div_signed <= ss_wdata[0];
            SSA_E_DIV_NSIGN: div_nsign <= ss_wdata[0];
            SSA_E_DIV_DSIGN: div_dsign <= ss_wdata[0];
            SSA_E_DIV_PEND: div_pend <= ss_wdata[0];
            SSA_E_DIV_LATE: div_late <= ss_wdata[0];
            SSA_E_SH_R: sh_r <= ss_wdata;
            SSA_E_SH_X: sh_x <= ss_wdata[7:0];
            SSA_E_SH_OTH: sh_oth <= ss_wdata[7:0];
            SSA_E_SH_CY: sh_cy <= ss_wdata[0];
            SSA_E_SH_OP: sh_op <= ss_wdata[2:0];
            SSA_E_SH_WF: sh_wf <= ss_wdata[0];
            SSA_E_SH_N: sh_n <= ss_wdata[7:0];
            SSA_E_SH_BUSY: sh_busy <= ss_wdata[0];
            SSA_E_SH_FBASE: sh_fbase <= ss_wdata;
            SSA_E_SH_RES: sh_res <= ss_wdata;
            SSA_E_SH_FL: sh_fl <= ss_wdata;
            SSA_E_OPC: opc <= ss_wdata[7:0];
            SSA_E_OPC2: opc2 <= ss_wdata[7:0];
            SSA_E_MRM: mrm <= ss_wdata[7:0];
            SSA_E_IMMB: immb <= ss_wdata[7:0];
            SSA_E_DISP: disp <= ss_wdata;
            SSA_E_A4_CNT: a4_cnt <= ss_wdata[7:0];
            SSA_E_A4_K: a4_k <= ss_wdata[7:0];
            SSA_E_A4_SRC: a4_src <= ss_wdata;
            SSA_E_A4_CARRY: a4_carry <= ss_wdata[0];
            SSA_E_A4_Z: a4_z <= ss_wdata[0];
            SSA_E_MEM_OP: mem_op <= ss_wdata;
            SSA_E_IVT_OFF: ivt_off <= ss_wdata;
            SSA_E_IVT_SEG: ivt_seg <= ss_wdata;
            SSA_E_TRAP_PSW: trap_psw <= ss_wdata;
            SSA_E_SEG_OVR_EN: seg_ovr_en <= ss_wdata[0];
            SSA_E_SEG_OVR: seg_ovr <= ss_wdata[1:0];
            SSA_E_LOCK_EN: lock_en <= ss_wdata[0];
            SSA_E_REP_EN: rep_en <= ss_wdata[0];
            SSA_E_REP_KIND: rep_kind <= ss_wdata[1:0];
            SSA_E_FLUSH_NOW: flush_now <= ss_wdata[0];
            SSA_E_STR_WR: str_wr <= ss_wdata[0];
            SSA_E_RSLOT: rslot <= ss_wdata[5:0];
            SSA_E_REP1_ABORT: rep1_abort <= ss_wdata[0];
            SSA_E_STR_DONE: str_done <= ss_wdata[0];
            SSA_E_CMP1: cmp1 <= ss_wdata;
            SSA_E_CMP_R2S: cmp_r2s <= ss_wdata[0];
            SSA_E_FL_CS: fl_cs <= ss_wdata;
            SSA_E_FL_IP: fl_ip <= ss_wdata;
            SSA_E_EA_SAVE_LO: ea_save[15:0] <= ss_wdata;
            SSA_E_EA_SAVE_HI: ea_save[19:16] <= ss_wdata[3:0];
            SSA_E_EA_SAVE_SEG: ea_save_seg <= ss_wdata[1:0];
            SSA_E_LDP2: ldp2 <= ss_wdata[0];
            SSA_E_FRET_PH: fret_ph <= ss_wdata[1:0];
            SSA_E_FACC: facc <= ss_wdata[1:0];
            SSA_E_IRET_PW: iret_pw <= ss_wdata[0];
            SSA_E_POPR_PEND: popr_pend <= ss_wdata[0];
            SSA_E_PREP_ACC: prep_acc <= ss_wdata[0];
            SSA_E_PRACC: pracc <= ss_wdata[2:0];
            SSA_E_W4SKIP: w4skip <= ss_wdata[0];
            SSA_E_PREP_BPD: prep_bpd <= ss_wdata[0];
            SSA_E_SHW: shw <= ss_wdata[8:0];
            SSA_E_POPM_HOLD: popm_hold <= ss_wdata[5:0];
            SSA_E_IE_OFF: ie_off <= ss_wdata[3:0];
            SSA_E_IE_LEN: ie_len <= ss_wdata[4:0];
            SSA_E_IE_FLD: ie_fld <= ss_wdata;
            SSA_E_IE_W0: ie_w0 <= ss_wdata;
            SSA_E_IE_MODE: ie_mode <= ss_wdata[1:0];
            SSA_E_IE_PH2: ie_ph2 <= ss_wdata[0];
            SSA_E_IE_DLY: ie_dly <= ss_wdata[11:0];
            SSA_E_IE_CHAIN: ie_chain <= ss_wdata[0];
            SSA_E_IE_RDYHOLD: ie_rdyhold <= ss_wdata[0];
            SSA_E_IE_LGOT: ie_lgot <= ss_wdata[0];
            SSA_E_INT_P: int_p <= ss_wdata[3:0];
            SSA_E_NMI_P: nmi_p <= ss_wdata[4:0];
            SSA_E_NMI_LATCH: nmi_latch <= ss_wdata[0];
            SSA_E_POLL_S1: poll_s1 <= ss_wdata[0];
            SSA_E_SHADOW: shadow <= ss_wdata[0];
            SSA_E_IE_PEND: ie_pend <= ss_wdata[0];
            SSA_E_IE_VAL: ie_val <= ss_wdata[0];
            SSA_E_PSW_OLD: psw_old <= ss_wdata;
            SSA_E_POP_PEND: pop_pend <= ss_wdata[0];
            SSA_E_IE_P: ie_p <= ss_wdata[3:0];
            SSA_E_WAITS_SEEN: waits_seen <= ss_wdata[0];
            SSA_E_POST_FLUSH: post_flush <= ss_wdata[0];
            SSA_E_INSN_IP: insn_ip <= ss_wdata;
            SSA_E_IVT_VEC: ivt_vec <= ss_wdata[7:0];
            SSA_E_HWAKE_IE0: hwake_ie0 <= ss_wdata[0];
            SSA_E_IRQ_DISP: irq_disp <= ss_wdata[0];
            SSA_E_IRQ_NMI_IVT: irq_nmi_ivt <= ss_wdata[0];
            SSA_E_EU_WR: eu_wr <= ss_wdata[0];
            SSA_E_EU_WORD: eu_word <= ss_wdata[0];
            SSA_E_EU_ADDR_LO: eu_addr[15:0] <= ss_wdata;
            SSA_E_EU_ADDR_HI: eu_addr[19:16] <= ss_wdata[3:0];
            SSA_E_EU_SEG: eu_seg <= ss_wdata[1:0];
            SSA_E_EU_WDATA: eu_wdata <= ss_wdata;
            SSA_E_EU_KIND: eu_kind <= ss_wdata[1:0];
            SSA_E_HALT_DISP: halt_disp <= ss_wdata[0];
            default: ;
        endcase
    end else if (srst) begin
        // flush_now defaults to 0 every cycle (was an ungated pulse
        // default); keep that at reset so q_flush is clean during RESET.
        flush_now <= 1'b0;
        // real reset flow: PS=FFFF, PC=0, PSW cleared; the sequencer
        // idles 7 cycles after release, then flush-redirects to FFFF0
        state    <= S_RESET;
        iret_pw  <= 1'b0;
        popr_pend <= 1'b0;
        ie_chain <= 1'b0;
        dly      <= 6'd7;
        wnext    <= S_HALT;
        opc      <= '0;
        opc2     <= '0;
        mrm      <= '0;
        immb     <= '0;
        disp     <= '0;
        a4_cnt   <= '0;
        a4_k     <= '0;
        a4_src   <= '0;
        a4_carry <= 1'b0;
        a4_z     <= 1'b0;
        mem_op   <= '0;
        ivt_off  <= '0;
        ivt_seg  <= '0;
        trap_psw <= '0;
        fl_cs    <= 16'hFFFF;
        fl_ip    <= '0;
        seg_ovr_en <= 1'b0;
        seg_ovr  <= 2'd0;
        rep_en   <= 1'b0;
        lock_en  <= 1'b0;
        str_wr   <= 1'b0;
        rslot    <= 6'd0;
        str_done <= 1'b0;
        int_p    <= '0;
        nmi_p    <= '0;
        nmi_latch <= 1'b0;
        rep1_abort <= 1'b0;
        poll_s1  <= 1'b1;
        shadow   <= 1'b0;
        ie_p     <= '0;
        post_flush <= 1'b0;
        waits_seen <= 1'b0;
        ie_pend  <= 1'b0;
        ie_val   <= 1'b0;
        psw_old  <= '0;
        pop_pend <= 1'b0;
        insn_ip  <= '0;
        ivt_vec  <= '0;
        hwake_ie0 <= 1'b0;
        irq_disp <= 1'b0;
        irq_nmi_ivt <= 1'b0;
        div_busy <= 1'b0;
        div_pend <= 1'b0;
        div_late <= 1'b0;
        sh_busy  <= 1'b0;
        eu_kind  <= K_MEM;
        halt_disp <= 1'b0;
        eu_wr    <= 1'b0;
        eu_word  <= 1'b0;
        eu_addr  <= '0;
        eu_seg   <= SEG_DS;
        eu_wdata <= '0;
        for (int i = 0; i < 8; i++) rf[i] <= '0;
        sr[SEG_ES] <= '0;
        sr[SEG_CS] <= 16'hFFFF;
        sr[SEG_SS] <= '0;
        sr[SEG_DS] <= '0;
        pc      <= '0;
        arch_ip <= '0;
        psw     <= 16'hF002;
        if (bkd_load) begin
            for (int i = 0; i < 8; i++) rf[i] <= bkd_regs[i*16 +: 16];
            sr[SEG_ES] <= bkd_regs[128 +: 16];
            sr[SEG_CS] <= bkd_regs[144 +: 16];
            sr[SEG_SS] <= bkd_regs[160 +: 16];
            sr[SEG_DS] <= bkd_regs[176 +: 16];
            pc         <= bkd_regs[192 +: 16];
            arch_ip    <= bkd_regs[192 +: 16];
            psw        <= (bkd_regs[208 +: 16] & 16'h0FD5) | 16'hF002;
            state      <= S_FIRST;
        end
    end else if (ce) begin
        // ---- "every-state" block: pulse defaults + pin pipelines ----
        // These run on every enabled clock (CE), NOT on CE-low fabric
        // clocks - moved inside the CE branch so one-cycle pulses are not
        // consumed by a default that fires faster than their CE-gated
        // producers/consumers (CE desync bug #1).
        flush_now <= 1'b0;
        if (rslot != 6'd0) rslot <= rslot - 6'd1;

        // pin pipelines + NMI edge latch (run in every state)
        int_p   <= {int_p[2:0], pin_int};
        nmi_p   <= {nmi_p[3:0], pin_nmi};
        // Taken-branch recognition boundary = the FLUSH cycle: the first
        // S_FIRST after S_JFLUSH samples the pin at flush-3 (measured: on a
        // controlled JMP-short sweep the chip recognizes the INT at the
        // target one delay EARLIER than the pop-anchored boundary - it is
        // anchored to the flush, not the fetch-limited target pop). One-cycle
        // pulse the cycle after any S_JFLUSH so irq_int taps int_p[3]/ie_p[3]
        // (= pin/IE at flush-3) there instead of the normal [2].
        post_flush <= (state == S_JFLUSH);
        if (bus_tw) waits_seen <= 1'b1;
        if (nmi_p[2] && !nmi_p[3]) nmi_latch <= 1'b1;   // set at edge+3
        poll_s1 <= pin_poll_n;
        ie_p    <= {ie_p[2:0], psw[9]};
        // REP first-iteration-boundary abort decision, latched at the fixed
        // pop-anchored edge pop+7 (rslot==6) with the standard edge-4 pin
        // tap (interrupt_model.md "REP abort"); consumed by the string
        // states below for accepts that land after the edge
        if (rslot == 6'd6) rep1_abort <= irq_rep;
        if (q_pop) popm_hold <= popm_rdy;
        // 8F.0 reg ghost pop (runs in any state). QUIRK (measured, all
        // 130 golden mod3 cases): the popped DATA IS DISCARDED - only
        // SP+2 commits; the destination register is untouched (POP SP
        // sees just the increment).
        if (popr_pend && eu_done) begin
            rf[4] <= rf[4] + 16'd2;
            popr_pend <= 1'b0;
        end

        // RETI's PSW stack pop completes after the flush (runs in any state)
        if (iret_pw && eu_done) begin
            psw_old  <= psw;                     // pre-IRET image (RR2/E1)
            psw <= (eu_rdata & 16'h0FD5) | 16'hF002;
            rf[4] <= rf[4] + 16'd6;
            iret_pw <= 1'b0;
            // ARM the boundary race at IRET's own boundary. E1 proved on
            // silicon (108/108 == int9d_race.hex, H-IDENTICAL) that mu01EA's
            // flag commit obeys the SAME race table as POP PSW's mu007A; the
            // shared consumer (:5221-5235) reverts to psw_old on class B when
            // an INT lands on the boundary with pre-IE=1. Reuses psw_old/
            // pop_pend (already in the SS map) - no new flop, no SS_VERSION
            // bump. Clears one boundary later at :2291 (NOP-exempt), same as
            // POP PSW. See docs/notes/... race_rom_physical_mechanism.md E1.
            pop_pend <= 1'b1;
        end

        // POP PSW consumes the popped image at the read's own data
        // edge (measured: the new IE shows in the PS bits during T4).
        // The commit is provisional until the next boundary (pop_pend).
        if (state == S_BUSW && opc == 8'h9D && eu_rd_now) begin
            psw_old  <= psw;
            psw      <= (eu_rdata_now & 16'h0FD5) | 16'hF002;
            pop_pend <= 1'b1;
        end

        // deferred EI/DI IE write lands when the queue holds a byte
        if (ie_pend && q_any) begin
            psw[9] <= ie_val;
            ie_pend <= 1'b0;
        end

        // ---- iterative divider: one restoring shift-subtract per clock ----
        // Runs while a divide is dispatched; the microsequence is idle in
        // S_WAITX meanwhile. On the final iteration it assembles the exact
        // architectural result into mem_op/disp (byte/word/signed arranged
        // just as the old combinational path did) so S_EX retires unchanged;
        // for IDIV it also latches the quotient-magnitude flags and the
        // late-trap decision (consumed at the S_WAITX terminal cycle).
        if (div_busy) begin
            logic        shin, qbit;
            logic [16:0] rsh;
            logic [15:0] magq, magr;
            shin = div_word ? div_quo[15] : div_quo[7];
            rsh  = {div_rem[15:0], shin};
            if (rsh >= {1'b0, div_den}) begin
                div_rem <= rsh - {1'b0, div_den};
                qbit = 1'b1;
            end else begin
                div_rem <= rsh;
                qbit = 1'b0;
            end
            div_quo <= {div_quo[14:0], qbit};
            div_cnt <= div_cnt - 6'd1;
            if (div_cnt == 6'd1) begin
                // final step: magq/magr are the values just latched above
                magq = {div_quo[14:0], qbit};
                magr = qbit ? (rsh[15:0] - div_den) : rsh[15:0];
                div_busy <= 1'b0;
                if (!div_signed) begin
                    // DIVU: unsigned; flags already set by the pre-check
                    if (div_word) begin
                        mem_op <= magq;            // AW = quotient
                        disp   <= magr;            // DW = remainder
                    end else begin
                        mem_op <= {8'd0, magq[7:0]};  // AL = quotient
                        disp   <= {8'd0, magr[7:0]};  // AH = remainder
                    end
                end else begin
                    // IDIV: sign fixup + quotient-magnitude flags + late trap
                    logic [15:0] qfix, rfix;
                    logic        qneg;
                    qneg = div_nsign ^ div_dsign;
                    if (div_word) begin
                        qfix = qneg      ? (~magq + 16'd1) : magq;
                        rfix = div_nsign ? (~magr + 16'd1) : magr;
                        mem_op <= qfix;            // AW = quotient
                        disp   <= rfix;            // DW = remainder
                        div_late <= magq > 16'd32767;
                        psw[FB_S]  <= magq[15];
                        psw[FB_Z]  <= magq == 16'd0;
                        psw[FB_P]  <= ~^magq[7:0];
                    end else begin
                        logic [7:0] q8, r8;
                        q8 = qneg      ? (~magq[7:0] + 8'd1) : magq[7:0];
                        r8 = div_nsign ? (~magr[7:0] + 8'd1) : magr[7:0];
                        mem_op <= {r8, q8};        // {AH=rem, AL=quot}
                        div_late <= magq[7:0] > 8'd127;
                        psw[FB_S]  <= magq[7];
                        psw[FB_Z]  <= magq[7:0] == 8'd0;
                        psw[FB_P]  <= ~^magq[7:0];
                    end
                    psw[FB_CY] <= 1'b0;
                    psw[FB_AC] <= 1'b0;
                    psw[FB_V]  <= 1'b0;
                end
            end
        end

        // ---- iterative shift/rotate: one single-bit shift per clock ----
        // Steps the shift micro-op's operand exactly sh_n times through the
        // wait window S_SHWAIT/S_WAITX already burns; each step mirrors one
        // iteration of the old combinational `shrot` loop body (word: 16-bit
        // value; byte: active lane x plus the shift-register sibling oth).
        // On the final step it assembles the architectural result (sh_res)
        // and the fitted flag laws (sh_fl) - CY, the SHL-vs-rotate V-flag,
        // and S/Z/P/AC for the shift sub-ops - from the final state, exactly
        // as shrot did. count=0 is handled at load (sh_load) and never runs.
        if (sh_busy) begin
            logic [15:0] rN;
            logic [7:0]  xN, othN;
            logic        cyN, msb, lsb, oc, msbf;
            logic [15:0] fl;
            rN = sh_r; xN = sh_x; othN = sh_oth; cyN = sh_cy;
            if (sh_wf) begin
                msb = sh_r[15]; lsb = sh_r[0];
                unique case (sh_op)
                    3'd0: begin rN = {sh_r[14:0], msb}; cyN = msb; end
                    3'd1: begin rN = {lsb, sh_r[15:1]}; cyN = lsb; end
                    3'd2: begin oc = sh_cy; cyN = msb; rN = {sh_r[14:0], oc}; end
                    3'd3: begin oc = sh_cy; cyN = lsb; rN = {oc, sh_r[15:1]}; end
                    3'd4, 3'd6: begin cyN = msb; rN = {sh_r[14:0], 1'b0}; end
                    3'd5: begin cyN = lsb; rN = {1'b0, sh_r[15:1]}; end
                    default: begin cyN = lsb; rN = {sh_r[15], sh_r[15:1]}; end
                endcase
                sh_r <= rN;
            end else begin
                msb = sh_x[7]; lsb = sh_x[0];
                unique case (sh_op)
                    3'd0: begin xN = {sh_x[6:0], msb}; cyN = msb;
                                othN = {sh_oth[6:0], msb}; end
                    3'd1: begin xN = {lsb, sh_x[7:1]}; cyN = lsb;
                                othN = {lsb, sh_oth[7:1]}; end
                    3'd2: begin oc = sh_cy; cyN = msb; xN = {sh_x[6:0], oc};
                                othN = {sh_oth[6:0], msb}; end
                    3'd3: begin oc = sh_cy; cyN = lsb; xN = {oc, sh_x[7:1]};
                                othN = {oc, sh_oth[7:1]}; end
                    3'd4, 3'd6: begin cyN = msb; xN = {sh_x[6:0], 1'b0};
                                othN = {sh_oth[6:0], msb}; end
                    3'd5: begin cyN = lsb; xN = {1'b0, sh_x[7:1]};
                                othN = {1'b0, sh_oth[7:1]}; end
                    default: begin cyN = lsb; xN = {sh_x[7], sh_x[7:1]};
                                othN = {1'b0, sh_oth[7:1]}; end
                endcase
                sh_x   <= xN;
                sh_oth <= othN;
            end
            sh_cy <= cyN;
            sh_n  <= sh_n - 8'd1;
            if (sh_n == 8'd1) begin
                sh_busy <= 1'b0;
                msbf = sh_wf ? rN[15] : xN[7];
                fl = sh_fbase;
                fl[FB_CY] = cyN;
                if (sh_op == 3'd0 || sh_op == 3'd2 ||
                    sh_op == 3'd4 || sh_op == 3'd6)
                    fl[FB_V] = msbf ^ cyN;                 // left family
                else
                    fl[FB_V] = msbf ^ (sh_wf ? rN[14] : xN[6]);
                if (sh_op[2]) begin                        // shifts 4-7
                    fl[FB_S]  = msbf;
                    fl[FB_Z]  = sh_wf ? (rN == 16'd0) : (xN == 8'd0);
                    fl[FB_P]  = sh_wf ? (~^rN[7:0]) : (~^xN);
                    fl[FB_AC] = 1'b0;
                end
                sh_fl  <= fl;
                sh_res <= sh_wf ? rN : {othN, xN};
            end
        end

        unique case (state)
            S_HALT: ;

            //----------------------------------------------------------------
            S_FIRST: begin
                if (irq_take) begin
                    // boundary recognition (interrupt_model.md): the
                    // next instruction is not executed; its address is
                    // the pushed PC (pc already points at it)
                    if (nmi_latch) begin
                        nmi_latch <= 1'b0;
                        ivt_vec   <= 8'd2;
                        dly <= 6'd6; wnext <= S_TRAP_IVT1;
                        irq_disp <= 1'b1;
                        irq_nmi_ivt <= 1'b1;   // NMI direct boundary->IVT wait
                        state <= S_WAITX;
                    end else begin
                        state <= S_IRQ_D;   // one internal decision cycle
                    end
                end else if (q_pop) begin
                    opc <= q_byte;
                    if (!rep_en && !seg_ovr_en) insn_ip <= pc;
                    pc  <= pc + 16'd1;
                    rslot <= 6'd12;    // REP retire-slot anchor (pop+12)
                    ivt_vec  <= '0;    // divide trap uses vector 0
                    irq_disp <= 1'b0;
                    irq_nmi_ivt <= 1'b0;
                    // Recognition shadow lasts exactly ONE boundary: it is
                    // consumed as the next opcode is popped past the shadowed
                    // S_FIRST (the block this cycle is combinational, so it
                    // still holds; the clear is registered). This makes the
                    // shadow lifetime independent of the completion path -
                    // several paths (MOV reg,imm, MOV Sreg fast, far-JMP
                    // S_JFLUSH) reach S_FIRST WITHOUT retire() and so used to
                    // leak a stale sreg-load/far-CALL shadow across many
                    // instructions in fetch-limited streams, deferring INT/NMI
                    // recognition ~2 cyc too long (measured chip-vs-TB). A
                    // sreg load re-sets shadow at its own completion, after
                    // this pop, so its one-boundary shadow is preserved.
                    shadow <= 1'b0;
                    eu_kind  <= K_MEM;
                    halt_disp <= (q_byte == 8'hF4);
                    ldp2 <= 1'b0;
                    // the POP-PSW provisional window SURVIVES across
                    // intervening NOPs (measured: d=9/10 tranche cases
                    // recognized one boundary late still revert to the
                    // pre-pop image per the race table). Physically the
                    // pre-pop latch lives until the next flags write;
                    // only NOP margins are exercised by the corpus, so
                    // clear on any non-NOP opcode (wider spans are
                    // mission-S fuzz territory)
                    if (q_byte != 8'h90) pop_pend <= 1'b0;
                    // EI/DI commit IE when the NEXT opcode byte is
                    // present: at the pop edge if a byte remains, else
                    // when the queue refills (measured on dry-queue EI)
                    if (q_byte == 8'hFB || q_byte == 8'hFA) begin
                        if (q_avail2) psw[9] <= q_byte[0];
                        else begin
                            ie_pend <= 1'b1;
                            ie_val  <= q_byte[0];
                        end
                    end
                    state <= S_DEC;
                end
            end

            //----------------------------------------------------------------
            S_DEC: begin
                if (op_modrm) begin
                    if (q_pop) begin
                        mrm <= q_byte;
                        pc  <= pc + 16'd1;
                        if (q_byte[7:6] == 2'd3) begin
                            // register form
                            if (op_srst || op_srld) begin
                                // sreg MOV reg forms retire in 2 cycles
                                // (faster than reg,reg MOV - measured)
                                logic [1:0] sx;
                                sx = srmap(q_byte[4:3]);
                                if (op_srst)
                                    rf[q_byte[2:0]] <= sr[sx];
                                else
                                    sr[sx] <= rf[q_byte[2:0]];
                                // BOTH the sreg store (8C) and load (8E)
                                // shadow recognition by one boundary
                                // (measured: an 8C sreg-store skips the
                                // next boundary's INT sample exactly like
                                // 8E - controlled pushedPC sweep)
                                shadow <= 1'b1;
                                arch_ip <= pc + 16'd1;
                                seg_ovr_en <= 1'b0;
                                rep_en     <= 1'b0;
                                lock_en    <= 1'b0;
                                state <= S_FIRST;
                            end else if (op_test) begin
                                // TEST rm,reg: flags + retire ON the
                                // modrm pop (fitted, 84)
                                logic [31:0] tt16;
                                logic [23:0] tt8;
                                tt16 = alu16(3'd4, rf[q_byte[2:0]],
                                             rf[q_byte[5:3]], psw);
                                tt8 = alu8(3'd4, reg8_get(q_byte[2:0]),
                                           reg8_get(q_byte[5:3]), psw);
                                psw <= opc[0] ? tt16[31:16] : tt8[23:8];
                                arch_ip <= pc + 16'd1;
                                state <= S_FIRST;
                            end else if (op_alu | op_movs8 | op_movs16 |
                                op_movl8 | op_movl16 |
                                op_xchg8 | op_xchg16)
                                state <= S_EX;
                            else if (op_fpo) begin
                                // ESC reg: retire ON the modrm pop
                                arch_ip <= pc + 16'd1;
                                state <= S_FIRST;
                            end
                            else if (op_alui || op_shimm || op_movri)
                                state <= S_AI_I8;   // imm byte(s) next
                            else if (op_grpfe && q_byte[5:3] <= 3'd1) begin
                                dly <= 6'd1;  wnext <= S_EX; state <= S_WAITX;
                            end else if (op_grpd0 || op_grpd1) begin
                                // by-1 forms: D0.4's fitted slot reused
                                // for the whole family (fit pending)
                                sh_load(q_byte[5:3], sh_word,
                                        sh_word ? rf[q_byte[2:0]]
                                        : {8'h00, reg8_get(q_byte[2:0])},
                                        8'd1);
                                dly <= 6'd3;  wnext <= S_EX; state <= S_WAITX;
                            end else if (op_grpd2 || op_grpd3) begin
                                // by-CL reg: full 8-bit count via the
                                // shift-burn state (base fit pending)
                                sh_load(q_byte[5:3], sh_word,
                                        sh_word ? rf[q_byte[2:0]]
                                        : {8'h00, reg8_get(q_byte[2:0])},
                                        rf[1][7:0]);
                                shw <= {1'b0, rf[1][7:0]};
                                state <= S_SHWAIT;
                            end else if ((op_grpf6 || op_grpf7) &&
                                         (q_byte[5:3] == 3'd0 ||
                                          q_byte[5:3] == 3'd1)) begin
                                state <= S_AIGAP;   // TEST imm pop+2 (/1=undoc /0)
                            end else if ((op_grpf6 || op_grpf7) &&
                                         (q_byte[5:3] == 3'd2 ||
                                          q_byte[5:3] == 3'd3)) begin
                                // NOT/NEG reg (fit pending)
                                dly <= 6'd1; wnext <= S_EX;
                                state <= S_WAITX;
                            end else if (op_grpf6 && q_byte[5:3] == 3'd5) begin
                                // IMUL8 reg: +4 when operand signs differ
                                // (extra correction pass; measured).
                                // ((a^b)&80h)!=0 == a[7]^b[7]: Quartus
                                // 17.1 can't parse a bit-select applied
                                // to a function-call result.
                                dly <= (((reg8_get(q_byte[2:0])
                                          ^ rf[0][7:0]) & 8'h80) != 8'h00)
                                       ? 6'd35 : 6'd31;
                                wnext <= S_EX;
                                state <= S_WAITX;
                            end else if (op_grpf7 && q_byte[5:3] == 3'd4) begin
                                // MULU16 reg
                                dly <= 6'd28; wnext <= S_EX;
                                state <= S_WAITX;
                            end else if (op_grpf7 && q_byte[5:3] == 3'd5) begin
                                // IMUL16 reg: +4 on sign mismatch (measured)
                                dly <= (rf[q_byte[2:0]][15] ^ rf[0][15])
                                       ? 6'd42 : 6'd38;
                                wnext <= S_EX;
                                state <= S_WAITX;
                            end else if (op_imuli) begin
                                state <= S_AI_I8;   // imm then multiply
                            end else if (op_grpff &&
                                         q_byte[5:3] <= 3'd1) begin
                                // INC/DEC rm16 reg (fit pending)
                                dly <= 6'd1; wnext <= S_EX;
                                state <= S_WAITX;
                            end else if (op_popm) begin
                                // POP reg via 8F.0: retire at pop+3;
                                // the stack read completes in flight
                                // (ghost load, measured)
                                eu_addr <= {sr[SEG_SS], 4'h0} +
                                           {4'h0, rf[4]};
                                eu_seg  <= SEG_SS;
                                eu_wr   <= 1'b0;
                                eu_word <= 1'b1;
                                popr_pend <= 1'b1;
                                dly <= 6'd2; wnext <= S_POPR;
                                state <= S_WAITX;
                            end else if (op_grpff &&
                                         (q_byte[5:3] == 3'd6 ||
                                          q_byte[5:3] == 3'd7)) begin
                                // PUSH reg via FF (/7=undoc /6): write ready pop+4
                                // (closure block: the earlier phase-
                                // dependent fit was aliased by the
                                // fetch alignment; constant slot fits
                                // all four golden geometries)
                                dly <= 6'd2;
                                wnext <= S_PUSH_CALC;
                                state <= S_WAITX;
                            end else if (op_grpff &&
                                         q_byte[5:3] == 3'd2) begin
                                // CALL rm (reg): push ready pop+4
                                // (constant slot - see FF.6 note)
                                fl_ip <= rf[q_byte[2:0]];
                                fl_cs <= sr[SEG_CS];
                                dly <= 6'd2;
                                wnext <= S_CALLFL;
                                state <= S_JWAIT;
                            end else if (op_grpff &&
                                         q_byte[5:3] == 3'd4) begin
                                // BR rm (reg): fit pending
                                fl_ip <= rf[q_byte[2:0]];
                                fl_cs <= sr[SEG_CS];
                                dly <= 6'd2; wnext <= S_JFLUSH;
                                state <= S_JWAIT;
                            end else if (op_grpf6 && q_byte[5:3] == 3'd6) begin
                                // DIVU8 reg -> shared iterative unit
                                logic [7:0] den8;
                                logic       early8;
                                den8   = reg8_get(q_byte[2:0]);
                                early8 = (den8 == 8'd0) ||
                                         (rf[0][15:8] >= den8);
                                psw <= psw_sub8f(rf[0][15:8], den8, psw);
                                if (early8) begin
                                    dly <= 6'd13; wnext <= S_TRAP_IVT1;
                                end else begin
                                    div_rem <= {9'd0, rf[0][15:8]};
                                    div_quo <= {8'd0, rf[0][7:0]};
                                    div_den <= {8'd0, den8};
                                    div_cnt <= 6'd8;
                                    div_word <= 1'b0; div_signed <= 1'b0;
                                    div_busy <= 1'b1; div_pend <= 1'b1;
                                    div_late <= 1'b0;
                                    dly <= 6'd19; wnext <= S_EX;
                                end
                                state <= S_WAITX;
                            end else if (op_grpf6 && q_byte[5:3] == 3'd4) begin
                                dly <= 6'd21; wnext <= S_EX; state <= S_WAITX;
                            end else if (op_grpf7 && q_byte[5:3] == 3'd6) begin
                                // DIVU16 reg -> shared iterative unit
                                logic [15:0] den16;
                                logic        early16;
                                den16   = rf[q_byte[2:0]];
                                early16 = (den16 == 16'd0) || (rf[2] >= den16);
                                psw <= psw_sub16(rf[2], den16, psw);
                                if (early16) begin
                                    dly <= 6'd12; wnext <= S_TRAP_IVT1;
                                end else begin
                                    div_rem <= {1'b0, rf[2]};
                                    div_quo <= rf[0];
                                    div_den <= den16;
                                    div_cnt <= 6'd16;
                                    div_word <= 1'b1; div_signed <= 1'b0;
                                    div_busy <= 1'b1; div_pend <= 1'b1;
                                    div_late <= 1'b0;
                                    dly <= 6'd25; wnext <= S_EX;
                                end
                                state <= S_WAITX;
                            end else if (op_grpf7 && q_byte[5:3] == 3'd7) begin
                                // IDIV16 reg (mission I timing law):
                                // early trap IVT ready @ +21, late trap
                                // and EX @ +44; +3 if dividend < 0. Late
                                // trap vs EX decided at the S_WAITX terminal
                                // cycle (both @ +44 - only the dest differs).
                                logic [31:0] num32, anum;
                                logic [15:0] den16, ad;
                                logic [5:0]  sfix;
                                logic        early;
                                num32 = {rf[2], rf[0]};
                                den16 = rf[q_byte[2:0]];
                                anum  = num32[31] ? (~num32 + 32'd1) : num32;
                                ad    = den16[15] ? (~den16 + 16'd1) : den16;
                                sfix  = rf[2][15] ? 6'd3 : 6'd0;
                                early = (den16 == 16'd0) ||
                                        (anum[31:16] >= ad);
                                if (early) begin
                                    psw <= psw_sub16(anum[31:16], ad, psw);
                                    dly <= 6'd21 + sfix; wnext <= S_TRAP_IVT1;
                                end else begin
                                    div_rem <= {1'b0, anum[31:16]};
                                    div_quo <= anum[15:0];
                                    div_den <= ad;
                                    div_cnt <= 6'd16;
                                    div_word <= 1'b1; div_signed <= 1'b1;
                                    div_nsign <= num32[31];
                                    div_dsign <= den16[15];
                                    div_busy <= 1'b1; div_pend <= 1'b1;
                                    div_late <= 1'b0;
                                    dly <= 6'd44 + sfix; wnext <= S_EX;
                                end
                                state <= S_WAITX;
                            end else if (op_grpf6 && q_byte[5:3] == 3'd7) begin
                                // IDIV8 reg: early @ +21, late @ +36,
                                // EX @ +37; +3 if dividend < 0. Byte form:
                                // late trap retires one cycle before EX
                                // (handled at the S_WAITX terminal).
                                logic [15:0] anum16;
                                logic  [7:0] den8, ad8;
                                logic [23:0] t8;
                                logic [5:0]  sfix;
                                logic        early;
                                den8   = reg8_get(q_byte[2:0]);
                                anum16 = rf[0][15] ? (~rf[0] + 16'd1) : rf[0];
                                ad8    = den8[7] ? (~den8 + 8'd1) : den8;
                                sfix   = rf[0][15] ? 6'd3 : 6'd0;
                                early  = (den8 == 8'd0) ||
                                         (anum16[15:8] >= ad8);
                                if (early) begin
                                    t8  = alu8(3'd5, anum16[15:8], ad8, psw);
                                    psw <= t8[23:8];
                                    dly <= 6'd21 + sfix; wnext <= S_TRAP_IVT1;
                                end else begin
                                    div_rem <= {9'd0, anum16[15:8]};
                                    div_quo <= {8'd0, anum16[7:0]};
                                    div_den <= {8'd0, ad8};
                                    div_cnt <= 6'd8;
                                    div_word <= 1'b0; div_signed <= 1'b1;
                                    div_nsign <= rf[0][15];
                                    div_dsign <= den8[7];
                                    div_busy <= 1'b1; div_pend <= 1'b1;
                                    div_late <= 1'b0;
                                    dly <= 6'd37 + sfix; wnext <= S_EX;
                                end
                                state <= S_WAITX;
                            end else
                                state <= S_HALT;
                        end else begin
                            // memory form; group ops with an unimplemented
                            // /reg field park the sequencer. F6/F7 /1 (=/0 TEST)
                            // and FF /7 (=/6 PUSH) are undoc aliases (v20) and
                            // NO LONGER park - they proceed to EA/read and are
                            // canonicalised by mrm_reg downstream.
                            if ((op_grpfe && q_byte[5:3] > 3'd1) ||
                                (op_popm && q_byte[5:3] != 3'd0))
                                state <= S_HALT;
                            else if ((q_byte[7:6] == 2'd0 &&
                                      q_byte[2:0] == 3'd6) ||
                                     q_byte[7:6] == 2'd2)
                                state <= S_DLO;
                            else
                                state <= S_EA1;   // mod0 reg-EA / mod1
                        end
                    end
                end else begin
                    // no modrm
                    if (op_0f) state <= S_0F;       // 2nd byte pops at F+2
                    else if (opc[7:3] == 5'b10111) state <= S_IMM_LO; // B8-BF
                    else if (opc[7:3] == 5'b10110) state <= S_IMM8;   // B0-B7
                    else if (opc[7:3] == 5'b01000) begin            // INC r16
                        {psw, rf[opc[2:0]]} <=
                            incdec16(1'b0, rf[opc[2:0]], psw);
                        retire();
                    end else if (opc[7:3] == 5'b01001) begin        // DEC r16
                        {psw, rf[opc[2:0]]} <=
                            incdec16(1'b1, rf[opc[2:0]], psw);
                        retire();
                    end else if (opc == 8'h90) state <= S_NOP;
                    else if (opc[7:3] == 5'b01010) state <= S_PUSH_CALC;
                    else if (opc == 8'h60) begin      // PUSH R
                        // first write ready pop+4 (measured)
                        issue_push(rf[0]);            // AW first
                        a4_cnt <= 8'd1;
                        dly <= 6'd2; wnext <= S_REQ;
                        state <= S_WAITX;
                    end else if (opc == 8'h61) begin  // POP R
                        eu_addr <= {sr[SEG_SS], 4'h0} + {4'h0, rf[4]};
                        eu_seg  <= SEG_SS;
                        eu_wr   <= 1'b0;
                        eu_word <= 1'b1;
                        mem_op  <= rf[4];             // 16-bit offset walk
                        pracc   <= 3'd0;
                        a4_cnt  <= 8'd0;
                        // The first stack read commits via the natural BIU
                        // eval-point mechanism (identical to POP r16 / S_REQ):
                        // measured A/B, POPA's chip read-start cycle equals a
                        // plain POP r16's at every prefetch phase. The old
                        // `bus_phase ? S_61G : S_61W` split forced a +1
                        // lead-in on every odd-parity dispatch, which only
                        // matches the chip when S_DEC lands on a fetch T4;
                        // it read 1-2 cycles late at T2/Ti phases. Dispatch
                        // straight to S_61W: its request-commit is phase-
                        // natural (S_61G retained only for its transition).
                        state   <= S_61W;
                    end
                    else if (opc[7:3] == 5'b01011 ||
                             opc == 8'hC3) begin      // POP r16 / RET
                        eu_addr <= {sr[SEG_SS], 4'h0} + {4'h0, rf[4]};
                        eu_seg  <= SEG_SS;
                        eu_wr   <= 1'b0;
                        eu_word <= 1'b1;
                        state   <= S_REQ;
                    end else if (opc == 8'hEB || op_jcc ||
                                 opc == 8'hE2 || opc == 8'hE3)
                        state <= S_JDISP;
                    else if (opc == 8'hE0 || opc == 8'hE1) begin
                        // DBNZNE/DBNZE: disp pops at F+4 (measured -
                        // two-cycle decode lead-in)
                        dly <= 6'd2; wnext <= S_JDISP; state <= S_WAITX;
                    end
                    else if (opc == 8'hE9 || opc == 8'hE8 ||
                             opc == 8'hC2 || opc == 8'hEA ||
                             opc == 8'hCA || opc == 8'h9A)
                        state <= S_JDLO;
                    else if (op_moff || op_moffw) state <= S_MLO;
                    else if (op_segp) begin
                        // segment override: retires as its own
                        // instruction (2 cycles, own F pop); the latch
                        // lives until the prefixed instruction retires
                        seg_ovr_en <= 1'b1;
                        seg_ovr    <= srmap(opc[4:3]);
                        arch_ip    <= pc;
                        state      <= S_FIRST;
                    end else if (op_repp) begin
                        rep_en  <= 1'b1;
                        rep_kind <= opc == 8'hF3 ? 2'd0 :
                                    opc == 8'hF2 ? 2'd1 :
                                    opc == 8'h65 ? 2'd2 : 2'd3;
                        arch_ip <= pc;
                        state   <= S_FIRST;
                    end else if (op_lockp) begin
                        // LOCK (F0): retires as its own 2-cycle instruction
                        // (own F pop); the latch lives until the locked
                        // instruction retires and drives BUSLOCK_N via the
                        // BIU (Stage 6). Transparent to prefetch (measured).
                        lock_en <= 1'b1;
                        arch_ip <= pc;
                        state   <= S_FIRST;
                    end else if (opc == 8'hF5) begin      // NOT1 CY
                        psw[0] <= ~psw[0];
                        retire();                         // close pop+2
                    end else if (opc == 8'hF8 || opc == 8'hF9) begin
                        psw[0] <= opc[0];                 // CLR1/SET1 CY
                        retire();
                    end else if (opc == 8'hFC || opc == 8'hFD) begin
                        psw[10] <= opc[0];                // CLR1/SET1 DIR
                        retire();
                    end else if (opc == 8'h9F) begin      // MOV AH, PSW
                        rf[0][15:8] <= psw[7:0];
                        retire();
                    end else if (opc == 8'h9E) begin      // MOV PSW, AH
                        psw[7:0] <= (rf[0][15:8] & 8'hD5) | 8'h02;
                        state <= S_NOP;   // SAHF closes pop+3
                    end else if (op_xchga) begin          // XCH AW, reg
                        rf[0] <= rf[opc[2:0]];
                        rf[opc[2:0]] <= rf[0];
                        state <= S_NOP;                   // fit pending
                    end else if (op_accimm || op_testai || op_pushi) begin
                        state <= S_AI_I8;                 // imm byte(s)
                    end else if (opc == 8'h9C || op_pushsr) begin
                        // PUSH PSW / sreg: r16-push pattern (write
                        // ready pop+2, fitted on 06/0E/16/1E)
                        state <= S_PUSH_CALC;
                    end else if (op_popsr) begin          // POP sreg
                        eu_addr <= {sr[SEG_SS], 4'h0} + {4'h0, rf[4]};
                        eu_seg  <= SEG_SS;
                        eu_wr   <= 1'b0;
                        eu_word <= 1'b1;
                        state   <= S_REQ;
                    end else if (opc == 8'hCB || op_iret) begin
                        // RETF / RETI: first stack word (fit pending)
                        fret_ph <= 2'd0;
                        eu_addr <= {sr[SEG_SS], 4'h0} + {4'h0, rf[4]};
                        eu_seg  <= SEG_SS;
                        eu_wr   <= 1'b0;
                        eu_word <= 1'b1;
                        state   <= S_REQ;
                    end else if (op_prep) begin           // PREPARE
                        state <= S_AI_I8;   // size16 + level8 follow
                    end else if (op_disp) begin           // DISPOSE
                        eu_addr <= {sr[SEG_SS], 4'h0} + {4'h0, rf[5]};
                        eu_seg  <= SEG_SS;
                        eu_wr   <= 1'b0;
                        eu_word <= 1'b1;
                        state   <= S_REQ;
                    end else if (opc == 8'hCC) begin      // BRK3
                        ivt_vec <= 8'd3;
                        dly <= 6'd5; wnext <= S_TRAP_IVT1;   // fit
                        state <= S_WAITX;
                    end else if (opc == 8'hCD) begin      // BRK imm8
                        state <= S_INTV;
                    end else if (opc == 8'hCE) begin      // BRKV
                        if (psw[11]) begin
                            ivt_vec <= 8'd4;
                            dly <= 6'd6; wnext <= S_TRAP_IVT1;
                            state <= S_WAITX;
                        end else state <= S_NOP;             // fit
                    end else if (opc == 8'h27 || opc == 8'h2F) begin
                        // ADJ4A/ADJ4S: close pop at +3 (fitted, 27)
                        state <= S_EX;
                    end else if (opc == 8'h37 || opc == 8'h3F) begin
                        // ADJBA/ADJBS: close pop at +7 (fitted, 37)
                        dly <= 6'd4; wnext <= S_EX; state <= S_WAITX;
                    end else if (opc == 8'hD4 || opc == 8'hD5) begin
                        state <= S_BCD_IMM;   // base byte pops next
                    end else if (opc == 8'h98) begin      // CVTBW
                        rf[0][15:8] <= {8{rf[0][7]}};
                        retire();
                    end else if (opc == 8'h99) begin      // CVTWL
                        dly <= 6'd2; wnext <= S_EX; state <= S_WAITX;
                    end else if (op_xlat) begin           // TRANS
                        eu_addr <= {sr[seg_ovr_en ? seg_ovr : SEG_DS],
                                    4'h0} +
                                   {4'h0, rf[3] + {8'd0, rf[0][7:0]}};
                        eu_seg  <= seg_ovr_en ? seg_ovr : SEG_DS;
                        eu_wr   <= 1'b0;
                        eu_word <= 1'b0;
                        dly <= 6'd1; state <= S_RSV;
                    end else if (op_str) begin
                        if (rep_en && rf[1] == 16'd0) begin
                            // REP with CW=0: uniform early-out
                            dly <= 6'd9; wnext <= S_EX; state <= S_WAITX;
                        end else if (op_stostr) begin
                            eu_addr <= {sr[SEG_ES], 4'h0} + {4'h0, rf[7]};
                            eu_seg  <= SEG_ES;
                            eu_wr   <= 1'b1;
                            eu_word <= opc[0];
                            eu_wdata <= rf[0];
                            // REP setup: first access 2 cycles later
                            // than the single form, and the extra wait
                            // does NOT reserve the bus (measured: a
                            // prefetch commits inside it)
                            if (rep_en) begin
                                dly <= 6'd2; wnext <= S_RSV;
                                state <= S_WAITX;
                            end else begin
                                dly <= 6'd1; state <= S_RSV;
                            end
                        end else if (op_scastr) begin     // CMPM: ES:IY rd
                            eu_addr <= {sr[SEG_ES], 4'h0} + {4'h0, rf[7]};
                            eu_seg  <= SEG_ES;
                            eu_wr   <= 1'b0;
                            eu_word <= opc[0];
                            if (rep_en) begin
                                dly <= 6'd2; wnext <= S_RSV;
                                state <= S_WAITX;
                            end else begin
                                dly <= 6'd1; state <= S_RSV;
                            end
                        end else if (op_outstr) begin     // OUTS: DS(sov):IX rd
                            // read the source element from DS(sov):IX
                            // first (LODS-style); the IOW to port DW
                            // follows at S_REQ with the read data
                            // forwarded (eu_fwd). Cycle timing is out of
                            // scope (no native 6E/6F captures); arch only.
                            eu_addr <= {sr[seg_ovr_en ? seg_ovr : SEG_DS],
                                        4'h0} + {4'h0, rf[6]};
                            eu_seg  <= seg_ovr_en ? seg_ovr : SEG_DS;
                            eu_wr   <= 1'b0;
                            eu_word <= opc[0];
                            eu_kind <= K_MEM;
                            if (rep_en) begin  // REP setup (see STM note)
                                dly <= 6'd2; wnext <= S_RSV;
                                state <= S_WAITX;
                            end else begin
                                dly <= 6'd1; state <= S_RSV;
                            end
                        end else if (op_instr) begin      // INS: port DW IOR
                            // read the port (IOR) first; the mem write to
                            // ES:IY (no segment override on the dest)
                            // follows at S_REQ with the port data
                            // forwarded (eu_fwd). Arch only (no 6C/6D
                            // native captures).
                            eu_addr <= {4'h0, rf[2]};
                            eu_seg  <= SEG_CS;
                            eu_wr   <= 1'b0;
                            eu_word <= opc[0];
                            eu_kind <= K_IO;
                            if (rep_en) begin  // REP setup (see STM note)
                                dly <= 6'd2; wnext <= S_RSV;
                                state <= S_WAITX;
                            end else begin
                                dly <= 6'd1; state <= S_RSV;
                            end
                        end else begin              // MOVBK / LDM / CMPBK
                            // REP CMPBK reads ES:IY FIRST, then DS:IX
                            // (measured - the single form reads DS:IX
                            // first); MOVBK/LDM/single-CMPBK read the
                            // DS(sov):IX side first
                            if (op_cmpstr && rep_en) begin
                                eu_addr <= {sr[SEG_ES], 4'h0} +
                                           {4'h0, rf[7]};
                                eu_seg  <= SEG_ES;
                            end else begin
                                eu_addr <= {sr[seg_ovr_en ? seg_ovr
                                               : SEG_DS],
                                            4'h0} + {4'h0, rf[6]};
                                eu_seg  <= seg_ovr_en ? seg_ovr : SEG_DS;
                            end
                            eu_wr   <= 1'b0;
                            eu_word <= opc[0];
                            str_wr  <= 1'b0;
                            cmp_r2s <= 1'b0;
                            if (rep_en) begin  // REP setup (see STM note)
                                dly <= 6'd2; wnext <= S_RSV;
                                state <= S_WAITX;
                            end else begin
                                dly <= 6'd1; state <= S_RSV;
                            end
                        end
                    end
                    else if (opc == 8'hF4) begin          // HALT
                        state   <= S_HALTED;   // BIU displays the
                                               // pseudo-cycle (halt_disp)
                    end else if (opc == 8'h9B) begin      // POLL
                        // initial check: synchronized pin; the wait
                        // loop samples LIVE (measured: a release in the
                        // sample cycle itself is caught)
                        if (!poll_s1) state <= S_NOP;     // low: 3-cyc nop
                        else begin
                            dly <= 6'd3;                  // 1st sample @F+4
                            state <= S_POLL_WAIT;
                        end
                    end else if (opc == 8'hFB ||
                                 opc == 8'hFA) begin      // EI / DI
                        retire();     // IE was written at the pop edge
                    end else if (opc == 8'h9D) begin      // POP PSW
                        eu_addr <= {sr[SEG_SS], 4'h0} + {4'h0, rf[4]};
                        eu_seg  <= SEG_SS;
                        eu_wr   <= 1'b0;
                        eu_word <= 1'b1;
                        state   <= S_REQ;
                    end else if (op_in || op_out) begin   // IN / OUT
                        if (opc[3]) begin            // EC/ED/EE/EF: port=DW
                            eu_addr <= {4'h0, rf[2]};
                            eu_seg  <= SEG_CS;
                            eu_wr   <= op_out;
                            eu_word <= opc[0];
                            eu_kind <= K_IO;
                            if (op_out) eu_wdata <= rf[0];
                            state   <= S_REQ;
                        end else
                            state <= S_IN_PORT;           // imm8 pops @F+2
                    end
                    else state <= S_HALT;
                end
            end

            //----------------------------------------------------------------
            S_IMM_LO: if (q_pop) begin
                disp[7:0] <= q_byte;
                pc <= pc + 16'd1;
                state <= S_IMM_HI;
            end

            //----------------------------------------------------------------
            // ALU rm,imm (80/81/83): imm byte(s) pop after the modrm/
            // displacement (reg forms) or after the operand read (mem
            // forms); execute in S_EX / write via S_RMWX. Structural
            // timing first pass - fit against the 80/81/83 goldens.
            //----------------------------------------------------------------
            S_TESTGAP: state <= S_AIGAP;   // TEST-imm mem extra gap
            S_AIGAP: state <= S_AI_I8;     // mem imm forms: pop done+2
            S_SHWAIT: begin
                // one cycle per count step, then the fitted base
                // slots. C0/C1 anchor at the count-imm pop (= mem
                // read done+2); D2/D3 anchor at the modrm pop / read
                // done, one slot later (fitted: D2.0 reg pop+9+CL,
                // +8 at 0; mem write T1 = read done+11+CL, cl=0
                // close done+9)
                if (shw != 9'd0) shw <= shw - 9'd1;
                else if (mrm_mod != 2'd3 && sh_cnt != 8'd0) begin
                    dly <= op_shimm ? 6'd5 : 6'd7;
                    state <= S_RMWX;
                end else if (mrm_mod != 2'd3) begin
                    dly <= op_shimm ? 6'd4 : 6'd6;
                    wnext <= S_EX; state <= S_WAITX;
                end else begin
                    dly <= op_shimm ? ((sh_cnt == 8'd0) ? 6'd4 : 6'd5)
                                    : ((sh_cnt == 8'd0) ? 6'd5 : 6'd6);
                    wnext <= S_EX; state <= S_WAITX;
                end
            end
            S_AI_I8: if (q_pop) begin
                disp[7:0] <= q_byte;
                pc <= pc + 16'd1;
                if (op_movri) begin
                    // MOV r/m,imm: C7 (word) pops a second imm byte; C6
                    // (byte) either writes the reg (mod3) or stores to the
                    // latched EA. Cadence fitted vs the C6.0 goldens.
                    if (opc[0]) state <= S_AI_I16;
                    else if (mrm_mod == 2'd3) begin
                        // C6 reg8: same one-idle-cycle tail as B0 (byte
                        // imm -> reg8); S_NOP supplies the extra cycle.
                        wr_reg8(mrm_rm, q_byte);
                        state <= S_NOP;
                    end else begin
                        // C6 byte store: reserve the bus (S_RSV) so the
                        // prefetcher cannot steal the write slot; write data
                        // is the sign-extended imm8 (unused lane = {8{s7}}).
                        eu_wr    <= 1'b1;
                        eu_wdata <= {{8{q_byte[7]}}, q_byte};
                        dly <= 6'd1; state <= S_RSV;
                    end
                end else if (op_grp81 || ((op_accimm || op_testai) && opc[0]) ||
                    opc == 8'h68 || opc == 8'h69 || op_prep ||
                    (op_grpf7 && mrm_reg == 3'd0))
                    state <= S_AI_I16;
                else if (op_pushi) begin              // 6A: push simm8
                    issue_push({{8{q_byte[7]}}, q_byte});
                    state <= S_REQ;
                end else if (op_accimm || op_testai)
                    state <= S_EX;
                else if (op_grpf6 && mrm_reg == 3'd0)
                    state <= S_EX;
                else if (op_imuli) begin
                    // 6B: +4 on sign mismatch rm vs simm8 (measured)
                    dly <= (q_byte[7] ^ ((mrm_mod == 2'd3)
                            ? rf[mrm_rm][15] : mem_op[15]))
                           ? 6'd40 : 6'd36;
                    wnext <= S_EX; state <= S_WAITX;
                end else if (op_alui &&
                             (mrm_mod == 2'd3 || mrm_reg == 3'd7)) begin
                    // reg forms + CMP-mem retire at imm-pop+4 (80.0)
                    dly <= 6'd2; wnext <= S_EX; state <= S_WAITX;
                end else if (op_shimm) begin
                    // C0/C1 fitted laws (FULL 8-bit count, no
                    // masking): mem write T1 = count-pop + 9 + count
                    // (count=0 writes nothing); reg close =
                    // count-pop + 8 + count (+7 at 0)
                    sh_load(mrm_reg, sh_word, sh_operand, q_byte);
                    shw <= {1'b0, q_byte};
                    state <= S_SHWAIT;
                end else if (mrm_mod != 2'd3 && mrm_reg != 3'd7) begin
                    dly <= 6'd3; state <= S_RMWX;   // write ready pop+4
                end else state <= S_EX;
            end
            S_BCD_IMM: if (q_pop) begin        // D4/D5 base byte
                disp[7:0] <= q_byte;
                pc <= pc + 16'd1;
                // CVTBD divides, CVTDB multiply-adds (fit pending)
                dly <= (opc == 8'hD4) ? 6'd11 : 6'd4;
                wnext <= S_EX; state <= S_WAITX;
            end
            S_AI_I16: if (q_pop) begin
                disp[15:8] <= q_byte;
                pc <= pc + 16'd1;
                if (op_movri) begin        // MOV rm16,imm16 (C7)
                    if (mrm_mod == 2'd3) begin
                        rf[mrm_rm] <= {q_byte, disp[7:0]};
                        arch_ip <= pc + 16'd1;
                        state <= S_FIRST;
                    end else begin
                        // C7 word store: the write is ready at the imm-hi
                        // pop's T3 eval (go straight to S_REQ; the pop-cycle
                        // reservation holds the bus). Measured: the write
                        // commits one eval earlier than an S_RSV lead-in.
                        eu_wr    <= 1'b1;
                        eu_wdata <= {q_byte, disp[7:0]};
                        state <= S_REQ;
                    end
                end else if (op_pushi) begin
                    // 68: the push write commits at the next phase-0 grid
                    // cycle. From a phase-1 imm pop it is ready pop+1
                    // (S_REQ direct). From a phase-0 pop with an in-flight
                    // prefetch it takes the S_PUSH_CALC reservation (pop+2)
                    // so the fetch cannot steal the slot. But a phase-0 pop
                    // in a bus-idle window (Ti) has no fetch to block: the
                    // extra calc cycle wrote 1 cycle late (Campaign 5 ph4/10)
                    // - commit pop+1 there too.
                    disp[15:8] <= q_byte;
                    if (bus_phase || bus_ts == 3'd0) begin
                        issue_push({q_byte, disp[7:0]});
                        state <= S_REQ;
                    end else
                        state <= S_PUSH_CALC;
                end else if (op_prep) begin
                    // PREPARE: BP push ready hi-pop+1; level byte
                    // pops at hi-pop+4 (measured)
                    issue_push(rf[5]);
                    cmp1 <= rf[4] - 16'd2;            // frame pointer
                    prep_acc <= 1'b0;
                    prep_bpd <= 1'b0;
                    dly <= 6'd3;
                    state <= S_PREP_L;
                end else if (op_accimm || op_testai) begin
                    // word acc-imm: execute AND retire on the hi-imm
                    // pop edge (close = open+4, fitted on 05)
                    logic [31:0] c16a;
                    logic [2:0] aop2;
                    aop2 = op_testai ? 3'd4 : opc[5:3];
                    c16a = alu16(aop2, rf[0], {q_byte, disp[7:0]}, psw);
                    psw <= c16a[31:16];
                    if (!op_testai && aop2 != 3'd7)
                        rf[0] <= c16a[15:0];
                    // retire on the pop edge (B8 pattern): pc is
                    // incremented THIS cycle, so arch_ip needs +1
                    arch_ip <= pc + 16'd1;
                    state <= S_FIRST;
                end else if (op_grpf7 && mrm_reg == 3'd0) begin
                    // TEST rm16,imm16: flags + retire ON the hi pop
                    logic [31:0] tw16;
                    tw16 = alu16(3'd4,
                                 (mrm_mod == 2'd3) ? rf[mrm_rm] : mem_op,
                                 {q_byte, disp[7:0]}, psw);
                    psw <= tw16[31:16];
                    arch_ip <= pc + 16'd1;
                    state <= S_FIRST;
                end else if (op_imuli) begin
                    // 69: +4 on sign mismatch rm vs imm16 (measured)
                    dly <= (q_byte[7] ^ ((mrm_mod == 2'd3)
                            ? rf[mrm_rm][15] : mem_op[15]))
                           ? 6'd39 : 6'd35;
                    wnext <= S_EX; state <= S_WAITX;
                end else if (op_alui &&
                             (mrm_mod == 2'd3 || mrm_reg == 3'd7)) begin
                    // 81.x reg/CMP-mem retire at imm-pop+3 (one
                    // earlier than the byte-imm forms; measured)
                    dly <= 6'd1; wnext <= S_EX; state <= S_WAITX;
                end else if (mrm_mod != 2'd3 && mrm_reg != 3'd7) begin
                    dly <= 6'd1; state <= S_RMWX;   // write ready pop+2
                end else state <= S_EX;
            end
            S_IMM_HI: if (q_pop) begin
                rf[opc[2:0]] <= {q_byte, disp[7:0]};
                pc <= pc + 16'd1;
                arch_ip <= pc + 16'd1;   // retire on the same edge as the pop
                state <= S_FIRST;
            end

            // MOV reg8, imm8 (B0-B7): single imm byte -> reg8. Unlike the
            // two-byte B8-BF (which retire ON the imm-hi pop), the byte-imm
            // form inserts ONE extra idle cycle before the next opcode can
            // pop (measured on the B0 goldens: the closing F pops one Ti
            // later than a naive pop-edge retire; absorbed when the queue is
            // dry, visible on prefetched variants). S_NOP supplies the cycle.
            S_IMM8: if (q_pop) begin
                wr_reg8(opc[2:0], q_byte);
                pc <= pc + 16'd1;
                state <= S_NOP;
            end

            S_NOP: retire();

            //----------------------------------------------------------------
            // control flow (mission E; offsets from the golden tranches):
            //   EB:  disp8 pop P -> flush @ P+3
            //   E9:  disp16 hi pop H -> flush @ H+3
            //   Jcc: taken flush @ P+4; not-taken retire end of P+1
            //   E2:  taken flush @ P+6; not-taken retire end of P+2
            //   E8:  push ready @ H+4, flush during the push status cycle
            //   C3/C2: SP read; flush @ done+1 (C2: imm16 pops first,
            //          read ready @ H+1)
            //----------------------------------------------------------------
            S_JDISP: if (q_pop) begin
                pc   <= pc + 16'd1;
                disp <= pc + 16'd1 + {{8{q_byte[7]}}, q_byte};  // target
                if (opc == 8'hEB) begin
                    dly <= 6'd2; wnext <= S_JFLUSH; state <= S_JWAIT;
                end else if (opc == 8'hE2) begin                // DBNZ
                    rf[1] <= rf[1] - 16'd1;
                    if (rf[1] != 16'd1) begin
                        dly <= 6'd5; wnext <= S_JFLUSH; state <= S_JWAIT;
                    end else begin
                        dly <= 6'd1; wnext <= S_JNT; state <= S_JWAIT;
                    end
                end else if (opc == 8'hE0 || opc == 8'hE1) begin
                    // DBNZNE/DBNZE (fitted: disp pop F+4; not-taken
                    // closes at pop+1, taken at open+16)
                    rf[1] <= rf[1] - 16'd1;
                    if (rf[1] != 16'd1 &&
                        (psw[FB_Z] == opc[0])) begin
                        dly <= 6'd4; wnext <= S_JFLUSH; state <= S_JWAIT;
                    end else begin
                        arch_ip <= pc + 16'd1;
                        state <= S_FIRST;
                    end
                end else if (opc == 8'hE3) begin                // BCWZ
                    // fitted: taken close open+15, not-taken pop+3
                    if (rf[1] == 16'd0) begin
                        dly <= 6'd5; wnext <= S_JFLUSH; state <= S_JWAIT;
                    end else begin
                        dly <= 6'd1; wnext <= S_JNT; state <= S_JWAIT;
                    end
                end else begin                                  // Jcc
                    if (jcc_taken) begin
                        dly <= 6'd3; wnext <= S_JFLUSH; state <= S_JWAIT;
                    end else state <= S_JNT;
                end
            end
            S_JDLO: if (q_pop) begin
                disp[7:0] <= q_byte;
                pc <= pc + 16'd1;
                state <= S_JDHI;
            end
            S_JDHI: if (q_pop) begin
                pc <= pc + 16'd1;
                if (opc == 8'hEA || opc == 8'h9A) begin
                    disp[15:8] <= q_byte;         // far target offset
                    state <= S_JSLO;              // seg words follow
                end else if (opc == 8'hC2 || opc == 8'hCA) begin
                    disp[15:8] <= q_byte;                       // pop count
                    fret_ph <= 2'd0;
                    eu_addr <= {sr[SEG_SS], 4'h0} + {4'h0, rf[4]};
                    eu_seg  <= SEG_SS;
                    eu_wr   <= 1'b0;
                    eu_word <= 1'b1;
                    state   <= S_REQ;                           // rdy @ H+1
                end else begin
                    disp <= pc + 16'd1 + {q_byte, disp[7:0]};   // target
                    if (opc == 8'hE9) begin
                        dly <= 6'd2; wnext <= S_JFLUSH; state <= S_JWAIT;
                    end else begin                 // E8: flush @ H+3 like E9,
                        dly <= 6'd2; wnext <= S_CALLFL; state <= S_JWAIT;
                    end                            // push ready @ H+4
                end
            end
            // branch-resolution wait (holds a bus reservation)
            S_JWAIT: begin
                // Jcc-w3 flush (+1-late redirect when queue full) DEFERRED: the
                // EU transition-delay approach over-shoots (the BIU redirect-
                // commit eval adds a 2nd idle); it needs a BIU-side one-idle
                // insertion on the Jcc flush redirect. Characterized, not landed.
                if (dly == 6'd1) begin
                    state <= wnext;
                    // near transfers redirect within CS; EA (far) has
                    // already latched its target in S_JSHI
                    // (FF rm forms latch fl_ip/fl_cs at dispatch)
                    if ((wnext == S_JFLUSH || wnext == S_CALLFL) &&
                        opc != 8'hEA && !op_grpff) begin
                        fl_cs <= sr[SEG_CS];
                        fl_ip <= disp;
                    end
                end
                dly <= dly - 6'd1;
            end
            S_JNT: retire();                                    // not taken
            // BR far (EA) segment bytes; flush at seghi-pop+2 (measured
            // on the boot capture; reservation from pop+1 via S_JWAIT)
            S_JSLO: if (q_pop) begin
                immb <= q_byte;
                pc   <= pc + 16'd1;
                state <= S_JSHI;
            end
            S_JSHI: if (q_pop) begin
                pc    <= pc + 16'd1;
                fl_cs <= {q_byte, immb};
                fl_ip <= disp;
                if (opc == 8'h9A) begin
                    // CALL far: PS push ready pop+4 (pop+5 when the
                    // seg-hi pop rides a bus T4 - a freshly pushed
                    // byte), then the FF.3 tail (flush at
                    // write-done+1, PC push at the flush end)
                    issue_push(sr[SEG_CS]);
                    dly <= bus_t4 ? 6'd4 : 6'd3;
                    wnext <= S_REQ;
                    state <= S_WAITX;
                end else begin
                    // EA: flush at pop+3 (measured)
                    dly <= 6'd2; wnext <= S_JFLUSH; state <= S_JWAIT;
                end
            end
            S_FCALLFL: begin     // q_flush high this cycle (comb)
                issue_push(sr[SEG_CS]);           // PS first, then PC
                mem_op <= pc;                     // return offset
                pc <= fl_ip;
                sr[SEG_CS] <= fl_cs;
                state <= S_FCALLP1;
            end
            S_FCFL2: begin       // FF.3 flush cycle (PC push pending)
                pc <= fl_ip;
                sr[SEG_CS] <= fl_cs;
                state <= S_CALLPUSH;
            end
            S_FCALLP1: if (eu_started) begin
                issue_push(mem_op);
                state <= S_FCALLP2;
            end
            S_FCALLP2: if (eu_started) state <= S_CALLW;
            //----------------------------------------------------------------
            // PREPARE (C8 size16, level8): push BP; for level>0 read
            // level-1 frame temps at BP-2k and push them, then push the
            // frame pointer; BP=frame, SP=frame-size. Structural
            // timing, fit pending the C8 goldens.
            //----------------------------------------------------------------
            S_PREP_L: begin
                if (eu_started) prep_acc <= 1'b1;
                if (eu_done) prep_bpd <= 1'b1;
                if (dly != 6'd0) dly <= dly - 6'd1;
                else if (q_pop) begin
                    a4_k <= {3'd0, q_byte[4:0]};  // level (mod 32)
                    pc <= pc + 16'd1;
                    a4_cnt <= 8'd1;
                    w4skip <= 1'b0;
                    if (q_byte[4:0] == 5'd0) begin
                        // retire at max(level-pop+4, push done) -
                        // fitted on all four level-0 geometries
                        // (queue-limited cases had masked the floor)
                        dly   <= 6'd3;
                        state <= S_PREP_W2;       // await BP push done
                    end
                    else begin
                        // frame push ready pop+7 (level 1); first
                        // pointer-copy read ready pop+7 with the bus
                        // reserved through the BP push (closure block:
                        // both split geometries pin the slot)
                        dly <= (q_byte[4:0] == 5'd1) ? 6'd6 : 6'd6;
                        wnext <= (q_byte[4:0] == 5'd1) ? S_PREP_W3A
                                                       : S_PREP_RDGO;
                        state <= S_WAITX;
                    end
                end
            end
            S_PREP_W2: begin
                if (eu_done) prep_bpd <= 1'b1;
                if (dly != 6'd0) dly <= dly - 6'd1;
                else if (eu_done || prep_bpd) begin // BP push done
                    rf[5] <= cmp1;
                    rf[4] <= rf[4] - disp;
                    retire();
                end
            end
            S_PREP_W3A: begin                     // frame push (level 1)
                issue_push(cmp1);
                state <= S_PREP_W3;
            end
            S_PREP_RDGO: if (eu_started) begin    // first pointer copy read
                issue_push(16'h0);                // data forwarded (eu_fwd)
                state <= S_PREP_PW2;
            end else
                state <= S_PREP_RD;               // not accepted: keep
                                                  // the request up
            // pointer copies pipeline read->write->read back-to-back:
            // the copy write commits at the read's T3 end with BIU
            // data forwarding; the next read (or the frame push)
            // chains at the write's T3 end (measured, level>=2)
            S_PREP_RD: if (eu_started) begin      // read accepted
                issue_push(16'h0);                // data forwarded (eu_fwd)
                state <= S_PREP_PW2;
            end
            S_PREP_PW2: if (eu_started) begin     // copy write accepted
                if ({1'b0, a4_cnt} < {1'b0, a4_k} - 9'd1) begin
                    a4_cnt <= a4_cnt + 8'd1;
                    eu_addr <= {sr[SEG_SS], 4'h0} +
                               {4'h0, rf[5] - 16'd2 -
                                {7'd0, a4_cnt, 1'b0}};
                    eu_seg <= SEG_SS; eu_wr <= 1'b0; eu_word <= 1'b1;
                    state <= S_PREP_RD;
                end else begin
                    issue_push(cmp1);             // frame ptr push
                    w4skip <= 1'b1;               // a copy done is pending
                    state <= S_PREP_W3;
                end
            end
            S_PREP_W3: begin
                if (eu_done) w4skip <= 1'b0;      // the last copy's done
                if (eu_started) state <= S_PREP_W4;
            end
            S_PREP_W4: if (eu_done) begin
                if (w4skip) w4skip <= 1'b0;       // the last copy's done
                else begin
                    // frame push done: BP/SP update, retire AT done
                    rf[5] <= cmp1;
                    rf[4] <= rf[4] - disp;
                    retire();
                end
            end
            S_INTV: if (q_pop) begin              // BRK imm8 vector
                ivt_vec <= q_byte;
                pc <= pc + 16'd1;
                // IVT read slot rides the bus grid: a pop on even
                // parity (T1/T3-aligned) reaches the arbiter one
                // cycle sooner (measured, 500-case CD tranche)
                dly <= bus_phase ? 6'd4 : 6'd3;
                wnext <= S_TRAP_IVT1;
                state <= S_WAITX;
            end
            // MOV AL/AW, moffs16 (A0/A1): direct address pops at F+2/F+3,
            // read ready hi+1, retire at done (boot capture)
            S_MLO: if (q_pop) begin
                disp[7:0] <= q_byte;
                pc <= pc + 16'd1;
                state <= S_MHI;
            end
            S_MHI: if (q_pop) begin
                pc <= pc + 16'd1;
                eu_addr <= {sr[seg_ovr_en ? seg_ovr : SEG_DS], 4'h0} +
                           {4'h0, {q_byte, disp[7:0]}};
                eu_seg  <= seg_ovr_en ? seg_ovr : SEG_DS;
                eu_wr   <= op_moffw;
                eu_word <= opc[0];
                if (op_moffw) eu_wdata <= rf[0];
                state   <= S_REQ;
            end
            // reset flow: 7 idle cycles after RESET release, then the
            // standard flush machinery redirects to FFFF:0000 (measured:
            // QS=E at release+7, first fetch T1 at release+9)
            S_RESET: begin
                if (dly == 6'd1) state <= S_JFLUSH;
                dly <= dly - 6'd1;
            end
            S_JFLUSH: begin      // q_flush high this cycle (comb)
                pc      <= fl_ip;
                arch_ip <= fl_ip;
                sr[SEG_CS] <= fl_cs;   // no-op for near flushes
                state   <= S_FIRST;
            end
            S_CALLFL: begin      // q_flush high this cycle (comb)
                issue_push(pc);  // return address = fall-through
                pc    <= fl_ip;
                state <= S_CALLPUSH;
            end
            S_CALLPUSH: if (eu_started) state <= S_CALLW;
            S_CALLW: if (eu_done) begin
                arch_ip <= pc;
                state <= S_FIRST;
            end
            S_RETF: begin        // q_flush high this cycle (comb)
                pc      <= fl_ip;
                arch_ip <= fl_ip;
                state   <= S_FIRST;
            end

            //----------------------------------------------------------------
            // 0F-prefixed forms: second byte pops at F+2, modrm at F+3
            //----------------------------------------------------------------
            S_0F: if (q_pop) begin
                opc2 <= q_byte;
                pc   <= pc + 16'd1;
                if (q_byte[7:4] == 4'h1 || q_byte == 8'h28 ||
                    q_byte == 8'h2A ||
                    q_byte == 8'h31 || q_byte == 8'h33 ||
                    q_byte == 8'h39 || q_byte == 8'h3B)
                    state <= S_DEC2;   // bit ops / ROL4 / ROR4 / INS/EXT
                else if (q_byte == 8'h20 || q_byte == 8'h22 ||
                         q_byte == 8'h26) begin        // ADD4S/SUB4S/CMP4S
                    a4_cnt  <= 8'(({1'b0, rf[1][7:0]} + 9'd1) >> 1);
                    a4_k    <= 8'd0;
                    a4_carry <= 1'b0;
                    a4_z    <= 1'b1;
                    dly     <= 6'd4;                   // src ready @ pop+5
                    state   <= S_A4_SETUP;
                end else
                    state <= S_HALT;
            end
            S_DEC2: if (q_pop) begin
                mrm <= q_byte;
                pc  <= pc + 16'd1;
                if (q_byte[7:6] == 2'd3) begin
                    if (op_insext)
                        // EXT imm4 (3B) pops its imm right after the
                        // mrm; INS imm4 (39) pops it mid-flow (fitted:
                        // mrm+6 when off=0, read-done+4+off otherwise)
                        state <= (ie_immf && !ie_ins) ? S_T1GAP
                                                      : S_IE_SET;
                    else if (op_bit1 && b1_imm)
                        state <= S_IMM3;               // imm pops at F+4
                    else if (op_bit1 && opc2[2:1] == 2'd1) begin
                        // CLR1 CL reg: close pop+4 (fitted 0F12/13)
                        dly <= 6'd2; wnext <= S_EX; state <= S_WAITX;
                    end else if (op_bit1) begin
                        // TEST1/SET1/NOT1 CL reg: close pop+3
                        dly <= 6'd1; wnext <= S_EX; state <= S_WAITX;
                    end else if (op_ror4) begin        // ROR4 reg
                        dly <= 6'd15; wnext <= S_EX; state <= S_WAITX;
                    end else begin                     // ROL4 reg
                        dly <= 6'd11; wnext <= S_EX; state <= S_WAITX;
                    end
                end else if (op_insext)
                    state <= S_HALT;   // mem-mod INS/EXT: undefined, parked
                else if ((q_byte[7:6] == 2'd0 && q_byte[2:0] == 3'd6) ||
                         q_byte[7:6] == 2'd2)
                    state <= S_DLO;
                else
                    state <= S_EA1;
            end
            S_T1GAP: state <= S_IMM3;                  // mem TEST1: done+2 pop
            S_IMM3: if (q_pop) begin
                immb <= q_byte;
                pc   <= pc + 16'd1;
                if (op_bit1 && opc2[2:1] != 2'd0 && mrm_mod != 2'd3) begin
                    // bit-op imm mem RMW: CLR1 done+7 (dly2);
                    // SET1/NOT1 slot swept vs goldens
                    dly <= (opc2[2:1] == 2'd1) ? 6'd2 : 6'd1;
                    state <= S_RMWX;
                end else if (op_bit1 && opc2[2:1] == 2'd1) begin
                    // CLR1 imm reg: close pop+3 (one hop)
                    dly <= 6'd1; wnext <= S_EX; state <= S_WAITX;
                end else if (op_insext)
                    state <= S_IE_SET;
                else
                    state <= S_EX;
            end

            //----------------------------------------------------------------
            // INS/EXT (0F 31/33/39/3B): bit-field insert/extract, mod3
            // only. Schedules (fitted, even-aligned; odd splits ride the
            // BIU): INS s<16 off>0: R1, re-read at R1+21+off, write at
            // R2+12+3len+2(off-1); off=0: late R1 (+~14), write at
            // R1+12+3len; s=16: write at R1+41; s>16: word-0 write at
            // R1+41, word-1 read at W1+8, word-1 write (off=0 law) at
            // R2+12+3(s-16); store-retire at the final write done.
            // EXT: single read, retire F = R1+33+off (s<16), +34+off
            // (s=16, incl. alias-raw), s>16: R2 at R1+9/10, F = R2+15
            // +off; runaway: F = R1+33+256*len.
            //----------------------------------------------------------------
            S_IE_SET: begin
                logic [3:0] o;
                logic [4:0] l;
                logic [5:0] ss;
                logic [15:0] awn;
                logic [7:0] rm8v, rg8v;   // Quartus 17.1: no bit-select
                rm8v = reg8_get(mrm_rm);  // on a function-call result
                rg8v = reg8_get(mrm_reg);
                o  = rm8v[3:0];
                l  = {1'b0, ie_immf ? immb[3:0]
                                    : rg8v[3:0]} + 5'd1;
                ss = {2'd0, o} + {1'd0, l};
                ie_ph2 <= 1'b0;
                ie_rdyhold <= 1'b0;
                ie_lgot    <= 1'b1;
                if (ie_ins) begin
                    ie_off  <= o;
                    ie_mode <= 2'd0;
                    eu_addr <= {sr[SEG_ES], 4'h0} + {4'h0, rf[7]};
                    eu_seg  <= SEG_ES;
                    if (ie_immf) begin
                        // 0F39: length rides in later (see S_IE_IMM);
                        // off=0 pops it at mrm+6 before the read
                        ie_lgot <= 1'b0;
                        ie_dly  <= (o == 4'd0) ? 12'(IEI_IMM0)
                                               : 12'(IE_R1D);
                    end else begin
                        // offset reg <- s mod 16 BEFORE the AW source
                        // read (aliased offset regs insert the updated
                        // field - measured)
                        wr_reg8(mrm_rm, {4'd0, ss[3:0]});
                        awn = rf[0];
                        if (mrm_rm == 3'd0)
                            awn = {rf[0][15:8], 4'd0, ss[3:0]};
                        if (mrm_rm == 3'd4)
                            awn = {4'd0, ss[3:0], rf[0][7:0]};
                        ie_fld <= awn & 16'((32'd1 << l) - 32'd1);
                        ie_len <= l;
                        if (o == 4'd0 && l == 5'd16) begin
                            // F1: full-word aligned insert - chip skips
                            // the ES:IY RMW read (0F39 imm twin below).
                            eu_wdata <= awn;
                            ie_dly   <= 12'(IE_W16R);
                        end else begin
                            ie_dly <= (o == 4'd0 && ss < 6'd16)
                                      ? 12'(IE_R1D0) : 12'(IE_R1D);
                        end
                    end
                end else begin
                    if (mrm_rm == 3'd0 || (mrm_rm == 3'd4 && l == 5'd16))
                    begin
                        ie_mode <= 2'd1;    // alias-raw: off=0, len=16
                        ie_off  <= 4'd0;
                        ie_len  <= 5'd16;
                    end else if (mrm_rm == 3'd4) begin
                        ie_mode <= 2'd2;    // runaway internal loop
                        ie_off  <= 4'd0;
                        ie_len  <= l;
                    end else begin
                        ie_mode <= 2'd0;
                        wr_reg8(mrm_rm, {4'd0, ss[3:0]});
                        ie_off  <= o;
                        ie_len  <= l;
                    end
                    eu_addr <= {sr[seg_ovr_en ? seg_ovr : SEG_DS], 4'h0} +
                               {4'h0, rf[6]};
                    eu_seg  <= seg_ovr_en ? seg_ovr : SEG_DS;
                    // EXT reg-form read maturity rides the mrm pop's
                    // bus phase (T1/T2 pop: ready pop+4; else the
                    // request reserves at pop+4 and matures pop+5) -
                    // the POP-mem law; INS does not (0F31 cold
                    // variants). The imm form (0F3B) is simpler:
                    // ready = imm pop + 3, no reservation (fitted)
                    ie_dly     <= ie_immf ? 12'd0 : 12'(IE_R1D);
                    ie_rdyhold <= !ie_immf && popm_hold != 6'd2;
                end
                eu_wr   <= 1'b0;
                eu_word <= 1'b1;
                eu_kind <= K_MEM;
                wnext   <= (ie_ins && !ie_immf &&
                            o == 4'd0 && l == 5'd16) ? S_IE_WR
                           : (ie_ins && ie_immf && o == 4'd0) ? S_IE_IMM
                                                             : S_IE_R1;
                state   <= S_IE_WAIT;
            end
            // 0F39 deferred imm4 pop (fitted: mrm+6 when off=0, read-
            // done+4+off otherwise); all later burns anchor at this pop
            S_IE_IMM: if (q_pop) begin
                logic [4:0] l;
                logic [5:0] ss;
                logic [15:0] awn, fld, mask;
                logic [31:0] msk32, fsh32;
                l  = {1'b0, q_byte[3:0]} + 5'd1;
                ss = {2'd0, ie_off} + {1'd0, l};
                pc <= pc + 16'd1;
                wr_reg8(mrm_rm, {4'd0, ss[3:0]});
                awn = rf[0];
                if (mrm_rm == 3'd0) awn = {rf[0][15:8], 4'd0, ss[3:0]};
                if (mrm_rm == 3'd4) awn = {4'd0, ss[3:0], rf[0][7:0]};
                mask = 16'((32'd1 << l) - 32'd1);
                fld  = awn & mask;
                ie_fld  <= fld;
                ie_len  <= l;
                ie_lgot <= 1'b1;
                msk32 = {16'd0, mask} << ie_off;
                fsh32 = {16'd0, fld} << ie_off;
                if (ie_off == 4'd0 && l == 5'd16) begin
                    // full-word insert: NO read - lone write at imm+34
                    eu_wdata <= fld;
                    ie_dly   <= 12'(IEI_W16);
                    wnext    <= S_IE_WR;
                end else if (ie_off == 4'd0) begin
                    ie_dly <= 12'(IEI_R1);   // late read at imm+14
                    wnext  <= S_IE_R1;
                end else if (ss < 6'd16) begin
                    ie_dly <= 12'(IEI_R2);   // re-read at imm+14
                    wnext  <= S_IE_R2;
                end else begin
                    // word-0 write at imm+34-off (merge the R1 data)
                    eu_wdata <= (eu_rdata & ~msk32[15:0]) | fsh32[15:0];
                    ie_dly   <= 12'(IEI_W1) - {8'd0, ie_off};
                    wnext    <= S_IE_WR;
                end
                state <= S_IE_WAIT;
            end
            S_IE_WAIT: begin
                if (ie_dly == 12'd0) begin
                    state <= wnext;
                    if (wnext == S_IE_WR) eu_wr <= 1'b1;
                end else
                    ie_dly <= ie_dly - 12'd1;
            end
            S_IE_R1: begin
                ie_rdyhold <= 1'b0;
                if (eu_started) state <= S_IE_R1W;
            end
            S_IE_R1W: if (eu_done) begin
                if (ie_ins && !ie_lgot) begin
                    // 0F39 off>0: imm pops at read-done + 4 + off
                    ie_dly <= 12'd2 + {8'd0, ie_off};
                    wnext  <= S_IE_IMM;
                    state  <= S_IE_WAIT;
                end else if (ie_ins) begin
                    eu_wdata <= (eu_rdata & ~ie_mask32[15:0]) |
                                ie_fsh[15:0];
                    if (ie_s < 6'd16 && ie_off != 4'd0) begin
                        ie_dly <= 12'(IE_G1) + {8'd0, ie_off};
                        wnext  <= S_IE_R2;      // re-read the same word
                    end else if (ie_s < 6'd16) begin
                        // split (odd) accesses: write burns anchor at
                        // the read's FIRST sub-cycle (-4) with a small-
                        // field floor +max(0, 4-len) (fitted, odd goldens)
                        ie_dly <= 12'(IE_GW0) +
                                  {5'd0, ie_len, 1'b0} + {6'd0, ie_len} -
                                  (eu_addr[0]
                                   ? 12'd4 - (ie_len < 5'd4
                                              ? 12'(6'd4 - {1'b0, ie_len})
                                              : 12'd0)
                                   : 12'd0);
                        wnext  <= S_IE_WR;
                    end else begin
                        ie_dly <= 12'(IE_GW16); // fixed word-0 write slot
                        wnext  <= S_IE_WR;
                    end
                    state <= S_IE_WAIT;
                end else if (ie_mode == 2'd0 && ie_s > 6'd16) begin
                    ie_w0   <= eu_rdata;
                    eu_addr <= ea_step2(eu_addr, eu_seg);
                    ie_dly  <= 12'(IE_R2G);
                    wnext   <= S_IE_R2;
                    state   <= S_IE_WAIT;
                end else begin
                    ie_fld <= (ie_mode == 2'd2) ? 16'd0
                              : 16'(({16'd0, eu_rdata} >> ie_off) &
                                    ((32'd1 << ie_len) - 32'd1));
                    ie_dly <= (ie_mode == 2'd2)
                              ? 12'(IE_TAIL) + {ie_len[3:0], 8'd0}
                              : (ie_s == 6'd16)
                                ? 12'(IE_TAIL) + 12'd1 + {8'd0, ie_off}
                                : 12'(IE_TAIL) + {8'd0, ie_off};
                    wnext <= S_EX;
                    state <= S_IE_WAIT;
                end
            end
            S_IE_R2: if (eu_started) state <= S_IE_R2W;
            S_IE_R2W: if (eu_done) begin
                if (ie_ins && !ie_ph2) begin
                    // no-carry path re-read: merge from the fresh data
                    eu_wdata <= (eu_rdata & ~ie_mask32[15:0]) |
                                ie_fsh[15:0];
                    ie_dly <= 12'(IE_GW) +
                              {5'd0, ie_len, 1'b0} + {6'd0, ie_len} +
                              {7'd0, ie_off, 1'b0} -
                              (eu_addr[0]
                               ? 12'd4 - (ie_len < 5'd4
                                          ? 12'(6'd4 - {1'b0, ie_len})
                                          : 12'd0)
                               : 12'd0);
                    wnext  <= S_IE_WR;
                    state  <= S_IE_WAIT;
                end else if (ie_ins) begin
                    // carry-out word 1: an off=0 insert of s-16 bits
                    logic [4:0] l2;
                    l2 = 5'(ie_s - 6'd16);
                    eu_wdata <= (eu_rdata & ~ie_mask32[31:16]) |
                                ie_fsh[31:16];
                    ie_dly <= 12'(IE_GW0) +
                              {5'd0, l2, 1'b0} + {6'd0, l2} -
                              (eu_addr[0]
                               ? 12'd4 - (l2 < 5'd4
                                          ? 12'(6'd4 - {1'b0, l2})
                                          : 12'd0)
                               : 12'd0);
                    wnext  <= S_IE_WR;
                    state  <= S_IE_WAIT;
                end else begin
                    ie_fld <= 16'(({eu_rdata, ie_w0} >> ie_off) &
                                  ((32'd1 << ie_len) - 32'd1));
                    ie_dly <= 12'(IE_TAIL2) + {8'd0, ie_off};
                    wnext  <= S_EX;
                    state  <= S_IE_WAIT;
                end
            end
            S_IE_WR: if (eu_started) begin
                state <= S_IE_WRW;
                // split word-0 write with a word-1 leg: chain the read
                // request in-flight (fitted: R2 T1 = W1 first-sub T1 + 8)
                if (eu_addr[0] && ie_s > 6'd16 && !ie_ph2) begin
                    ie_chain <= 1'b1;
                    eu_wr    <= 1'b0;
                    eu_addr  <= ea_step2(eu_addr, eu_seg);  // F4a: INS split w0->w1
                end
            end
            S_IE_WRW: begin
                if (ie_chain) begin
                    if (eu_started) begin   // chained word-1 read accepted
                        ie_chain <= 1'b0;   // (write completes this cycle)
                        ie_ph2   <= 1'b1;
                        state    <= S_IE_R2W;
                    end
                end else if (eu_done) begin
                    if (ie_s > 6'd16 && !ie_ph2) begin
                        ie_ph2  <= 1'b1;
                        eu_wr   <= 1'b0;
                        eu_addr <= ea_step2(eu_addr, eu_seg);
                        ie_dly  <= 12'(IE_W2R2);
                        wnext   <= S_IE_R2;
                        state   <= S_IE_WAIT;
                    end else begin
                        if (ie_s >= 6'd16) rf[7] <= rf[7] + 16'd2;
                        psw <= ie_psw_ins;
                        retire();
                    end
                end
            end

            //----------------------------------------------------------------
            // ADD4S: per-byte loop src(DS0:IX+k) rd, dst(DS1:IY+k) rd, wr.
            // Ready offsets measured: src @ pop2+5 / wrdone+1, dst @
            // srcdone+2, write @ dstdone+4, retire @ last wrdone+4.
            //----------------------------------------------------------------
            S_A4_SETUP: begin
                if (dly == 6'd1) begin
                    // 4S source honours the segment-override prefix (DS0
                    // default). BUGFIX: was hardcoded SEG_DS -> override
                    // (ES/CS/SS) source string mis-read; whole result + CY
                    // wrong. Verified on v20 0F20/0F22/0F26: fail <=> override.
                    eu_addr <= {sr[seg_ovr_en ? seg_ovr : SEG_DS], 4'h0}
                               + {4'h0, rf[6]};
                    eu_seg  <= seg_ovr_en ? seg_ovr : SEG_DS;
                    eu_wr   <= 1'b0;
                    eu_word <= 1'b0;
                    state   <= (a4_cnt == 8'd0) ? S_HALT : S_A4_SRC;
                end
                dly <= dly - 6'd1;
            end
            S_A4_SRC: if (eu_started) state <= S_A4_SRCW;
            // src->dst read->read: STAYS on eu_done. MEASURED (this campaign):
            // marching the dst read early on eu_rdone made w1/w3 drift WORSE
            // (core overshoots faster) - the chip's "dst @ srcdone+2" law
            // tracks the src read's STRETCHED completion (eu_done), NOT the
            // bus-grid-early point. Read->read here is not a bus-grid march.
            S_A4_SRCW: if (eu_done) begin
                a4_src  <= eu_rdata;
                eu_addr <= {sr[SEG_ES], 4'h0} + {4'h0, rf[7] + {8'd0, a4_k}};
                eu_seg  <= SEG_ES;
                state   <= S_A4_G1;
            end
            S_A4_G1: state <= S_A4_DST;
            S_A4_DST: if (eu_started) state <= S_A4_DSTW;
            S_A4_DSTW: if (eu_done) begin
                // {badj/pad, braw/wrap, fire, sibx, prez, res}; ADD4S driven
                // sibling = src_o + dst_o + fire + sibx - 1, plus the
                // second-rail high-nibble carry when sibx did not cover it;
                // SUB4S sibling = dst_o - src_o - braw - badj + 1
                // (closure block: exact on all 1020 golden writes)
                logic [13:0] s;
                if (opc2 == 8'h20)
                    s = bcd_add8(eu_rdata[7:0], a4_src[7:0], a4_carry);
                else
                    s = bcd_sub8(eu_rdata[7:0], a4_src[7:0], a4_carry);
                a4_carry <= s[10];
                // F2 unified-law Z stage: both directions expose their
                // per-byte zq term at s[13] (V30 mu-line-2 intermediate ==
                // 0). ADD = raw 9-bit byte sum == 0; SUB/CMP = {dec[3:0],
                // dlo} == 0. P=Z mirror at S_A4_END unchanged.
                a4_z     <= a4_z && s[13];
                if (opc2 == 8'h20)
                    mem_op <= {a4_src[15:8] + eu_rdata[15:8] +
                               {7'd0, s[10]} + {7'd0, s[9]} +
                               {7'd0, s[11]} - 8'd1, s[7:0]};
                else
                    mem_op <= {eu_rdata[15:8] - a4_src[15:8] -
                               {7'd0, s[11]} - {7'd0, s[12]} + 8'd1,
                               s[7:0]};
                if (opc2 == 8'h26) begin        // CMP4S: no write-back
                    if (a4_cnt > 8'd1) begin
                        a4_cnt  <= a4_cnt - 8'd1;
                        a4_k    <= a4_k + 8'd1;
                        eu_addr <= {sr[seg_ovr_en ? seg_ovr : SEG_DS], 4'h0}
                                   + {4'h0, rf[6] + {8'd0, a4_k} + 16'd1};
                        eu_seg  <= seg_ovr_en ? seg_ovr : SEG_DS;
                        eu_wr   <= 1'b0;
                        // next src T1 = this dst T1 + 14 (measured)
                        dly <= 6'd8; wnext <= S_A4_SRC;
                        state <= S_WAITX;
                    end else begin
                        // close = last read T1 + 17 (borrow) / 18
                        dly   <= s[10] ? 6'd13 : 6'd14;
                        state <= S_A4_END;
                    end
                end else begin
                    dly      <= 6'd3;
                    state    <= S_A4_G2;
                end
            end
            S_A4_G2: begin
                if (dly == 6'd1) begin
                    eu_wr    <= 1'b1;
                    eu_wdata <= mem_op;
                    state    <= S_A4_WR;
                end
                dly <= dly - 6'd1;
            end
            S_A4_WR: if (eu_started) state <= S_A4_WRW;
            S_A4_WRW: if (eu_done) begin
                if (a4_cnt > 8'd1) begin
                    a4_cnt  <= a4_cnt - 8'd1;
                    a4_k    <= a4_k + 8'd1;
                    eu_addr <= {sr[seg_ovr_en ? seg_ovr : SEG_DS], 4'h0}
                               + {4'h0, rf[6] + {8'd0, a4_k} + 16'd1};
                    eu_seg  <= seg_ovr_en ? seg_ovr : SEG_DS;
                    eu_wr   <= 1'b0;
                    state   <= S_A4_SRC;
                end else begin
                    // retire at wrdone+4 with final carry, +5 without
                    // (measured: the no-carry path costs one extra cycle)
                    dly   <= a4_carry ? 6'd4 : 6'd5;
                    state <= S_A4_END;
                end
            end
            S_A4_END: begin
                if (dly == 6'd1) begin
                    // undefined-flag law: S=AC=CY(out), P=Z(out), V=0
                    psw[FB_CY] <= a4_carry;
                    psw[FB_S]  <= a4_carry;
                    psw[FB_AC] <= a4_carry;
                    psw[FB_Z]  <= a4_z;
                    psw[FB_P]  <= a4_z;
                    psw[FB_V]  <= 1'b0;
                    retire();
                end
                dly <= dly - 6'd1;
            end

            //----------------------------------------------------------------
            // effective-address path
            //----------------------------------------------------------------
            S_EA1: begin
                if (op_lea && mrm_mod == 2'd0) begin
                    // LDEA mod0: one EA cycle, retire at cycle 2, no
                    // bus access or reservation (measured: F@3)
                    rf[mrm_reg] <= ea_base;
                    retire();
                end else if (op_movri && mrm_mod == 2'd0) begin
                    // MOV r/m,imm mod0 reg-EA: no operand read, so the EA
                    // is latched in this single cycle and the imm pops next
                    // (the read forms need the extra S_EA2 cycle; measured:
                    // the imm pops at modrm-pop+2 for the no-disp form).
                    setup_access(ea_base);
                    state <= S_AI_I8;
                end else
                    state <= (mrm_mod == 2'd1) ? S_DISP8 : S_EA2;
            end
            S_EA2: begin                                  // mod0, reg EA
                setup_access(ea_base);
                // MOV r/m,imm: latch EA now, pop the imm, THEN write
                // (no operand read). Cadence fitted vs the C6/C7 goldens.
                state <= op_movri ? S_AI_I8 : S_REQ;  // ready @ 4
            end
            // displacement pops retry on a 2-cycle grain when the queue
            // runs dry (measured on cold-start traces; modrm/imm/F pops
            // poll every cycle instead)
            S_DISP8: if (q_pop) begin                     // mod1 disp pop @ 3
                disp <= {{8{q_byte[7]}}, q_byte};
                pc <= pc + 16'd1;
                if (op_lea) begin
                    rf[mrm_reg] <= ea_base + {{8{q_byte[7]}}, q_byte};
                    arch_ip <= pc + 16'd1;
                    seg_ovr_en <= 1'b0;
                    rep_en     <= 1'b0;
                    lock_en    <= 1'b0;
                    state <= S_FIRST;
                end else begin
                    setup_access(ea_base + {{8{q_byte[7]}}, q_byte});
                    // the sreg store (8C) follows the READER ready
                    // schedule (d0/d1 @ 4, d2 @ 5 - measured)
                    if (op_movri) state <= S_AI_I8;  // pop imm, then write
                    else if (op_popm) begin
                        dly <= popm_rdy - 6'd1;
                        wnext <= S_REQ; state <= S_WAITX;
                    end else if (is_store && !op_srst) begin // d1 store
                        dly <= 6'd1; state <= S_RSV;
                    end else state <= S_REQ;              // d1 load: rdy @ 4
                end
            end else if (!q_avail) begin
                dret <= S_DISP8; state <= S_DSTALL;
            end
            // else: T2-blocked with data available - retry next cycle
            S_DLO: if (q_pop) begin                       // disp16 low @ 2
                disp[7:0] <= q_byte;
                pc <= pc + 16'd1;
                state <= S_DGAP;
            end
            // dry: re-poll every cycle (chip pops at first availability -
            // measured, Campaign 4 phase matrix 3e:disp16 ph1; the old
            // DSTALL 2-grain here was an aliased cold-trace fit)
            S_DGAP: state <= S_DHI;
            S_DHI: if (q_pop) begin                       // disp16 high @ 4
                disp[15:8] <= q_byte;
                pc <= pc + 16'd1;
                if (op_lea) begin
                    rf[mrm_reg] <= ea_base + {q_byte, disp[7:0]};
                    arch_ip <= pc + 16'd1;
                    seg_ovr_en <= 1'b0;
                    rep_en     <= 1'b0;
                    lock_en    <= 1'b0;
                    state <= S_FIRST;
                end else begin
                    setup_access(ea_base + {q_byte, disp[7:0]});
                    // d2 loads and the sreg store (uniform @5): rdy @ 5;
                    // other d2 stores: rdy @ 7
                    if (op_movri) state <= S_AI_I8;  // pop imm, then write
                    else if (op_popm) begin
                        dly <= popm_rdy - 6'd1;
                        wnext <= S_REQ; state <= S_WAITX;
                    end else if (is_reader || op_srst) state <= S_REQ;
                    // d2 stores: ready @ 6 (hi-pop+2, same as the d1
                    // store schedule). The old rdy@7 was a phase-aliased
                    // golden fit - the Campaign 4 store phase matrix
                    // (st8/st16 x prefix x phase) shows the chip's write
                    // catching a T3 eval at hi+2 (fz151 class).
                    else begin dly <= 6'd1; state <= S_RSV; end
                end
            end else if (!q_avail) begin
                dret <= S_DHI; state <= S_DSTALL;
            end
            // else: T2-blocked with data available - retry next cycle
            S_DSTALL: state <= dret;

            //----------------------------------------------------------------
            // bus access issue / wait
            //----------------------------------------------------------------
            S_RSV: begin
                if (dly == 6'd1) state <= S_REQ;
                dly <= dly - 6'd1;
            end
            //----------------------------------------------------------------
            // string micro-loop (mission J): the MOVBK write is requested
            // WHILE its read is in flight and commits at the read's T3
            // edge (data forwarded inside the BIU, eu_fwd); REP chains
            // the next element's access the same way - reads and writes
            // run back to back (slopes 8/4, measured).
            //----------------------------------------------------------------
            S_REQ: if (eu_started) begin
                if (op_movstr) begin           // read accepted: queue write
                    eu_addr <= {sr[SEG_ES], 4'h0} + {4'h0, rf[7]};
                    eu_seg  <= SEG_ES;
                    eu_wr   <= 1'b1;
                    state   <= S_STRW;
                end else if (op_outstr) begin  // OUTS: mem read accepted -> IOW
                    // issue the I/O write to port DW; the write data is
                    // the DS:IX read, forwarded inside the BIU (eu_fwd).
                    eu_addr <= {4'h0, rf[2]};
                    eu_seg  <= SEG_CS;
                    eu_wr   <= 1'b1;
                    eu_word <= opc[0];
                    eu_kind <= K_IO;
                    state   <= S_OUTS_W;
                end else if (op_instr) begin   // INS: port read accepted -> MEMW
                    // issue the mem write to ES:IY (no override); the
                    // write data is the port read, forwarded (eu_fwd).
                    eu_addr <= {sr[SEG_ES], 4'h0} + {4'h0, rf[7]};
                    eu_seg  <= SEG_ES;
                    eu_wr   <= 1'b1;
                    eu_word <= opc[0];
                    eu_kind <= K_MEM;
                    state   <= S_INS_W;
                end else if (op_cmpstr) begin  // rd1 accepted: queue rd2
                    if (rep_en) begin          // REP order: ES then DS
                        eu_addr <= {sr[seg_ovr_en ? seg_ovr : SEG_DS],
                                    4'h0} + {4'h0, rf[6]};
                        eu_seg  <= seg_ovr_en ? seg_ovr : SEG_DS;
                    end else begin
                        eu_addr <= {sr[SEG_ES], 4'h0} + {4'h0, rf[7]};
                        eu_seg  <= SEG_ES;
                    end
                    eu_wr   <= 1'b0;
                    cmp_r2s <= 1'b0;
                    state   <= S_CMPW1;
                end else if (op_scastr) begin  // ES:IY read accepted
                    state   <= S_SCASW;
                end else if (op_stostr) begin  // write accepted
                    rf[7] <= rf[7] + str_step;
                    if (rep_en) begin
                        rf[1] <= rf[1] - 16'd1;
                        if (rf[1] != 16'd1 &&
                            ((rslot == 6'd6 && irq_rep) ||
                             (rslot <  6'd6 && rep1_abort))) begin
                            // FIRST-boundary abort. The decision edge
                            // is POP-ANCHORED at pop+7 (rslot==6) with
                            // the edge-4 pin tap, and the flush is
                            // invariant at pop+16 = edge+9 (all 35
                            // first-iter aborts of the INT.F3AA
                            // tranche; the write-accept slot floats
                            // +-1 beneath both). Accept at the edge:
                            // decide live; accept after: use the
                            // latched rep1_abort. Accept BEFORE the
                            // edge issues the next write and the
                            // parallel pop+7 check below withdraws it.
                            // flush = accept+dly+1 = pop+16 with
                            // dly = rslot+2 (rslot reads 13-j at
                            // pop+j). Chained-iteration aborts
                            // (S_STRS) stay write-anchored, accept+9.
                            state <= S_IRQ_REPW;
                            dly   <= rslot + 6'd2;
                        end else if (rf[1] != 16'd1) begin
                            eu_addr <= {sr[SEG_ES], 4'h0} +
                                       {4'h0, rf[7] + str_step};
                            state <= S_STRS;
                        end else begin
                            // REP cx=1: retire at the pop+12 slot or
                            // the write's completion, whichever later
                            str_done <= 1'b0;
                            state <= S_STRE;
                        end
                    end else state <= S_BUSW;
                end else if (op_popm && mrm_mod != 2'd3) begin
                    // POP mem: stack read accepted; pipeline the EA
                    // write with forwarded data (commits at the
                    // read's T3 end; measured, all 500 golden cases)
                    eu_addr <= ea_save;
                    eu_seg  <= ea_save_seg;
                    eu_wr   <= 1'b1;
                    eu_word <= 1'b1;
                    ldp2    <= 1'b0;
                    facc    <= 2'd0;
                    state   <= S_POPMW;
                end else if (op_retf || op_iret) begin
                    // rd1 accepted: pipeline rd2 (CS / IP+CS+PSW pops)
                    facc <= 2'd1;
                    eu_addr <= {sr[SEG_SS], 4'h0} +
                               {4'h0, rf[4] + 16'd2};
                    state <= S_FRETW;
                end else state <= S_BUSW;
            end
            S_POPR: retire();
            S_61G: state <= S_61W;
            S_61W: begin
                if (eu_started) begin
                    logic [15:0] noff;
                    pracc <= pracc + 3'd1;
                    // the saved-SP slot is never read (measured);
                    // the offset wraps at 64K within SS
                    noff = mem_op + ((pracc == 3'd2) ? 16'd4 : 16'd2);
                    mem_op <= noff;
                    eu_addr <= {sr[SEG_SS], 4'h0} + {4'h0, noff};
                end
                if (eu_done) begin
                    // pop order: IY,IX,BP,(skip SP),BW,DW,CW,AW
                    case (a4_cnt[2:0])
                        3'd0: rf[7] <= eu_rdata;
                        3'd1: rf[6] <= eu_rdata;
                        3'd2: rf[5] <= eu_rdata;
                        3'd3: rf[3] <= eu_rdata;
                        3'd4: rf[2] <= eu_rdata;
                        3'd5: rf[1] <= eu_rdata;
                        default: rf[0] <= eu_rdata;
                    endcase
                    rf[4] <= rf[4] + ((a4_cnt[2:0] == 3'd2) ? 16'd4
                                                            : 16'd2);
                    a4_cnt <= a4_cnt + 8'd1;
                    if (a4_cnt[2:0] == 3'd6) retire();
                end
            end
            S_POPMW: begin
                if (eu_started) facc <= 2'd1;
                if (eu_done) begin
                    if (!ldp2) begin           // stack read done
                        ldp2 <= 1'b1;
                        rf[4] <= rf[4] + 16'd2;
                    end else retire();         // EA write done
                end
            end
            S_FRETW: begin
                if (eu_started) begin
                    facc <= facc + 2'd1;
                    eu_addr <= {sr[SEG_SS], 4'h0} +
                               {4'h0, rf[4] + 16'd4};   // rd3 (RETI PSW)
                end
                if (eu_done) begin
                    if (fret_ph == 2'd0) begin
                        fl_ip <= eu_rdata;
                        fret_ph <= 2'd1;
                    end else begin
                        fl_cs <= eu_rdata;
                        fret_ph <= 2'd2;
                        if (!op_iret) begin           // RETF: flush done+3
                            rf[4] <= rf[4] + 16'd4 +
                                     ((opc == 8'hCA) ? disp : 16'd0);
                            dly <= 6'd1; wnext <= S_JFLUSH;
                            state <= S_WAITX;
                        end else begin
                            // RETI flushes at the CS pop; the PSW read
                            // completes in flight (measured: prefetch
                            // chains at the PSW read's T3)
                            iret_pw <= 1'b1;
                            state <= S_JFLUSH;
                        end
                    end
                end
            end
            // REP cx=1 slot-bound retire (measured: the closing F sits
            // at pop+13 regardless of when the single write completes)
            S_STRE: begin
                if (eu_done) str_done <= 1'b1;
                if (rslot <= 6'd1 && (str_done || eu_done)) retire();
            end
            //----------------------------------------------------------------
            // CMPBK/CMPM (A6/A7/AE/AF) + REPE/REPNE/REPC/REPNC.
            // Structural first pass (fit against goldens pending):
            // singles set flags at the (last) read's done and retire;
            // REP iterations chain the next read at the compare.
            //----------------------------------------------------------------
            S_CMPW1: begin                     // await DS:IX data
                if (eu_started) cmp_r2s <= 1'b1;
                if (eu_done) begin
                    cmp1  <= eu_rdata;
                    state <= S_CMPW2;
                end
            end
            S_CMPW2: if (eu_done) begin        // 2nd data: compare
                // operand roles follow the read order: single form
                // cmp1=DS:IX (a), rdata=ES:IY (b); REP form reversed
                logic [15:0] nf;
                logic [15:0] a_op, b_op;
                logic        cont;
                a_op = rep_en ? eu_rdata : cmp1;
                b_op = rep_en ? cmp1 : eu_rdata;
                if (opc[0]) begin
                    logic [31:0] c16;
                    c16 = alu16(3'd7, a_op, b_op, psw);
                    nf = c16[31:16];
                end else begin
                    logic [23:0] c8;
                    c8 = alu8(3'd7, a_op[7:0], b_op[7:0], psw);
                    nf = c8[23:8];
                end
                psw   <= nf;
                rf[6] <= rf[6] + str_step;
                rf[7] <= rf[7] + str_step;
                if (rep_en) begin
                    rf[1] <= rf[1] - 16'd1;
                    cont = (rf[1] != 16'd1) &&
                           (rep_kind == 2'd0 ?  nf[FB_Z]  :
                            rep_kind == 2'd1 ? !nf[FB_Z]  :
                            rep_kind == 2'd2 ?  nf[FB_CY] : !nf[FB_CY]);
                    if (cont) begin
                        // next iteration leads with ES:IY again; its
                        // read T1 lands at the previous T1+10
                        eu_addr <= {sr[SEG_ES], 4'h0} +
                                   {4'h0, rf[7] + str_step};
                        eu_seg  <= SEG_ES;
                        eu_wr   <= 1'b0;
                        dly <= 6'd3; wnext <= S_RSV; state <= S_WAITX;
                    end else begin
                        // termination close = last done + 10 (measured,
                        // cx-exhaust and condition-fail alike)
                        dly <= 6'd8; wnext <= S_EX; state <= S_WAITX;
                    end
                end else retire();
            end
            S_SCASW: if (eu_done) begin        // CMPM: acc - [ES:IY]
                logic [15:0] nf;
                logic        cont;
                if (opc[0]) begin
                    logic [31:0] c16;
                    c16 = alu16(3'd7, rf[0], eu_rdata, psw);
                    nf = c16[31:16];
                end else begin
                    logic [23:0] c8;
                    c8 = alu8(3'd7, rf[0][7:0], eu_rdata[7:0], psw);
                    nf = c8[23:8];
                end
                psw   <= nf;
                rf[7] <= rf[7] + str_step;
                if (rep_en) begin
                    rf[1] <= rf[1] - 16'd1;
                    cont = (rf[1] != 16'd1) &&
                           (rep_kind == 2'd0 ?  nf[FB_Z]  :
                            rep_kind == 2'd1 ? !nf[FB_Z]  :
                            rep_kind == 2'd2 ?  nf[FB_CY] : !nf[FB_CY]);
                    if (cont) begin
                        // next ES:IY read T1 = previous T1+10 (measured)
                        eu_addr <= {sr[SEG_ES], 4'h0} +
                                   {4'h0, rf[7] + str_step};
                        eu_seg  <= SEG_ES;
                        eu_wr   <= 1'b0;
                        dly <= 6'd3; wnext <= S_RSV; state <= S_WAITX;
                    end else begin
                        // termination close = last done + 12 (measured)
                        dly <= 6'd10; wnext <= S_EX; state <= S_WAITX;
                    end
                end else state <= S_EX;    // single: retire at done+1
            end
            S_CMPNXT, S_SCASNXT: state <= S_HALT;  // placeholders
            S_STRW: if (eu_started) begin      // MOVBK write accepted
                rf[6] <= rf[6] + str_step;
                rf[7] <= rf[7] + str_step;
                if (rep_en) begin
                    rf[1] <= rf[1] - 16'd1;
                    if (rf[1] != 16'd1) begin
                        eu_addr <= {sr[seg_ovr_en ? seg_ovr : SEG_DS],
                                    4'h0} + {4'h0, rf[6] + str_step};
                        eu_seg  <= seg_ovr_en ? seg_ovr : SEG_DS;
                        eu_wr   <= 1'b0;
                        state <= S_STRR;
                    end else state <= S_BUSW;
                end else state <= S_BUSW;
            end
            S_STRR: if (eu_started) begin      // next read accepted
                eu_addr <= {sr[SEG_ES], 4'h0} + {4'h0, rf[7]};
                eu_seg  <= SEG_ES;
                eu_wr   <= 1'b1;
                state   <= S_STRW;
            end
            // OUTS I/O write accepted: advance IX only (source pointer);
            // REP repeats CW times unconditionally (carry/zero gate only
            // CMPS/SCAS). The next element's DS(sov):IX read is chained.
            S_OUTS_W: if (eu_started) begin
                rf[6] <= rf[6] + str_step;
                if (rep_en) begin
                    rf[1] <= rf[1] - 16'd1;
                    if (rf[1] != 16'd1) begin
                        eu_addr <= {sr[seg_ovr_en ? seg_ovr : SEG_DS],
                                    4'h0} + {4'h0, rf[6] + str_step};
                        eu_seg  <= seg_ovr_en ? seg_ovr : SEG_DS;
                        eu_wr   <= 1'b0;
                        eu_kind <= K_MEM;
                        state <= S_OUTS_R;
                    end else state <= S_BUSW;
                end else state <= S_BUSW;
            end
            S_OUTS_R: if (eu_started) begin    // next mem read accepted -> IOW
                eu_addr <= {4'h0, rf[2]};
                eu_seg  <= SEG_CS;
                eu_wr   <= 1'b1;
                eu_word <= opc[0];
                eu_kind <= K_IO;
                state   <= S_OUTS_W;
            end
            // INS mem write accepted: advance IY only (dest pointer); REP
            // repeats CW times unconditionally. Next element's IOR is chained.
            S_INS_W: if (eu_started) begin
                rf[7] <= rf[7] + str_step;
                if (rep_en) begin
                    rf[1] <= rf[1] - 16'd1;
                    if (rf[1] != 16'd1) begin
                        eu_addr <= {4'h0, rf[2]};   // port DW (next IOR)
                        eu_seg  <= SEG_CS;
                        eu_wr   <= 1'b0;
                        eu_kind <= K_IO;
                        state <= S_INS_R;
                    end else state <= S_BUSW;
                end else state <= S_BUSW;
            end
            S_INS_R: if (eu_started) begin     // next port read accepted -> MEMW
                eu_addr <= {sr[SEG_ES], 4'h0} + {4'h0, rf[7]};
                eu_seg  <= SEG_ES;
                eu_wr   <= 1'b1;
                eu_word <= opc[0];
                eu_kind <= K_MEM;
                state   <= S_INS_W;
            end
            S_STRS: if (eu_started) begin      // next STM write accepted
                rf[7] <= rf[7] + str_step;
                if (rep_en) begin
                    rf[1] <= rf[1] - 16'd1;
                    if (rf[1] != 16'd1 && irq_rep) begin
                        state <= S_IRQ_REPW;
                        dly   <= 6'd8;
                    end else if (rf[1] != 16'd1) begin
                        eu_addr <= {sr[SEG_ES], 4'h0} +
                                   {4'h0, rf[7] + str_step};
                        state <= S_STRS;
                    end else state <= S_BUSW;
                end else state <= S_BUSW;
            end
            S_BUSW: if (opc == 8'h60 && a4_cnt < 8'd8 && eu_wdone) begin
                // PUSHA (0x60) inter-write march: issue the next stack write
                // from the current write's ZERO-WAIT completion point
                // (eu_wdone) instead of eu_done - the trap-chain law
                // (biu_model mission H). Under waits eu_done is stretched by
                // one per waited access, so issuing the next write on eu_done
                // lands its request one cycle late per wait and a prefetch
                // commits in the widened inter-write gap (measured fz84007
                // w1: chip runs 8 contiguous MEMW, the eu_done TB spliced a
                // CODE fetch between writes). eu_wdone fires at the write's
                // T4 at zero waits (== eu_done there, so golden is bit-exact)
                // and at the first Tw under waits, keeping the next request
                // ready for the deferred completion eval. reg order
                // AW,CW,DW,BW,SP(orig),BP,IX,IY.
                issue_push((a4_cnt == 8'd4) ? rf[4] + 16'd8
                                            : rf[a4_cnt[2:0]]);
                a4_cnt <= a4_cnt + 8'd1;
                state <= S_REQ;
            end else if (((op_grpff && mrm_reg == 3'd3) || opc == 8'h9A) &&
                         eu_wr && eu_wdone) begin
                // CALL far (9A / FF.3): march the PC push from the CS push's
                // ZERO-WAIT completion (eu_wdone), not eu_done - the same
                // trap-chain law as PUSHA. Under waits eu_done stretches +1
                // per waited access, so issuing the PC push at eu_done lands
                // its request late and the chip's contiguous CS;IP pushes get
                // an idle gap (measured fz84007 w1: chip runs the two pushes
                // back-to-back, the eu_done TB inserts 2 idle cycles). At zero
                // waits eu_wdone==eu_done so golden is bit-exact (the eu_done
                // far-CALL branch below becomes unreachable, harmless).
                issue_push(pc);
                state <= S_FCFL2;
            end else if (eu_done) begin
                if (op_moff) begin                        // A0 / A1
                    if (opc[0]) rf[0] <= eu_rdata;
                    else        rf[0][7:0] <= eu_rdata[7:0];
                    retire();
                end else if (op_moffw) begin              // A2 / A3
                    retire();
                end else if (op_xlat) begin               // TRANS
                    rf[0][7:0] <= eu_rdata[7:0];
                    retire();
                end else if (op_lodstr) begin             // LDM
                    if (opc[0]) rf[0] <= eu_rdata;
                    else        rf[0][7:0] <= eu_rdata[7:0];
                    rf[6] <= rf[6] + str_step;
                    if (rep_en) begin       // REP LDM loop (fitted)
                        rf[1] <= rf[1] - 16'd1;
                        if (rf[1] != 16'd1) begin
                            eu_addr <= {sr[seg_ovr_en ? seg_ovr : SEG_DS],
                                        4'h0} + {4'h0, rf[6] + str_step};
                            eu_seg  <= seg_ovr_en ? seg_ovr : SEG_DS;
                            eu_wr   <= 1'b0;
                            dly <= 6'd2; wnext <= S_RSV; state <= S_WAITX;
                        end else begin
                            dly <= 6'd7; wnext <= S_EX; state <= S_WAITX;
                        end
                    end else retire();
                end else if (op_stostr || op_movstr) begin // STM / MOVBK end
                    // REP (cx>=2) termination: one extra cycle after the
                    // last write's done (measured); singles retire at
                    // done. Split (odd word) writes close at done
                    // directly (measured on the F3A5/F3AB tranches)
                    if (rep_en) begin
                        if (opc[0] && eu_addr[0]) retire();
                        else state <= S_EX;
                    end else retire();
                end else if (op_outstr) begin             // OUTS end (IOW done)
                    // final I/O write complete; restore the data-access
                    // kind and retire (REP takes S_EX's +1-cycle close,
                    // like STM/MOVBK - timing is out of scope, arch only).
                    eu_kind <= K_MEM;
                    if (rep_en) state <= S_EX;
                    else        retire();
                end else if (op_instr) begin              // INS end (MEMW done)
                    // final mem write (ES:IY) complete; IY already stepped
                    // in S_INS_W. REP close mirrors the STM/MOVBK split-close
                    // law (line ~4249, silicon-fitted): a SPLIT final word write
                    // (word opc AND odd write address) closes at done; an aligned
                    // word OR any byte takes S_EX's +1 close. Family 6 (task #24):
                    // the op_instr branch never received this - word REP INS at an
                    // odd ES:IY closed one cycle late. eu_addr[0] is the physical
                    // write-address parity (== initial-DI parity, loop-invariant).
                    eu_kind <= K_MEM;
                    if (rep_en) begin
                        if (opc[0] && eu_addr[0]) retire();
                        else                      state <= S_EX;
                    end else retire();
                end else if (op_ret) begin                // C3 / C2
                    rf[4] <= rf[4] + 16'd2 +
                             ((opc == 8'hC2) ? disp : 16'd0);
                    fl_cs <= sr[SEG_CS];
                    fl_ip <= eu_rdata;
                    state <= S_RETF;                      // flush @ done+1
                end else if (opc == 8'h9D) begin          // POP PSW
                    rf[4] <= rf[4] + 16'd2;
                    retire();     // psw was consumed at the data edge
                end else if (op_in) begin                 // IN acc
                    if (opc[0]) rf[0] <= eu_rdata;
                    else        rf[0][7:0] <= eu_rdata[7:0];
                    eu_kind <= K_MEM;
                    retire();
                end else if (op_out) begin                // OUT
                    eu_kind <= K_MEM;
                    retire();
                end else if (op_popsr) begin              // POP sreg
                    logic [1:0] sx2;
                    sx2 = srmap(opc[4:3]);
                    sr[sx2] <= eu_rdata;
                    rf[4] <= rf[4] + 16'd2;
                    shadow <= 1'b1;      // sreg loads shadow recognition
                    retire();
                end else if (((op_grpff && mrm_reg == 3'd3) ||
                              opc == 8'h9A) && eu_wr) begin
                    // CALL far: PS push done; pre-issue the PC push
                    // and flush next cycle
                    issue_push(pc);
                    state <= S_FCFL2;
                end else if (op_pushsr || opc == 8'h9C || op_pushi ||
                             (op_grpff && mrm_reg == 3'd6 && eu_wr)) begin
                    retire();
                end else if (opc == 8'h60) begin          // PUSH R next
                    if (a4_cnt < 8'd8) begin
                        // reg order AW,CW,DW,BW,SP(orig),BP,IX,IY;
                        // next write ready at done+1 (measured cadence)
                        issue_push((a4_cnt == 8'd4) ? rf[4] + 16'd8
                                                    : rf[a4_cnt[2:0]]);
                        a4_cnt <= a4_cnt + 8'd1;
                        state <= S_REQ;
                    end else retire();
                end else if (opc[7:3] == 5'b01011) begin  // POP r16
                    rf[4] <= rf[4] + 16'd2;
                    rf[opc[2:0]] <= eu_rdata;             // POP SP: load wins
                    retire();
                end else if (op_popm) begin               // POP r16 via 8F
                    rf[4] <= rf[4] + 16'd2;
                    rf[mrm_rm] <= eu_rdata;
                    retire();
                end else if (opc[7:3] == 5'b01010) begin  // PUSH r16
                    retire();
                end else if (is_store || op_movri) begin  // MOV 88/89 / C6/C7 store
                    retire();
                end else begin
                    mem_op <= eu_rdata;
                    if (op_srld)
                        state <= S_LD_W2;   // sreg load: writeback done+1
                    else if (op_test) begin
                        // TEST mem: flags + retire at the read done
                        logic [31:0] tm16;
                        logic [23:0] tm8;
                        tm16 = alu16(3'd4, eu_rdata, rf[mrm_reg], psw);
                        tm8 = alu8(3'd4, eu_rdata[7:0],
                                   reg8_get(mrm_reg), psw);
                        psw <= opc[0] ? tm16[31:16] : tm8[23:8];
                        state <= S_NOP;   // close done+3
                    end
                    else if (is_load ||
                             (op_alu && (opc[5:3] == 3'd7 || opc[1])))
                        state <= S_LD_W1;   // MOV load / CMP mem /
                                            // ALU reg,mem load-op (d=1)
                    else if ((op_grpf6 || op_grpf7) && mrm_reg == 3'd0)
                        state <= S_TESTGAP; // TEST imm pops at done+3
                    else if (op_alui || op_imuli)
                        state <= S_AIGAP;   // imm pops at done+2
                    else if ((op_grpf6 || op_grpf7) &&
                             (mrm_reg == 3'd2 || mrm_reg == 3'd3)) begin
                        dly <= 6'd3; state <= S_RMWX;   // NOT/NEG mem
                    end else if (op_grpf6 && mrm_reg == 3'd5) begin
                        // IMUL8 mem: +4 on sign mismatch (measured)
                        dly <= (eu_rdata[7] ^ rf[0][7]) ? 6'd36 : 6'd32;
                        wnext <= S_EX; state <= S_WAITX;
                    end else if (op_grpf7 && mrm_reg == 3'd4) begin
                        dly <= 6'd29; wnext <= S_EX; state <= S_WAITX;
                    end else if (op_grpf7 && mrm_reg == 3'd5) begin
                        // IMUL16 mem: +4 on sign mismatch (measured)
                        dly <= (eu_rdata[15] ^ rf[0][15]) ? 6'd43 : 6'd39;
                        wnext <= S_EX; state <= S_WAITX;
                    end else if (op_grpff && mrm_reg <= 3'd1) begin
                        dly <= 6'd3; state <= S_RMWX;   // INC/DEC mem16
                    end else if (op_grpff && mrm_reg == 3'd6) begin
                        // PUSH mem: write ready done+5 (write commits
                        // read-end+8, all 365 golden mem cases)
                        issue_push(eu_rdata);
                        dly <= 6'd4; wnext <= S_REQ;
                        state <= S_WAITX;
                    end else if (op_grpff && (mrm_reg == 3'd2 ||
                                              mrm_reg == 3'd4)) begin
                        // CALL/BR rm (mem): flush at read done+4
                        fl_ip <= eu_rdata;
                        fl_cs <= sr[SEG_CS];
                        dly <= (mrm_reg == 3'd2) ? 6'd3 : 6'd2;
                        wnext <= (mrm_reg == 3'd2) ? S_CALLFL : S_JFLUSH;
                        state <= S_JWAIT;
                    end else if (op_grpff && (mrm_reg == 3'd3 ||
                                              mrm_reg == 3'd5)) begin
                        // CALL/BR far mem: two pointer words
                        if (!ldp2) begin
                            // second pointer word: BR ready done+4,
                            // CALL done+5 (measured)
                            ldp2 <= 1'b1;
                            fl_ip <= eu_rdata;
                            eu_addr <= ea_step2(eu_addr, eu_seg);
                            eu_wr <= 1'b0;
                            dly <= (mrm_reg == 3'd3) ? 6'd4 : 6'd3;
                            wnext <= S_REQ;
                            state <= S_WAITX;
                        end else if (mrm_reg == 3'd5) begin
                            // BR far mem: flush at CS-done+2
                            fl_cs <= eu_rdata;
                            dly <= 6'd1;
                            wnext <= S_JFLUSH;
                            state <= S_JWAIT;
                        end else begin
                            // CALL far mem: PS push ready done+5, then
                            // flush at write-done+1 with the PC push
                            // committing at the flush cycle end
                            fl_cs <= eu_rdata;
                            issue_push(sr[SEG_CS]);
                            dly <= 6'd4; wnext <= S_REQ;
                            state <= S_WAITX;
                        end
                    end else if (op_popm) begin
                        // stack word read; now write it to the saved EA
                        rf[4] <= rf[4] + 16'd2;
                        eu_addr <= ea_save;
                        eu_seg  <= ea_save_seg;
                        eu_wr   <= 1'b1;
                        eu_word <= 1'b1;
                        eu_wdata <= eu_rdata;
                        state <= S_WREQ;
                    end else if (op_fpo) begin
                        state <= S_NOP;      // ESC mem: close done+3
                    end else if (op_chk && !ldp2) begin
                        // CHKIND: lower bound read; upper at EA+2
                        // ready done+3 (hi read commits lo-end+6, all
                        // 500 golden cases)
                        ldp2 <= 1'b1;
                        cmp1 <= eu_rdata;
                        eu_addr <= ea_step2(eu_addr, eu_seg);
                        eu_wr <= 1'b0;
                        dly <= 6'd2; wnext <= S_REQ;
                        state <= S_WAITX;
                    end else if (op_chk) begin
                        // signed bounds check (0xFFFF upper = -1 traps)
                        logic [15:0] xi, xl, xh;
                        xi = rf[mrm_reg] ^ 16'h8000;
                        xl = cmp1 ^ 16'h8000;
                        xh = eu_rdata ^ 16'h8000;
                        if (xi < xl) begin
                            // below-lower trap: early-out, IVT read
                            // 3 cycles sooner than above-upper (measured)
                            ivt_vec <= 8'd5;
                            dly <= 6'd5; wnext <= S_TRAP_IVT1;
                            state <= S_WAITX;
                        end else if (xi > xh) begin
                            ivt_vec <= 8'd5;
                            dly <= 6'd8; wnext <= S_TRAP_IVT1;
                            state <= S_WAITX;
                        end else begin
                            dly <= 6'd2; wnext <= S_EX;  // close done+3
                            state <= S_WAITX;
                        end
                    end else if (op_disp) begin
                        // DISPOSE: SP = BP + 2 (pop), BP = popped word
                        rf[4] <= rf[5] + 16'd2;
                        rf[5] <= eu_rdata;
                        retire();            // fit pending
                    end else if (op_ldptr && !ldp2) begin
                        // LES/LDS: offset word read; segment word at
                        // EA+2 ready done+3 (commits lo-end+6; all
                        // 1000 golden cases C4+C5)
                        ldp2 <= 1'b1;
                        eu_addr <= ea_step2(eu_addr, eu_seg);
                        eu_wr <= 1'b0;
                        dly <= 6'd2; wnext <= S_REQ;
                        state <= S_WAITX;
                    end else if (op_ldptr) begin
                        rf[mrm_reg] <= mem_op;
                        if (opc[0]) sr[SEG_DS] <= eu_rdata;   // C5 LDS
                        else        sr[SEG_ES] <= eu_rdata;   // C4 LES
                        shadow <= 1'b1;      // sreg load shadow
                        retire();
                    end
                    else if (op_bit1 && b1_imm)
                        state <= S_T1GAP;                 // imm pop done+2
                    else if (op_bit1 && opc2[2:1] == 2'd0) begin
                        dly <= 6'd2; wnext <= S_EX;       // TEST1 CL mem
                        state <= S_WAITX;                 // (fit pending)
                    end else if (op_bit1) begin
                        // CLR1 CL mem wT1 done+7; SET1/NOT1 done+6
                        dly <= (opc2[2:1] == 2'd1) ? 6'd4 : 6'd3;
                        state <= S_RMWX;
                    end
                    else if (op_rol4) begin
                        dly <= 6'd10; state <= S_RMWX;    // wr ready done+11
                    end else if (op_ror4) begin
                        dly <= 6'd14; state <= S_RMWX;    // wT1 done+17
                    end else if (op_alu || op_xchg8 || op_xchg16) begin
                        dly <= 6'd2; state <= S_RMWX;     // wr ready done+3
                    end else if (op_grpfe) begin
                        dly <= 6'd3; state <= S_RMWX;     // done+4
                    end else if (op_grpd0 || op_grpd1) begin
                        // operand from eu_rdata: mem_op is only assigned
                        // (NBA) this cycle and not yet visible
                        sh_load(mrm_reg, sh_word, eu_rdata, 8'd1);
                        dly <= 6'd5; state <= S_RMWX;     // done+6
                    end else if (op_grpd2 || op_grpd3) begin
                        // by-CL mem: full count via the burn state
                        sh_load(mrm_reg, sh_word, eu_rdata, rf[1][7:0]);
                        shw <= {1'b0, rf[1][7:0]};
                        state <= S_SHWAIT;
                    end else if (op_shimm) begin
                        state <= S_AIGAP;   // count byte pops at done+2
                    end else if (op_grpf6 && mrm_reg == 3'd7) begin
                        // IDIV8 mem: reg-form law + 1 -> shared iterative unit
                        logic [15:0] anum16;
                        logic  [7:0] den8, ad8;
                        logic [23:0] t8;
                        logic [5:0]  sfix;
                        logic        early;
                        den8   = eu_rdata[7:0];
                        anum16 = rf[0][15] ? (~rf[0] + 16'd1) : rf[0];
                        ad8    = den8[7] ? (~den8 + 8'd1) : den8;
                        sfix   = rf[0][15] ? 6'd3 : 6'd0;
                        early  = (den8 == 8'd0) || (anum16[15:8] >= ad8);
                        if (early) begin
                            t8  = alu8(3'd5, anum16[15:8], ad8, psw);
                            psw <= t8[23:8];
                            dly <= 6'd22 + sfix; wnext <= S_TRAP_IVT1;
                        end else begin
                            div_rem <= {9'd0, anum16[15:8]};
                            div_quo <= {8'd0, anum16[7:0]};
                            div_den <= {8'd0, ad8};
                            div_cnt <= 6'd8;
                            div_word <= 1'b0; div_signed <= 1'b1;
                            div_nsign <= rf[0][15]; div_dsign <= den8[7];
                            div_busy <= 1'b1; div_pend <= 1'b1;
                            div_late <= 1'b0;
                            dly <= 6'd38 + sfix; wnext <= S_EX;
                        end
                        state <= S_WAITX;
                    end else if (op_grpf6 && mrm_reg == 3'd6) begin
                        // DIVU8 mem (reg law + 1) -> shared iterative unit
                        logic [7:0] den8;
                        logic       early8;
                        den8   = eu_rdata[7:0];
                        early8 = (den8 == 8'd0) || (rf[0][15:8] >= den8);
                        psw <= psw_sub8f(rf[0][15:8], den8, psw);
                        if (early8) begin
                            dly <= 6'd14; wnext <= S_TRAP_IVT1;
                        end else begin
                            div_rem <= {9'd0, rf[0][15:8]};
                            div_quo <= {8'd0, rf[0][7:0]};
                            div_den <= {8'd0, den8};
                            div_cnt <= 6'd8;
                            div_word <= 1'b0; div_signed <= 1'b0;
                            div_busy <= 1'b1; div_pend <= 1'b1;
                            div_late <= 1'b0;
                            dly <= 6'd20; wnext <= S_EX;
                        end
                        state <= S_WAITX;
                    end else if (op_grpf6) begin
                        dly <= 6'd22; wnext <= S_EX; state <= S_WAITX;
                    end else if (op_grpf7 && mrm_reg == 3'd7) begin
                        // IDIV16 mem: reg-form law + 1 -> shared iterative unit
                        logic [31:0] num32, anum;
                        logic [15:0] den16, ad;
                        logic [5:0]  sfix;
                        logic        early;
                        num32 = {rf[2], rf[0]};
                        den16 = eu_rdata;
                        anum  = num32[31] ? (~num32 + 32'd1) : num32;
                        ad    = den16[15] ? (~den16 + 16'd1) : den16;
                        sfix  = rf[2][15] ? 6'd3 : 6'd0;
                        early = (den16 == 16'd0) || (anum[31:16] >= ad);
                        if (early) begin
                            psw <= psw_sub16(anum[31:16], ad, psw);
                            dly <= 6'd22 + sfix; wnext <= S_TRAP_IVT1;
                        end else begin
                            div_rem <= {1'b0, anum[31:16]};
                            div_quo <= anum[15:0];
                            div_den <= ad;
                            div_cnt <= 6'd16;
                            div_word <= 1'b1; div_signed <= 1'b1;
                            div_nsign <= num32[31]; div_dsign <= den16[15];
                            div_busy <= 1'b1; div_pend <= 1'b1;
                            div_late <= 1'b0;
                            dly <= 6'd45 + sfix; wnext <= S_EX;
                        end
                        state <= S_WAITX;
                    end else if (op_grpf7) begin
                        // DIVU16 mem -> shared iterative unit
                        logic [15:0] den16;
                        logic        early16;
                        den16   = eu_rdata;
                        early16 = (den16 == 16'd0) || (rf[2] >= den16);
                        psw <= psw_sub16(rf[2], den16, psw);
                        if (early16) begin
                            dly <= 6'd13; wnext <= S_TRAP_IVT1;
                        end else begin
                            div_rem <= {1'b0, rf[2]};
                            div_quo <= rf[0];
                            div_den <= den16;
                            div_cnt <= 6'd16;
                            div_word <= 1'b1; div_signed <= 1'b0;
                            div_busy <= 1'b1; div_pend <= 1'b1;
                            div_late <= 1'b0;
                            dly <= 6'd26; wnext <= S_EX;
                        end
                        state <= S_WAITX;
                    end else state <= S_HALT;
                end
            end

            S_LD_W1: state <= S_LD_W2;
            S_LD_W2: begin
                logic [1:0] sx;
                sx = srmap(mrm_reg[1:0]);
                if (op_movl8)       wr_reg8(mrm_reg, mem_op[7:0]);
                else if (op_movl16) rf[mrm_reg] <= mem_op;
                else if (op_srld)   sr[sx] <= mem_op;
                else if (op_test) begin                   // TEST mem
                    logic [31:0] t16;
                    logic [23:0] t8;
                    t16 = alu16(3'd4, mem_op, rf[mrm_reg], psw);
                    t8  = alu8(3'd4, mem_op[7:0], reg8_get(mrm_reg), psw);
                    psw <= opc[0] ? t16[31:16] : t8[23:8];
                end
                else if (op_alu) begin
                    // CMP mem (op 7): flags only. ALU reg,mem load-op
                    // (d=1): flags + register writeback. Width = opc[0].
                    // mem_op holds the read operand (rm_word/rm_byte).
                    if (opc[0]) begin
                        psw <= ex_alu16[31:16];
                        if (opc[5:3] != 3'd7 && opc[1])
                            rf[mrm_reg] <= ex_alu16[15:0];
                    end else begin
                        psw <= ex_alu[23:8];
                        if (opc[5:3] != 3'd7 && opc[1])
                            wr_reg8(mrm_reg, ex_alu[7:0]);
                    end
                end
                retire();
                if (op_srld) shadow <= 1'b1;   // sreg-load shadow
            end

            //----------------------------------------------------------------
            // generic wait, then execute (reg forms, MUL/DIV finish)
            //----------------------------------------------------------------
            S_WAITX: begin
                if (div_pend) begin
                    // divide retirement: the iterative unit has long since
                    // finished and latched q/r/flags + the late-trap flag.
                    // word forms: late trap and normal EX share the terminal
                    // cycle (dly==1), only the destination differs. byte
                    // forms: late trap retires one cycle earlier (dly==2)
                    // than EX (dly==1) - the measured +1 EX/late split.
                    if (!div_word && div_late && dly == 6'd2) begin
                        state    <= S_TRAP_IVT1;
                        div_pend <= 1'b0;
                        eu_addr  <= {10'h0, ivt_vec, 2'b00};
                        eu_seg   <= SEG_CS;
                        eu_wr    <= 1'b0;
                        eu_word  <= 1'b1;
                        eu_kind  <= K_MEM;
                    end else if (dly == 6'd1) begin
                        div_pend <= 1'b0;
                        if (div_late) begin
                            state   <= S_TRAP_IVT1;
                            eu_addr <= {10'h0, ivt_vec, 2'b00};
                            eu_seg  <= SEG_CS;
                            eu_wr   <= 1'b0;
                            eu_word <= 1'b1;
                            eu_kind <= K_MEM;
                        end else
                            state <= S_EX;
                    end
                    dly <= dly - 6'd1;
                end else begin
                    if (dly == 6'd1) begin
                        state <= wnext;
                        if (wnext == S_TRAP_IVT1) begin
                            // request params must be valid on state entry
                            eu_addr <= {10'h0, ivt_vec, 2'b00};
                            eu_seg  <= SEG_CS;
                            eu_wr   <= 1'b0;
                            eu_word <= 1'b1;
                            eu_kind <= K_MEM;
                        end
                        if (wnext == S_PREP_RDGO) begin
                            // first pointer-copy read: ready ON state
                            // entry (level-pop+7; closure block)
                            eu_addr <= {sr[SEG_SS], 4'h0} +
                                       {4'h0, rf[5] - 16'd2};
                            eu_seg  <= SEG_SS;
                            eu_wr   <= 1'b0;
                            eu_word <= 1'b1;
                            eu_kind <= K_MEM;
                        end
                        if (wnext == S_RSV) dly <= 6'd1;   // REP setup tail
                    end
                    if (!(dly == 6'd1 && wnext == S_RSV))
                        dly <= dly - 6'd1;
                end
            end

            S_EX: begin
                if (opc == 8'h27 || opc == 8'h2F) begin  // ADJ4A/ADJ4S
                    // DAA/DAS. V = signed overflow of EITHER fix add
                    // (measured on the 27 goldens: +6 crossing 0x80
                    // OR +60 crossing 0x80; DAS mirrored)
                    logic [7:0] al, r1, r2;
                    logic lowfix, hifix;
                    al = rf[0][7:0];
                    lowfix = (al[3:0] > 4'd9) || psw[FB_AC];
                    // V30 deviation from the 8086: with AC set the
                    // high-fix threshold moves to >0x9F (measured
                    // exactly on the 27/2F goldens, 1000/1000)
                    hifix  = (al > (psw[FB_AC] ? 8'h9F : 8'h99)) ||
                             psw[FB_CY];
                    if (opc == 8'h27) begin
                        r1 = al + (lowfix ? 8'h06 : 8'h00);
                        r2 = r1 + (hifix ? 8'h60 : 8'h00);
                        psw[FB_V] <= (lowfix && !al[7] && r1[7]) ||
                                     (hifix && !r1[7] && r2[7]);
                    end else begin
                        r1 = al - (lowfix ? 8'h06 : 8'h00);
                        r2 = r1 - (hifix ? 8'h60 : 8'h00);
                        psw[FB_V] <= (lowfix && al[7] && !r1[7]) ||
                                     (hifix && r1[7] && !r2[7]);
                    end
                    rf[0][7:0] <= r2;
                    psw[FB_AC] <= lowfix;
                    psw[FB_CY] <= hifix;
                    psw[FB_S]  <= r2[7];
                    psw[FB_Z]  <= r2 == 8'd0;
                    psw[FB_P]  <= ~^r2;
                end else if (opc == 8'h37 || opc == 8'h3F) begin
                    // ADJBA/ADJBS (fitted on the 37/3F goldens):
                    // r1 = AL +/- 6 when firing; AL' = r1 & 0F,
                    // AH' = AH +/- 1; CY=AC=fire; S/Z/P of the
                    // PRE-MASK r1; V = signed overflow of the +/-6
                    // step (same family law as ADJ4A/ADJ4S)
                    logic fire;
                    logic [7:0] al, r1;
                    al = rf[0][7:0];
                    fire = (al[3:0] > 4'd9) || psw[FB_AC];
                    if (opc == 8'h37) begin
                        r1 = fire ? al + 8'h06 : al;
                        if (fire) rf[0][15:8] <= rf[0][15:8] + 8'd1;
                        psw[FB_V] <= fire && !al[7] && r1[7];
                    end else begin
                        r1 = fire ? al - 8'h06 : al;
                        if (fire) rf[0][15:8] <= rf[0][15:8] - 8'd1;
                        psw[FB_V] <= fire && al[7] && !r1[7];
                    end
                    rf[0][7:0] <= {4'd0, r1[3:0]};
                    psw[FB_AC] <= fire;
                    psw[FB_CY] <= fire;
                    psw[FB_S]  <= r1[7];
                    psw[FB_Z]  <= r1 == 8'd0;
                    psw[FB_P]  <= ~^r1;
                end else if (opc == 8'hD4) begin       // CVTBD (AAM)
                    logic [7:0] q8, r8;
                    if (disp[7:0] == 8'd0) begin
                        // V20 undocumented: AAM base 0 does NOT trap (unlike the
                        // 8086 DIV0). AH=0xFF, AL preserved; flags S/Z/P from AL.
                        // v20-verified on all 47 base-0 cases.
                        q8 = 8'hFF;
                        r8 = rf[0][7:0];
                    end else begin
                        q8 = rf[0][7:0] / disp[7:0];
                        r8 = rf[0][7:0] % disp[7:0];
                    end
                    rf[0] <= {q8, r8};
                    psw[FB_S] <= r8[7];
                    psw[FB_Z] <= r8 == 8'd0;
                    psw[FB_P] <= ~^r8;
                    psw[FB_V] <= 1'b0;
                    psw[FB_AC] <= 1'b0;
                    psw[FB_CY] <= 1'b0;
                end else if (opc == 8'hD5) begin       // CVTDB (AAD)
                    // V30: the immediate base is IGNORED - always
                    // AH*10+AL (measured on the D5 goldens; CVTBD
                    // does use its base). AC/CY from the final add.
                    logic [7:0] mulb;
                    logic [23:0] ad;
                    mulb = rf[0][15:8] * 8'd10;
                    ad = alu8(3'd0, mulb, rf[0][7:0], psw);
                    rf[0] <= {8'd0, ad[7:0]};
                    // V = the add's signed overflow (measured; the
                    // manual's V=0 is wrong for CVTDB)
                    psw <= ad[23:8];
                end else if (opc == 8'h99) begin                // CVTWL
                    rf[2] <= {16{rf[0][15]}};
                end else if (op_str) begin
                    // REP with CW=0 early-out: no effects
                end else if (op_test1) begin
                    // t = op AND (1<<n): Z=(t==0), S=t[7], P=par(t),
                    // AC=CY=V=0 (undefined_flags.md TEST1 law)
                    logic [7:0] t;
                    t = rm_byte & (8'd1 << immb[2:0]);
                    psw[FB_Z]  <= t == 8'd0;
                    psw[FB_S]  <= t[7];
                    psw[FB_P]  <= ~^t;
                    psw[FB_AC] <= 1'b0;
                    psw[FB_CY] <= 1'b0;
                    psw[FB_V]  <= 1'b0;
                end else if (op_bit1) begin
                    // TEST1/CLR1/SET1/NOT1 generalized (CL and imm
                    // forms, byte/word; 0F18 keeps its fitted branch
                    // above). Laws: TEST1 sets Z/S/P of the masked
                    // value, AC=CY=V=0; the others touch NO flags.
                    logic [3:0] bidx;
                    logic [15:0] bmask, bop, bt;
                    bidx = b1_imm ? (opc2[0] ? immb[3:0]
                                             : {1'b0, immb[2:0]})
                                  : (opc2[0] ? rf[1][3:0]
                                             : {1'b0, rf[1][2:0]});
                    bmask = 16'd1 << bidx;
                    bop = (mrm_mod == 2'd3)
                          ? (opc2[0] ? rf[mrm_rm] : {8'd0, reg8_get(mrm_rm)})
                          : mem_op;
                    if (opc2[2:1] == 2'd0) begin       // TEST1
                        bt = bop & bmask;
                        psw[FB_Z]  <= bt == 16'd0;
                        psw[FB_S]  <= opc2[0] ? bt[15] : bt[7];
                        psw[FB_P]  <= ~^bt[7:0];
                        psw[FB_AC] <= 1'b0;
                        psw[FB_CY] <= 1'b0;
                        psw[FB_V]  <= 1'b0;
                    end else if (mrm_mod == 2'd3) begin
                        logic [15:0] br;
                        br = (opc2[2:1] == 2'd1) ? (bop & ~bmask) :
                             (opc2[2:1] == 2'd2) ? (bop | bmask)  :
                                                   (bop ^ bmask);
                        if (opc2[0]) rf[mrm_rm] <= br;
                        else         wr_reg8(mrm_rm, br[7:0]);
                    end
                end else if (op_insext) begin
                    // EXT retire: AW <- field (0 on runaway); IX +2 on
                    // carry (alias-raw counts as s=16); flags per law
                    rf[0] <= ie_fld;
                    if (ie_mode != 2'd2 && ie_s >= 6'd16)
                        rf[6] <= rf[6] + 16'd2;
                    psw <= (ie_mode == 2'd2) ? ie_psw_run : ie_psw_ext;
                end else if (op_rol4) begin            // reg form
                    wr_reg8(mrm_rm, {rm_byte[3:0], rf[0][3:0]});
                    rf[0][7:0] <= {rf[0][3:0], rm_byte[7:4]};
                end else if (op_ror4) begin            // reg form
                    // measured: AL takes the ENTIRE operand byte
                    // (upper nibble too - undocumented side effect)
                    wr_reg8(mrm_rm, {rf[0][3:0], rm_byte[7:4]});
                    rf[0][7:0] <= rm_byte;
                end else if (op_xchg8) begin
                    wr_reg8(mrm_rm, reg8_get(mrm_reg));
                    wr_reg8(mrm_reg, reg8_get(mrm_rm));
                end else if (op_xchg16) begin
                    rf[mrm_rm]  <= rf[mrm_reg];
                    rf[mrm_reg] <= rf[mrm_rm];
                end else if (op_alu) begin
                    // reg-form ALU: width (opc[0]) + direction (opc[1]);
                    // CMP (op 7) writes flags only
                    if (opc[0]) begin
                        psw <= ex_alu16[31:16];
                        if (opc[5:3] != 3'd7) begin
                            if (opc[1]) rf[mrm_reg] <= ex_alu16[15:0];
                            else        rf[mrm_rm]  <= ex_alu16[15:0];
                        end
                    end else begin
                        psw <= ex_alu[23:8];
                        if (opc[5:3] != 3'd7) begin
                            if (opc[1]) wr_reg8(mrm_reg, ex_alu[7:0]);
                            else        wr_reg8(mrm_rm,  ex_alu[7:0]);
                        end
                    end
                end else if (op_alui) begin
                    // reg forms + CMP-mem: flags (and reg writeback)
                    if (op_grp80) begin
                        psw <= ex_ai8[23:8];
                        if (mrm_mod == 2'd3 && mrm_reg != 3'd7)
                            wr_reg8(mrm_rm, ex_ai8[7:0]);
                    end else begin
                        psw <= ex_ai16[31:16];
                        if (mrm_mod == 2'd3 && mrm_reg != 3'd7)
                            rf[mrm_rm] <= ex_ai16[15:0];
                    end
                end else if (op_accimm || op_testai) begin
                    // ALU acc,imm / TEST acc,imm: op from the opcode
                    // (TEST = AND flags, no writeback)
                    logic [2:0] aop;
                    aop = op_testai ? 3'd4 : opc[5:3];
                    if (opc[0]) begin
                        logic [31:0] c16;
                        c16 = alu16(aop, rf[0], disp, psw);
                        psw <= c16[31:16];
                        if (!op_testai && aop != 3'd7)
                            rf[0] <= c16[15:0];
                    end else begin
                        logic [23:0] c8;
                        c8 = alu8(aop, rf[0][7:0], disp[7:0], psw);
                        psw <= c8[23:8];
                        if (!op_testai && aop != 3'd7)
                            rf[0][7:0] <= c8[7:0];
                    end
                end else if (op_test) begin
                    // TEST rm,reg: AND flags only
                    if (opc[0]) begin
                        logic [31:0] c16;
                        c16 = alu16(3'd4, (mrm_mod == 2'd3) ? rf[mrm_rm]
                                                            : mem_op,
                                    rf[mrm_reg], psw);
                        psw <= c16[31:16];
                    end else begin
                        logic [23:0] c8;
                        c8 = alu8(3'd4, rm_byte, reg8_get(mrm_reg), psw);
                        psw <= c8[23:8];
                    end
                end else if (op_movs8)  wr_reg8(mrm_rm, reg8_get(mrm_reg));
                else if (op_movs16) rf[mrm_rm]  <= rf[mrm_reg];
                else if (op_movl8)  wr_reg8(mrm_reg, reg8_get(mrm_rm));
                else if (op_movl16) rf[mrm_reg] <= rf[mrm_rm];
                else if (op_grpfe) begin
                    psw <= ex_inc[23:8];
                    wr_reg8(mrm_rm, ex_inc[7:0]);
                end else if (op_shrot) begin
                    psw <= sh_fl;                       // iterative unit
                    if (mrm_mod == 2'd3) begin
                        if (sh_word) rf[mrm_rm] <= sh_res;
                        else         wr_reg8(mrm_rm, sh_res[7:0]);
                    end
                end else if (op_grpd0) begin
                    psw <= ex_shl[23:8];
                    wr_reg8(mrm_rm, ex_shl[7:0]);
                end else if ((op_grpf6 || op_grpf7) &&
                             mrm_reg == 3'd0) begin
                    // TEST rm,imm: AND flags only (imm in disp)
                    logic [31:0] t16;
                    logic [23:0] t8;
                    t16 = alu16(3'd4, (mrm_mod == 2'd3) ? rf[mrm_rm]
                                                        : mem_op,
                                disp, psw);
                    t8  = alu8(3'd4, rm_byte, disp[7:0], psw);
                    psw <= op_grpf7 ? t16[31:16] : t8[23:8];
                end else if ((op_grpf6 || op_grpf7) &&
                             mrm_reg == 3'd2) begin
                    // NOT rm (reg form): no flags
                    if (op_grpf7) rf[mrm_rm] <= ~rf[mrm_rm];
                    else          wr_reg8(mrm_rm, ~rm_byte);
                end else if ((op_grpf6 || op_grpf7) &&
                             mrm_reg == 3'd3) begin
                    // NEG rm (reg form): 0 - rm
                    logic [31:0] n16;
                    logic [23:0] n8;
                    n16 = alu16(3'd5, 16'd0, rf[mrm_rm], psw);
                    n8  = alu8(3'd5, 8'd0, rm_byte, psw);
                    if (op_grpf7) begin
                        psw <= n16[31:16];
                        rf[mrm_rm] <= n16[15:0];
                    end else begin
                        psw <= n8[23:8];
                        wr_reg8(mrm_rm, n8[7:0]);
                    end
                end else if (op_grpf6 && mrm_reg == 3'd5) begin
                    // MUL (IMUL8): AW = AL x rm8 signed;
                    // CY=V = AH != sext(AL) (fit pending)
                    logic signed [15:0] ms;
                    ms = $signed({{8{rm_byte[7]}}, rm_byte}) *
                         $signed({{8{rf[0][7]}}, rf[0][7:0]});
                    rf[0] <= ms;
                    psw[FB_CY] <= ms[15:8] != {8{ms[7]}};
                    psw[FB_V]  <= ms[15:8] != {8{ms[7]}};
                    // S/Z/AC/P = flags of the internal self-add
                    // lo+lo (last multiplier shift stage; measured)
                    psw[FB_S]  <= ms[6];
                    psw[FB_Z]  <= ms[6:0] == 7'd0;
                    psw[FB_AC] <= ms[3];
                    psw[FB_P]  <= ~(^ms[6:0]);
                end else if (op_grpf7 && mrm_reg == 3'd4) begin
                    // MULU16: {DW,AW} = AW x rm16; CY=V=(DW!=0)
                    logic [31:0] mw;
                    mw = {16'd0, (mrm_mod == 2'd3) ? rf[mrm_rm] : mem_op}
                         * {16'd0, rf[0]};
                    rf[0] <= mw[15:0];
                    rf[2] <= mw[31:16];
                    psw[FB_CY] <= mw[31:16] != 16'd0;
                    psw[FB_V]  <= mw[31:16] != 16'd0;
                end else if (op_grpf7 && mrm_reg == 3'd5) begin
                    // MUL (IMUL16): {DW,AW} signed; CY=V on overflow
                    logic signed [31:0] mws;
                    mws = $signed((mrm_mod == 2'd3) ? rf[mrm_rm] : mem_op)
                          * $signed(rf[0]);
                    rf[0] <= mws[15:0];
                    rf[2] <= mws[31:16];
                    psw[FB_CY] <= mws[31:16] != {16{mws[15]}};
                    psw[FB_V]  <= mws[31:16] != {16{mws[15]}};
                    // S/Z/AC/P from the internal lo+lo self-add (measured)
                    psw[FB_S]  <= mws[14];
                    psw[FB_Z]  <= mws[14:0] == 15'd0;
                    psw[FB_AC] <= mws[3];
                    psw[FB_P]  <= ~(^mws[6:0]);
                end else if (op_grpf6 && mrm_reg == 3'd6) begin
                    // DIVU8 writeback: AL=q, AH=r
                    rf[0] <= {disp[7:0], mem_op[7:0]};
                end else if (op_grpf6 && mrm_reg == 3'd7) begin
                    // IDIV8 writeback: AL=q, AH=r (flags set at dispatch)
                    rf[0] <= mem_op;
                end else if (op_grpf6) begin
                    // MULU8: AW = AL * op8; CY=V = (AH != 0);
                    // S/Z/AC/P preserved (undefined_flags.md)
                    logic [15:0] m;
                    m = {8'd0, rm_byte} * {8'd0, rf[0][7:0]};
                    rf[0] <= m;
                    psw[FB_CY] <= m[15:8] != 8'd0;
                    psw[FB_V]  <= m[15:8] != 8'd0;
                end else if (op_imuli) begin
                    // MUL reg,rm,imm: reg = rm x imm signed (fit pending)
                    logic signed [31:0] mi;
                    logic [15:0] iop;
                    iop = (opc == 8'h6B) ? {{8{disp[7]}}, disp[7:0]}
                                         : disp;
                    mi = $signed((mrm_mod == 2'd3) ? rf[mrm_rm] : mem_op)
                         * $signed(iop);
                    rf[mrm_reg] <= mi[15:0];
                    psw[FB_CY] <= mi[31:16] != {16{mi[15]}};
                    psw[FB_V]  <= mi[31:16] != {16{mi[15]}};
                    // S/Z/AC/P from the internal lo+lo self-add (measured)
                    psw[FB_S]  <= mi[14];
                    psw[FB_Z]  <= mi[14:0] == 15'd0;
                    psw[FB_AC] <= mi[3];
                    psw[FB_P]  <= ~(^mi[6:0]);
                end else if (op_grpff) begin
                    // FF.0/FF.1 INC/DEC rm16 (reg form)
                    {psw, rf[mrm_rm]} <=
                        incdec16(mrm_reg == 3'd1, rf[mrm_rm], psw);
                end else if (op_grpf7) begin
                    // DIVU16 (no trap): AW=quot (mem_op), DW=rem (disp);
                    // flags were set by the pre-check compare at dispatch
                    rf[0] <= mem_op;
                    rf[2] <= disp;
                end
                retire();
            end

            //----------------------------------------------------------------
            // RMW writeback (byte ops: sibling lane preserved from the read)
            //----------------------------------------------------------------
            S_RMWX: begin
                if (dly == 6'd1) begin
                    state <= S_WREQ;
                    eu_wr <= 1'b1;
                    if (op_xchg8) begin
                        eu_wdata <= src_pair;
                        wr_reg8(mrm_reg, mem_op[7:0]);
                    end else if (op_xchg16) begin
                        eu_wdata <= rf[mrm_reg];
                        rf[mrm_reg] <= mem_op;
                    end else if (op_rol4) begin
                        // driven pair = {AL_new, mem_new}: AL rides the
                        // sibling lane of the internal operand pair
                        eu_wdata <= {rf[0][3:0], mem_op[7:4],
                                     mem_op[3:0], rf[0][3:0]};
                        rf[0][7:0] <= {rf[0][3:0], mem_op[7:4]};
                    end else if (op_ror4) begin
                        // ROR4 mem: AL' = whole mem byte; driven pair
                        // = {AL_new, mem_new} (measured)
                        eu_wdata <= {mem_op[7:0],
                                     rf[0][3:0], mem_op[7:4]};
                        rf[0][7:0] <= mem_op[7:0];
                    end else if (op_bit1) begin
                        // CLR1/SET1/NOT1 mem: only the addressed bit
                        // changes; sibling lane preserved from the read
                        logic [3:0] bidx;
                        logic [15:0] bmask;
                        bidx = b1_imm ? (opc2[0] ? immb[3:0]
                                                 : {1'b0, immb[2:0]})
                                      : (opc2[0] ? rf[1][3:0]
                                                 : {1'b0, rf[1][2:0]});
                        bmask = 16'd1 << bidx;
                        eu_wdata <= (opc2[2:1] == 2'd1) ? (mem_op & ~bmask)
                                  : (opc2[2:1] == 2'd2) ? (mem_op | bmask)
                                  :                       (mem_op ^ bmask);
                    end else if (op_alui) begin
                        eu_wdata <= ai_wide;
                        psw <= op_grp80 ? ex_ai8[23:8] : ex_ai16[31:16];
                    end else if ((op_grpf6 || op_grpf7) &&
                                 mrm_reg == 3'd2) begin
                        eu_wdata <= ~mem_op;    // NOT mem: no flags
                    end else if ((op_grpf6 || op_grpf7) &&
                                 mrm_reg == 3'd3) begin
                        // NEG mem (byte forms drive the negated pair -
                        // sibling GUESS, fit vs goldens)
                        logic [31:0] n16;
                        logic [23:0] n8;
                        n16 = alu16(3'd5, 16'd0, mem_op, psw);
                        n8  = alu8(3'd5, 8'd0, mem_op[7:0], psw);
                        eu_wdata <= n16[15:0];
                        psw <= op_grpf7 ? n16[31:16] : n8[23:8];
                    end else if (op_grpff) begin
                        logic [31:0] idw;
                        idw = {16'd0, 16'd0};
                        {idw[31:16], idw[15:0]} =
                            incdec16(mrm_reg == 3'd1, mem_op, psw);
                        eu_wdata <= idw[15:0];
                        psw <= idw[31:16];
                    end else if (op_shrot) begin
                        // measured split: by-1 forms (D0/D1) operate on
                        // the full internal pair; count forms (C0/C1/
                        // D2/D3) loop on the byte and PRESERVE the
                        // sibling lane (C0.0 goldens). Value/flags now
                        // come from the iterative unit (sh_res/sh_fl).
                        eu_wdata <= sh_res;
                        psw <= sh_fl;
                    end else if (op_alu && opc[0]) begin
                        // word ALU RMW (d=0): drive the 16-bit result
                        eu_wdata <= ex_alu16[15:0];
                        psw      <= ex_alu16[31:16];
                    end else begin
                        eu_wdata <= rmw_wide;
                        if (op_alu)        psw <= ex_alu[23:8];
                        else if (op_grpfe) psw <= ex_inc[23:8];
                        else               psw <= ex_shl[23:8];
                    end
                end
                dly <= dly - 6'd1;
            end
            S_WREQ: if (eu_started) state <= S_WBUSW;
            S_WBUSW: if (eu_done) begin
                retire();
                if (op_srst) shadow <= 1'b1;  // mem-form 8C also shadows
            end

            //----------------------------------------------------------------
            // PUSH r16 (PUSH SP pushes the decremented value, 8086-style)
            //----------------------------------------------------------------
            S_PUSH_CALC: begin
                if (opc == 8'h9C)
                    issue_push(psw);
                else if (op_pushi)
                    issue_push(disp);            // 68 imm16
                else if (op_pushsr)
                    issue_push(sr[srmap(opc[4:3])]);
                else if (op_grpff)               // FF.6 reg
                    issue_push(rf[mrm_rm] -
                               ((mrm_rm == 3'd4) ? 16'd2 : 16'd0));
                else
                    issue_push(rf[opc[2:0]] -
                               ((opc[2:0] == 3'd4) ? 16'd2 : 16'd0));
                state <= S_REQ;
            end

            //----------------------------------------------------------------
            // divide trap sequence (timing from F7.6 golden traces)
            //----------------------------------------------------------------
            S_TRAP_IVT1: if (eu_started) begin
                eu_addr <= eu_addr + 20'd2;   // vector high word
                state <= S_TRAP_IVT2;
            end
            S_TRAP_IVT2: begin
                if (eu_done) ivt_off <= eu_rdata;
                if (eu_started) state <= S_TRAP_IVT2W;
            end
            S_TRAP_IVT2W: if (eu_done) begin
                ivt_seg  <= eu_rdata;
                fl_cs    <= eu_rdata;
                fl_ip    <= ivt_off;
                // psw already carries the divide residue (pre-check
                // compare); latch the to-be-pushed value here, and clear
                // the live IE/TF at once - the PS status bits of the push
                // cycles already show IE=0 (measured)
                trap_psw <= psw;
                // POP-PSW boundary race (closure block): pop_pend marks
                // recognition at POP PSW's own boundary; the measured
                // race table picks class B (revert to the pre-pop
                // image) vs class A (keep the popped image) from the
                // two flag words - see the race_rom declaration
                // the revert needs pre-IE=1 (a pre-IE=0 pop cannot race:
                // recognition waited for the popped IE, silicon commits
                // the popped image; measured 89/89) - and the same
                // table covers own-boundary AND one-NOP-late
                // recognitions (7/7 tranche late races)
                if (pop_pend && psw_old[9] && race_B)
                    psw <= psw_old & ~16'h0300;
                else
                    psw <= psw & ~16'h0300;
                pop_pend <= 1'b0;
                dly <= 6'd2; state <= S_TRAP_W1;
            end
            S_TRAP_W1: begin
                if (dly == 6'd1) begin
                    state <= S_TRAP_PSW;
                    issue_push(trap_psw);
                end
                dly <= dly - 6'd1;
            end
            S_TRAP_PSW: if (eu_started) begin
                state <= S_TRAP_PSWW;
                dly   <= 6'd11;      // fixed microcode slot: PS at started+12
            end
            // The trap chain issues the next push at the EARLIER of two
            // paths (measured on the F7.6 waits tranches): the
            // completion-gated path - early write-done strobe (ready
            // edge under waits, the old T4 law at zero waits) + 2 - and
            // the microcode's own fixed 12-cycle push slot from
            // eu_started. Splits + waits can stretch the bus cycle past
            // the slot; the request then waits only on BIU arbitration.
            S_TRAP_PSWW: begin
                if (eu_wdone) begin
                    dly <= 6'd1; state <= S_TRAP_W2;
                end else if (dly == 6'd1) begin
                    state <= S_TRAP_PS;
                    issue_push(sr[SEG_CS]);
                end else begin
                    dly <= dly - 6'd1;
                end
            end
            S_TRAP_W2: begin
                if (dly == 6'd1) begin
                    state <= S_TRAP_PS;
                    issue_push(sr[SEG_CS]);
                end
                dly <= dly - 6'd1;
            end
            S_TRAP_PS: if (eu_started) state <= S_TRAP_PSW2W;
            S_TRAP_PSW2W: if (eu_wdone) begin
                state <= S_TRAP_FLUSH;
                flush_now <= 1'b1;
                issue_push(pc);
                pc <= ivt_off;
                sr[SEG_CS] <= ivt_seg;
            end
            //----------------------------------------------------------------
            // REP interruption tail: wait for the in-flight access, then
            // rewind pc to the first prefix, flush + refetch there
            // (measured: the resume refetch precedes the INTA pair),
            // then the INT/NMI entry sequence.
            //----------------------------------------------------------------
            S_IRQ_REPW: begin
                dly <= dly - 6'd1;
                if (eu_done) begin
                    pc <= insn_ip;
                    fl_cs <= sr[SEG_CS];
                    fl_ip <= insn_ip;
                end
                if (dly == 6'd1) state <= S_IRQ_REPFL;
            end
            S_IRQ_REPFL: begin   // q_flush high this cycle (comb)
                if (nmi_latch) begin
                    nmi_latch <= 1'b0;
                    ivt_vec   <= 8'd2;
                    dly <= 6'd8; wnext <= S_TRAP_IVT1;
                    irq_disp <= 1'b1;
                    state <= S_WAITX;
                end else begin
                    dly <= 6'd2;
                    state <= S_INT_W0;
                end
            end

            //----------------------------------------------------------------
            // INT entry: 2 INTA bus cycles 7 apart (vector byte taken
            // from the second), then the divide-trap IVT/push/flush
            // chain (interrupt_model.md)
            //----------------------------------------------------------------
            S_IRQ_D: begin      // INT dispatch: request ready 2 cycles
                eu_addr <= '0;  // after the blocked boundary pop slot
                eu_seg  <= SEG_CS;
                eu_wr   <= 1'b0;
                eu_word <= 1'b1;
                eu_kind <= K_INTA;
                state   <= S_INT_A1;
            end
            S_INT_W0: begin
                if (dly == 6'd1) begin
                    state <= S_INT_A1;
                    eu_addr <= '0;
                    eu_seg  <= SEG_CS;
                    eu_wr   <= 1'b0;
                    eu_word <= 1'b1;
                    eu_kind <= K_INTA;
                end
                dly <= dly - 6'd1;
            end
            S_INT_A1:  if (eu_started) state <= S_INT_A1W;
            S_INT_A1W: if (eu_done) state <= S_INT_G;
            S_INT_G:   begin
                state <= S_INT_A2;    // one gap cycle: T1-to-T1 = 7
                eu_kind <= K_INTA;
            end
            S_INT_A2:  if (eu_started) state <= S_INT_A2W;
            S_INT_A2W: if (eu_done) begin
                ivt_vec <= eu_rdata[7:0];
                eu_kind <= K_MEM;
                dly <= 6'd4; wnext <= S_TRAP_IVT1;   // IVT T1 = T4+7
                irq_disp <= 1'b1;
                state <= S_WAITX;
            end

            //----------------------------------------------------------------
            // HALT: one pseudo bus cycle, then idle with the bus held.
            // Wake: NMI -> vector 2; INT with IE=1 -> INTA sequence;
            // INT with IE=0 -> resume at the next instruction WITHOUT
            // vectoring (measured, != 8086).
            //----------------------------------------------------------------
            S_HALTED: begin
                if (nmi_latch) begin
                    nmi_latch <= 1'b0;
                    ivt_vec   <= 8'd2;
                    halt_disp <= 1'b0;
                    dly <= 6'd7; wnext <= S_TRAP_IVT1;
                    state <= S_HWAIT;
                end else if (int_p[1] && psw[9]) begin
                    halt_disp <= 1'b0;
                    dly <= 6'd3; wnext <= S_INT_A1;
                    state <= S_HWAIT;
                end else if (int_p[2]) begin
                    halt_disp <= 1'b0;
                    retire();          // masked INT releases the halt
                end
            end
            S_HWAIT: begin   // wake wait (bus stays held: no prefetch)
                if (dly == 6'd1) begin
                    state <= wnext;
                    if (wnext == S_TRAP_IVT1) begin
                        eu_addr <= {10'h0, ivt_vec, 2'b00};
                        eu_seg  <= SEG_CS;
                        eu_wr   <= 1'b0;
                        eu_word <= 1'b1;
                        eu_kind <= K_MEM;
                        irq_disp <= 1'b1;
                    end else begin     // S_INT_A1
                        eu_addr <= '0;
                        eu_seg  <= SEG_CS;
                        eu_wr   <= 1'b0;
                        eu_word <= 1'b1;
                        eu_kind <= K_INTA;
                    end
                end
                dly <= dly - 6'd1;
            end

            //----------------------------------------------------------------
            // POLL: pin low = 3-cycle no-op; else sample every 5 clocks,
            // resume 4 cycles after the satisfied sample
            //----------------------------------------------------------------
            S_POLL_WAIT: begin
                if (dly == 6'd1) begin
                    if (!pin_poll_n) begin  // live sample every 5 clocks
                        dly <= 6'd3;        // next F = sample + 4
                        state <= S_POLL_X;
                    end else
                        dly <= 6'd5;   // next sample in 5 clocks
                end else
                    dly <= dly - 6'd1;
            end
            S_POLL_X: begin
                if (dly == 6'd1) retire();
                dly <= dly - 6'd1;
            end

            S_IN_PORT: if (q_pop) begin     // E4-E7 imm8 port @F+2
                pc <= pc + 16'd1;
                eu_addr <= {12'h0, q_byte};
                eu_seg  <= SEG_CS;
                eu_wr   <= op_out;
                eu_word <= opc[0];
                eu_kind <= K_IO;
                if (op_out) eu_wdata <= rf[0];
                state   <= S_REQ;
            end

            S_TRAP_FLUSH: state <= eu_started ? S_TRAP_PCW : S_TRAP_PC;
            S_TRAP_PC:    if (eu_started) state <= S_TRAP_PCW;
            S_TRAP_PCW:   if (eu_done) begin
                arch_ip <= pc;
                state <= S_FIRST;
            end

            default: state <= S_HALT;
        endcase

        // REP first-boundary abort, pop-anchored parallel check: when
        // the first write was accepted BEFORE the pop+7 decision edge,
        // the FSM is already in S_STRS with the next write pending
        // (its accept is >= pop+9, so no clash with the accept branch
        // above). On an abort decision the pending request is
        // withdrawn (eu_req is combinational from state) and the
        // flush lands at pop+16 = edge+9 (dly=8 from the edge).
        if (state == S_STRS && rslot == 6'd6 && irq_rep &&
            rep_en && op_stostr) begin
            state <= S_IRQ_REPW;
            dly   <= 6'd8;
        end
    end
end

//----------------------------------------------------------------------------
// backdoor observation
//----------------------------------------------------------------------------
// The divide-trap chain holds the bus across its whole IVT-read/push
// sequence up to the flush (measured on the waits tranches: cold traps
// with a non-full queue show no prefetch in the inter-access gaps).
// This hold blocks prefetch commits but is NOT an EU request: it must
// not feed the mid-cycle-commit request history (eu_req_p1/p2) - the
// w1 traps' PS push takes the idle-end slot, not the mid-cycle one.
// The post-flush PC-push wait (S_TRAP_PCW) does not hold: the handler
// prefetch commits at that push's T3 edge.
assign eu_hold = state == S_TRAP_IVT2W || state == S_TRAP_W1 ||
                 state == S_TRAP_PSWW  || state == S_TRAP_W2 ||
                 state == S_TRAP_PSW2W ||
                 (state == S_HALTED && !(int_p[2] && !psw[9])) ||
                 (state == S_HWAIT && wnext == S_TRAP_IVT1) ||
                 (state == S_WAITX && (wnext == S_TRAP_IVT1 ||
                                       wnext == S_INT_A1) && irq_disp);

assign dbg_regs = {psw, arch_ip, sr[SEG_DS], sr[SEG_SS], sr[SEG_CS],
                   sr[SEG_ES], rf[7], rf[6], rf[5], rf[4], rf[3], rf[2],
                   rf[1], rf[0]};
assign dbg_pend = popr_pend || iret_pw;

endmodule
