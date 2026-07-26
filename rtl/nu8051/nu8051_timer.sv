//============================================================================
// Imported from nu8051 (machine-cycle-accurate Intel 8051/8052 core, SystemVerilog)
// Source: rtl/nu8051_timer.sv, commit e2c97de0fabb3b3baa1eb98a15284bc74ef92f4e
// Do not hand-edit; re-import from upstream. The M72 integration wrapper is
// rtl/mcu.sv (distilled from nu8051 rtl/m72/mcu.sv), NOT a raw re-import.
//============================================================================
//============================================================================
//
//  nu8051_timer - timer/counter SFR block (core_design.md §5.4 row 3, §6.2)
//
//  TIMERS 0 AND 1 ARE COMPLETE HERE (work package C5.2, periph §4.1-§4.5):
//  all four modes on both timers, C/T counter mode with the manual's sampled
//  1->0 pin detector, GATE off the INTx# PIN (PQ1), the mode-3 split and the
//  timer-1 rules that go with it (PQ4).  The engine first appeared in
//  Phase 3 because the pilot corpus needed it (QUESTION-P3-2: with timer 0
//  in mode 3, timer 1 free-runs regardless of TR1, and the pilot randomizes
//  TMOD while freezing TR0/TR1); C5.2 completed and directed-verified it -
//  tests/directed_rtl/c5_2/ carries the derivation of every instant below.
//
//  Phase mapping (core_design §2.2 / §6.2 / §6.1 [D-10]):
//    tk_s3p1  increments apply (PQ3 [D-07]) - both timer and counter mode.
//             Suppressed in PRIME and RESET_HOLD (a PRIME cycle stands in
//             for a machine cycle the reference has already counted).
//    tk_s5p2  T0/T1 pin sampling, 1->0 edge arms a pending count; TF0/TF1
//             transfer out of the tf0_stage/tf1_stage flops (D1.2 F-7)
//
//  Cadence and latency, derived once here and asserted by the c5_2 suite:
//
//    timer mode (C/T=0): +1 per machine cycle in which the enable term is
//      true, at that cycle's S3P1 edge.  An SFR read of TLx/THx sees the
//      increment iff its capture edge is at or after S3P1: the R3 "late"
//      slot (S5P1, the slot `MOV A,direct` uses) does, the R1/R2 slots
//      (S2P1/S2P2, e.g. the source of `MOV direct,direct`) do not - PQ3's
//      N-1 read rule is slot-dependent (QUESTION-P52-1).
//
//    counter mode (C/T=1): the pin is sampled at S5P2 of every machine
//      cycle; a high sample in cycle N-1 followed by a low sample in cycle N
//      arms `tX_pend`, and the count appears in the register at S3P1 of
//      cycle N+1 (periph §4.1 verbatim).  Pin-to-count latency is therefore
//      7 oscillator periods from the S5P2 that sees the low level, 19 from
//      the S5P2 that saw the preceding high, and the sustained rate ceiling
//      is one count per 2 machine cycles = osc/24.  A level that is not
//      present at any S5P2 is never seen at all.
//
//    `tX_pend` is a one-machine-cycle PULSE, not a request: it is cleared at
//      every S3P1 whether or not the enable term let it count (periph §4.2
//      Figure 6 puts the run/GATE AND *between* the transition detector and
//      the counter clock).  So an edge that arrives while TRx=0 or the gate
//      is shut is lost, not banked - QUESTION-P52-3.
//
//    TF0/TF1 set at S5P2 of the overflowing cycle (periph §4.5) via the
//      tf0_stage/tf1_stage flops [D1.2 F-7], which is also the irq snapshot
//      edge, so the poll sees the flag in the next cycle (timing §6).
//
//  TCON.IE0/IE1 latching and the §6.4 vectoring clears arrive with C5.3 as
//  WRITES from nu8051_irq (`irq_iex_*` / `irq_tfx_clr`): the rules are the
//  interrupt unit's, the register is this module's.
//
//  TIMER 2 IS COMPLETE HERE TOO (work package C5.5, periph §4.6, Figures
//  11/12/13/16), under P_8052.  With P_8052 = 0 not one flop of it moves: the
//  five SFR addresses do not answer (so they read NU8051_CFG_UNIMPL_RD from
//  the core's PQ5 mux), `t2con_q` reads 00H, `t2_ovf` never pulses, and the
//  interrupt unit's sixth source is dead.
//
//  Timer 2's three modes and the instants this file implements:
//
//    capture      (CP/RL2 = 1, RCLK = TCLK = 0): a plain 16-bit up-counter;
//                 overflow sets TF2 and the count wraps.  EXEN2 = 1 adds the
//                 T2EX path: a 1-to-0 transition captures TL2/TH2 into
//                 RCAP2L/RCAP2H and sets EXF2 (Figure 12).
//    auto-reload  (CP/RL2 = 0, RCLK = TCLK = 0): overflow sets TF2 and LOADS
//                 TL2/TH2 from RCAP2L/RCAP2H.  EXEN2 = 1: a T2EX 1-to-0
//                 transition triggers the same LOAD and sets EXF2 (Figure 13).
//                 LOAD, not add - PQ10 (the reference model adds RCAP2 to the
//                 running count on the T2EX path; the manual's RELOAD line
//                 drives the same transfer gates the overflow path does).
//    baud-rate    (RCLK = 1 or TCLK = 1, CP/RL2 ignored): a TH2 rollover
//                 reloads from RCAP2 and DOES NOT set TF2 ("will not generate
//                 an interrupt"), and the timer function is clocked at OSC/2,
//                 not OSC/12 - Figure 16's "NOTE: OSC. FREQ. IS DIVIDED BY 2,
//                 NOT 12".  EXEN2 = 1 still sets EXF2 on a T2EX transition but
//                 causes no reload ("an additional external interrupt").
//
//  Phase mapping, derived in tests/directed_rtl/c5_5/README.md:
//    the count applies at S3P1 (timer and counter mode alike) - periph §4.1
//      names "T0, T1, or 8052 T2" in the same sentence as the S3P1 rule - or
//      at every OSC/2 tick in baud mode, where the register is 6 counts per
//      machine cycle wide of the machine-cycle grid entirely.
//    tk_s5p2  T2/T2EX pin sampling and 1->0 detection (the C5.2 detector,
//             `t2_pend`/`t2ex_pend` one-cycle pulses); the T2EX event's EXF2
//             is set HERE, at S5P2 - the manual is explicit: "the Timer 2 flag
//             EXF2 and the Serial Port flags RI and TI are set at S5P2. The
//             values are not actually polled by the circuitry until the next
//             machine cycle" (3-25).  Its capture/reload half runs at the
//             S3P1 of the same cycle (the count-apply edge), so `t2ex_pend`
//             spans S5P2 -> S3P1 -> S5P2 and is cleared by the S5P2 that sets
//             the flag.  EXEN2 cannot change in between (SFR writes commit at
//             S6P2), so one gate term serves both halves - PQ9.
//    tk_s2p2  TF2's set instant, out of `tf2_stage` [D1.2 F-7] - TQ8: "the
//             Timer 2 flag TF2 is set at S2P2 and is polled in the same cycle
//             in which the timer overflows" (3-25), the one flag that is not
//             on the uniform S5P2 sample.  The overflow is computed by the
//             S3P1 (or OSC/2) apply and staged to the NEXT S2P2, which is
//             what the ratified save-state inventory allocates `tf2_stage`
//             for; the interrupt unit polls that flag LIVE in the cycle it is
//             set instead of out of its S6P2 snapshot, which is what makes
//             the "same cycle" true and buys the source one machine cycle of
//             latency (see QUESTION-P55-1 for the residue the manual leaves).
//
//============================================================================

`timescale 1ns/1ps

module nu8051_timer #(
    parameter bit P_8052 = 1'b1
)(
    input  logic        CLK,
    input  logic        CE,
    input  logic        rst_hold,

    input  logic        tk_s2p2,         // C5.5: TF2's set instant (TQ8)
    input  logic        tk_s3p1,
    input  logic        tk_s5p2,
    input  logic        prime_cycle,     // §2.8: sample pins, arm no edge

    // SFR bus
    input  logic  [7:0] sfr_addr,
    output logic  [7:0] sfr_rdata,
    output logic        sfr_hit,
    input  logic        sfr_we,
    input  logic  [7:0] sfr_wdata,

    // pins / gate terms
    input  logic        T0, T1,
    input  logic        T2, T2EX,        // 8052 counter / capture-reload pins
    input  logic        int0_pin,        // S5P2-sampled INT0 pin level (PQ1)
    input  logic        int1_pin,

    // C5.3: TCON carries the four external-interrupt bits (IT0/IE0/IT1/IE1)
    // and the two timer flags the interrupt system consumes and clears.  The
    // REGISTER lives here (D1.2 §2.6, map 0x080 - one register, one symbol);
    // the RULES live in nu8051_irq, which hands the writes back over these
    // three ports: `irq_iex_*` is the S5P2 level/edge maintenance and
    // `irq_tfx_clr` the §6.4 vectoring clear at the S6P2 edge of ILCALL1.
    input  logic  [1:0] irq_iex_we,      // {IE1, IE0} write enable
    input  logic  [1:0] irq_iex_d,       // {IE1, IE0} new value
    input  logic  [1:0] irq_tfx_clr,     // {TF1, TF0} vectoring clear

    output logic  [7:0] tcon_q,          // to the irq unit (flags + IT bits)
    // C5.5: T2CON to the irq unit (TF2/EXF2 = the sixth source, periph §6.1)
    // and to the UART (RCLK/TCLK = the two baud-source selects, periph §5.8).
    // Reads 00H when P_8052 = 0, so neither consumer needs its own guard.
    output logic  [7:0] t2con_q,

    // Timer-1 overflow pulse for the serial port's baud generator (periph
    // §5.4).  It is the OVERFLOW, not TF1: while timer 0 is in mode 3 the
    // TF1 flag belongs to TH0 and timer 1 sets no flag at all, but "it can
    // serve as the serial-port baud generator" (periph §4.4) - so the UART
    // is fed from timer 1's own carry-out, at the S3P1 apply edge that
    // produces it ([D-07]).
    output logic        t1_ovf,

    // Timer-2 overflow pulse for the serial port's baud generator (periph
    // §5.8 / Figure 16): the RCLK/TCLK muxes take it with NO divide-by-2
    // (the SMOD stage sits only in the timer-1 branch).  It rides the edge
    // that produced the rollover - S3P1 in counter mode, an OSC/2 tick in the
    // baud-rate mode's timer function, where it is not machine-cycle aligned
    // at all.
    output logic        t2_ovf,

    // save state (savestate_design §5.1/§5.2, map 0x080-0x096).  The eleven
    // timer-2 addresses (0x08C-0x096) are mapped UNCONDITIONALLY (§2's
    // P_8052 rule): on a P_8052 = 0 build they read 0 and their writes are
    // ignored, so the map, the counts, the tag and ss_addr_of iteration are
    // identical for both builds - one snapshot format, one tooling path.
    // The gate is a parameter conditional rather than §5.1's `generate`: a
    // generate cannot split one `case`, and naming the eleven symbols again
    // in an else-arm would break §7.6's "exactly twice" count.  Same constant
    // fold, one textual occurrence per arm (savestate_design §3.4).
    input  logic  [9:0] ss_addr,
    input  logic [15:0] ss_wdata,
    input  logic        ss_we,
    output logic [15:0] ss_rdata

`ifdef NU8051_BACKDOOR
    ,
    input  logic        sfr_bkd_we
`endif
);

`include "nu8051_defs.svh"
    import nu8051_ss_pkg::*;

    logic [7:0] tcon_r, tmod_r, tl0_r, th0_r, tl1_r, th1_r;
    logic [7:0] t2con_r, rcap2l_r, rcap2h_r, tl2_r, th2_r;
    logic       t0_smpl, t1_smpl, t0_pend, t1_pend;
    logic       tf0_stage, tf1_stage;     // D1.2 F-7 (mapped state)
    // C5.5 timer-2 engine state (savestate_design §2.6 P_8052 group)
    logic       t2_smpl, t2_pend, t2ex_smpl, t2ex_pend;
    logic       t2_osc2;                  // OSC/2 state-time prescaler
    logic       tf2_stage;                // overflow -> S2P2 (TQ8, F-7)

    assign tcon_q  = tcon_r;
    // P_8052 = 0: the register never moves (no address answers, so nothing
    // writes it), but publish the constant anyway so the consumers are
    // structurally dead rather than merely quiet.
    assign t2con_q = P_8052 ? t2con_r : 8'h00;

    wire [1:0] mode0 = tmod_r[1:0];
    wire [1:0] mode1 = tmod_r[5:4];
    wire       ct0   = tmod_r[2];
    wire       gate0 = tmod_r[3];
    wire       ct1   = tmod_r[6];
    wire       gate1 = tmod_r[7];
    wire       tr0   = tcon_r[TCON_TR0];
    wire       tr1   = tcon_r[TCON_TR1];

    // ---- SFR read mux ------------------------------------------------------
    always_comb begin
        sfr_hit = 1'b1;
        case (sfr_addr)
            SFR_TCON: sfr_rdata = tcon_r;
            SFR_TMOD: sfr_rdata = tmod_r;
            SFR_TL0:  sfr_rdata = tl0_r;
            SFR_TL1:  sfr_rdata = tl1_r;
            SFR_TH0:  sfr_rdata = th0_r;
            SFR_TH1:  sfr_rdata = th1_r;
            default:  begin sfr_rdata = 8'h00; sfr_hit = 1'b0; end
        endcase
        if (P_8052) begin
            case (sfr_addr)
                SFR_T2CON:  begin sfr_rdata = t2con_r;  sfr_hit = 1'b1; end
                SFR_RCAP2L: begin sfr_rdata = rcap2l_r; sfr_hit = 1'b1; end
                SFR_RCAP2H: begin sfr_rdata = rcap2h_r; sfr_hit = 1'b1; end
                SFR_TL2:    begin sfr_rdata = tl2_r;    sfr_hit = 1'b1; end
                SFR_TH2:    begin sfr_rdata = th2_r;    sfr_hit = 1'b1; end
                default: ;
            endcase
        end
    end

    // ---- counting engine ---------------------------------------------------
    // Enable term, periph §4.2 Figure 6:
    //     count = TRx AND (!GATEx OR INTx#_pin) AND (C/T ? Tx 1->0 : 1)
    // GATE consumes the S5P2-SAMPLED INTx# pin level (PQ1 - the manual; MAME
    // gates on the latched IEx flag and carries a TODO saying so).
    // `int0_pin`/`int1_pin` are nu8051_irq's sample registers
    // (SSA_I_INT0_SMPL/_INT1_SMPL), so a level driven during cycle N first
    // gates the increment at S3P1 of cycle N+1.
    wire gate0_ok = !gate0 || int0_pin;
    wire gate1_ok = !gate1 || int1_pin;
    wire run0 = tr0 && gate0_ok;
    wire run1 = tr1 && gate1_ok;
    wire d0 = run0 && (ct0 ? t0_pend : 1'b1);
    // Timer 1 while timer 0 is in mode 3 (periph §4.4 / PQ4): TR1 and TF1 are
    // stolen by TH0, so timer 1 is switched on and off by moving it into and
    // out of mode 3 and runs regardless of TR1; it is forced to TIMER
    // operation (C/T1 ignored) and GATE1 is unavailable - the DS5002FP
    // wording the reference model implements, which the Intel Ch3 text does
    // not contradict.
    wire t1_free = (mode0 == 2'd3);
    wire d1 = t1_free ? 1'b1
                      : (run1 && (ct1 ? t1_pend : 1'b1));

    // next-value helpers -----------------------------------------------------
    logic [7:0] tl0_n, th0_n, tl1_n, th1_n;
    logic       ov0, ov1;

    task automatic step_pair(input  logic [1:0] md,
                             input  logic       en,
                             input  logic [7:0] tl, th,
                             output logic [7:0] tln, thn,
                             output logic       ov);
        logic [16:0] c;
        begin
            tln = tl; thn = th; ov = 1'b0;
            if (en) begin
                case (md)
                    2'd0: begin                       // 13-bit (periph §4.4)
                        // THx + the LOW 5 BITS of TLx; TLx's upper 3 bits are
                        // "indeterminate and should be ignored" per the
                        // manual - this core writes them back as 0 on the
                        // first increment (deterministic choice, matches the
                        // reference model; QUESTION-P52-4).
                        c   = {4'b0, th, tl[4:0]} + 17'd1;
                        ov  = c[13];
                        thn = c[12:5];
                        tln = {3'b000, c[4:0]};
                    end
                    2'd1: begin                       // 16-bit
                        c   = {1'b0, th, tl} + 17'd1;
                        ov  = c[16];
                        thn = c[15:8];
                        tln = c[7:0];
                    end
                    2'd2: begin                       // 8-bit auto-reload
                        // Overflow LOADS TLx from THx and leaves THx alone
                        // (periph §4.4).  The reload rides the same S3P1 edge
                        // as the increment that overflowed - the manual gives
                        // no separate instant for it (QUESTION-P52-2), and
                        // one edge per machine cycle is the [D-07] rule.
                        c   = {9'b0, tl} + 17'd1;
                        ov  = c[8];
                        tln = ov ? th : c[7:0];
                    end
                    default: begin                    // mode 3: 8-bit halves
                        c   = {9'b0, tl} + 17'd1;
                        ov  = c[8];
                        tln = c[7:0];
                    end
                endcase
            end
        end
    endtask

    // Timer 0 in mode 3 splits into two 8-bit counters (periph §4.4): TL0
    // keeps the timer-0 control bits (C/T0, GATE0, TR0, INT0#) and TF0 - that
    // is the `d0`/`step_pair(mode0,...)` path above - while TH0 is LOCKED into
    // a timer function (machine cycles only, no C/T, no GATE) and takes over
    // TR1 and TF1.
    wire        th0_split_en = t1_free && tr1;
    wire [8:0]  th0_split_c  = {1'b0, th0_r} + 9'd1;

    // Timer 1 in mode 3 "simply holds its count - the effect is the same as
    // setting TR1 = 0" (periph §4.4), whatever timer 0 is doing.
    wire t1_step = d1 && (mode1 != 2'd3);

    always_comb begin
        step_pair(mode0, d0, tl0_r, th0_r, tl0_n, th0_n, ov0);
        step_pair(mode1, t1_step, tl1_r, th1_r, tl1_n, th1_n, ov1);
    end

    // TF write terms resolved at the S3P1 apply edge into the staging flops
    // (transferred to TCON at S5P2 of the same cycle, periph §4.5).  In mode 3
    // the TL0 half is what sets TF0 - the same `d0 && ov0` term, since
    // step_pair's mode-3 arm counts TL0 as an 8-bit register.
    wire tf0_set = d0 && ov0;
    // A mode-3 free-running timer 1 must NOT set TF1 - the flag belongs to
    // TH0 then ("no interrupts will be generated by timer 1 while timer 0 is
    // using the TF1 flag", periph §4.4).
    wire tf1_set = t1_free ? (th0_split_en && th0_split_c[8])
                           : (t1_step && ov1);

    // Timer 1's own carry-out, exported to the UART (the baud clock).  It
    // rides `tk_s3p1` like the increment that produced it, and it is alive
    // in the t1_free (timer-0 mode 3) case where TF1 is not.
    assign t1_ovf = tk_s3p1 && t1_step && ov1;

    // ---- timer 2 (periph §4.6, Figures 11/12/13/16) ------------------------
    wire rclk2  = t2con_r[T2CON_RCLK];
    wire tclk2  = t2con_r[T2CON_TCLK];
    wire exen2  = t2con_r[T2CON_EXEN2];
    wire tr2    = t2con_r[T2CON_TR2];
    wire ct2    = t2con_r[T2CON_CT2];
    wire cprl2  = t2con_r[T2CON_CPRL2];
    // Table 2 (3-12): RCLK+TCLK beats CP/RL2 - "when RCLK = 1 or TCLK = 1
    // this bit is ignored and the timer is forced to auto-reload".
    wire baud2  = rclk2 || tclk2;
    wire cap2   = cprl2 && !baud2;

    // The OSC/2 state-time clock of the baud-rate mode (Figure 16's note).
    // `t2_osc2` is the divider flop itself: it toggles on every oscillator
    // period, so the tick lands on the edge ending every P2 phase - six per
    // machine cycle, one per STATE, which is what "incremented every state
    // time" means.  Suppressed in PRIME like every other count ([D-10]); the
    // toggle is not, so the divider keeps its phase across the boundary (a
    // machine cycle is an even number of oscillator periods).
    wire osc2_tk = P_8052 && t2_osc2 && !prime_cycle;

    // The count input, Figure 12/13/16's CONTROL gate: TR2 AND (C/T2 ? a T2
    // pin transition : the internal clock).  PQ8: TR2 - the reference model
    // gates the pin path on TCON.TR1, which is a plain bug.
    wire t2_clk = ct2 ? (tk_s3p1 && t2_pend)
                      : (baud2 ? osc2_tk : tk_s3p1);
    wire t2_run = P_8052 && tr2 && t2_clk;

    wire [16:0] t2_nx = {1'b0, th2_r, tl2_r} + 17'd1;
    wire        ov2   = t2_nx[16];

    // The UART's baud clock (periph §5.8): the rollover itself, whatever edge
    // produced it.  TF2 is a different question - it is suppressed in exactly
    // this mode - so the two are separate terms.
    assign t2_ovf = t2_run && ov2;

    // The T2EX event.  Figures 12/13/16 gate it with EXEN2 alone: TR2 is in
    // the counter's clock path, not in the transition detector's, so a
    // capture/reload arrives even with the timer stopped (QUESTION-P55-2).
    wire t2ex_ev = P_8052 && t2ex_pend && exen2;

    wire hit_tcon  = (sfr_addr == SFR_TCON);
    wire hit_tmod  = (sfr_addr == SFR_TMOD);
    wire hit_tl0   = (sfr_addr == SFR_TL0);
    wire hit_tl1   = (sfr_addr == SFR_TL1);
    wire hit_th0   = (sfr_addr == SFR_TH0);
    wire hit_th1   = (sfr_addr == SFR_TH1);
    wire hit_t2con = P_8052 && (sfr_addr == SFR_T2CON);
    wire hit_rc2l  = P_8052 && (sfr_addr == SFR_RCAP2L);
    wire hit_rc2h  = P_8052 && (sfr_addr == SFR_RCAP2H);
    wire hit_tl2   = P_8052 && (sfr_addr == SFR_TL2);
    wire hit_th2   = P_8052 && (sfr_addr == SFR_TH2);

    // ---- save-state read mux (registered, savestate_design §5.1) ----------
    always_ff @(posedge CLK) begin
        case (ss_addr)
            SSA_T_TCON:      ss_rdata <= {8'b0, tcon_r};
            SSA_T_TMOD:      ss_rdata <= {8'b0, tmod_r};
            SSA_T_TL0:       ss_rdata <= {8'b0, tl0_r};
            SSA_T_TH0:       ss_rdata <= {8'b0, th0_r};
            SSA_T_TL1:       ss_rdata <= {8'b0, tl1_r};
            SSA_T_TH1:       ss_rdata <= {8'b0, th1_r};
            SSA_T_T0_SMPL:   ss_rdata <= {15'b0, t0_smpl};
            SSA_T_T0_PEND:   ss_rdata <= {15'b0, t0_pend};
            SSA_T_T1_SMPL:   ss_rdata <= {15'b0, t1_smpl};
            SSA_T_T1_PEND:   ss_rdata <= {15'b0, t1_pend};
            SSA_T_TF0_STAGE: ss_rdata <= {15'b0, tf0_stage};
            SSA_T_TF1_STAGE: ss_rdata <= {15'b0, tf1_stage};
            // ---- P_8052 group: read 0 / write ignored when off ----
            SSA_T_T2CON:     ss_rdata <= P_8052 ? {8'b0, t2con_r}   : 16'h0000;
            SSA_T_RCAP2L:    ss_rdata <= P_8052 ? {8'b0, rcap2l_r}  : 16'h0000;
            SSA_T_RCAP2H:    ss_rdata <= P_8052 ? {8'b0, rcap2h_r}  : 16'h0000;
            SSA_T_TL2:       ss_rdata <= P_8052 ? {8'b0, tl2_r}     : 16'h0000;
            SSA_T_TH2:       ss_rdata <= P_8052 ? {8'b0, th2_r}     : 16'h0000;
            SSA_T_T2_SMPL:   ss_rdata <= P_8052 ? {15'b0, t2_smpl}  : 16'h0000;
            SSA_T_T2_PEND:   ss_rdata <= P_8052 ? {15'b0, t2_pend}  : 16'h0000;
            SSA_T_T2EX_SMPL: ss_rdata <= P_8052 ? {15'b0, t2ex_smpl}: 16'h0000;
            SSA_T_T2EX_PEND: ss_rdata <= P_8052 ? {15'b0, t2ex_pend}: 16'h0000;
            SSA_T_T2_OSC2:   ss_rdata <= P_8052 ? {15'b0, t2_osc2}  : 16'h0000;
            SSA_T_TF2_STAGE: ss_rdata <= P_8052 ? {15'b0, tf2_stage}: 16'h0000;
            default:         ss_rdata <= 16'h0000;
        endcase
    end

    always_ff @(posedge CLK) begin
        // ---- save-state write decode (§5.2), restore-priority position ----
        if (ss_we) begin
            case (ss_addr)
                SSA_T_TCON:      tcon_r    <= ss_wdata[7:0];
                SSA_T_TMOD:      tmod_r    <= ss_wdata[7:0];
                SSA_T_TL0:       tl0_r     <= ss_wdata[7:0];
                SSA_T_TH0:       th0_r     <= ss_wdata[7:0];
                SSA_T_TL1:       tl1_r     <= ss_wdata[7:0];
                SSA_T_TH1:       th1_r     <= ss_wdata[7:0];
                SSA_T_T0_SMPL:   t0_smpl   <= ss_wdata[0];
                SSA_T_T0_PEND:   t0_pend   <= ss_wdata[0];
                SSA_T_T1_SMPL:   t1_smpl   <= ss_wdata[0];
                SSA_T_T1_PEND:   t1_pend   <= ss_wdata[0];
                SSA_T_TF0_STAGE: tf0_stage <= ss_wdata[0];
                SSA_T_TF1_STAGE: tf1_stage <= ss_wdata[0];
                SSA_T_T2CON:     if (P_8052) t2con_r   <= ss_wdata[7:0];
                SSA_T_RCAP2L:    if (P_8052) rcap2l_r  <= ss_wdata[7:0];
                SSA_T_RCAP2H:    if (P_8052) rcap2h_r  <= ss_wdata[7:0];
                SSA_T_TL2:       if (P_8052) tl2_r     <= ss_wdata[7:0];
                SSA_T_TH2:       if (P_8052) th2_r     <= ss_wdata[7:0];
                SSA_T_T2_SMPL:   if (P_8052) t2_smpl   <= ss_wdata[0];
                SSA_T_T2_PEND:   if (P_8052) t2_pend   <= ss_wdata[0];
                SSA_T_T2EX_SMPL: if (P_8052) t2ex_smpl <= ss_wdata[0];
                SSA_T_T2EX_PEND: if (P_8052) t2ex_pend <= ss_wdata[0];
                SSA_T_T2_OSC2:   if (P_8052) t2_osc2   <= ss_wdata[0];
                SSA_T_TF2_STAGE: if (P_8052) tf2_stage <= ss_wdata[0];
                default: ;
            endcase
        end else if (CE) begin
            if (rst_hold) begin
                tcon_r <= 8'h00; tmod_r <= 8'h00;
                tl0_r  <= 8'h00; th0_r  <= 8'h00;
                tl1_r  <= 8'h00; th1_r  <= 8'h00;
                t2con_r <= 8'h00; rcap2l_r <= 8'h00; rcap2h_r <= 8'h00;
                tl2_r   <= 8'h00; th2_r    <= 8'h00;
                t0_smpl <= T0;   t1_smpl <= T1;
                t0_pend <= 1'b0; t1_pend <= 1'b0;
                tf0_stage <= 1'b0; tf1_stage <= 1'b0;
                t2_smpl <= T2;   t2ex_smpl <= T2EX;
                t2_pend <= 1'b0; t2ex_pend <= 1'b0;
                t2_osc2 <= 1'b0; tf2_stage <= 1'b0;
            end else begin
                // ---- S3P1: apply counts (PQ3 [D-07]) ----
                if (tk_s3p1) begin
                    if (d0) begin
                        tl0_r <= tl0_n;
                        // mode 2: THx is the reload constant, never counted;
                        // mode 3: TH0 has its own path below
                        if ((mode0 != 2'd2) && !t1_free) th0_r <= th0_n;
                    end
                    if (th0_split_en) th0_r <= th0_split_c[7:0];
                    if (t1_step) begin
                        tl1_r <= tl1_n;
                        if (mode1 != 2'd2) th1_r <= th1_n;
                    end
                    // The detector output is a one-cycle PULSE at the counter
                    // clock input, gated by run/GATE on its way in (periph
                    // §4.2 Figure 6): consumed here whether or not it counted,
                    // so an edge seen while TRx=0 or the gate is shut is lost
                    // rather than banked (QUESTION-P52-3).
                    t0_pend <= 1'b0;
                    t1_pend <= 1'b0;
                    t2_pend <= 1'b0;
                    if (tf0_set) tf0_stage <= 1'b1;
                    if (tf1_set) tf1_stage <= 1'b1;
                end
                // ---- timer 2: the count, then the T2EX transfer ----
                // Ordered AFTER the timer-0/1 block and before the T2EX one
                // so a reload triggered by T2EX overrides the increment of
                // the same edge - Figure 13's RELOAD line drives the transfer
                // gates directly, and "load" is PQ10's whole point.
                if (t2_run) begin
                    if (ov2) begin
                        if (cap2) begin
                            // capture mode just wraps (Figure 12 has no
                            // reload path at all)
                            tl2_r <= t2_nx[7:0];
                            th2_r <= t2_nx[15:8];
                        end else begin
                            tl2_r <= rcap2l_r;
                            th2_r <= rcap2h_r;
                        end
                        // "TF2 will not be set when either RCLK = 1 or
                        // TCLK = 1" (Figure 11) - and in that mode a rollover
                        // "does not set TF2 and will not generate an
                        // interrupt" (3-17), so the staging flop stays clear.
                        if (!baud2) tf2_stage <= 1'b1;
                    end else begin
                        tl2_r <= t2_nx[7:0];
                        th2_r <= t2_nx[15:8];
                    end
                end
                // The T2EX half that moves registers rides the count-apply
                // edge; its flag half is at S5P2 below (periph §6.3/3-25).
                if (tk_s3p1 && t2ex_ev) begin
                    if (cap2) begin
                        // "a 1-to-0 transition at T2EX causes the current
                        // value in TL2/TH2 to be captured into RCAP2L/RCAP2H"
                        // - the count itself is untouched, so the increment
                        // above stands and the captured value is the one the
                        // register showed during this cycle.
                        rcap2l_r <= tl2_r;
                        rcap2h_r <= th2_r;
                    end else if (!baud2) begin
                        // auto-reload: LOAD, never add (PQ10)
                        tl2_r <= rcap2l_r;
                        th2_r <= rcap2h_r;
                    end
                    // baud-rate mode: "a 1-to-0 transition at T2EX sets EXF2
                    // but does not cause a reload" - nothing to do here.
                end
                // ---- S5P2: pin sampling + flag transfer (periph §4.1/§4.5) ----
                // The samples run in PRIME too, so the sample registers hold
                // the injection boundary's pin levels, but edge DETECTION is
                // suppressed there - no phantom edge (core_design §2.8/§6.1,
                // [D-10]).
                if (tk_s5p2) begin
                    // C5.5: the T2EX event's FLAG half.  "The Timer 2 flag
                    // EXF2 and the Serial Port flags RI and TI are set at
                    // S5P2.  The values are not actually polled by the
                    // circuitry until the next machine cycle" (3-25) - so
                    // EXF2 is an ordinary S5P2 source, unlike TF2.  The pulse
                    // is consumed here whether or not EXEN2 let it do
                    // anything (PQ9 + the QUESTION-P52-3 rule), and this
                    // clear is written BEFORE the detector below so a fresh
                    // transition on the same edge re-arms it.
                    if (P_8052 && t2ex_pend) begin
                        if (exen2) t2con_r[T2CON_EXF2] <= 1'b1;
                        t2ex_pend <= 1'b0;
                    end
                    if (!prime_cycle) begin
                        if (t0_smpl && !T0) t0_pend <= 1'b1;
                        if (t1_smpl && !T1) t1_pend <= 1'b1;
                        if (P_8052) begin
                            if (t2_smpl   && !T2)   t2_pend   <= 1'b1;
                            if (t2ex_smpl && !T2EX) t2ex_pend <= 1'b1;
                        end
                    end
                    t0_smpl <= T0;
                    t1_smpl <= T1;
                    t2_smpl   <= T2;
                    t2ex_smpl <= T2EX;
                    if (tf0_stage) begin
                        tcon_r[TCON_TF0] <= 1'b1;
                        tf0_stage        <= 1'b0;
                    end
                    if (tf1_stage) begin
                        tcon_r[TCON_TF1] <= 1'b1;
                        tf1_stage        <= 1'b0;
                    end
                end
                // ---- S2P2: TF2's set instant (TQ8) ----
                // "The Timer 0 and Timer 1 flags, TF0 and TF1, are set at
                // S5P2 of the cycle in which the timers overflow... However,
                // the Timer 2 flag TF2 is set at S2P2 and is polled in the
                // same cycle in which the timer overflows" (3-25).  S2P2
                // precedes the S3P1 apply that produces the overflow, so the
                // flag rides the staging flop into the NEXT cycle's S2P2 -
                // and nu8051_irq polls THAT flag live, in the cycle it is
                // set, instead of out of its S6P2 snapshot.  QUESTION-P55-1
                // carries the one reading the manual leaves open.
                // The OSC/2 divider toggles on every oscillator period so the
                // baud-mode tick keeps its phase; it is not an architectural
                // event and runs in PRIME as well (the count it clocks does
                // not - `osc2_tk`).
                if (P_8052) t2_osc2 <= !t2_osc2;
                if (tk_s2p2 && tf2_stage) begin
                    t2con_r[T2CON_TF2] <= 1'b1;
                    tf2_stage          <= 1'b0;
                end
                // ---- C5.3: the interrupt unit's TCON maintenance ----
                // Two different phases: `irq_iex_*` fires at S5P2 (the
                // level/edge latch, periph §6.5) and `irq_tfx_clr` at the
                // S6P2 of ILCALL1 (the §6.4 vectoring clear).  Placed BEFORE
                // the architectural write block so a software write to TCON
                // in the same cycle wins, which is the general TQ6 rule the
                // S5P2-vs-S6P2 ordering already gives the TF transfer above.
                if (irq_iex_we[0]) tcon_r[TCON_IE0] <= irq_iex_d[0];
                if (irq_iex_we[1]) tcon_r[TCON_IE1] <= irq_iex_d[1];
                if (irq_tfx_clr[0]) tcon_r[TCON_TF0] <= 1'b0;
                if (irq_tfx_clr[1]) tcon_r[TCON_TF1] <= 1'b0;
                // ---- S6P2: architectural SFR writes (TQ6) ----
                // S6P2 is after S3P1 and S5P2 of the same cycle, so a software
                // write always beats that cycle's increment and that cycle's
                // TF transfer: `MOV TL0,#d` while running leaves exactly d,
                // and `CLR TF0` in the overflow cycle wins.  Nothing here
                // clears TLx/THx on a mode or TRx change - "setting the run
                // flag does not clear the registers" (periph §4.4), and the
                // manual says nothing else about changing TMOD/TCON live
                // (QUESTION-P52-5).
                if (sfr_we) begin
                    if (hit_tcon)  tcon_r   <= sfr_wdata;
                    if (hit_tmod)  tmod_r   <= sfr_wdata;
                    if (hit_tl0)   tl0_r    <= sfr_wdata;
                    if (hit_tl1)   tl1_r    <= sfr_wdata;
                    if (hit_th0)   th0_r    <= sfr_wdata;
                    if (hit_th1)   th1_r    <= sfr_wdata;
                    if (hit_t2con) t2con_r  <= sfr_wdata;
                    if (hit_rc2l)  rcap2l_r <= sfr_wdata;
                    if (hit_rc2h)  rcap2h_r <= sfr_wdata;
                    if (hit_tl2)   tl2_r    <= sfr_wdata;
                    if (hit_th2)   th2_r    <= sfr_wdata;
                end
            end
        end
`ifdef NU8051_BACKDOOR
        else if (sfr_bkd_we) begin
            if (hit_tcon)  tcon_r   <= sfr_wdata;
            if (hit_tmod)  tmod_r   <= sfr_wdata;
            if (hit_tl0)   tl0_r    <= sfr_wdata;
            if (hit_tl1)   tl1_r    <= sfr_wdata;
            if (hit_th0)   th0_r    <= sfr_wdata;
            if (hit_th1)   th1_r    <= sfr_wdata;
            if (hit_t2con) t2con_r  <= sfr_wdata;
            if (hit_rc2l)  rcap2l_r <= sfr_wdata;
            if (hit_rc2h)  rcap2h_r <= sfr_wdata;
            if (hit_tl2)   tl2_r    <= sfr_wdata;
            if (hit_th2)   th2_r    <= sfr_wdata;
        end
`endif
    end

    initial begin
        tcon_r = 8'h00; tmod_r = 8'h00; tl0_r = 8'h00; th0_r = 8'h00;
        tl1_r = 8'h00; th1_r = 8'h00;
        t2con_r = 8'h00; rcap2l_r = 8'h00; rcap2h_r = 8'h00;
        tl2_r = 8'h00; th2_r = 8'h00;
        t0_smpl = 1'b1; t1_smpl = 1'b1; t0_pend = 1'b0; t1_pend = 1'b0;
        tf0_stage = 1'b0; tf1_stage = 1'b0;
        t2_smpl = 1'b1; t2ex_smpl = 1'b1; t2_pend = 1'b0; t2ex_pend = 1'b0;
        t2_osc2 = 1'b0; tf2_stage = 1'b0;
        ss_rdata = 16'h0000;
    end

endmodule
