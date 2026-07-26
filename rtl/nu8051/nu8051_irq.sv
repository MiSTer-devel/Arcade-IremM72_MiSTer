//============================================================================
// Imported from nu8051 (machine-cycle-accurate Intel 8051/8052 core, SystemVerilog)
// Source: rtl/nu8051_irq.sv, commit e2c97de0fabb3b3baa1eb98a15284bc74ef92f4e
// Do not hand-edit; re-import from upstream. The M72 integration wrapper is
// rtl/mcu.sv (distilled from nu8051 rtl/m72/mcu.sv), NOT a raw re-import.
//============================================================================
//============================================================================
//
//  nu8051_irq - interrupt unit (core_design.md §5.4 row 5, §6.3)
//
//  COMPLETE HERE (work package C5.3, timing §6, periph §6): the S5P2 request
//  snapshot over all five 8051 sources, the two-level priority resolve, the
//  two in-progress flip-flops, the INT0#/INT1# level-vs-edge rules and the
//  §6.4 flag-clearing table.  C5.5 added the 8052's sixth source (TF2 + EXF2
//  -> 002BH, ET2 = IE.5, PT2 = IP.5, last in the polling order, neither flag
//  hardware-cleared) - with TF2's same-cycle poll as a live bypass, since
//  3-25 sets that one flag at S2P2 instead of S5P2 (TQ8).  The injected LCALL itself (shape I, §2.3) and
//  the three blocking conditions live in the sequencer - this module owns
//  WHICH source wins and WHETHER the level admits it; the seq owns WHEN.
//
//  ------------------------------------------------------------------
//  The cadence, derived once (timing §6, periph §6.3):
//
//    S5P2 of cycle N     the flags are SAMPLED - INT0#/INT1# are inverted
//                        and latched into TCON.IE0/IE1 here, and the timer
//                        transfers TF0/TF1 out of its staging flops here
//                        (periph §4.5).  Nothing that feeds a request can
//                        move again before the end of the cycle.
//    S6P2 of cycle N     the enabled request set is RESOLVED out of that
//                        settled state and latched into {irq_req, irq_src,
//                        irq_prio} (D1.2 F-3, map 0x0E4-0x0E6)
//    cycle N+1           the seq polls the snapshot and applies the take at
//                        THAT cycle's S6P2 edge
//
//  WHY THE SNAPSHOT LATCHES AT S6P2 AND NOT AT S5P2 [QUESTION-P53-2].  The
//  seq applies its take decision at an S6P2 edge (§2.2's commit row - there
//  is no other instant an instruction boundary exists at).  A snapshot flop
//  clocked at S5P2 of cycle N is therefore ALREADY VISIBLE at S6P2 of cycle
//  N, which collapses Figure 24's C1 and C2 into one machine cycle and makes
//  the minimum response 2 cycles instead of 3.  Latching the resolved
//  request at the END of the sampling cycle restores "the samples are polled
//  during the FOLLOWING machine cycle" with the one mapped flop the D1.2
//  inventory allows - and it is exactly equivalent to an S5P2 latch,
//  because every term is a REGISTER that S5P2 has already settled:
//  TCON.IE0/IE1 (written at S5P2 from the pin), TCON.TF0/TF1 (transferred at
//  S5P2), SCON.RI/TI, IE and IP (all written at S6P2, so the snapshot sees
//  their pre-edge values, which is what an S5P2 read would have returned).
//  No raw pin appears in the request expression, so nothing sub-cycle can
//  leak past the S5P2 sampling instant.
//
//  Blocking rule 1 (an interrupt of equal or higher priority already in
//  progress) is applied at PRESENTATION, combinationally against the live
//  in-progress flip-flops - not baked into the latched snapshot.  Two
//  manual sentences force that:
//
//    * "every polling cycle is new" (timing §6): the blocking conditions are
//      re-evaluated by the polling cycle, only the FLAG sample is a cycle old.
//    * Figure 24's nested window: a high-priority request latched at S5P2 of
//      the first injected-LCALL cycle C3 "will be vectored to during C5 and
//      C6" - at C3's S5P2 the low level's in-progress FF is not yet set (it
//      commits at S6P2 of C3, [D-08]), so a snapshot-time test would let a
//      SECOND low-priority request through in C4.  The presentation-time test
//      sees the FF set and denies it, while still admitting the high one.
//
//  The resolve order is (priority level, then the fixed polling order
//  IE0 -> TF0 -> IE1 -> TF1 -> RI+TI -> TF2+EXF2), and testing the winner
//  against the in-progress FFs is equivalent to testing every request:
//  priority is the first sort key, so nothing that loses to the winner can
//  outrank it (periph §6.2).
//
//  Flag clearing on vectoring (periph §6.4) is driven from the seq's
//  `irq_ack` pulse at the S6P2 edge of ILCALL1 [D-08]:
//
//    | source                    | cleared by hardware |
//    | IE0/IE1, edge   (ITx = 1) | yes                 |
//    | IE0/IE1, level  (ITx = 0) | NO - the flag tracks the pin            |
//    | TF0/TF1                   | yes, always         |
//    | RI/TI (and TF2/EXF2)      | NEVER - the ISR must clear them         |
//
//  The four TCON bits involved (IE0/IE1/IT0/IT1) and TF0/TF1 are TCON
//  storage, which lives in nu8051_timer (D1.2 §2.6, map 0x080) - one
//  register, one symbol.  So the RULES live here and the WRITES are handed
//  to the timer over `irq_iex_we/_d` (the S5P2 maintenance) and
//  `irq_tfx_clr` (the vectoring clear).
//
//  Reset (§7.2): IE 00H, IP 00H, both in-progress FFs clear, snapshot empty.
//
//============================================================================

`timescale 1ns/1ps

module nu8051_irq #(
    parameter bit P_8052 = 1'b1
)(
    input  logic        CLK,
    input  logic        CE,
    input  logic        rst_hold,

    input  logic        tk_s5p2,
    input  logic        tk_s6p2,
    input  logic        prime_cycle,     // §2.8: sample pins, arm no edge

    input  logic  [7:0] sfr_addr,
    output logic  [7:0] sfr_rdata,
    output logic        sfr_hit,
    input  logic        sfr_we,
    input  logic  [7:0] sfr_wdata,

    input  logic        INT0_N, INT1_N,
    output logic        int0_pin, int1_pin,   // S5P2 pin-level samples

    // source flags owned by other modules
    input  logic  [7:0] tcon_q,          // TF1 TR1 TF0 TR0 IE1 IT1 IE0 IT0
    input  logic        ri_q, ti_q,      // SCON.RI / SCON.TI
    input  logic  [7:0] t2con_q,         // C5.5: TF2/EXF2 (00H when !P_8052)

    // TCON maintenance handed back to the timer (the register lives there)
    output logic  [1:0] irq_iex_we,      // {IE1, IE0} write enable
    output logic  [1:0] irq_iex_d,       // {IE1, IE0} new value
    output logic  [1:0] irq_tfx_clr,     // {TF1, TF0} vectoring clear

    // C4.7: RETI's re-arm pulse from the sequencer, asserted at the RETI
    // instruction's S6P2 commit edge (core_design §6.3 "RETI clears the
    // current-level in-progress FF at its S6P2 commit; RET does not",
    // periph §6.6).  The level cleared is the CURRENT one - high if a
    // high-priority routine is in progress, otherwise low (periph §6.2:
    // MAME's `clear_current_irq`, and the manual's "the interrupt in
    // progress is done").
    input  logic        reti_clr,

    // C5.3: the sequencer's vectoring acknowledge - one pulse at the S6P2
    // edge of ILCALL1 [D-08], carrying the source/level it latched at the
    // take decision (map 0x012/0x013).
    input  logic        irq_ack,
    input  logic  [2:0] irq_ack_src,
    input  logic        irq_ack_prio,

    output logic        irq_req,
    output logic  [2:0] irq_src,
    output logic        irq_prio,
    output logic  [1:0] irq_ipl,         // {ipl_hi, ipl_lo} observation tap

    // save state (savestate_design §5.1/§5.2, map 0x0E0-0x0E8)
    input  logic  [9:0] ss_addr,
    input  logic [15:0] ss_wdata,
    input  logic        ss_we,
    output logic [15:0] ss_rdata

`ifdef NU8051_BACKDOOR
    ,
    input  logic        sfr_bkd_we,
    input  logic        bkd_load         // §2.8 boundary force
`endif
);

`include "nu8051_defs.svh"
    import nu8051_ss_pkg::*;

    // polling order (periph §6.1 table); also the SSA_I_IRQ_SRC encoding
    localparam logic [2:0] SRC_IE0 = 3'd0;
    localparam logic [2:0] SRC_TF0 = 3'd1;
    localparam logic [2:0] SRC_IE1 = 3'd2;
    localparam logic [2:0] SRC_TF1 = 3'd3;
    localparam logic [2:0] SRC_SER = 3'd4;
    localparam logic [2:0] SRC_T2  = 3'd5;

    logic [7:0] ie_r, ip_r;
    logic       int0_s, int1_s;
    logic       ipl_lo, ipl_hi;
    logic       req_r, prio_r;
    logic [2:0] src_r;

    assign int0_pin = int0_s;
    assign int1_pin = int1_s;
    assign irq_ipl  = {ipl_hi, ipl_lo};

    // ---- external-interrupt pin rules (periph §6.5) ------------------------
    // ITx = 0 (level)      : IEx simply follows the inverted pin, latched at
    //                        S5P2 of every machine cycle.  The external source
    //                        owns the flag; hardware never clears it.
    // ITx = 1 (transition) : a sample HIGH in one cycle followed by a sample
    //                        LOW in the next sets IEx, which then holds until
    //                        vectoring (or software) clears it.
    // Edge DETECTION is suppressed in the PRIME cycle (core_design §2.8 /
    // §6.1 [D-10]) so an injected boundary cannot manufacture a fall; the
    // level path is not an edge and needs no such guard - it just restates
    // the pin, which is what an S5P2 in any real preceding cycle would have
    // left in the flag.
    wire it0 = tcon_q[TCON_IT0];
    wire it1 = tcon_q[TCON_IT1];

    wire int0_low  = ~INT0_N;
    wire int1_low  = ~INT1_N;
    wire int0_fall = int0_s && int0_low && !prime_cycle;
    wire int1_fall = int1_s && int1_low && !prime_cycle;

    wire ie0_hw_we = it0 ? int0_fall : 1'b1;
    wire ie1_hw_we = it1 ? int1_fall : 1'b1;
    wire ie0_hw_d  = it0 ? 1'b1 : int0_low;
    wire ie1_hw_d  = it1 ? 1'b1 : int1_low;

    // ---- the vectoring clears (periph §6.4) --------------------------------
    wire ack_clr_ie0 = irq_ack && (irq_ack_src == SRC_IE0) && it0;
    wire ack_clr_ie1 = irq_ack && (irq_ack_src == SRC_IE1) && it1;

    // The S5P2 maintenance and the S6P2 vectoring clear are different phases,
    // so the two write sources cannot collide; the clear writes 0 by taking
    // the `_d` term to 0 when the S5P2 arm is idle.
    assign irq_iex_we = {(tk_s5p2 && ie1_hw_we) | ack_clr_ie1,
                         (tk_s5p2 && ie0_hw_we) | ack_clr_ie0};
    assign irq_iex_d  = {(tk_s5p2 && ie1_hw_we) & ie1_hw_d,
                         (tk_s5p2 && ie0_hw_we) & ie0_hw_d};
    assign irq_tfx_clr = {irq_ack && (irq_ack_src == SRC_TF1),
                          irq_ack && (irq_ack_src == SRC_TF0)};

    // ---- the request snapshot (timing §6; latch edge per the header) -------
    // Every term is a settled REGISTER - TCON's four flag bits were written
    // by this cycle's S5P2 (the IEx pin latch here, the TFx staging transfer
    // in the timer), SCON/IE/IP change only at S6P2 and so present their
    // pre-edge values.  Source order = the polling order, so "lowest set
    // index wins" IS the manual's within-level tie-break (periph §6.2).
    // Bit 5 (TF2 + EXF2) is the 8052's sixth source (C5.5).  EXF2 belongs in
    // this latched set exactly like the other five: 3-25 puts it on the
    // uniform sample - "the Timer 2 flag EXF2 and the Serial Port flags RI
    // and TI are set at S5P2.  The values are not actually polled by the
    // circuitry until the next machine cycle".  TF2 is the exception and is
    // handled by the live bypass below; carrying it here as well costs
    // nothing (the flag is not hardware-cleared, so a TF2 that survives into
    // the next cycle is a legitimate request of this cycle too) and keeps the
    // ORed source honest when only one of the two flags is set.
    logic [5:0] pend;
    always_comb begin
        pend           = 6'b0;
        pend[SRC_IE0]  = tcon_q[TCON_IE0];
        pend[SRC_TF0]  = tcon_q[TCON_TF0];
        pend[SRC_IE1]  = tcon_q[TCON_IE1];
        pend[SRC_TF1]  = tcon_q[TCON_TF1];
        pend[SRC_SER]  = ri_q | ti_q;
        pend[SRC_T2]   = P_8052 && (t2con_q[T2CON_TF2] | t2con_q[T2CON_EXF2]);
    end
    // EA (IE.7) is the global enable; IE[5:0] the individual masks
    wire [5:0] enb  = pend & ie_r[5:0] & {6{ie_r[7]}};
    wire [5:0] hi   = enb & ip_r[5:0];
    wire       any_hi = |hi;
    wire [5:0] sel  = any_hi ? hi : enb;

    logic [2:0] win;
    always_comb begin
        win = SRC_IE0;
        for (int i = 5; i >= 0; i--)
            if (sel[i]) win = 3'(i);
    end

    // ---- TF2's live bypass (C5.5 / TQ8) ------------------------------------
    // Every other flag is "sampled at S5P2 and polled during the FOLLOWING
    // machine cycle"; TF2 alone "is set at S2P2 and is polled in the same
    // cycle" (3-25).  S2P2 is before this module's S6P2 snapshot edge but
    // after the PREVIOUS one, so the same-cycle poll cannot come out of the
    // latched set at all - it is a second, combinational request presented
    // alongside it.  Enabling and priority use the LIVE IE/IP, which is
    // exactly equivalent to the snapshot's copy: both registers move only at
    // S6P2, so their value throughout the polling cycle IS the value the
    // previous S6P2 latched.
    //
    // Resolution against the latched winner is the polling order itself: the
    // 8052's sixth source is LAST within a level, so it wins only when the
    // latched set is empty or when it is the high-priority one and the
    // latched winner is not (periph §6.2).  Testing the chosen candidate
    // against the in-progress flip-flops is then still complete - the loser
    // is never of a level the winner's admission does not cover.
    wire t2_live      = P_8052 && t2con_q[T2CON_TF2]
                                && ie_r[SRC_T2] && ie_r[7];
    wire t2_live_prio = ip_r[SRC_T2];
    wire live_wins    = t2_live && (!req_r || (t2_live_prio && !prio_r));

    // ---- presentation: blocking rule 1, live (see the header) --------------
    wire       req_pre  = req_r || t2_live;
    wire [2:0] src_pre  = live_wins ? SRC_T2 : src_r;
    wire       prio_pre = live_wins ? t2_live_prio : prio_r;
    wire admit = prio_pre ? !ipl_hi : (!ipl_hi && !ipl_lo);
    assign irq_req  = req_pre && admit;
    assign irq_src  = src_pre;
    assign irq_prio = prio_pre;

    always_comb begin
        sfr_hit = 1'b1;
        case (sfr_addr)
            SFR_IE:  sfr_rdata = ie_r;
            SFR_IP:  sfr_rdata = ip_r;
            default: begin sfr_rdata = 8'h00; sfr_hit = 1'b0; end
        endcase
    end

    wire hit_ie = (sfr_addr == SFR_IE);
    wire hit_ip = (sfr_addr == SFR_IP);

    // ---- save-state read mux (registered, savestate_design §5.1) ----------
    always_ff @(posedge CLK) begin
        case (ss_addr)
            SSA_I_IE:         ss_rdata <= {8'b0, ie_r};
            SSA_I_IP:         ss_rdata <= {8'b0, ip_r};
            SSA_I_IPL_LO:     ss_rdata <= {15'b0, ipl_lo};
            SSA_I_IPL_HI:     ss_rdata <= {15'b0, ipl_hi};
            SSA_I_IRQ_REQ:    ss_rdata <= {15'b0, req_r};
            SSA_I_IRQ_SRC:    ss_rdata <= {13'b0, src_r};
            SSA_I_IRQ_PRIO:   ss_rdata <= {15'b0, prio_r};
            SSA_I_INT0_SMPL:  ss_rdata <= {15'b0, int0_s};
            SSA_I_INT1_SMPL:  ss_rdata <= {15'b0, int1_s};
            default:          ss_rdata <= 16'h0000;
        endcase
    end

    always_ff @(posedge CLK) begin
        // ---- save-state write decode (§5.2), restore-priority position ----
        if (ss_we) begin
            case (ss_addr)
                SSA_I_IE:        ie_r   <= ss_wdata[7:0];
                SSA_I_IP:        ip_r   <= ss_wdata[7:0];
                SSA_I_IPL_LO:    ipl_lo <= ss_wdata[0];
                SSA_I_IPL_HI:    ipl_hi <= ss_wdata[0];
                SSA_I_IRQ_REQ:   req_r  <= ss_wdata[0];
                SSA_I_IRQ_SRC:   src_r  <= ss_wdata[2:0];
                SSA_I_IRQ_PRIO:  prio_r <= ss_wdata[0];
                SSA_I_INT0_SMPL: int0_s <= ss_wdata[0];
                SSA_I_INT1_SMPL: int1_s <= ss_wdata[0];
                default: ;
            endcase
        end else if (CE) begin
            if (rst_hold) begin
                ie_r   <= 8'h00;
                ip_r   <= 8'h00;
                int0_s <= INT0_N;
                int1_s <= INT1_N;
                ipl_lo <= 1'b0;
                ipl_hi <= 1'b0;
                req_r  <= 1'b0;
                src_r  <= SRC_IE0;
                prio_r <= 1'b0;
            end else begin
                if (tk_s5p2) begin
                    int0_s <= INT0_N;
                    int1_s <= INT1_N;
                end
                // the resolved snapshot the NEXT machine cycle polls
                if (tk_s6p2) begin
                    req_r  <= |enb;
                    src_r  <= win;
                    prio_r <= any_hi;
                end
                // in-progress flip-flops.  The set and the clear are S6P2
                // edges of DIFFERENT cycles (ILCALL1 vs a RETI commit), so
                // they cannot contend.
                if (irq_ack) begin
                    if (irq_ack_prio) ipl_hi <= 1'b1;
                    else              ipl_lo <= 1'b1;
                end
                if (reti_clr) begin
                    // the CURRENT level - high wins (periph §6.6); a RETI
                    // with nothing in progress is functionally a RET
                    if (ipl_hi)      ipl_hi <= 1'b0;
                    else if (ipl_lo) ipl_lo <= 1'b0;
                end
                if (sfr_we) begin
                    if (hit_ie) ie_r <= sfr_wdata;
                    if (hit_ip) ip_r <= sfr_wdata;
                end
            end
        end
`ifdef NU8051_BACKDOOR
        else begin
            if (sfr_bkd_we) begin
                if (hit_ie) ie_r <= sfr_wdata;
                if (hit_ip) ip_r <= sfr_wdata;
            end
            // canonical instruction-boundary state (§2.8 / D1.2 §3.2): no
            // phantom poll, no interrupt in progress.  Ordered after the
            // storage write so the boundary force always wins if a TB ever
            // drives both in one parked tick.  The pin-sample registers are
            // NOT forced here - the PRIME cycle's S5P2 reloads them from the
            // vector's own levels with edge detection off, which is exactly
            // D1.2 §3.2's "default = current pin levels, no phantom edge".
            if (bkd_load) begin
                ipl_lo <= 1'b0;
                ipl_hi <= 1'b0;
                req_r  <= 1'b0;
                src_r  <= SRC_IE0;
                prio_r <= 1'b0;
            end
        end
`endif
    end

    initial begin
        ie_r = 8'h00; ip_r = 8'h00; int0_s = 1'b1; int1_s = 1'b1;
        ipl_lo = 1'b0; ipl_hi = 1'b0;
        req_r = 1'b0; src_r = SRC_IE0; prio_r = 1'b0;
        ss_rdata = 16'h0000;
    end

endmodule
