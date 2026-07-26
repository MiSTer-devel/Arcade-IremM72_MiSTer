//============================================================================
// Imported from nu8051 (machine-cycle-accurate Intel 8051/8052 core, SystemVerilog)
// Source: rtl/nu8051_core.sv, commit e2c97de0fabb3b3baa1eb98a15284bc74ef92f4e
// Do not hand-edit; re-import from upstream. The M72 integration wrapper is
// rtl/mcu.sv (distilled from nu8051 rtl/m72/mcu.sv), NOT a raw re-import.
//============================================================================
//============================================================================
//
//  nu8051_core - top level (core_design.md §1.2 port list, §10 item 8)
//
//  PHASE 3 (PLAN §4 P3).  Wiring, tick-strobe fan-out, SFR-bus aggregation
//  with the PQ5 unimplemented-read constant, the NU8051_BACKDOOR group
//  (§1.9) and the seven ratified sim-only observation signals.
//
//  Hierarchy (core_design §10 file plan):
//    nu8051_seq    sequencer: phase/cycle counters, fetch slots, micro-
//                  schedule, PC rules, reset sequencing, bus drive, §2.8
//                  boundary force
//    nu8051_iram   256B single-port sync RAM + backdoor byte port
//    nu8051_alu    §4 combinational op set (instantiated inside nu8051_seq,
//                  which owns every operand mux; MUL/DIV engine = C4.3)
//    nu8051_sfr    SP/DPL/DPH/PCON/PSW/ACC/B, live PSW.P parity
//    nu8051_ports  P0-P3 latches, S5P1 pin samples, alt folds, TQ4 clobber
//    nu8051_timer  timer SFRs + timer-0/1 engine (C5.2); TCON storage, whose
//                  four interrupt bits nu8051_irq maintains (C5.3)
//    nu8051_uart   SCON/SBUF storage (engines = C5.4)
//    nu8051_irq    IE/IP, the S5P2 request snapshot, priority resolve and the
//                  in-progress flip-flops (C5.3)
//
//  The port list and parameters are the FROZEN Gate-P1 surface, transcribed
//  from core_design §1.1/§1.2/§1.9 - unchanged from the T2.3 stub.
//
//  Not yet implemented (later phases): MOVX, the branch/call shapes and
//  MUL/DIV (Phase 4 chunks C4.2-C4.8); timers-2/UART/interrupt behaviour
//  (Phase 5); the SS save-state path (Phase 6) - SS_RDATA/SS_ERR still read 0.
//
//============================================================================

`timescale 1ns/1ps

module nu8051_core #(
    parameter bit P_8052 = 1'b1,
    parameter int ROM_AW = 13            // 12 = 4K, 13 = 8K
)(
    input  logic        CLK,
    input  logic        CE,              // one pulse per oscillator period
    input  logic        RESET,           // active high, min 24 CE ticks

    // de-muxed external bus (code fetch + MOVX)
    output logic [15:0] MEM_ADDR,
    output logic  [7:0] MEM_DOUT,
    input  logic  [7:0] MEM_DIN,
    output logic        MEM_ALE,
    output logic        MEM_PSEN_N,
    output logic        MEM_RD_N,
    output logic        MEM_WR_N,
    input  logic        EA_N,            // latched during RESET (TQ7)

    // internal program ROM (platform-supplied BRAM)
    output logic [ROM_AW-1:0] ROM_ADDR,
    input  logic  [7:0] ROM_DATA,

    // GPIO ports P0..P3
    input  logic  [7:0] P0_IN,  P1_IN,  P2_IN,  P3_IN,
    output logic  [7:0] P0_OUT, P1_OUT, P2_OUT, P3_OUT,
    output logic  [7:0] P0_OEN, P1_OEN, P2_OEN, P3_OEN,

    // dedicated alternate-function pins
    input  logic        INT0_N, INT1_N,  // level/edge per IT0/IT1
    input  logic        T0, T1,          // counter inputs
    input  logic        T2, T2EX,        // sampled only when P_8052
    input  logic        RXD_IN,
    output logic        RXD_OUT, RXD_OE, // mode-0 data out + its OE
    output logic        TXD,

    // save-state (v30 savestate_v2 idiom; map defined by D1.2)
    input  logic  [9:0] SS_ADDR,
    input  logic [15:0] SS_WDATA,
    input  logic        SS_WE,
    output logic [15:0] SS_RDATA,
    output logic        SS_ERR

`ifdef NU8051_BACKDOOR
    // verification-only state backdoor (Phases 3-5; superseded by the SS
    // bus from Phase 6; compiled out of synthesis builds) - §1.9
    ,
    input  logic        bkd_load,        // pulse while CE parked: force §2.8 boundary state
    input  logic [15:0] bkd_pc,
    input  logic  [8:0] bkd_addr,        // [8]=0 IRAM byte, [8]=1 SFR storage
    input  logic  [7:0] bkd_wdata,
    input  logic        bkd_we,
    output logic  [7:0] bkd_rdata
`endif
);

    import nu8051_ss_pkg::*;

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
    // internal buses
    // ------------------------------------------------------------------
    logic  [7:0] iram_addr, iram_rdata, iram_wdata;
    logic        iram_we;

    logic  [7:0] seq_sfr_addr, seq_sfr_wdata;
    logic        seq_sfr_rmw, seq_sfr_we;
    logic  [7:0] sfr_addr, sfr_wdata, sfr_rdata;
    logic        sfr_rmw, sfr_we;

    logic  [7:0] rd_sfr, rd_ports, rd_timer, rd_uart, rd_irq;
    logic        hit_sfr, hit_ports, hit_timer, hit_uart, hit_irq;

    logic  [7:0] acc_q, b_q, psw_q, sp_q;
    logic [15:0] dptr_q;
    logic  [7:0] acc_wdata, b_wdata, sp_wdata;
    logic [15:0] dptr_wdata;
    logic        acc_we, b_we, sp_we, dptr_we;
    logic        flag_we_c, flag_c, flag_we_ac, flag_ac, flag_we_ov, flag_ov;

    logic  [3:0] ph_w;
    logic        tk_s2p2;
    logic        tk_s3p1, tk_s5p1, tk_s5p2, tk_s6p1, tk_s6p2;
    logic        bus_active, rst_hold, prime_cycle;
    logic  [1:0] mcyc_w;
    logic  [2:0] seqstate_w;
    logic [15:0] pc_w;
    logic        retire_w;
    logic [31:0] hash_w;

    logic        int0_pin, int1_pin;
    logic        irq_req;
    // C4.7: the sequencer's RETI re-arm pulse (§6.3), consumed by the irq
    // unit's in-progress flip-flops since C5.3.
    logic        irq_reti;
    logic  [2:0] irq_src;
    logic        irq_prio;
    // C5.3: the vectoring acknowledge (S6P2 of ILCALL1, [D-08]) and the two
    // TCON-maintenance channels between the irq unit (the rules) and the
    // timer (the register).
    logic        irq_ack, irq_ack_prio;
    logic  [2:0] irq_ack_src;
    logic  [1:0] irq_iex_we, irq_iex_d, irq_tfx_clr, irq_ipl;
    logic  [7:0] tcon_q;
    // C5.5: T2CON crosses the timer boundary twice - to the irq unit as the
    // sixth source's two flags, to the UART as the two baud-source selects.
    logic  [7:0] t2con_q;
    logic        ri_q, ti_q, txd_r;
    logic        pcon_smod, t1_ovf, t2_ovf;
    logic  [7:0] p2_latch_q;
    // C5.6: PCON power control (periph §7, core_design §6.5)
    logic        pcon_idl, pcon_pd, pcon_idl_clr, pd_freeze, cpu_idle;

    // ------------------------------------------------------------------
    // C5.6: the power-down clock gate.
    //
    // "The oscillator is stopped ... all functions are stopped" (periph §7).
    // This core has no oscillator to stop - it has CE - so power down is
    // modelled as the CE gate every peripheral sees, and the sequencer freezes
    // itself on the same condition.  Gating CE rather than auditing each
    // module's strobes is what makes "all functions" true by construction:
    // the UART's ÷16 chains, timer 2's OSC/2 divider and the port latches have
    // no clock at all while it is asserted, so nothing needs to know about the
    // mode.  RESET opens the gate again (QUESTION-P56-3), which is how the
    // reset that is power down's only exit reaches the SFR block that holds
    // PCON.PD.
    // ------------------------------------------------------------------
    wire ce_g = CE && !pd_freeze;

    // ------------------------------------------------------------------
    // SFR bus arbitration.  While CE is parked the NU8051_BACKDOOR port
    // borrows the bus (§1.9): storage access, no side effects, `sfr_rmw`
    // forced so a port read returns the LATCH (which is what "SFR storage"
    // means for a port, savestate_design §5.5 lowering).
    // ------------------------------------------------------------------
`ifdef NU8051_BACKDOOR
    wire bkd_bus = !CE;
    wire sfr_bkd_we = bkd_bus && bkd_we && bkd_addr[8];
    assign sfr_addr  = bkd_bus ? bkd_addr[7:0] : seq_sfr_addr;
    assign sfr_wdata = bkd_bus ? bkd_wdata     : seq_sfr_wdata;
    assign sfr_rmw   = bkd_bus ? 1'b1          : seq_sfr_rmw;
    assign sfr_we    = bkd_bus ? 1'b0          : seq_sfr_we;
`else
    assign sfr_addr  = seq_sfr_addr;
    assign sfr_wdata = seq_sfr_wdata;
    assign sfr_rmw   = seq_sfr_rmw;
    assign sfr_we    = seq_sfr_we;
`endif

    // PQ5: unimplemented SFR reads return NU8051_CFG_UNIMPL_RD, writes are
    // dropped (no owner claims the address, so no storage moves).
    wire sfr_any_hit = hit_sfr | hit_ports | hit_timer | hit_uart | hit_irq;
    assign sfr_rdata = !sfr_any_hit ? NU8051_CFG_UNIMPL_RD
                     : ((hit_sfr   ? rd_sfr   : 8'h00)
                      | (hit_ports ? rd_ports : 8'h00)
                      | (hit_timer ? rd_timer : 8'h00)
                      | (hit_uart  ? rd_uart  : 8'h00)
                      | (hit_irq   ? rd_irq   : 8'h00));

    // ------------------------------------------------------------------
    // sequencer
    // ------------------------------------------------------------------
    nu8051_seq #(.P_8052(P_8052), .ROM_AW(ROM_AW)) u_seq (
        .CLK        (CLK),
        .CE         (CE),
        .RESET      (RESET),
        .EA_N       (EA_N),
        .MEM_ADDR   (MEM_ADDR),
        .MEM_DOUT   (MEM_DOUT),
        .MEM_DIN    (MEM_DIN),
        .MEM_ALE    (MEM_ALE),
        .MEM_PSEN_N (MEM_PSEN_N),
        .MEM_RD_N   (MEM_RD_N),
        .MEM_WR_N   (MEM_WR_N),
        .ROM_ADDR   (ROM_ADDR),
        .ROM_DATA   (ROM_DATA),
        .iram_addr  (iram_addr),
        .iram_rdata (iram_rdata),
        .iram_we    (iram_we),
        .iram_wdata (iram_wdata),
        .sfr_addr   (seq_sfr_addr),
        .sfr_rmw    (seq_sfr_rmw),
        .sfr_rdata  (sfr_rdata),
        .sfr_we     (seq_sfr_we),
        .sfr_wdata  (seq_sfr_wdata),
        .acc_q      (acc_q),
        .b_q        (b_q),
        .psw_q      (psw_q),
        .sp_q       (sp_q),
        .dptr_q     (dptr_q),
        // C4.8 / TQ10: the P2 latch is the MOVX @Ri high address byte
        .p2_q       (p2_latch_q),
        .acc_we     (acc_we),
        .acc_wdata  (acc_wdata),
        .b_we       (b_we),
        .b_wdata    (b_wdata),
        .sp_we      (sp_we),
        .sp_wdata   (sp_wdata),
        .dptr_we    (dptr_we),
        .dptr_wdata (dptr_wdata),
        .flag_we_c  (flag_we_c),  .flag_c  (flag_c),
        .flag_we_ac (flag_we_ac), .flag_ac (flag_ac),
        .flag_we_ov (flag_we_ov), .flag_ov (flag_ov),
        .ph_o       (ph_w),
        .tk_s2p2    (tk_s2p2),
        .tk_s3p1    (tk_s3p1),
        .tk_s5p1    (tk_s5p1),
        .tk_s5p2    (tk_s5p2),
        .tk_s6p1    (tk_s6p1),
        .tk_s6p2    (tk_s6p2),
        .bus_active (bus_active),
        .prime_cycle(prime_cycle),
        .irq_req    (irq_req),
        .irq_src    (irq_src),
        .irq_prio   (irq_prio),
        .irq_reti   (irq_reti),
        .irq_ack    (irq_ack),
        .irq_ack_src(irq_ack_src),
        .irq_ack_prio(irq_ack_prio),
        .pcon_idl   (pcon_idl),
        .pcon_pd    (pcon_pd),
        .pcon_idl_clr(pcon_idl_clr),
        .pd_freeze  (pd_freeze),
        .cpu_idle   (cpu_idle),
        .mcyc_o     (mcyc_w),
        .seqstate_o (seqstate_w),
        .pc_o       (pc_w),
        .rst_hold_o (rst_hold),
        .retire_o   (retire_w),
        .hash_o     (hash_w),
        .ss_addr    (ss_addr_q),
        .ss_wdata   (ss_wdata_q),
        .ss_we      (ss_we_q),
        .ss_rdata   (ss_rd_seq),
        .ss_rdata_alu(ss_rd_alu)
`ifdef NU8051_BACKDOOR
        ,
        .bkd_load   (bkd_load),
        .bkd_pc     (bkd_pc)
`endif
    );

    // ------------------------------------------------------------------
    // memories / SFR owners
    // ------------------------------------------------------------------
    logic [7:0] iram_bkd_rdata;

    nu8051_iram u_iram (
        .CLK        (CLK),
        .CE         (ce_g),
        .addr       (iram_addr),
        .rdata      (iram_rdata),
        .we         (iram_we),
        .wdata      (iram_wdata),
        .ss_sel     (iram_ss_sel),
        .ss_addr    (ss_addr_q[7:0]),
        .ss_we      (ss_we_q),
        .ss_wdata   (ss_wdata_q[7:0])
`ifdef NU8051_BACKDOOR
        ,
        .bkd_we     (bkd_we && !bkd_addr[8]),
        .bkd_addr   (bkd_addr[7:0]),
        .bkd_wdata  (bkd_wdata),
        .bkd_rdata  (iram_bkd_rdata)
`endif
    );

    nu8051_sfr u_sfr (
        .CLK        (CLK),
        .CE         (ce_g),
        .rst_hold   (rst_hold),
        .sfr_addr   (sfr_addr),
        .sfr_rdata  (rd_sfr),
        .sfr_hit    (hit_sfr),
        .sfr_we     (sfr_we),
        .sfr_wdata  (sfr_wdata),
        .acc_q      (acc_q),
        .b_q        (b_q),
        .psw_q      (psw_q),
        .sp_q       (sp_q),
        .dptr_q     (dptr_q),
        .pcon_smod  (pcon_smod),
        .pcon_idl   (pcon_idl),
        .pcon_pd    (pcon_pd),
        .pcon_idl_clr(pcon_idl_clr),
        .flag_we_c  (flag_we_c),  .flag_c  (flag_c),
        .flag_we_ac (flag_we_ac), .flag_ac (flag_ac),
        .flag_we_ov (flag_we_ov), .flag_ov (flag_ov),
        .acc_we     (acc_we),     .acc_wdata (acc_wdata),
        .b_we       (b_we),       .b_wdata   (b_wdata),
        .sp_we      (sp_we),      .sp_wdata  (sp_wdata),
        .dptr_we    (dptr_we),    .dptr_wdata(dptr_wdata),
        .ss_addr    (ss_addr_q),
        .ss_wdata   (ss_wdata_q),
        .ss_we      (ss_we_q),
        .ss_rdata   (ss_rd_sfr)
`ifdef NU8051_BACKDOOR
        , .sfr_bkd_we (sfr_bkd_we)
`endif
    );

    nu8051_ports u_ports (
        .CLK        (CLK),
        .CE         (ce_g),
        .rst_hold   (rst_hold),
        .tk_s5p1    (tk_s5p1),
        .tk_s6p2    (tk_s6p2),
        .bus_active (bus_active),
        .sfr_addr   (sfr_addr),
        .sfr_rmw    (sfr_rmw),
        .sfr_rdata  (rd_ports),
        .sfr_hit    (hit_ports),
        .sfr_we     (sfr_we),
        .sfr_wdata  (sfr_wdata),
        .P0_IN      (P0_IN), .P1_IN(P1_IN), .P2_IN(P2_IN), .P3_IN(P3_IN),
        .P0_OUT     (P0_OUT), .P1_OUT(P1_OUT), .P2_OUT(P2_OUT), .P3_OUT(P3_OUT),
        .P0_OEN     (P0_OEN), .P1_OEN(P1_OEN), .P2_OEN(P2_OEN), .P3_OEN(P3_OEN),
        // C4.8 / QUESTION-P48-2: the MOVX strobes are NOT folded into the
        // core's P3_OUT.  §1.5 lists P3.6/P3.7 <- WR#/RD# as a core-side
        // fold, but that is the same class of behaviour §1.6 assigns to
        // `nu8051_pins` for the other two bus-multiplexed ports (P0's
        // address/data mux and P2's DPH/PCH emission are recreated in the
        // wrapper from MEM_ADDR + the strobes, while the core's P0_OUT/
        // P2_OUT show the LATCH at all times, §1.5).  Keeping the strobes
        // out of the core's P3_OUT makes the three bus-multiplexed ports
        // consistent, and it is what the two independent verification
        // artifacts require: CAD-11 (port pins change only at S1P1, while a
        // folded strobe would move P3_OUT at S4P1 when RD#/WR# rises) and
        // the vector corpus, whose `P` bus-op lists carry no P3 event for
        // any MOVX.  The wrapper has MEM_RD_N/MEM_WR_N as pass-through
        // outputs, so nothing is lost at the pin surface; the fold
        // mechanism itself stays live for TXD / mode-0 RXD.
        .alt_rd_n   (1'b1),
        .alt_wr_n   (1'b1),
        .alt_txd    (txd_r),
        .alt_rxd_oe (RXD_OE),
        .alt_rxd_out(RXD_OUT),
        .p2_latch_q (p2_latch_q),
        .ss_addr    (ss_addr_q),
        .ss_wdata   (ss_wdata_q),
        .ss_we      (ss_we_q),
        .ss_rdata   (ss_rd_ports)
`ifdef NU8051_BACKDOOR
        , .sfr_bkd_we (sfr_bkd_we)
`endif
    );

    nu8051_timer #(.P_8052(P_8052)) u_timer (
        .CLK        (CLK),
        .CE         (ce_g),
        .rst_hold   (rst_hold),
        .tk_s2p2    (tk_s2p2),
        .tk_s3p1    (tk_s3p1),
        .tk_s5p2    (tk_s5p2),
        .prime_cycle(prime_cycle),
        .sfr_addr   (sfr_addr),
        .sfr_rdata  (rd_timer),
        .sfr_hit    (hit_timer),
        .sfr_we     (sfr_we),
        .sfr_wdata  (sfr_wdata),
        .T0         (T0),
        .T1         (T1),
        .T2         (T2),
        .T2EX       (T2EX),
        .int0_pin   (int0_pin),
        .int1_pin   (int1_pin),
        .irq_iex_we (irq_iex_we),
        .irq_iex_d  (irq_iex_d),
        .irq_tfx_clr(irq_tfx_clr),
        .tcon_q     (tcon_q),
        .t2con_q    (t2con_q),
        .t1_ovf     (t1_ovf),
        .t2_ovf     (t2_ovf),
        .ss_addr    (ss_addr_q),
        .ss_wdata   (ss_wdata_q),
        .ss_we      (ss_we_q),
        .ss_rdata   (ss_rd_timer)
`ifdef NU8051_BACKDOOR
        , .sfr_bkd_we (sfr_bkd_we)
`endif
    );

    nu8051_uart u_uart (
        .CLK        (CLK),
        .CE         (ce_g),
        .rst_hold   (rst_hold),
        .prime_cycle(prime_cycle),
        .ph         (ph_w),
        .tk_s5p2    (tk_s5p2),
        .tk_s6p2    (tk_s6p2),
        .smod       (pcon_smod),
        .t1_ovf     (t1_ovf),
        // T2CON.5 = RCLK, T2CON.4 = TCLK (periph §4.6 Figure 11); this file
        // does not include nu8051_defs.svh, so the two selects are named
        // here rather than symbolically.
        .rclk       (t2con_q[5]),
        .tclk       (t2con_q[4]),
        .t2_ovf     (t2_ovf),
        .RXD_IN     (RXD_IN),
        .sfr_addr   (sfr_addr),
        .sfr_rdata  (rd_uart),
        .sfr_hit    (hit_uart),
        .sfr_we     (sfr_we),
        .sfr_wdata  (sfr_wdata),
        .ri_q       (ri_q),
        .ti_q       (ti_q),
        .txd_r      (txd_r),
        .rxd_out    (RXD_OUT),
        .rxd_oe     (RXD_OE),
        .ss_addr    (ss_addr_q),
        .ss_wdata   (ss_wdata_q),
        .ss_we      (ss_we_q),
        .ss_rdata   (ss_rd_uart)
`ifdef NU8051_BACKDOOR
        , .sfr_bkd_we (sfr_bkd_we)
`endif
    );

    nu8051_irq #(.P_8052(P_8052)) u_irq (
        .CLK        (CLK),
        .CE         (ce_g),
        .rst_hold   (rst_hold),
        .tk_s5p2    (tk_s5p2),
        .tk_s6p2    (tk_s6p2),
        .prime_cycle(prime_cycle),
        .sfr_addr   (sfr_addr),
        .sfr_rdata  (rd_irq),
        .sfr_hit    (hit_irq),
        .sfr_we     (sfr_we),
        .sfr_wdata  (sfr_wdata),
        .INT0_N     (INT0_N),
        .INT1_N     (INT1_N),
        .int0_pin   (int0_pin),
        .int1_pin   (int1_pin),
        .tcon_q     (tcon_q),
        .ri_q       (ri_q),
        .ti_q       (ti_q),
        .t2con_q    (t2con_q),
        .irq_iex_we (irq_iex_we),
        .irq_iex_d  (irq_iex_d),
        .irq_tfx_clr(irq_tfx_clr),
        .reti_clr   (irq_reti),
        .irq_ack    (irq_ack),
        .irq_ack_src(irq_ack_src),
        .irq_ack_prio(irq_ack_prio),
        .irq_req    (irq_req),
        .irq_src    (irq_src),
        .irq_prio   (irq_prio),
        .irq_ipl    (irq_ipl),
        .ss_addr    (ss_addr_q),
        .ss_wdata   (ss_wdata_q),
        .ss_we      (ss_we_q),
        .ss_rdata   (ss_rd_irq)
`ifdef NU8051_BACKDOOR
        , .sfr_bkd_we (sfr_bkd_we)
        , .bkd_load   (bkd_load)
`endif
    );

    assign TXD = txd_r;

    // ==================================================================
    // save state (savestate_design §1.2/§1.3/§5, Phase-6 A1)
    //
    // THE COHERENCE RULE, verbatim from §6 and audited per block in A1:
    // **while CE == 0, core state changes only via SS writes.**  Reads are
    // free-running (a registered mux that never touches state); writes are
    // legal only while the platform has parked CE and RESET is low.  Reads
    // taken at any parked instant therefore form a coherent snapshot, and
    // writes in any order reproduce it exactly.
    // ==================================================================

    // ---- single command staging point (§1.2) -------------------------
    // Every module-facing copy is the STAGED command, never the raw port.
    logic  [9:0] ss_addr_q;
    logic [15:0] ss_wdata_q;
    logic        ss_we_q;
    logic        ss_sel_iram_q, ss_sel_tag_q;
    logic  [2:0] ss_sel_mod_q;

    // Address decode of the STAGED address, combinational: it drives the
    // IRAM port mux, which samples on the same edge the flop modules
    // register their read words.  The IRAM window is 0x200-0x2FF, so the
    // decode is addr[9] & ~addr[8] - NOT addr[9] alone, which would alias
    // the reserved 0x300-0x3FF onto the array and break both the
    // "reserved addresses read 0" rule (§1.2) and the §7.4 probes
    // (savestate_design §3.4 adjudication).
    wire ss_iram_hit = (ss_addr_q >= SS_IRAM_BASE)
                    && (ss_addr_q < SS_IRAM_BASE + 10'(SS_IRAM_COUNT));
    // INV-MUX (§5.4): the SS side never touches the port on a CE-high edge.
    wire iram_ss_sel = !CE && ss_iram_hit;

    always_ff @(posedge CLK) begin
        ss_addr_q     <= SS_ADDR;
        ss_wdata_q    <= SS_WDATA;
        ss_we_q       <= SS_WE;
        ss_sel_iram_q <= ss_iram_hit;
        ss_sel_tag_q  <= (ss_addr_q == SSA_TAG);
        ss_sel_mod_q  <= ss_region_of(ss_addr_q);
    end

    // ---- registered read mux (§1.2): SS_RDATA valid 2 CLKs after SS_ADDR
    logic [15:0] ss_rd_seq, ss_rd_alu, ss_rd_sfr, ss_rd_ports,
                 ss_rd_timer, ss_rd_uart, ss_rd_irq;
    logic [15:0] ss_mod_rdata;
    always_comb begin
        case (ss_sel_mod_q)
            3'd0:    ss_mod_rdata = ss_rd_seq;
            3'd1:    ss_mod_rdata = ss_rd_alu;
            3'd2:    ss_mod_rdata = ss_rd_sfr;
            3'd3:    ss_mod_rdata = ss_rd_ports;
            3'd4:    ss_mod_rdata = ss_rd_timer;
            3'd5:    ss_mod_rdata = ss_rd_uart;
            default: ss_mod_rdata = ss_rd_irq;
        endcase
    end

    assign SS_RDATA = ss_sel_tag_q  ? SS_TAG
                    : ss_sel_iram_q ? {8'h00, iram_rdata}
                                    : ss_mod_rdata;

    // ---- tag / sticky SS_ERR (§1.2) ----------------------------------
    // A tag WRITE is the integrity check: a mismatch sets SS_ERR, a matching
    // write or RESET clears it.  The hardware stays dumb - a mismatched
    // restore still writes all state; the platform treats SS_ERR as fatal.
    always_ff @(posedge CLK) begin
        if (RESET) SS_ERR <= 1'b0;
        else if (ss_we_q && (ss_addr_q == SSA_TAG))
            SS_ERR <= (ss_wdata_q != SS_TAG);
    end

    initial begin
        ss_addr_q = 10'h000; ss_wdata_q = 16'h0000; ss_we_q = 1'b0;
        ss_sel_iram_q = 1'b0; ss_sel_tag_q = 1'b0; ss_sel_mod_q = 3'd0;
        SS_ERR = 1'b0;
    end

    // ---- park-contract assertions (§1.3 / §6 A-SS1..A-SS4) -----------
`ifndef SYNTHESIS
    always @(posedge CLK) begin
        if (SS_WE && CE)
            $error("A-SS1: SS_WE while CE high (core not frozen)");
        if (SS_WE && RESET)
            $error("A-SS2: SS_WE during RESET");
        if (CE && ss_we_q)
            $error("A-SS3: CE resumed with SS staging undrained");
        // A-SS4 (INV-MUX).  Written on the ADDRESS decode, not on the mux
        // select: `CE && iram_ss_sel` is vacuous by construction (the select
        // carries !CE), while the address decode catches the real hazard -
        // a platform that leaves SS_ADDR inside the window while the core
        // runs would have the window steal the IRAM port on the CE-low
        // edges of a divided duty and clobber `iram_rdata` between an
        // address issue and its arrival edge.  SS_ADDR must be parked
        // outside 0x200-0x2FF whenever CE is running.
        if (CE && ss_iram_hit)
            $error("A-SS4: IRAM window addressed while CE high (INV-MUX)");
    end
`endif

    // ------------------------------------------------------------------
    // NU8051_BACKDOOR observation contract (§1.9, ratified QUESTION-T23-1).
    // These are NOT ports: the TB reads them hierarchically.
    // ------------------------------------------------------------------
`ifdef NU8051_BACKDOOR
    wire  [3:0] bkd_ph        = ph_w;
    // C5.6: the two power modes, exported as observation only (§1.9).  The TB
    // needs them because they change what the bus pins MEAN: ALE/PSEN# hold
    // high in idle and low in power down (periph §7), and in power down the
    // phase counter does not run at all - so CAD-1/2/3 and CAD-13 have to be
    // told, and CAD-15 checks the §7 levels in their place.
    wire        bkd_idle      = cpu_idle;
    wire        bkd_pd        = pd_freeze;
    // RESET_HOLD joins them in the TB's "not fetching" predicate: it is the
    // third state in which the fetch pipeline is stopped while ALE/PSEN# sit
    // at forced levels (§7.1), and a mid-case reset (C5.6's power-down exit)
    // is the first thing in the project that puts it inside a trace.
    wire        bkd_rst_hold  = rst_hold;
    wire  [1:0] bkd_mcyc      = mcyc_w;
    wire  [2:0] bkd_seqstate  = seqstate_w;
    wire        bkd_retire    = retire_w;
    wire [15:0] bkd_retire_pc = pc_w;
    wire [15:0] bkd_pc_cur    = pc_w;
    wire [31:0] bkd_state_hash = hash_w
                              ^ {acc_q, b_q, psw_q, sp_q}
                              ^ {dptr_q, p2_latch_q, 8'h00}
                              ^ {P0_OUT, P1_OUT, P2_OUT, P3_OUT}
                              ^ {24'h0, 3'b0, int0_pin, int1_pin,
                                 tcon_q[5] /* TF0 */, ri_q, ti_q}
                              // C5.3: the interrupt unit's own CE-gated
                              // state - the snapshot the next cycle polls
                              // and the two in-progress flip-flops.  A park
                              // across the S5P2 snapshot edge or inside an
                              // ILCALL must be invisible (§1.9 / CAD-12).
                              ^ {20'h0, 2'b0, irq_ipl, irq_prio, irq_src,
                                 3'b0, irq_req}
                              // C5.4: the UART's own CE-gated engine state.
                              // A park anywhere inside a frame - mid shift
                              // clock, mid majority window, between the two
                              // double-buffered halves - must be invisible
                              // (§1.9 / CAD-12).
                              ^ {u_uart.tx_shift, u_uart.rx_shift,
                                 u_uart.tx_bitcnt, u_uart.rx_bitcnt,
                                 u_uart.tx_send, u_uart.tx_dataf,
                                 u_uart.tx_req, u_uart.rx_recv,
                                 u_uart.rx_pend, u_uart.rxd_prev}
                              ^ {11'h0, u_uart.tx_div16, u_uart.rx_div16,
                                 u_uart.rx_maj, u_uart.smod_div,
                                 u_uart.m2_div2, u_uart.tx_bnd_pend,
                                 u_uart.sbuf_tx_r}
                              // C5.5: timer 2's own CE-gated engine state -
                              // the two pin pipelines, the OSC/2 prescaler of
                              // the baud-rate mode and TF2's S2P2 staging
                              // flop.  A park across any of those edges must
                              // be invisible (§1.9 / CAD-12).
                              ^ {26'h0, u_timer.t2_smpl, u_timer.t2_pend,
                                 u_timer.t2ex_smpl, u_timer.t2ex_pend,
                                 u_timer.t2_osc2, u_timer.tf2_stage};

    // registered SFR-side backdoor read; the IRAM side is registered inside
    // nu8051_iram with identical (1 parked CLK) latency, so the two halves
    // stay aligned behind one `bkd_addr[8]` pipeline stage.
    logic [7:0] sfr_bkd_rdata;
    logic       bkd_sel_q;
    always_ff @(posedge CLK) begin
        if (!CE) begin
            sfr_bkd_rdata <= sfr_rdata;
            bkd_sel_q     <= bkd_addr[8];
        end
    end
    assign bkd_rdata = bkd_sel_q ? sfr_bkd_rdata : iram_bkd_rdata;

    initial begin
        sfr_bkd_rdata = 8'h00;
        bkd_sel_q     = 1'b0;
    end
`endif

    // ------------------------------------------------------------------
    // unused-input sink for the pins the later phases consume
    // ------------------------------------------------------------------
    wire _unused_ok = &{1'b0,
                        tk_s6p1, b_q, sp_q, dptr_q, p2_latch_q,
                        // C5.3: observation taps consumed only by the
                        // NU8051_BACKDOOR digest (§1.9)
                        irq_ipl, tcon_q,
                        // C5.6: observation-only outside NU8051_BACKDOOR
                        cpu_idle, 1'b0};

endmodule
