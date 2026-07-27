//============================================================================
// Imported from nec_test (https://github.com/.../nec_test)
// Source: hdl/rtl/core, commit 650dc971cb170d7fd27c073049be99b1c1bbb278
// Cycle-accurate NEC V30 (uPD70116) max-mode core, verified vs golden traces.
// Do not hand-edit; re-import from upstream. No license header upstream (same author).
//============================================================================

//============================================================================
//
//  v30_core - cycle-accurate NEC V30 (uPD70116) CPU core, max mode
//
//  Campaign 3: EU + BIU verified against golden traces captured from the
//  real chip (tests/v30/v0.1) via hdl/tb/tb_v30_core.sv / sw/check_core.py.
//
//  The port list mirrors the physical chip's maximum-mode pins as seen by
//  the harness (hdl/rtl/nec_bus.sv), so Campaign 4 can instantiate this
//  core behind the same nec_bus interface used for the socketed part:
//    AD[19:0]  muxed address/data; AD[19:16] carry PS3-0 during T2-T4
//    BS[2:0]   bus-cycle status (8086 S2-S0 compatible)
//    QS[1:0]   queue status (00 none, 01 first byte, 10 flush, 11 byte)
//    RD_N, UBE_N, BUSLOCK_N, READY, RESET, INT, NMI, POLL_N, CLK
//
//  V30_BACKDOOR (verification only, set by the testbench build): adds
//  state-injection/observation ports so golden test cases can start from
//  an arbitrary architectural state without a load routine, plus a
//  scripted queue-consumer mode that replaces the EU for BIU-only
//  verification. The backdoor is compiled out of synthesis builds; the
//  normal reset flow (vector fetch at FFFF0h) runs in synthesis builds
//  (implemented in Campaign 3 mission G; see v30_biu's reset-vector
//  sequencing note, boot-capture verified by sw/check_boot.py).
//
//  INT/NMI/POLL, HALT, and wait states are now implemented (Campaign 3
//  mission blocks 3-4). Still not implemented: BUSLOCK (BUSLOCK_N is
//  tied high), and the deferred opcode families noted in the closure
//  checkpoint (INM/OUTM 6C-6F, BRKEM/8080-emulation mode, the 0x82
//  alias).
//
//============================================================================

/* verilator lint_off WIDTHEXPAND */
/* verilator lint_off UNUSEDPARAM */
/* verilator lint_on UNUSEDPARAM */
/* verilator lint_on WIDTHEXPAND */

module v30_core (
    input             CLK,
    input             CE,        // clock-enable: advance core state this clk
    input             CE_HALF,   // clock-enable for the T1 negedge half-cycle
    input             RESET,
    input             READY,
    input             INT,
    input             NMI,
    input             POLL_N,
    inout      [19:0] AD,
    output      [1:0] QS,
    output      [2:0] BS,
    output            RD_N,
    output            UBE_N,
    output            BUSLOCK_N,
    input [v30_ss_pkg::SS_ADDR_W-1:0] SS_ADDR,
    input      [15:0] SS_WDATA,
    input             SS_WE,
    output     [15:0] SS_RDATA,
    output reg        SS_ERR,
    output            SS_BUS_QUIET
`ifdef V30_BACKDOOR
    ,
    input             bkd_load,     // pulse while RESET=1: inject state
    input     [223:0] bkd_regs,     // {psw,ip,ds,ss,cs,es,di,si,bp,sp,bx,dx,cx,ax}
    input      [47:0] bkd_queue,    // queue bytes, entry 0 first
    input       [2:0] bkd_qlen,
    input      [15:0] bkd_fetch_ip, // BIU fetch offset (= ip + qlen)
    input             scr_en,       // scripted-consumer mode (BIU-only test)
    input       [1:0] scr_qop,      // per-cycle queue op, QS encoding
    output    [223:0] dbg_regs,     // ip slot holds the retired-instruction IP
    output            dbg_first_pop,
    output            dbg_pend
`endif
);

import v30_ss_pkg::*;

`ifndef V30_BACKDOOR
logic         bkd_load = 1'b0;
logic [223:0] bkd_regs = '0;
logic  [47:0] bkd_queue = '0;
logic   [2:0] bkd_qlen = '0;
logic  [15:0] bkd_fetch_ip = '0;
logic         scr_en = 1'b0;
logic   [1:0] scr_qop = '0;
`endif

wire  [7:0] q_byte;
wire        q_avail, q_avail2, q_fresh, q_any;
wire        eu_pop, eu_first, eu_flush;
wire [15:0] eu_flush_cs, eu_flush_ip;
wire        eu_req, eu_hold, eu_ready, eu_wr, eu_fwd, eu_word;
wire        eu_soon, eu_soon_ea, eu_soon_ivt, bus_phase, bus_t4, flush_fast;
wire        eu_soon_strio;                   // Family-7 strio idle-window lead (task #24)
wire        grid_phase;
wire        eu_lock, core_buslock_n, eu_mem_acc;
wire        eu_rsv_dhi, eu_rsv_push_calc;   // Phase 3 reservation-class hints
wire        eu_rsv_lead;                     // eu_req=0 onset lead hint
wire        eu_rsv_strio;                    // Family-5 strio T3-eval veto (task #24)
wire        eu_rdone, bus_tw;
wire        eu_defer_wr;
wire [2:0]  bus_ts;
wire  [1:0] eu_kind;
wire        eu_wrap;
wire [19:0] eu_addr;
wire  [1:0] eu_seg;
wire [15:0] eu_wdata;
wire        eu_started, eu_done, eu_wdone, eu_t1;
wire [15:0] eu_rdata;
wire        eu_rd_now;
wire [15:0] eu_rdata_now;
wire        psw_ie;
wire        halt_disp;
wire [15:0] ss_eu_rdata;
wire [15:0] ss_biu_rdata;
wire        ss_biu_bus_quiet;
reg  [8:0]  ss_addr_q;
reg  [15:0] ss_wdata_q;
reg         ss_we_q;
reg         ss_sel_eu_q, ss_sel_tag_q;

always_ff @(posedge CLK) begin
    ss_addr_q    <= SS_ADDR;
    ss_wdata_q   <= SS_WDATA;
    ss_we_q      <= SS_WE;
    ss_sel_eu_q  <= ss_addr_q[8];
    ss_sel_tag_q <= (ss_addr_q == SSA_TAG);
end

assign SS_RDATA = ss_sel_tag_q ? SS_TAG
                : ss_sel_eu_q  ? ss_eu_rdata : ss_biu_rdata;

always_ff @(posedge CLK) begin
    if (RESET) SS_ERR <= 1'b0;
    else if (ss_we_q && ss_addr_q == SSA_TAG)
        SS_ERR <= (ss_wdata_q != SS_TAG);
end

assign SS_BUS_QUIET = ss_biu_bus_quiet;

`ifndef SYNTHESIS
always @(posedge CLK) begin
    if (SS_WE && CE)    $error("SS_WE asserted while CE high (core not frozen)");
    if (SS_WE && RESET) $error("SS_WE asserted during RESET");
    // Resume-drain contract (A2): the platform must NOT re-enable CE until the
    // SS command staging has drained. ss_we_q still high at a CE cycle means the
    // core's FSM takes its `if (ss_we)` branch and SKIPS its state advance -> a
    // phantom wait on resume. The TB honours this via a parked drain posedge +
    // negedge CE release; this assertion mechanises the rule so the next
    // integration (MiSTer wrapper) cannot repeat the bug silently.
    if (CE && ss_we_q)  $error("CE resumed with SS command staging undrained (ss_we_q high)");
end
`endif

// scripted-consumer override (BIU-only verification)
wire q_pop   = scr_en ? scr_qop[0]              : eu_pop;
wire q_first = scr_en ? (scr_qop == 2'b01)      : eu_first;
wire q_flush = scr_en ? 1'b0                    : eu_flush;
wire qs_e;   // E display timing is BIU-generated (measured law)

// queue status pins: 00 none, 01 first byte, 10 flush, 11 subsequent byte
assign QS = qs_e   ? 2'b10
          : q_pop  ? (q_first ? 2'b01 : 2'b11)
          : 2'b00;

wire [19:0] ad_o;
wire        ad_oe_addr, ad_oe_ps, ad_oe_data;

v30_biu u_biu (
    .clk        (CLK),
    .ce         (CE),
    .ce_half    (CE_HALF),
    .srst       (RESET),
    .bs         (BS),
    .ad_o       (ad_o),
    .ad_oe_addr (ad_oe_addr),
    .ad_oe_ps   (ad_oe_ps),
    .ad_oe_data (ad_oe_data),
    .ube_n      (UBE_N),
    .rd_n       (RD_N),
    .ad_i       (AD[15:0]),
    .ready      (READY),
    .psw_ie     (psw_ie),
    .halt_disp  (halt_disp),
    .q_byte     (q_byte),
    .q_avail    (q_avail),
    .q_avail2   (q_avail2),
    .q_fresh    (q_fresh),
    .q_any      (q_any),
    .qs_e       (qs_e),
    .q_pop      (q_pop),
    .q_flush    (q_flush),
    .flush_cs   (eu_flush_cs),
    .flush_ip   (eu_flush_ip),
    .eu_req     (scr_en ? 1'b0 : eu_req),
    .eu_soon    (scr_en ? 1'b0 : eu_soon),
    .eu_soon_ea (scr_en ? 1'b0 : eu_soon_ea),
    .eu_soon_ivt(scr_en ? 1'b0 : eu_soon_ivt),
    .eu_soon_strio(scr_en ? 1'b0 : eu_soon_strio),
    .flush_fast (scr_en ? 1'b0 : flush_fast),
    .eu_defer_wr(scr_en ? 1'b0 : eu_defer_wr),
    .eu_mem_acc (scr_en ? 1'b0 : eu_mem_acc),
    .bus_phase  (bus_phase),
    .grid_phase (grid_phase),
    .eu_lock    (scr_en ? 1'b0 : eu_lock),
    .buslock_n  (core_buslock_n),
    .bus_t4     (bus_t4),
    .bus_tw     (bus_tw),
    .bus_ts     (bus_ts),
    .eu_hold    (scr_en ? 1'b0 : eu_hold),
    .eu_ready   (eu_ready),
    .eu_rsv_dhi (scr_en ? 1'b0 : eu_rsv_dhi),
    .eu_rsv_push_calc (scr_en ? 1'b0 : eu_rsv_push_calc),
    .eu_rsv_lead (scr_en ? 1'b0 : eu_rsv_lead),
    .eu_rsv_strio(scr_en ? 1'b0 : eu_rsv_strio),
    .eu_wr      (eu_wr),
    .eu_fwd     (eu_fwd),
    .eu_word    (eu_word),
    .eu_kind    (eu_kind),
    .eu_wrap    (eu_wrap),
    .eu_addr    (eu_addr),
    .eu_seg     (eu_seg),
    .eu_wdata   (eu_wdata),
    .eu_started (eu_started),
    .eu_done    (eu_done),
    .eu_wdone   (eu_wdone),
    .eu_rdone   (eu_rdone),
    .eu_t1      (eu_t1),
    .eu_rdata   (eu_rdata),
    .eu_rd_now  (eu_rd_now),
    .eu_rdata_now (eu_rdata_now),
    .bkd_load   (bkd_load),
    .bkd_cs     (bkd_regs[144 +: 16]),
    .bkd_ip     (bkd_fetch_ip),
    .bkd_queue  (bkd_queue),
    .bkd_qlen   (bkd_qlen),
    .ss_addr    (ss_addr_q),
    .ss_wdata   (ss_wdata_q),
    .ss_we      (ss_we_q),
    .ss_rdata   (ss_biu_rdata),
    .ss_bus_quiet(ss_biu_bus_quiet)
);

v30_eu u_eu (
    .clk        (CLK),
    .ce         (CE),
    .srst       (RESET),
    .q_byte     (q_byte),
    .q_avail    (q_avail),
    .q_avail2   (q_avail2),
    .q_fresh    (q_fresh),
    .q_any      (q_any),
    .q_pop      (eu_pop),
    .q_first    (eu_first),
    .q_flush    (eu_flush),
    .flush_cs   (eu_flush_cs),
    .flush_ip   (eu_flush_ip),
    .eu_req     (eu_req),
    .eu_soon    (eu_soon),
    .eu_soon_ea (eu_soon_ea),
    .eu_soon_ivt(eu_soon_ivt),
    .eu_soon_strio(eu_soon_strio),
    .flush_fast (flush_fast),
    .eu_defer_wr(eu_defer_wr),
    .eu_mem_acc (eu_mem_acc),
    .eu_rsv_dhi (eu_rsv_dhi),
    .eu_rsv_push_calc (eu_rsv_push_calc),
    .eu_rsv_lead (eu_rsv_lead),
    .eu_rsv_strio(eu_rsv_strio),
    .bus_phase  (bus_phase),
    .grid_phase (grid_phase),
    .eu_lock    (eu_lock),
    .bus_t4     (bus_t4),
    .bus_tw     (bus_tw),
    .bus_ts     (bus_ts),
    .eu_hold    (eu_hold),
    .eu_ready   (eu_ready),
    .eu_wr      (eu_wr),
    .eu_fwd     (eu_fwd),
    .eu_word    (eu_word),
    .eu_kind    (eu_kind),
    .eu_wrap    (eu_wrap),
    .eu_addr    (eu_addr),
    .eu_seg     (eu_seg),
    .eu_wdata   (eu_wdata),
    .eu_started (eu_started),
    .eu_done    (eu_done),
    .eu_wdone   (eu_wdone),
    .eu_rdone   (eu_rdone),
    .eu_t1      (eu_t1),
    .eu_rdata   (eu_rdata),
    .eu_rd_now  (eu_rd_now),
    .eu_rdata_now (eu_rdata_now),
    .psw_ie     (psw_ie),
    .halt_disp  (halt_disp),
    .pin_int    (INT),
    .pin_nmi    (NMI),
    .pin_poll_n (POLL_N),
    .bkd_load   (bkd_load),
    .bkd_regs   (bkd_regs),
    .ss_addr    (ss_addr_q),
    .ss_wdata   (ss_wdata_q),
    .ss_we      (ss_we_q),
    .ss_rdata   (ss_eu_rdata)
`ifdef V30_BACKDOOR
    ,
    .dbg_regs      (dbg_regs),
    .dbg_first_pop (dbg_first_pop),
    .dbg_pend      (dbg_pend)
`else
    ,
    /* verilator lint_off PINCONNECTEMPTY */
    .dbg_regs      (),
    .dbg_first_pop (),
    .dbg_pend      ()
    /* verilator lint_on PINCONNECTEMPTY */
`endif
);

// AD drive (simple en?val:'z forms only - Verilator requirement).
// AD[19:16] carry the address during address phases and PS3-0 during
// the data phase; AD[15:0] additionally carry write data.
assign AD[15:0]  = (ad_oe_addr | ad_oe_data) ? ad_o[15:0]  : 16'hzzzz;
assign AD[19:16] = (ad_oe_addr | ad_oe_ps)   ? ad_o[19:16] : 4'hz;

assign BUSLOCK_N = core_buslock_n;

wire _unused = &{1'b0, scr_qop[1]};

endmodule
