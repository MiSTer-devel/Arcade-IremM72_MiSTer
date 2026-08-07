//============================================================================
// Imported from nec_test hdl/rtl/ucore at HEAD
// 29dcc5b05fdc0bf6e72e05b0c309b859c9a41d20 (2026-08-07); the ucore
// directory's last content change is 5403671558a5c20ff4036c6d59ebd06124f3fd78.
// The M72 integration keeps this upstream module name and interface.
//============================================================================
//
//============================================================================
//
//  v30_core (ucore) - the ROM-driven NEC V30 (uPD70116) core, max mode.
//
//  This is the ucore TOP.  It is a DROP-IN alternative to hdl/rtl/core/
//  v30_core.sv: same module name, same port list, same package name
//  (`v30_ss_pkg`, supplied here by hdl/rtl/ucore/v30u_ss_pkg.sv), so the two
//  cores are selected by the RTL FILE LIST alone -- `sw/check_core.py
//  --core {fsm,ucore}` and, later, hdl/files_ucore.qip.  hdl/tb/tb_v30_core.sv
//  is not parameterised on the engine.
//
//  Governance: hdl/rtl/ucore/README.md.  The correctness target is "identical
//  to sim/ clock-for-clock"; the BIU below is a transliteration of
//  sim/biu_timed.{h,cpp}, mechanism by mechanism.
//
//  STAGE U1: BIU only.  `v30u_eu` is a tied-off placeholder and the EU side is
//  held inert exactly as the FSM core's scripted-consumer mode holds it.
//
//  SCRIPTED-CONSUMER MODE (V30_BACKDOOR, verification only).  `scr_en` hands
//  the queue port to `scr_qop`, using the QS encoding as the command:
//      2'b00  idle
//      2'b01  pop, and this pop is an F (instruction first byte)
//      2'b11  pop, and this pop is an S (subsequent byte)
//      2'b10  FLUSH + redirect to bkd_cs : bkd_fetch_ip
//  A pop is a DEMAND: it is served on the first clock the front byte is ripe,
//  which is M8's `pop = max(demand, ready)`.  The FSM core's scripted mode has
//  no flush command (its `q_flush` is tied low under `scr_en`); ucore spends
//  the otherwise-unused E encoding on it so the F1/F3/M12/M19 flush family is
//  reachable without an EU.  Recorded in docs/notes/ucore_provenance.md.
//
//============================================================================

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
    output     [19:0] AD_OE,     // the pads' own output enable (task #37)
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

//----------------------------------------------------------------------------
// BIU <-> EU
//----------------------------------------------------------------------------
wire  [7:0] q_byte;
wire        q_ripe, q_ripe_lead_n;
wire  [3:0] q_cnt;
wire        eu_ghost_full, eu_ghost_idle, eu_ghost_stack_first, eu_rd_wait;
wire        eu_pop, eu_first, eu_flush;
wire        eu_flush_pre, eu_flush_rep, eu_flush_stage, eu_flush_pend;
wire        eu_flush_nmi, eu_flush_int_live;
wire [15:0] eu_flush_cs, eu_flush_cs_old, eu_flush_ip;
wire        eu_flush_cs_we;
wire        eu_post, eu_post_hold, eu_ghost_preview, eu_halt_irq, eu_vector_post;
wire        eu_word, eu_pair, eu_pair2, eu_split;
wire        eu_slot_busy, eu_slot_busy_n, eu_access_active;
wire        eu_direct_fetch, eu_fetch_tail;
wire  [2:0] eu_bs;
wire [19:0] eu_addr, eu_addr2;
wire  [1:0] eu_seg, eu_seg2;
wire [15:0] eu_wdata, eu_rdata_n;
wire        eu_rd_done_n, eu_wr_done_n, eu_wr_eval;
wire        eu_rd_edge;
wire [15:0] eu_rd_edge_d;
wire        eu_opr_free;
wire        eu_susp, eu_resume, eu_halt, eu_unhalt, biu_halted;
wire        eu_unhalt_disp;                       // F43 (SM3 sitting 6)
wire        eu_bnd_take, eu_bnd_post;   // SM3 sitting 11 (H1's remains)
wire        psw_ie, md8080;
wire [15:0] ss_eu_rdata, ss_biu_rdata;
wire        ss_biu_bus_quiet;
reg   [8:0] ss_addr_q;
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
    // SS command staging has drained.  ss_we_q still high at a CE cycle means
    // the core takes its `if (ss_we)` branch and SKIPS its state advance -> a
    // phantom wait on resume.
    if (CE && ss_we_q)  $error("CE resumed with SS command staging undrained (ss_we_q high)");
end
`endif

//----------------------------------------------------------------------------
// scripted-consumer override (BIU-only verification) -- see the header
//----------------------------------------------------------------------------
wire q_pop   = scr_en ? (scr_qop == 2'b01 || scr_qop == 2'b11) : eu_pop;
wire q_first = scr_en ? (scr_qop == 2'b01)                     : eu_first;
wire q_flush = scr_en ? (scr_qop == 2'b10)                     : eu_flush;
wire [15:0] flush_cs = scr_en ? bkd_regs[144 +: 16] : eu_flush_cs;
wire [15:0] flush_ip = scr_en ? bkd_fetch_ip        : eu_flush_ip;

wire [19:0] ad_o;
wire        ad_oe_addr, ad_oe_ps, ad_oe_data;

v30u_biu u_biu (
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
    .qs         (QS),
    .ad_i       (AD[15:0]),
    .ready      (READY),
    .psw_ie     (psw_ie),
    .md8080     (md8080),
    .q_byte     (q_byte),
    .q_ripe     (q_ripe),
    .q_ripe_lead_n(q_ripe_lead_n),
    .q_cnt_o    (q_cnt),
    .q_pop      (q_pop),
    .q_first    (q_first),
    .q_flush    (q_flush),
    .flush_pre  (scr_en ? 1'b0 : eu_flush_pre),
    .flush_rep  (scr_en ? 1'b0 : eu_flush_rep),
    .flush_stage(scr_en ? 1'b0 : eu_flush_stage),
    .flush_pend (scr_en ? 1'b0 : eu_flush_pend),
    .flush_nmi  (scr_en ? 1'b0 : eu_flush_nmi),
    .flush_int_live(scr_en ? 1'b0 : eu_flush_int_live),
    .flush_cs   (flush_cs),
    .flush_cs_old(eu_flush_cs_old),
    .flush_cs_we(scr_en ? 1'b0 : eu_flush_cs_we),
    .flush_ip   (flush_ip),
    .eu_post    (scr_en ? 1'b0 : eu_post),
    .eu_post_hold(scr_en ? 1'b0 : eu_post_hold),
    .eu_ghost_preview(scr_en ? 1'b0 : eu_ghost_preview),
    .eu_halt_irq(scr_en ? 1'b0 : eu_halt_irq),
    .eu_vector_post(scr_en ? 1'b0 : eu_vector_post),
    .eu_bs      (eu_bs),
    .eu_addr    (eu_addr),
    .eu_addr2   (eu_addr2),
    .eu_split   (eu_split),
    .eu_seg     (eu_seg),
    .eu_seg2    (eu_seg2),
    .eu_word    (eu_word),
    .eu_slot_busy (eu_slot_busy),
    .eu_slot_busy_n (eu_slot_busy_n),
    .eu_access_active(eu_access_active),
    .eu_direct_fetch(eu_direct_fetch),
    .eu_fetch_tail(eu_fetch_tail),
    .eu_ghost_full(eu_ghost_full),
    .eu_ghost_idle(eu_ghost_idle),
    .eu_ghost_stack_first(eu_ghost_stack_first),
    .eu_rd_wait(eu_rd_wait),
    .eu_pair    (scr_en ? 1'b0 : eu_pair),
    .eu_pair2   (eu_pair2),
    .eu_wdata   (eu_wdata),
    .eu_rdata_n (eu_rdata_n),
    .eu_rd_done_n (eu_rd_done_n),
    .eu_rd_edge (eu_rd_edge),
    .eu_rd_edge_d (eu_rd_edge_d),
    .eu_wr_done_n (eu_wr_done_n),
    .eu_wr_eval   (eu_wr_eval),
    .eu_opr_free(eu_opr_free),
    .eu_susp    (scr_en ? 1'b0 : eu_susp),
    .eu_resume  (scr_en ? 1'b0 : eu_resume),
    .eu_halt    (scr_en ? 1'b0 : eu_halt),
    .eu_unhalt  (scr_en ? 1'b0 : eu_unhalt),
    .eu_unhalt_disp(scr_en ? 1'b0 : eu_unhalt_disp),
    .halted_o   (biu_halted),
    .eu_bnd_take(scr_en ? 1'b0 : eu_bnd_take),
    .eu_bnd_post(eu_bnd_post),
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

v30u_eu u_eu (
    .clk        (CLK),
    .ce         (CE),
    .srst       (RESET),
    .q_byte     (q_byte),
    .q_ripe     (q_ripe),
    .q_ripe_lead_n(q_ripe_lead_n),
    .q_cnt      (q_cnt),
    .q_pop      (eu_pop),
    .q_first    (eu_first),
    .q_flush    (eu_flush),
    .flush_pre  (eu_flush_pre),
    .flush_rep  (eu_flush_rep),
    .flush_stage(eu_flush_stage),
    .flush_pend (eu_flush_pend),
    .flush_nmi  (eu_flush_nmi),
    .flush_int_live(eu_flush_int_live),
    .flush_cs   (eu_flush_cs),
    .flush_cs_old(eu_flush_cs_old),
    .flush_cs_we(eu_flush_cs_we),
    .flush_ip   (eu_flush_ip),
    .eu_post    (eu_post),
    .eu_post_hold(eu_post_hold),
    .eu_ghost_preview(eu_ghost_preview),
    .eu_halt_irq(eu_halt_irq),
    .eu_vector_post(eu_vector_post),
    .eu_bs      (eu_bs),
    .eu_addr    (eu_addr),
    .eu_addr2   (eu_addr2),
    .eu_split   (eu_split),
    .eu_seg     (eu_seg),
    .eu_seg2    (eu_seg2),
    .eu_word    (eu_word),
    .eu_slot_busy (eu_slot_busy),
    .eu_slot_busy_n (eu_slot_busy_n),
    .eu_access_active(eu_access_active),
    .eu_direct_fetch(eu_direct_fetch),
    .eu_fetch_tail(eu_fetch_tail),
    .eu_ghost_full(eu_ghost_full),
    .eu_ghost_idle(eu_ghost_idle),
    .eu_ghost_stack_first(eu_ghost_stack_first),
    .eu_rd_wait(eu_rd_wait),
    .eu_pair    (eu_pair),
    .eu_pair2   (eu_pair2),
    .eu_wdata   (eu_wdata),
    .eu_rdata_n (eu_rdata_n),
    .eu_rd_done_n (eu_rd_done_n),
    .eu_rd_edge (eu_rd_edge),
    .eu_rd_edge_d (eu_rd_edge_d),
    .eu_wr_done_n (eu_wr_done_n),
    .eu_wr_eval   (eu_wr_eval),
    .eu_opr_free(eu_opr_free),
    .eu_susp    (eu_susp),
    .eu_resume  (eu_resume),
    .eu_halt    (eu_halt),
    .eu_unhalt  (eu_unhalt),
    .eu_unhalt_disp(eu_unhalt_disp),
    .halted     (biu_halted),
    .eu_bnd_take(eu_bnd_take),
    .eu_bnd_post(eu_bnd_post),
    .psw_ie     (psw_ie),
    .md8080     (md8080),
    .pin_int    (INT),
    .pin_nmi    (NMI),
    .pin_poll_n (POLL_N),
    .bkd_load   (bkd_load),
    .bkd_regs   (bkd_regs),
`ifdef V30_BACKDOOR
    .dbg_regs      (dbg_regs),
    .dbg_first_pop (dbg_first_pop),
    .dbg_pend      (dbg_pend),
`else
    /* verilator lint_off PINCONNECTEMPTY */
    .dbg_regs      (),
    .dbg_first_pop (),
    .dbg_pend      (),
    /* verilator lint_on PINCONNECTEMPTY */
`endif
    .ss_addr    (ss_addr_q),
    .ss_wdata   (ss_wdata_q),
    .ss_we      (ss_we_q),
    .ss_rdata   (ss_eu_rdata)
);

// AD drive (simple en?val:'z forms only - Verilator requirement).
// AD[19:16] carry the address during address phases and PS3-0 during the
// data phase; AD[15:0] additionally carry write data.
assign AD[15:0]  = (ad_oe_addr | ad_oe_data) ? ad_o[15:0]  : 16'hzzzz;
assign AD[19:16] = (ad_oe_addr | ad_oe_ps)   ? ad_o[19:16] : 4'hz;

// AD_OE -- THE PADS' OWN OUTPUT ENABLE, PUBLISHED (task #37, user-approved
// 2026-08-04).  This is a WIRE, not a rule: it is bit-for-bit the SAME
// expression the two `assign AD[...]` statements above already use to decide
// whether to drive, so it adds no logic and cannot disagree with the drive.
// The real part has two pad-enable groups (AD0-15, A16-19/PS0-3) and so does
// this; the 20-bit form just replicates them so a consumer needs no knowledge
// of where the groups split.
//
// WHY IT EXISTS.  ucore_provenance.md §56.3a's retention intervention needs an
// "is anyone driving" term, and §56.3a forbids MANUFACTURING one in the
// harness (that would be a fitted rule in the exact place the intervention
// must not have one).  It does not forbid the core TELLING the truth: the
// V30's pads have an output enable, so publishing it is the part's own
// structure, not the instrument's.  §59.7.1's blocker -- Quartus 17.1 folds
// `net === 1'bz` on an internal tri-state and deletes the retention register
// -- is removed by construction, because the model no longer asks the net.
assign AD_OE = {{4{ad_oe_addr | ad_oe_ps}}, {16{ad_oe_addr | ad_oe_data}}};

// BUSLOCK is not implemented (inherited scope note; the FSM core drives it
// from the EU's LOCK prefix, which U2 restores).
assign BUSLOCK_N = 1'b1;

endmodule
