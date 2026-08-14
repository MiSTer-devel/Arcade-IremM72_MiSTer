//============================================================================
//  Irem M72 for MiSTer FPGA - V30 core bus adapter
//
//  Wraps the cycle-accurate nec_test v30_core and presents a lean,
//  word-aligned bus to m72.v: address+byte-enables, separate read/write
//  strobes for memory and IO, an INTA/vector handshake, and a code-fetch flag
//  (BS==CODE) so the sim DebugLink can tell prefetch from data reads.
//
//  --- THE DE-MUXED CORE (2026-08-14) --------------------------------------
//  Built with `V30_MUXED_AD` UNDEFINED, so the core publishes the three
//  quantities the real part's 20 pins share as ports of their own -- ADDR_O /
//  DATA_O / STATUS_O -- and takes read data on DATA_I.  What that deletes from
//  this adapter is the whole pin-multiplex layer:
//
//    * the `tri [19:0] AD` net, its `drive_en` one-shot and the tri-state
//      assign that drove read data back onto it;
//    * `ad_q` / `ube_n_q`, the free-running pin copies;
//    * the T1 address-phase capture, which existed ONLY to sample AD before
//      the core switched it from address to write data.  There is no
//      turnaround to race any more, so there is nothing to be early for;
//    * `CE_HALF`, which existed only to mark that turnaround.  The core now
//      takes ONE clock enable.
//
//  What stays is the max-mode bus-controller model, because the core still
//  announces with BS and derives nothing else: the T-state tracker is what
//  turns {BS, READY} into m72.v's strobe levels.  There is deliberately no
//  "address valid" pin on the core -- that would be a pin the die does not
//  have -- so this module still derives `addr_valid` itself.
//
//  Timing model (CE-freeze with catch-up):
//    ce : advances a T-state; 8MHz average rate.  The core is frozen by
//    withholding it while an SDRAM access is outstanding, then catches up
//    (m72.v owns that pacing).  READY (m72.v-driven) inserts real Tw states
//    for the sprite/tile RAM wait-state feature; it is orthogonal to the
//    SDRAM CE-freeze.
//
//  Distilled from nec_test/hdl/rtl/nec_bus.sv T-state tracker (large mode,
//  minus wait-states / random / capture / power sequencing / harness), and
//  the v30_core instantiation in nec_test/hdl/rtl/system_large.sv.
//============================================================================

module v30_bus #(
    parameter SS_IDX = -1
) (
    input             clk,
    input             ce,          // CPU-clock advance strobe (T-state)
    input             reset,       // active high
    input             ready,       // V30 READY: low at T3/Tw inserts Tw (M72 wait-states)

    // word-aligned unified bus
    output     [19:0] cpu_addr,    // bit0 forced 0
    output      [1:0] cpu_be,      // [0]=low byte (A0==0), [1]=high byte (~UBE_N)
    output     [15:0] cpu_dout,    // write data (valid during a write cycle)
    input      [15:0] cpu_din,     // read data (sampled by the core at T3/T4)

    // access strobes
    output            mem_rd,      // level, T2 .. T3 (CODE or MEMR)
    output            io_rd,       // level, T2 .. T3 (IOR)
    output            mem_wr_pending, // address-valid MEMW, for early READY generation
    output            mem_wr,      // level, held during T3 (MEMW)
    output            io_wr,       // level, held during T3 (IOW)
    output            code_fetch,  // current cycle is a prefetch (BS==CODE)

    // interrupt handshake
    input             int_req,
    input       [7:0] int_vector,
    output            int_ack,     // level, held during T3 of the 2nd INTA

    // full register file for the sim CPU window (zeros unless V30_BACKDOOR)
    output    [223:0] dbg_regs,

    // savestate: streams the core's 228-entry SS register file
    ssbus_if.slave    ssbus,
    input             ss_restore_done, // pulse: reset this adapter to bus-idle
    output            ss_quiet         // core BIU is bus-quiet (safe to freeze)
);

import v30_ss_pkg::*;

// Bus status codes (8086 S2-S0 compatible)
localparam bit [2:0] BS_INTA = 3'b000;
localparam bit [2:0] BS_IOR  = 3'b001;
localparam bit [2:0] BS_IOW  = 3'b010;
localparam bit [2:0] BS_HALT = 3'b011;
localparam bit [2:0] BS_CODE = 3'b100;
localparam bit [2:0] BS_MEMR = 3'b101;
localparam bit [2:0] BS_MEMW = 3'b110;
localparam bit [2:0] BS_PASV = 3'b111;

// T-state encoding (ST_TW value matches the BIU's, v30u_biu.sv ST_TW=3'd4)
localparam bit [2:0] ST_TI = 3'd0;
localparam bit [2:0] ST_T1 = 3'd1;
localparam bit [2:0] ST_T2 = 3'd2;
localparam bit [2:0] ST_T3 = 3'd3;
localparam bit [2:0] ST_TW = 3'd4;
localparam bit [2:0] ST_T4 = 3'd5;

//----------------------------------------------------------------------------
// Core instance (de-muxed bus)
//----------------------------------------------------------------------------
wire  [2:0] BS;
wire        RD_N, UBE_N;
wire [19:0] ADDR_O;
wire [15:0] DATA_O;
wire  [3:0] STATUS_O;
reg  [15:0] rdata_q;

`ifdef V30_BACKDOOR
wire [223:0] core_dbg_regs;
assign dbg_regs = core_dbg_regs;
`else
assign dbg_regs = 224'd0;
`endif

// Savestate regfile access.  Contract (v30_core.sv assertions): SS_WE only
// while CE is withheld (the core is frozen whenever the ssbus is active),
// exactly one clk per entry, and the 2-deep staging must drain before CE
// resumes (the m72 controller's post-restore drain covers this).  SS_RDATA
// is valid 2 clks after SS_ADDR presents.
wire [8:0]  ss_core_addr = ss_addr_of(int'(ssbus.addr));
wire [15:0] ss_core_rdata;
reg  [1:0]  ss_rd_delay;
reg         ss_wr_done;
reg         ss_tag_reject;
wire        ss_core_is_tag = (ss_core_addr == SSA_TAG);
// Always pass the tag so a later compatible restore can clear a sticky
// mismatch.  Once a bad tag is seen, acknowledge but discard the remainder
// of that CPU section rather than loading an incompatible register map.
wire        ss_core_we = ssbus.access(SS_IDX) & ssbus.write & ~ss_wr_done &
                         (~ss_tag_reject | ss_core_is_tag);
wire        ss_err /* verilator public_flat */;

v30_core u_core (
    .CLK        (clk),
    .CE         (ce),
    .RESET      (reset),
    .READY      (ready),
    .INT        (int_req),
    .NMI        (1'b0),
    .POLL_N     (1'b1),
    .DATA_I     (rdata_q),
    .ADDR_O     (ADDR_O),
    .DATA_O     (DATA_O),
    .STATUS_O   (STATUS_O),
    .QS         (),
    .BS         (BS),
    .RD_N       (RD_N),
    .UBE_N      (UBE_N),
    .BUSLOCK_N  (),
    .SS_ADDR    (ss_core_addr),
    .SS_WDATA   (ssbus.data[15:0]),
    .SS_WE      (ss_core_we),
    .SS_RDATA   (ss_core_rdata),
    .SS_ERR     (ss_err),
    .SS_BUS_QUIET(ss_quiet)
`ifdef V30_BACKDOOR
    ,
    .bkd_load     (1'b0),
    .bkd_regs     (224'd0),
    .bkd_queue    (48'd0),
    .bkd_qlen     (3'd0),
    .bkd_fetch_ip (16'd0),
    .scr_en       (1'b0),
    .scr_qop      (2'd0),
    .dbg_regs     (core_dbg_regs),
    .dbg_first_pop(),
    .dbg_pend     ()
`endif
);

//----------------------------------------------------------------------------
// Savestate slave: the ucore's 228 x 16-bit regfile entries streamed via the
// ssbus. The package supplies the dense index-to-address mapping.
// Reads respect the 2-clk SS_ADDR->SS_RDATA staging; writes pulse SS_WE for
// exactly one clk per entry (ss_wr_done holds it off while the master waits
// for the ack to propagate through the mux).
//----------------------------------------------------------------------------
always_ff @(posedge clk) begin
    ssbus.setup(SS_IDX, v30_ss_pkg::SS_COUNT, 1);

    if (reset)
        ss_tag_reject <= 1'b0;
    else if (ss_core_we && ss_core_is_tag)
        ss_tag_reject <= (ssbus.data[15:0] != SS_TAG);

    if (ssbus.access(SS_IDX)) begin
        if (ssbus.write) begin
            ss_wr_done <= 1;
            ssbus.write_ack(SS_IDX);
        end else if (ssbus.read) begin
            ss_rd_delay <= { ss_rd_delay[0], 1'b1 };
            if (ss_rd_delay[1]) begin
                ssbus.read_response(SS_IDX, { 48'd0, ss_core_rdata });
            end
        end
    end else begin
        ss_rd_delay <= 0;
        ss_wr_done <= 0;
    end
end

//----------------------------------------------------------------------------
// Registered BS sample (mirrors nec_bus: status is one clk old at the FSM
// edge, reproducing the verified end-of-cycle timing relationship).  BS only
// changes on a `ce` edge, so at the next `ce` this carries the announcement
// the tracker is meant to act on; the register is what keeps the announcement
// out of the tracker's own combinational cone.
//----------------------------------------------------------------------------
reg  [2:0] bs_q;
always_ff @(posedge clk) bs_q <= BS;

//----------------------------------------------------------------------------
// T-state tracker (advances under ce)
//----------------------------------------------------------------------------
reg  [2:0] t_state;
reg  [2:0] lat_type;
reg        is_read_cycle, is_write_cycle;
reg        addr_valid;
reg        inta_prev, inta_second;
reg [19:0] addr_lat;
reg [19:0] addr_ann;   // ADDR_O captured at T1 entry
reg  [1:0] be_lat;

wire bs_active = bs_q != BS_PASV;

wire [2:0] next_t =
    (t_state == ST_TI) ? (bs_active ? ST_T1 : ST_TI) :
    (t_state == ST_T1) ? ST_T2 :
    (t_state == ST_T2) ? ST_T3 :
    (t_state == ST_T3) ? (ready ? ST_T4 : ST_TW) : // READY low -> Tw (mirrors BIU)
    (t_state == ST_TW) ? (ready ? ST_T4 : ST_TW) : // loop Tw until READY sampled high
    /* ST_T4 */          (bs_active ? ST_T1 : ST_TI);

wire read_type  = (bs_q == BS_CODE) || (bs_q == BS_MEMR) ||
                  (bs_q == BS_IOR)  || (bs_q == BS_INTA);
wire write_type = (bs_q == BS_MEMW) || (bs_q == BS_IOW);

always_ff @(posedge clk) begin
    if (reset || ss_restore_done) begin
        // On ss_restore_done the adapter returns to bus-idle: the savestate
        // quiesce guarantees no cycle was in flight at save time, so every
        // other adapter register is dead state until the next T1.
        t_state        <= ST_TI;
        lat_type       <= BS_PASV;
        is_read_cycle  <= 1'b0;
        is_write_cycle <= 1'b0;
        addr_valid     <= 1'b0;
        inta_prev      <= 1'b0;
        inta_second    <= 1'b0;
    end else begin
        if (ce) begin
            t_state <= next_t;

            // `ADDR_O` IS CAPTURED AT T1 ENTRY, `UBE_N` INSIDE T1, AND THAT
            // IS NOT A STYLE CHOICE -- the two do not change owner together.
            //
            // `ADDR_O` follows the ANNOUNCEMENT: at the `ce` that enters T1
            // the announcement for this cycle is still up and it already
            // carries the address about to run, and the core re-points it at
            // the NEXT cycle as soon as THAT is announced, which can be inside
            // this cycle.  `UBE_N` is the running cycle's pin and only becomes
            // this cycle's at the entering edge itself.
            //
            // MEASURED, both directions, against the muxed build's 6,886
            // accesses: capturing both at entry gives correct addresses and
            // the PREVIOUS cycle's byte enables (first wrong at access 14);
            // capturing both inside T1 gives correct byte enables and picks up
            // a later announcement's address (first wrong at access 93).  Each
            // at its own instant is the only combination that reproduces the
            // muxed build exactly.
            if (next_t == ST_T1) begin
                lat_type       <= bs_q;
                is_read_cycle  <= read_type;
                is_write_cycle <= write_type;
                addr_ann       <= ADDR_O;
                if (bs_q == BS_INTA) begin
                    inta_second <= inta_prev;
                    inta_prev   <= 1'b1;
                end else begin
                    inta_second <= 1'b0;
                    inta_prev   <= 1'b0;
                end
            end

            // THE ADDRESS AND BYTE ENABLES ARE CAPTURED *INSIDE* T1, on the
            // `ce` that leaves it -- not on the one that enters it.
            //
            // `ADDR_O` and `UBE_N` do NOT change owner together.  At the
            // entering edge the announcement is still up, so `ADDR_O` already
            // carries the cycle about to run while `UBE_N` still carries the
            // one just finished: latching there gets a correct address with
            // the PREVIOUS cycle's byte enables.  MEASURED -- addresses
            // matched the muxed build on every access and `cpu_be` did not,
            // from the 14th access on.  One `ce` later both are the running
            // cycle's (`r_cur_addr` / `r_cur_ube_n`) and they agree.
            //
            // This is the same CONTENT the muxed build captured off the pins
            // during T1; only the instant moves, from mid-T1 (it had `ce_half`
            // to land on) to the end of T1.
            if (t_state == ST_T1) begin
                addr_lat   <= {addr_ann[19:1], 1'b0};
                be_lat     <= {~UBE_N, ~addr_ann[0]};
                addr_valid <= 1'b1;
            end

            if (next_t == ST_T4 || next_t == ST_TI)
                addr_valid <= 1'b0;
        end
    end
end

// Read data into the core's DATA_I.  Registered exactly as it was when it was
// driven onto AD, so the value the core samples at T3/T4 is unchanged; with no
// shared pad there is no drive enable and it is simply always presented.
// INTA cycles present the vector number in the low byte.
always_ff @(posedge clk)
    rdata_q <= (lat_type == BS_INTA) ? {8'h00, int_vector} : cpu_din;

//----------------------------------------------------------------------------
// Outputs
//----------------------------------------------------------------------------
assign cpu_addr   = addr_lat;
assign cpu_be     = be_lat;
// `DATA_O` is the owning cycle's write word and is meaningful from that
// cycle's T1 through its T4, which covers every instant `mem_wr` / `io_wr`
// below can be asserted, so it needs no latch of its own.
assign cpu_dout   = DATA_O;
assign code_fetch = (lat_type == BS_CODE);

// read strobes: level from T2 through T3 (addr_valid), cleared at T4
assign mem_rd = addr_valid && ((lat_type == BS_CODE) || (lat_type == BS_MEMR));
assign io_rd  = addr_valid && (lat_type == BS_IOR);

// The ucore evaluates READY before entering T3, so wait-generating devices
// need to see a write request as soon as the T1 address has been latched.
// Keep this separate from mem_wr: consumers must still commit writes only in
// T3/Tw, when the core is driving valid write data.
assign mem_wr_pending = addr_valid && (lat_type == BS_MEMW);

// write strobes: level held across T3 and any Tw. At zero waits there is no Tw
// so this is the single-T3 pulse as before; under waits it holds the request so
// the wait-stated sprite/tile RAM can commit the write at its own gate.
assign mem_wr = (lat_type == BS_MEMW) && (t_state == ST_T3 || t_state == ST_TW);
assign io_wr  = (lat_type == BS_IOW)  && (t_state == ST_T3 || t_state == ST_TW);

// INTA acknowledge: level held during T3 of the second INTA cycle
assign int_ack = (lat_type == BS_INTA) && inta_second && (t_state == ST_T3);

endmodule
