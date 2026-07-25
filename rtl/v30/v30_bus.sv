//============================================================================
//  Irem M72 for MiSTer FPGA - V30 core bus adapter
//
//  Wraps the cycle-accurate nec_test v30_core (max-mode, muxed AD/BS bus) and
//  presents a lean, word-aligned bus to m72.v: address+byte-enables, separate
//  read/write strobes for memory and IO, an INTA/vector handshake, and a
//  code-fetch flag (BS==CODE) so the sim DebugLink can tell prefetch from data
//  reads.
//
//  Timing model (CE-freeze with catch-up):
//    ce      : advances a T-state; 8MHz average rate
//    ce_half : the T1 address-latch strobe, one fabric clock after each ce
//  The core is frozen by withholding ce/ce_half while an SDRAM access is
//  outstanding, then catches up at one phase per fabric clock (16MHz burst)
//  until it matches the steady 8MHz reference (m72.v owns that pacing).
//  READY is tied high (no Tw).
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
    input             ce_half,     // T1 address-latch strobe (2 clks after ce)
    input             reset,       // active high

    // word-aligned unified bus
    output     [19:0] cpu_addr,    // bit0 forced 0
    output      [1:0] cpu_be,      // [0]=low byte (A0==0), [1]=high byte (~UBE_N)
    output     [15:0] cpu_dout,    // write data (valid during a write cycle)
    input      [15:0] cpu_din,     // read data (sampled by the core at T3/T4)

    // access strobes
    output            mem_rd,      // level, T1-half .. T4 (CODE or MEMR)
    output            io_rd,       // level, T1-half .. T4 (IOR)
    output            mem_wr,      // level, held during T3 (MEMW)
    output            io_wr,       // level, held during T3 (IOW)
    output            code_fetch,  // current cycle is a prefetch (BS==CODE)

    // interrupt handshake
    input             int_req,
    input       [7:0] int_vector,
    output            int_ack,     // level, held during T3 of the 2nd INTA

    // full register file for the sim CPU window (zeros unless V30_BACKDOOR)
    output    [223:0] dbg_regs,

    // savestate: streams the core's 202-entry SS register file
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

// T-state encoding (TW unused, READY tied high)
localparam bit [2:0] ST_TI = 3'd0;
localparam bit [2:0] ST_T1 = 3'd1;
localparam bit [2:0] ST_T2 = 3'd2;
localparam bit [2:0] ST_T3 = 3'd3;
localparam bit [2:0] ST_T4 = 3'd5;

//----------------------------------------------------------------------------
// Core instance (private muxed AD net)
//----------------------------------------------------------------------------
tri  [19:0] AD;
wire  [2:0] BS;
wire        RD_N, UBE_N;
reg  [15:0] rdata_q;
reg         drive_en;

// harness drives read/INTA data onto AD[15:0] during read cycles
assign AD[15:0]  = drive_en ? rdata_q : 16'hzzzz;

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
wire        ss_core_we = ssbus.access(SS_IDX) & ssbus.write & ~ss_wr_done;
wire        ss_err /* verilator public_flat */;

v30_core u_core (
    .CLK        (clk),
    .CE         (ce),
    .CE_HALF    (ce_half),
    .RESET      (reset),
    .READY      (1'b1),
    .INT        (int_req),
    .NMI        (1'b0),
    .POLL_N     (1'b1),
    .AD         (AD),
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
// Savestate slave: 202 x 16-bit regfile entries streamed via the ssbus.
// Reads respect the 2-clk SS_ADDR->SS_RDATA staging; writes pulse SS_WE for
// exactly one clk per entry (ss_wr_done holds it off while the master waits
// for the ack to propagate through the mux).
//----------------------------------------------------------------------------
always_ff @(posedge clk) begin
    ssbus.setup(SS_IDX, v30_ss_pkg::SS_COUNT, 1);

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
// Registered pin samples (mirrors nec_bus: status/data are one clk old at the
// FSM edge, reproducing the verified end-of-cycle timing relationship).
//----------------------------------------------------------------------------
reg  [2:0] bs_q;
reg        ube_n_q;
reg [19:0] ad_q;

always_ff @(posedge clk) begin
    bs_q    <= BS;
    ube_n_q <= UBE_N;
    ad_q    <= AD;
end

// T1 address-phase pin capture on the FALLING clock edge (the tb_v30_core.sv
// scheme): AD holds the address and UBE_N is stable across the half-clock
// between the T1-entering CE and the core's CE_HALF processing edge.
// Posedge-delayed copies can belong to the neighbouring bus cycle when
// cycles run back-to-back (string ops corrupted their byte enables), and
// live posedge sampling races the core's switch to the data phase.
reg [19:0] addr_neg;
reg        ube_neg;
always_ff @(negedge clk) begin
    if (ce_half && t_state == ST_T1) begin
        addr_neg <= AD;
        ube_neg  <= UBE_N;
    end
end

//----------------------------------------------------------------------------
// T-state tracker (advances under ce)
//----------------------------------------------------------------------------
reg  [2:0] t_state;
reg  [2:0] lat_type;
reg        is_read_cycle, is_write_cycle;
reg        addr_valid;
reg        inta_prev, inta_second;
reg [19:0] addr_lat;
reg  [1:0] be_lat;
reg [15:0] dout_lat;

wire bs_active = bs_q != BS_PASV;

wire [2:0] next_t =
    (t_state == ST_TI) ? (bs_active ? ST_T1 : ST_TI) :
    (t_state == ST_T1) ? ST_T2 :
    (t_state == ST_T2) ? ST_T3 :
    (t_state == ST_T3) ? ST_T4 :             // READY==1: never Tw
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
        drive_en       <= 1'b0;
    end else begin
        // drive read data onto AD one clk after entering T2 (nec_bus timing)
        if (ce) begin
            t_state <= next_t;

            if (next_t == ST_T1) begin
                lat_type       <= bs_q;
                is_read_cycle  <= read_type;
                is_write_cycle <= write_type;
                if (bs_q == BS_INTA) begin
                    inta_second <= inta_prev;
                    inta_prev   <= 1'b1;
                end else begin
                    inta_second <= 1'b0;
                    inta_prev   <= 1'b0;
                end
            end

            if (next_t == ST_T2 && is_read_cycle)
                drive_en <= 1'b1;

            // capture write data while the core drives it (T2/T3)
            if ((t_state == ST_T2 || t_state == ST_T3) && is_write_cycle)
                dout_lat <= ad_q[15:0];

            if (next_t == ST_T4 || next_t == ST_TI)
                drive_en <= 1'b0;
        end

        // Address / byte-enable latch on the T1 half-cycle strobe, from the
        // negedge-captured pin samples (see below) - race-free against both
        // the address-drive and the data-phase switch.
        if (ce_half && t_state == ST_T1) begin
            addr_lat   <= {addr_neg[19:1], 1'b0};
            be_lat     <= {~ube_neg, ~addr_neg[0]};
            addr_valid <= 1'b1;
        end else if (ce && (next_t == ST_T4 || next_t == ST_TI)) begin
            addr_valid <= 1'b0;
        end
    end
end

// read data mux: INTA cycles present the vector number on AD[7:0]
always_ff @(posedge clk)
    rdata_q <= (lat_type == BS_INTA) ? {8'h00, int_vector} : cpu_din;

//----------------------------------------------------------------------------
// Outputs
//----------------------------------------------------------------------------
assign cpu_addr   = addr_lat;
assign cpu_be     = be_lat;
assign cpu_dout   = dout_lat;
assign code_fetch = (lat_type == BS_CODE);

// read strobes: level from T1-half through T3 (addr_valid), cleared at T4
assign mem_rd = addr_valid && ((lat_type == BS_CODE) || (lat_type == BS_MEMR));
assign io_rd  = addr_valid && (lat_type == BS_IOR);

// write strobes: level held for the single T3 CPU-clock (exactly one ce edge)
assign mem_wr = (lat_type == BS_MEMW) && (t_state == ST_T3);
assign io_wr  = (lat_type == BS_IOW)  && (t_state == ST_T3);

// INTA acknowledge: level held during T3 of the second INTA cycle
assign int_ack = (lat_type == BS_INTA) && inta_second && (t_state == ST_T3);

endmodule
