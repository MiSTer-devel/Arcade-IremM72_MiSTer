
//============================================================================
//  Irem M72 for MiSTer FPGA - Dualport memory with mailbox functionality
//
//  Copyright (C) 2022 Martin Donlon
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 2 of the License, or (at your option)
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//============================================================================

// Based on the MB8421

// Left port is 16-bit, right port in 8-bit
// 
module dualport_mailbox_2kx16 #(
    parameter SS_IDX = -1
) (
    input reset,

    input clk_l,
    input cs_l,
    input [11:1] addr_l,
    input [15:0] din_l,
    output [15:0] dout_l,
    input [1:0] we_l,
    output int_l,

    input clk_r,
    input cs_r,
    input [11:0] addr_r,
    input [7:0] din_r,
    output [7:0] dout_r,
    input we_r,
    output int_r,

    // savestate: SSIDX_MCU_MAILBOX.  Streamed over the LEFT (16-bit) port,
    // which the V30 has parked during a savestate pause.  Words 0..2047 are
    // the RAM (both byte lanes forced); word 2048 is the packed 4-bit
    // interrupt handshake {int_l_rq,int_l_ack,int_r_rq,int_r_ack}.
    ssbus_if.slave ssbus
);

wire [7:0] dout_0_l, dout_1_l;
wire [7:0] dout_0_r, dout_1_r;

// Savestate left-port hijack.  addr 0..2047 -> RAM word (both lanes), the
// handshake word 2048 is served in the blocks below.
wire        ss_acc    = ssbus.access(SS_IDX);
wire        ss_ram    = ss_acc & (ssbus.addr < 2048);
wire        ss_hs_wr  = ss_acc & ssbus.write & (ssbus.addr == 2048);
wire [10:0] addr_a_ss = ss_ram ? ssbus.addr[10:0] : addr_l[11:1];
wire  [7:0] data0_a_ss = ss_ram ? ssbus.data[7:0]  : din_l[7:0];
wire  [7:0] data1_a_ss = ss_ram ? ssbus.data[15:8] : din_l[15:8];
wire        wr0_a_ss   = ss_ram ? ssbus.write : we_l[0];
wire        wr1_a_ss   = ss_ram ? ssbus.write : we_l[1];

assign dout_l = { dout_1_l, dout_0_l };
assign dout_r = addr_r[0] ? dout_1_r : dout_0_r;

assign int_l = int_l_rq != int_l_ack;
assign int_r = int_r_rq != int_r_ack;

reg int_l_rq = 0;
reg int_l_ack = 0;
reg int_r_rq = 0;
reg int_r_ack = 0;

always @(posedge clk_l or posedge reset) begin
    if (reset) begin
        int_l_ack <= 0;
        int_r_rq <= 0;
    end else begin
        if (cs_l) begin
            // The 16-bit left port is a pair of MB8421s; the MCU-side interrupt
            // comes from the high-byte (odd address) device only: byte 0xfff is
            // the command byte (the dbreed i8751 polls it for changes), and the
            // CPU writes 0xffe before 0xfff, so ringing on the 0xfff write makes
            // command + interrupt atomic.  Ringing on either lane double-triggers
            // the MCU when the CPU's byte writes straddle the MCU's acknowledge.
            if (we_l[1] && addr_l[11:1] == 'h7ff) int_r_rq <= ~int_r_ack;
            if (we_l == 2'b00 && addr_l[11:1] == 'h7fe) int_l_ack <= int_l_rq;
        end
        // Savestate restore of the two clk_l-owned handshake bits (frozen under
        // pause, so no live event competes).  Highest priority.
        if (ss_hs_wr) begin
            int_l_ack <= ssbus.data[2];
            int_r_rq  <= ssbus.data[1];
        end
    end
end

always @(posedge clk_r or posedge reset) begin
    if (reset) begin
        int_l_rq <= 0;
        int_r_ack <= 0;
    end else begin
        if (cs_r) begin
            if (we_r && addr_r[11:1] == 'h7fe) int_l_rq <= ~int_l_ack;
            if (~we_r && addr_r[11:1] == 'h7ff) int_r_ack <= int_r_rq;
        end
        // Savestate restore of the two clk_r-owned handshake bits.
        if (ss_hs_wr) begin
            int_l_rq  <= ssbus.data[3];
            int_r_ack <= ssbus.data[0];
        end
    end
end

// Savestate slave protocol: enumeration, RAM/handshake reads, acks.  RAM
// writes force both byte lanes via the port-A mux above; handshake writes
// land in the two clocked blocks above (single driver per bit).
reg ram_rd_delay = 1'b0;
always @(posedge clk_l) begin
    ssbus.setup(SS_IDX, 2049, 1);   // 2048 RAM words + 1 handshake word

    if (ss_acc) begin
        if (ssbus.write) begin
            ssbus.write_ack(SS_IDX);
        end else if (ssbus.read) begin
            if (ssbus.addr == 2048) begin
                ssbus.read_response(SS_IDX,
                    { 60'd0, int_l_rq, int_l_ack, int_r_rq, int_r_ack });
            end else begin
                // dpramv registered read: data valid 1 clk after address
                if (ram_rd_delay)
                    ssbus.read_response(SS_IDX, { 48'd0, dout_1_l, dout_0_l });
                ram_rd_delay <= 1'b1;
            end
        end
    end else begin
        ram_rd_delay <= 1'b0;
    end
end


dpramv #(.widthad_a(11)) ram_0(
    .clock_a(clk_l),
    .address_a(addr_a_ss),
    .q_a(dout_0_l),
    .wren_a(wr0_a_ss),
    .data_a(data0_a_ss),

    .clock_b(clk_r),
    .address_b(addr_r[11:1]),
    .q_b(dout_0_r),
    .wren_b(we_r & ~addr_r[0]),
    .data_b(din_r)
);

dpramv #(.widthad_a(11)) ram_1(
    .clock_a(clk_l),
    .address_a(addr_a_ss),
    .q_a(dout_1_l),
    .wren_a(wr1_a_ss),
    .data_a(data1_a_ss),

    .clock_b(clk_r),
    .address_b(addr_r[11:1]),
    .q_b(dout_1_r),
    .wren_b(we_r & addr_r[0]),
    .data_b(din_r)
);

endmodule