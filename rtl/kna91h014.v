//============================================================================
//  Irem M72 for MiSTer FPGA - Palette chip
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

module kna91h014 #(
    parameter SS_IDX = -1
) (
    input CLK_32M,

    input [7:0] CB,	// Pins 3-10.
    input [7:0] CA,	// Pins 11-18.
    
    input SELECT,	// Pin 50. "S"
    
    input E1_N,		// Pin 52.
    input E2_N,		// Pin 51. CBLK.

    input G,		// Pin 30. G_N.
    
    input MWR,	// Pin 29.
    input MRD,	// Pin 28.

    input [15:0] DIN,	// Pins 25, 22-19 (split to input for Verilog).
    output [15:0] DOUT,	// Pins 25, 22-19 (split to output for Verilog).
    output DOUT_VALID,
    
    input [19:0] A,	// Pins 53-60

    output reg [4:0] RED,	// Pins 47-43.
    output reg [4:0] GRN,	// Pins 42-40, 37-36.
    output reg [4:0] BLU,	// Pins 35-31.

    ssbus_if.slave ssbus
);

wire [7:0] A_IN = A[8:1];
wire [2:0] A_S = { A[11], A[10], A[0] };

reg [7:0] color_addr;

always @(posedge CLK_32M) begin
    color_addr <= SELECT ? CA : CB;
end

// Palette RAMs...
reg [4:0] ram_a [256];
reg [4:0] ram_b [256];
reg [4:0] ram_c [256];

// RAM Addr decoding...
wire ram_a_cs = A_S==3'b000 | A_S==3'b110;
wire ram_b_cs = A_S==3'b010;
wire ram_c_cs = A_S==3'b100;

// Write enable, and addr decoding for RAM writes.
wire wr_ena = G & MWR;
wire rd_ena = G & MRD;

wire ram_wr_a = ram_a_cs & wr_ena;
wire ram_wr_b = ram_b_cs & wr_ena;
wire ram_wr_c = ram_c_cs & wr_ena;

reg [4:0] red_lat;
reg [4:0] grn_lat;
reg [4:0] blu_lat;

always @(posedge CLK_32M)
begin
    // Savestate restore writes take priority; CPU writes are impossible in
    // that window (MWR frozen while paused).
    if (ssbus.access(SS_IDX) & ssbus.write) begin
        ram_a[ssbus.addr[7:0]] <= ssbus.data[4:0];
        ram_b[ssbus.addr[7:0]] <= ssbus.data[9:5];
        ram_c[ssbus.addr[7:0]] <= ssbus.data[14:10];
    end else begin
        if (ram_wr_a)
            ram_a[A_IN] <= DIN[4:0];
        else
            red_lat <= ram_a[A_IN];

        if (ram_wr_b)
            ram_b[A_IN] <= DIN[4:0];
        else
            grn_lat <= ram_b[A_IN];

        if (ram_wr_c)
            ram_c[A_IN] <= DIN[4:0];
        else
            blu_lat <= ram_c[A_IN];
    end
end

// DOUT read driver...
assign DOUT = { 11'd0,
    (ram_a_cs) ? red_lat :
    (ram_b_cs) ? grn_lat :
    (ram_c_cs) ? blu_lat : 5'h00 };
assign DOUT_VALID = rd_ena;

// Latch RAM outputs...

always @(posedge CLK_32M) begin
    if (~G) begin
        RED <= ram_a[color_addr];
        GRN <= ram_b[color_addr];
        BLU <= ram_c[color_addr];
    end
end

// Savestate slave: streams the three 5-bit palette planes as one 16-bit word
// per entry ({0, blu, grn, red}).  The array writes live in the block above;
// this block does the enumeration, reads and acks.
reg [14:0] ss_rdata;
reg ss_read_delay;

always @(posedge CLK_32M) begin
    ss_rdata <= { ram_c[ssbus.addr[7:0]], ram_b[ssbus.addr[7:0]], ram_a[ssbus.addr[7:0]] };

    ssbus.setup(SS_IDX, 256, 1);

    if (ssbus.access(SS_IDX)) begin
        if (ssbus.write) begin
            ssbus.write_ack(SS_IDX);
        end else if (ssbus.read) begin
            if (ss_read_delay) begin
                ssbus.read_response(SS_IDX, { 49'd0, ss_rdata });
            end
            ss_read_delay <= 1;
        end
    end else begin
        ss_read_delay <= 0;
    end
end

endmodule
