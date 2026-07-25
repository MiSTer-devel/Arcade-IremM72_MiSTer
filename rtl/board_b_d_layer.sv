//============================================================================
//  Irem M72 for MiSTer FPGA - Background layer
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

module board_b_d_layer #(
    parameter SS_IDX_RAM0 = -1
) (
    input CLK_32M,
    input CE_PIX,

    input [15:0] DIN,
    output [15:0] DOUT,
    input [19:0] A,
    input RD,
    input WR,

    input [15:0] IO_DIN,
    input [1:0] IO_BE,

    input VSCK,
    input HSCK,
    input NL,

    input [8:0] VE,
    input [8:0] HE,

    output [3:0] BIT,
    output reg [3:0] COL,
    output reg CP15,
    output reg CP8,

    input [31:0] sdr_data,
    output [20:0] sdr_addr,
    output sdr_req,
    input sdr_rdy,

    input enabled,
    input paused,

    input m84,

    // savestate: one chunk per VRAM bank (indices SS_IDX_RAM0+0..3) plus the
    // scroll/pixel-pipeline registers
    ssbus_if.slave ssbus_ram0,
    ssbus_if.slave ssbus_ram1,
    ssbus_if.slave ssbus_ram2,
    ssbus_if.slave ssbus_ram3,
    ssbus_if.slave ssbus_regs,
    input ss_restore
);

localparam SS_IDX_REGS = SS_IDX_RAM0 + 4;

assign DOUT = A[1] ? { dout_11, dout_10 } : { dout_01 , dout_00 };

wire [7:0] dout_00, dout_01, dout_10, dout_11;

// Savestate access hijacks the scan-out port (B) of each bank; only active
// while the core is savestate-quiesced, so the worst case is a few frames of
// on-screen garbage during the streaming itself.
wire [11:0] scan_addr = {SV[8:3], SH[8:3]};

wire [11:0] ram_addr_b[4];
wire  [7:0] ram_data_b[4];
wire        ram_wren_b[4];

ram_ss_adaptor #(.WIDTH(8), .WIDTHAD(12), .SS_IDX(SS_IDX_RAM0 + 0)) ram_00_ss(
    .clk(CLK_32M),
    .wren_in(1'd0), .addr_in(scan_addr), .data_in(8'd0),
    .wren_out(ram_wren_b[0]), .addr_out(ram_addr_b[0]), .data_out(ram_data_b[0]),
    .q(ram_00_dout),
    .ssbus(ssbus_ram0)
);
ram_ss_adaptor #(.WIDTH(8), .WIDTHAD(12), .SS_IDX(SS_IDX_RAM0 + 1)) ram_01_ss(
    .clk(CLK_32M),
    .wren_in(1'd0), .addr_in(scan_addr), .data_in(8'd0),
    .wren_out(ram_wren_b[1]), .addr_out(ram_addr_b[1]), .data_out(ram_data_b[1]),
    .q(ram_01_dout),
    .ssbus(ssbus_ram1)
);
ram_ss_adaptor #(.WIDTH(8), .WIDTHAD(12), .SS_IDX(SS_IDX_RAM0 + 2)) ram_10_ss(
    .clk(CLK_32M),
    .wren_in(1'd0), .addr_in(scan_addr), .data_in(8'd0),
    .wren_out(ram_wren_b[2]), .addr_out(ram_addr_b[2]), .data_out(ram_data_b[2]),
    .q(ram_10_dout),
    .ssbus(ssbus_ram2)
);
ram_ss_adaptor #(.WIDTH(8), .WIDTHAD(12), .SS_IDX(SS_IDX_RAM0 + 3)) ram_11_ss(
    .clk(CLK_32M),
    .wren_in(1'd0), .addr_in(scan_addr), .data_in(8'd0),
    .wren_out(ram_wren_b[3]), .addr_out(ram_addr_b[3]), .data_out(ram_data_b[3]),
    .q(ram_11_dout),
    .ssbus(ssbus_ram3)
);

dualport_ram_unreg #(.WIDTHAD(12)) ram_00
(
    .clock_a(CLK_32M),
    .address_a(A[13:2]),
    .q_a(dout_00),
    .wren_a(WR & ~A[1]),
    .data_a(DIN[7:0]),

    .clock_b(CLK_32M),
    .address_b(ram_addr_b[0]),
    .data_b(ram_data_b[0]),
    .wren_b(ram_wren_b[0]),
    .q_b(ram_00_dout)
);

dualport_ram_unreg #(.WIDTHAD(12)) ram_01
(
    .clock_a(CLK_32M),
    .address_a(A[13:2]),
    .q_a(dout_01),
    .wren_a(WR & ~A[1]),
    .data_a(DIN[15:8]),

    .clock_b(CLK_32M),
    .address_b(ram_addr_b[1]),
    .data_b(ram_data_b[1]),
    .wren_b(ram_wren_b[1]),
    .q_b(ram_01_dout)
);

dualport_ram_unreg #(.WIDTHAD(12)) ram_10
(
    .clock_a(CLK_32M),
    .address_a(A[13:2]),
    .q_a(dout_10),
    .wren_a(WR & A[1]),
    .data_a(DIN[7:0]),

    .clock_b(CLK_32M),
    .address_b(ram_addr_b[2]),
    .data_b(ram_data_b[2]),
    .wren_b(ram_wren_b[2]),
    .q_b(ram_10_dout)
);

dualport_ram_unreg #(.WIDTHAD(12)) ram_11
(
    .clock_a(CLK_32M),
    .address_a(A[13:2]),
    .q_a(dout_11),
    .wren_a(WR & A[1]),
    .data_a(DIN[15:8]),

    .clock_b(CLK_32M),
    .address_b(ram_addr_b[3]),
    .data_b(ram_data_b[3]),
    .wren_b(ram_wren_b[3]),
    .q_b(ram_11_dout)
);

reg [31:0] rom_data;
wire [3:0] BITF, BITR;

kna6034201 kna6034201(
    .clock(CLK_32M),
    .CE_PIXEL(CE_PIX),
    .LOAD(SH[2:0] == 3'b111),
    .byte_1(enabled ? rom_data[7:0] : 8'h00),
    .byte_2(enabled ? rom_data[15:8] : 8'h00),
    .byte_3(enabled ? rom_data[23:16] : 8'h00),
    .byte_4(enabled ? rom_data[31:24] : 8'h00),
    .bit_1(BITF[0]),
    .bit_2(BITF[1]),
    .bit_3(BITF[2]),
    .bit_4(BITF[3]),
    .bit_1r(BITR[0]),
    .bit_2r(BITR[1]),
    .bit_3r(BITR[2]),
    .bit_4r(BITR[3])
);

wire [8:0] SV = VE + adj_v;
wire [8:0] SH = ( ( m84 ? HE - 9'd4 : HE ) + adj_h ) ^ { 6'b0, {3{NL}} };

reg [8:0] adj_v;
reg [8:0] adj_h;

reg HREV1, VREV, HREV2;
reg [15:0] COD;

wire [2:0] RV = SV[2:0] ^ {3{VREV}};

wire [7:0] ram_00_dout, ram_01_dout, ram_10_dout, ram_11_dout;
wire [15:0] attrib_0 = { ram_01_dout, ram_00_dout };
wire [15:0] attrib_1 = { ram_11_dout, ram_10_dout };


assign BIT = (HREV2 ^ NL) ? BITR : BITF;

reg [17:0] paused_offsets[512];
reg [8:0] ve_latch;

always @(posedge CLK_32M) begin
    ve_latch <= VE;
    if (ssbus_regs.access(SS_IDX_REGS) & ssbus_regs.write) begin
        // Savestate restore writes take priority (the paused replay below
        // would otherwise overwrite adj_v/adj_h the very next clock).
        case (ssbus_regs.addr[2:0])
        3'd0: adj_v <= ssbus_regs.data[8:0];
        3'd1: adj_h <= ssbus_regs.data[8:0];
        default: begin end
        endcase
    end else if (paused) begin
        // During a savestate restore paused_offsets holds pre-restore (or, in
        // a fresh process, uninitialized) data; the replay would clobber the
        // restored adj_v/adj_h, so it is gated until the core resumes.  The
        // recording resumes on unpause and repopulates within one frame.
        if (~ss_restore) {adj_v, adj_h} <= paused_offsets[ve_latch];
    end else begin
        if (VSCK & IO_BE[0]) adj_v[7:0] <= IO_DIN[7:0];
        if (HSCK & IO_BE[0]) adj_h[7:0] <= IO_DIN[7:0];
        if (VSCK & IO_BE[1]) adj_v[8]   <= IO_DIN[8];
        if (HSCK & IO_BE[1]) adj_h[8]   <= IO_DIN[8];
        paused_offsets[ve_latch] <= {adj_v, adj_h};
    end
end

// Savestate regs slave.  COD and the pixel-pipeline latches free-run with
// CE_PIX during pause, so their restore writes land in the pipeline block
// below with top priority; enumeration/reads/acks live here.
always @(posedge CLK_32M) begin
    ssbus_regs.setup(SS_IDX_REGS, 5, 1);

    if (ssbus_regs.access(SS_IDX_REGS)) begin
        if (ssbus_regs.write) begin
            ssbus_regs.write_ack(SS_IDX_REGS);
        end else if (ssbus_regs.read) begin
            case (ssbus_regs.addr[2:0])
            3'd0: ssbus_regs.read_response(SS_IDX_REGS, { 55'd0, adj_v });
            3'd1: ssbus_regs.read_response(SS_IDX_REGS, { 55'd0, adj_h });
            3'd2: ssbus_regs.read_response(SS_IDX_REGS, { 48'd0, COD });
            3'd3: ssbus_regs.read_response(SS_IDX_REGS, { 57'd0, HREV1, VREV, HREV2, COL });
            default: ssbus_regs.read_response(SS_IDX_REGS, { 62'd0, CP15, CP8 });
            endcase
        end
    end
end

always @(posedge CLK_32M) begin
    reg do_rom;

    sdr_req <= 0;
    do_rom <= 0;

    if (do_rom) begin
        sdr_addr <= {COD[15:0], RV[2:0], 2'b00};
        sdr_req <= 1;
    end else if (sdr_rdy) begin
        rom_data <= sdr_data;
    end

    // Savestate restore writes for the free-running pixel-pipeline latches
    if (ssbus_regs.access(SS_IDX_REGS) & ssbus_regs.write) begin
        case (ssbus_regs.addr[2:0])
        3'd2: COD <= ssbus_regs.data[15:0];
        3'd3: { HREV1, VREV, HREV2, COL } <= ssbus_regs.data[6:0];
        3'd4: { CP15, CP8 } <= ssbus_regs.data[1:0];
        default: begin end
        endcase
    end else if (CE_PIX) begin
        if (SH[2:0] == 2'b001) begin
            if (m84) begin
                COD <= attrib_0;
                {VREV, HREV1} <= attrib_1[6:5];
            end else begin
                COD <= { 2'b00, attrib_0[13:0]};
                { VREV, HREV1 } <= attrib_0[15:14];
            end
            do_rom <= 1;
        end
        if (SH[2:0] == 3'b111) begin
            COL <= attrib_1[3:0];
            if (m84) begin
                CP15 <= attrib_1[8];
                CP8 <= attrib_1[7];
            end else begin
                CP15 <= attrib_1[7];
                CP8 <= attrib_1[6];
            end
            HREV2 <= HREV1;
        end
    end
end


endmodule