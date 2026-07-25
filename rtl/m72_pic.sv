//============================================================================
//  Irem M72 for MiSTer FPGA - Programmable interrupt controller
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

module m72_pic #(
    parameter SS_IDX = -1
) (
    input clk,
    input ce,
    input reset,

    input cs,
    input wr,
    input rd,
    input a0,

    input [7:0] din,

    output reg int_req,
    output reg [7:0] int_vector,
    input int_ack,

    input [7:0] intp,

    ssbus_if.slave ssbus
);

typedef enum bit [2:0] {
    UNINIT,
    INIT_IW2,
    INIT_IW3,
    INIT_IW4,
    INIT_DONE
} state_t;
state_t init_state = UNINIT;

reg [7:0] IW1, IW2, IW3, IW4;
reg [7:0] IMW, IRR, ISR;
reg [7:0] PFCW;
reg [7:0] MCW;

wire iw4_write = IW1[0];
wire iw4_not_written = ~IW1[0];
wire single_mode = IW1[1];
wire extended_mode = ~IW1[1];
wire address_gap_4 = IW1[2];
wire address_gap_8 = ~IW1[2];
wire level_triggered = IW1[3];
wire edge_triggered = ~IW1[3];

reg [7:0] intp_latch = 0;

always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
        init_state <= UNINIT;
        int_req <= 0;
        intp_latch <= 0;
    end else if (ssbus.access(SS_IDX) & ssbus.write) begin
        // Savestate restore writes (the ce-gated body below is frozen)
        case (ssbus.addr[3:0])
        4'd0: IW1 <= ssbus.data[7:0];
        4'd1: IW2 <= ssbus.data[7:0];
        4'd2: IW3 <= ssbus.data[7:0];
        4'd3: IW4 <= ssbus.data[7:0];
        4'd4: IMW <= ssbus.data[7:0];
        4'd5: PFCW <= ssbus.data[7:0];
        4'd6: MCW <= ssbus.data[7:0];
        4'd7: IRR <= ssbus.data[7:0];
        4'd8: ISR <= ssbus.data[7:0];
        4'd9: intp_latch <= ssbus.data[7:0];
        4'd10: init_state <= state_t'(ssbus.data[2:0]);
        default: {int_req, int_vector} <= ssbus.data[8:0];
        endcase
    end else if (ce) begin
        if (cs & wr) begin
            if (~a0) begin
                if (din[4]) begin
                    init_state <= INIT_IW2;
                    IW1 <= din;
                    PFCW <= 0;
                    MCW <= 0;
                    IMW <= 0;
                    IRR <= 0;
                    ISR <= 0;
                end else if (~din[4] & ~din[3]) begin
                    PFCW <= din;
                end else if (~din[4] & din[3]) begin
                    MCW <= din;
                end
            end

            if (a0) begin
                case (init_state)
                INIT_IW2: begin
                    IW2 <= din;
                    if (extended_mode) init_state <= INIT_IW3;
                    else if (iw4_write) init_state <= INIT_IW4;
                    else init_state <= INIT_DONE;
                end
                INIT_IW3: begin
                    IW3 <= din;
                    if (iw4_write) init_state <= INIT_IW4;
                    else init_state <= INIT_DONE;
                end
                INIT_IW4: begin
                    IW4 <= din;
                    init_state <= INIT_DONE;
                end
                INIT_DONE: begin
                    IMW <= din;
                end
                endcase
            end
        end

        if (init_state == INIT_DONE) begin
            intp_latch <= intp;

            if (int_req) begin
                if (int_ack) begin
                    int_req <= 0;
                end
            end else begin
                bit [7:0] trig;
                int p;
                bit t;

                if (edge_triggered)
                    trig = intp & ~intp_latch;
                else
                    trig = intp;
                
                t = 0;
                for( p = 0; p < 8 && !t; p = p + 1 ) begin
                    if (intp[p]) begin
                        if (trig[p] & ~IMW[p]) begin
                            int_req <= 1;
                            // Full 8-bit vector NUMBER (was a byte offset that
                            // dropped IW2[7] and appended 2'b00 for the VHDL core).
                            int_vector <= {IW2[7:3], p[2:0]};
                        end
                        t = 1;
                    end
                end
            end
        end
    end
end

// Savestate slave: enumeration, reads and acks (writes live above)
always_ff @(posedge clk) begin
    ssbus.setup(SS_IDX, 12, 1);

    if (ssbus.access(SS_IDX)) begin
        if (ssbus.write) begin
            ssbus.write_ack(SS_IDX);
        end else if (ssbus.read) begin
            case (ssbus.addr[3:0])
            4'd0: ssbus.read_response(SS_IDX, { 56'd0, IW1 });
            4'd1: ssbus.read_response(SS_IDX, { 56'd0, IW2 });
            4'd2: ssbus.read_response(SS_IDX, { 56'd0, IW3 });
            4'd3: ssbus.read_response(SS_IDX, { 56'd0, IW4 });
            4'd4: ssbus.read_response(SS_IDX, { 56'd0, IMW });
            4'd5: ssbus.read_response(SS_IDX, { 56'd0, PFCW });
            4'd6: ssbus.read_response(SS_IDX, { 56'd0, MCW });
            4'd7: ssbus.read_response(SS_IDX, { 56'd0, IRR });
            4'd8: ssbus.read_response(SS_IDX, { 56'd0, ISR });
            4'd9: ssbus.read_response(SS_IDX, { 56'd0, intp_latch });
            4'd10: ssbus.read_response(SS_IDX, { 61'd0, init_state });
            default: ssbus.read_response(SS_IDX, { 55'd0, int_req, int_vector });
            endcase
        end
    end
end

endmodule