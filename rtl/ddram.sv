//============================================================================
//  Copyright (C) 2026 Martin Donlon
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <https://www.gnu.org/licenses/>.
//============================================================================

interface ddr_if;
    logic        acquire;

    logic [31:0] addr;
    logic [63:0] wdata;
    logic [63:0] rdata;
    logic        read;
    logic        write;
    logic  [7:0] burstcnt;
    logic  [7:0] byteenable;
    logic        busy;
    logic        rdata_ready;

    modport to_host(
        output addr, wdata, read, write, burstcnt, byteenable, acquire,
        input rdata, busy, rdata_ready
    );

    modport from_host(
        output rdata, busy, rdata_ready,
        input addr, wdata, read, write, burstcnt, byteenable, acquire
    );


endinterface

module ddr_mux(
    input clk,

    ddr_if.to_host x,

    ddr_if.from_host a,
    ddr_if.from_host b
);

reg a_active = 0;

always_comb begin
    a.rdata = x.rdata;
    b.rdata = x.rdata;

    if (a_active) begin
        x.addr = a.addr;
        x.wdata = a.wdata;
        x.read = a.read;
        x.write = a.write;
        x.burstcnt = a.burstcnt;
        x.byteenable = a.byteenable;

        a.busy = x.busy;
        a.rdata_ready = x.rdata_ready;
        a.rdata = x.rdata;

        b.busy = 1;
        b.rdata_ready = 0;
    end else begin
        x.addr = b.addr;
        x.wdata = b.wdata;
        x.read = b.read;
        x.write = b.write;
        x.burstcnt = b.burstcnt;
        x.byteenable = b.byteenable;

        b.busy = x.busy;
        b.rdata_ready = x.rdata_ready;
        b.rdata = x.rdata;

        a.busy = 1;
        a.rdata_ready = 0;
    end
end

assign x.acquire = a.acquire | b.acquire;

always_ff @(posedge clk) begin
    if (a.acquire & ~b.acquire) a_active <= 1;
    if (~a.acquire & b.acquire) a_active <= 0;
end

endmodule


