/*  This file is part of JT51.

    JT51 is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    JT51 is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with JT51.  If not, see <http://www.gnu.org/licenses/>.
    
    Author: Jose Tejada Gomez. Twitter: @topapate
    Version: 1.0
    Date: 27-10-2016
    */

// The rst scheme makes this module synthesizable as LUT shift register. This
// saves 18% of logic cells at JT51 top level (3347 vs 2747)
// for the reset value to be applied, the reset must be enabled for as many
// clock cycles as stages are in the shift register
module jt51_sh #(parameter width=5, stages=32, rstval=1'b0 ) (
    input                           rst,
    input                           clk,
    input                           cen,
    input       [width-1:0]         din,
    output      [width-1:0]         drop
);

reg [stages-1:0] bits[width-1:0];

// Local change (kept across jt51 updates): the clocked shift lives at module
// scope rather than inside the genvar loop.  Functionally identical to the
// upstream per-genvar always, but it lets util/state_module.py inject its
// restore-write and read-mux once at module scope instead of replicating them
// per genvar bit, which produced a multidriven auto_ss_data_out and crippled
// sim speed.  Mirrors the jt12_sh structure the generator is known to handle.
integer k;
always @(posedge clk) if(cen) begin
    for (k=0; k < width; k=k+1)
        bits[k] <= {bits[k][stages-2:0], din[k]};
end

genvar i;
generate
    for (i=0; i < width; i=i+1) begin: bit_shifter
        assign drop[i] = rst ? rstval[0] : bits[i][stages-1];
    end
endgenerate

endmodule
