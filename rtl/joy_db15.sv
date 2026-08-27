//============================================================================
//  DB15 SNAC reader for the MiSTer user port
//
//  Based on "Control module for DB15 Splitter of Antonio Villena by Aitor
//  Pelaez (NeuroRulez)" as used in jotego's jtframe (joydb15.v), rewritten to
//  run entirely on a single clock with a clock enable instead of a divided
//  clock, and with the serial input synchronised.
//
//  The two pads are read out of a 74HC165 chain: JOY_LOAD is pulsed low to
//  latch the switches, then each JOY_CLK rising edge shifts the next bit out
//  on JOY_DATA.  Outputs are active high.
//
//      [3:0]  Up(3) Down(2) Left(1) Right(0)
//      [9:4]  A(4) B(5) C(6) D(7) E(8) F(9)
//      [10]   Start
//      [11]   Select
//      [15:12] unused, always 0
//============================================================================

module joy_db15
(
    input         clk,
    input         rst,

    output        JOY_CLK,
    output        JOY_LOAD,
    input         JOY_DATA,

    output [15:0] joystick1,
    output [15:0] joystick2
);

// Half period of JOY_CLK, in clk cycles.  The MiSTer user port pins are open
// drain with only a weak pull-up, so the line is driven slowly to give the
// rising edges time to settle.  At clk = 32MHz this gives a 62.5kHz JOY_CLK,
// so the whole 26 bit frame is read out ~2.4k times a second.
parameter DIV_BITS = 8;

reg [DIV_BITS-1:0] div = 0;
reg                joy_clk = 0;
wire               tick = &div;
wire               joy_clk_rise = tick & ~joy_clk;

always @(posedge clk) begin
    if (rst) begin
        div <= 0;
        joy_clk <= 0;
    end else begin
        div <= div + 1'd1;
        if (tick) joy_clk <= ~joy_clk;
    end
end

// JOY_DATA comes in off a cable, synchronise it before use
reg [1:0] data_sync = 2'b11;
always @(posedge clk) data_sync <= { data_sync[0], JOY_DATA };
wire joy_data = data_sync[1];

reg [15:0] joy1 = 16'hFFFF, joy2 = 16'hFFFF;
reg        joy_renew = 1'b1;
reg  [4:0] joy_count = 5'd0;

always @(posedge clk) begin
    if (rst) begin
        joy_count <= 5'd0;
        joy_renew <= 1'b1;
        joy1 <= 16'hFFFF;
        joy2 <= 16'hFFFF;
    end else if (joy_clk_rise) begin
        joy_renew <= (joy_count != 5'd0);
        joy_count <= (joy_count == 5'd25) ? 5'd0 : joy_count + 1'd1;

        case (joy_count)
            5'd2  : joy1[7]  <= joy_data;   // P1 D
            5'd3  : joy1[6]  <= joy_data;   // P1 C
            5'd4  : joy1[5]  <= joy_data;   // P1 B
            5'd5  : joy1[4]  <= joy_data;   // P1 A
            5'd6  : joy1[0]  <= joy_data;   // P1 Right
            5'd7  : joy1[1]  <= joy_data;   // P1 Left
            5'd8  : joy1[2]  <= joy_data;   // P1 Down
            5'd9  : joy1[3]  <= joy_data;   // P1 Up
            5'd10 : joy2[0]  <= joy_data;   // P2 Right
            5'd11 : joy2[1]  <= joy_data;   // P2 Left
            5'd12 : joy2[2]  <= joy_data;   // P2 Down
            5'd13 : joy2[3]  <= joy_data;   // P2 Up
            5'd14 : joy1[9]  <= joy_data;   // P1 F
            5'd15 : joy1[8]  <= joy_data;   // P1 E
            5'd16 : joy1[11] <= joy_data;   // P1 Select
            5'd17 : joy1[10] <= joy_data;   // P1 Start
            5'd18 : joy2[9]  <= joy_data;   // P2 F
            5'd19 : joy2[8]  <= joy_data;   // P2 E
            5'd20 : joy2[11] <= joy_data;   // P2 Select
            5'd21 : joy2[10] <= joy_data;   // P2 Start
            5'd22 : joy2[7]  <= joy_data;   // P2 D
            5'd23 : joy2[6]  <= joy_data;   // P2 C
            5'd24 : joy2[5]  <= joy_data;   // P2 B
            5'd25 : joy2[4]  <= joy_data;   // P2 A
            default:;
        endcase
    end
end

assign JOY_CLK  = joy_clk;
assign JOY_LOAD = joy_renew;

//----LS FEDCBAUDLR
assign joystick1 = ~joy1;
assign joystick2 = ~joy2;

endmodule
