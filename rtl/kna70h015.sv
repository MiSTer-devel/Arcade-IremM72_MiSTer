module kna70h015 (
    input DCLK,

    input [15:0] D,
    input ISET,
    input NL, // TODO
    input S24H, // TODO

    output CLD_UNKNOWN, // TODO
    output CPBLK, // TODO
    output reg [8:0] VE,
    output reg [8:0] V,
    output reg [9:0] HE,
    output reg [9:0] H,

    // These are inputs from two PROMs on the original board, but combined into this module here
    output reg HBLK,
    output reg VBLK,
    output reg INT_D, // TODO
    output reg HINT, // TODO

    // Output from PROMs in original board
    output reg HS,
    output reg VS
);

reg [8:0] v_count;
reg [8:0] h_int_line;
reg [9:0] h_count;

/* From MAME
Legend of Hero TONMA
(c)1989 Irem

M72 System
Horizontal Freq. = 15.625KHz
H.Period         = 64.0us (512)
H.Blank          = 16.0us (128)
H.Sync Pulse     = 5.0us (40)
Vertical Freq.   = 55.02Hz
V.Period         = 18.176ms (284)
V.Blank          = 1.792ms (28)
V.Sync Pulse     = 384us (6)

*/

always @(posedge DCLK) begin
    if (ISET) h_int_line <= D[8:0];

    h_count <= h_count + 10'd1;
    H <= h_count;
    HE <= 0;

    HINT <= 0;

    if (h_count < 64) begin
        HBLK <= 1;
    end else if (h_count < 448) begin
        HBLK <= 0;
        HE <= h_count - 10'd64;
    end else if (h_count == 10'd511) begin
        v_count <= v_count + 9'd1;
    end else begin
        HBLK <= 1;
    end

    V <= v_count;
    VE <= 0;
    VBLK <= 1;

    HINT <= v_count == ( h_int_line - 9'd128 );

    if (v_count < 9'd256) begin
        VE <= v_count;
        VBLK <= 0;
    end else if (v_count == 9'd283) begin
        v_count <= 9'd0;
    end

    HS <= (h_count < 10'd20 || h_count > 10'd490 );
    VS <= (v_count >= 9'd270 && v_count < 9'd276 );
end


endmodule