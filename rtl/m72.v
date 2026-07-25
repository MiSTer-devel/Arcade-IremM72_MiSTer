//============================================================================
//  Irem M72 for MiSTer FPGA - Main module
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

import m72_pkg::*;

module m72 (
    input CLK_32M,
    input CLK_96M,

    input reset_n,
    output reg ce_pix,

    input board_cfg_t board_cfg,
    
    input z80_reset_n,

    output [7:0] R,
    output [7:0] G,
    output [7:0] B,

    output HSync,
    output VSync,
    output HBlank,
    output VBlank,

    output [15:0] AUDIO_L,
    output [15:0] AUDIO_R,

    input [1:0] coin,
    input [1:0] start_buttons,
    input [3:0] p1_joystick,
    input [3:0] p2_joystick,
    input [3:0] p1_buttons,
    input [3:0] p2_buttons,
    input service_button,
    input [15:0] dip_sw,

    input pause_rq,

    output [24:0] sdr_sprite_addr,
    input [63:0] sdr_sprite_dout,
    output sdr_sprite_req,
    input sdr_sprite_rdy,

    output [24:0] sdr_bg_addr,
    input [31:0] sdr_bg_dout,
    output sdr_bg_req,
    input sdr_bg_rdy,

    output [24:0] sdr_cpu_addr,
    input [15:0] sdr_cpu_dout,
    output [15:0] sdr_cpu_din,
    output sdr_cpu_req,
    output sdr_cpu_mem_rq,
    input sdr_cpu_rdy,
    output [1:0] sdr_cpu_wr_sel,

    input [16:0] hs_address,
    output [7:0] hs_data_out,
    output hs_data_ready,
    input [7:0] hs_data_in,
    input hs_read_enable,
    input hs_write_enable,
    

    input clk_bram,
    input bram_wr,
    input [7:0] bram_data,
    input [19:0] bram_addr,
    input [4:0] bram_cs,

    input en_layer_a,
    input en_layer_b,
    input en_sprites,
    input en_layer_palette,
    input en_sprite_palette,
    input en_audio_filters,

    input sprite_freeze,

    input video_timing_t video_timing,

    output ddr_debug_data_t ddr_debug_data,

    // Full V30 register file for the sim CPU window (zeros unless V30_BACKDOOR).
    output [223:0] dbg_v30_regs,
    // code_fetch flag latched with the last CPU SDRAM request (sim DebugLink).
    output reg sdr_cpu_code
);

// Divide 32Mhz clock by 4 for pixel clock
reg paused = 0;
reg [8:0] paused_v;
reg [9:0] paused_h;

always @(posedge CLK_32M) begin
    if (pause_rq & ~paused) begin
        if (~ls245_en & ~DBEN & ~mem_rq_active) begin
            paused <= 1;
            paused_v <= V;
            paused_h <= H;
        end
    end else if (~pause_rq & paused) begin
        paused <= ~(V == paused_v && H == paused_h);
    end
end

reg ce_cpu, ce_cpu_half, ce_mcu;
wire ce_mcu_nopause;

// CE-freeze stall: withhold the CPU clock train while an SDRAM access is
// outstanding. The new-request term covers the 1-clk gap between a request
// being decoded and mem_rq_active going high, so the core never advances a
// T-state mid-request. The CE train free-runs during reset (the v30_core
// needs a running clock to execute its boot/reset-vector sequence); the core
// itself is held in reset via reset_n.
wire cpu_stall = mem_rq_active | cpu_new_sdr_req;

// Steady-rate CPU clock with catch-up (ce_steady mechanism, as in the IGSPGM
// core): a free-running reference counts the target 8MHz CPU clocks; the CPU
// counter chases it, issuing CE/CE_HALF phases back-to-back (one per fabric
// clock, 16MHz effective) whenever it is behind. SDRAM stalls therefore
// defer CPU cycles instead of losing them, and the average CPU rate stays at
// exactly 8MHz - the real board has no such stalls. The reference pauses
// with `paused` so no backlog accumulates across the pause feature.
reg [1:0] ce_steady_div;
reg [9:0] ce_steady_count;
reg [10:0] ce_cpu_count; // [0] selects the CE (0) / CE_HALF (1) phase

always @(posedge CLK_32M) begin
    ce_cpu <= 0;
    ce_cpu_half <= 0;
    ce_mcu <= 0;

    if (~paused) begin
        ce_steady_div <= ce_steady_div + 2'd1;
        if (&ce_steady_div) ce_steady_count <= ce_steady_count + 10'd1;

        if (~cpu_stall && ce_cpu_count[10:1] != ce_steady_count) begin
            ce_cpu      <= ~ce_cpu_count[0];
            ce_cpu_half <=  ce_cpu_count[0];
            ce_cpu_count <= ce_cpu_count + 11'd1;
        end

        ce_mcu <= reset_n ? ce_mcu_nopause : 1'b0;
    end
end

wire ce_pix_half, ce_mcu_half;
jtframe_frac_cen #(2) pixel_cen
(
    .clk(CLK_32M),
    .n(10'd1),
    .m(10'd4),
    .cen({ce_pix_half, ce_pix})
);

// The mc8051 core executes roughly one instruction per cen, but a real 8751
// takes 12 oscillator clocks per machine cycle.  Divide the 8MHz oscillator
// rate by 12 (cen = 32M/48) so the MCU runs at authentic speed relative to
// the cycle-accurate V30 - the dbreed i8751 handshake is timing sensitive.
// TODO: scale the 57/60Hz alternates the same way (n/m are 10-bit, needs
// reduced fractions).
jtframe_frac_cen #(2) mcu_cen
(
    .clk(CLK_32M),
    .n(10'd1),
    .m(video_timing == VIDEO_57HZ ? 10'd50 : video_timing == VIDEO_60HZ ? 10'd52 : 10'd48),
    .cen({ce_mcu_half, ce_mcu_nopause})
);

wire clock = CLK_32M;

/* Global signals from schematics. Strobes come straight from the v30_bus
 * adapter now: reads are levels (T1-half..T4), writes are held during T3. The
 * bus is word-aligned (cpu_mem_addr[0]==0) with per-byte enables in cpu_be, so
 * the old word_shuffle / cpu_word_* realignment machinery is gone. */
wire mem_rd, mem_wr, io_rd, io_wr, code_fetch;

wire IOWR = io_wr; // IO Write
wire IORD = io_rd; // IO Read
wire MWR = mem_wr; // Mem Write
wire MRD = mem_rd; // Mem Read
wire DBEN = io_rd | io_wr | mem_rd | mem_wr;

wire TNSL;

wire m84 = board_cfg.m84;

wire [19:0] cpu_mem_addr;   // word-aligned CPU address (bit0 == 0)
wire [1:0]  cpu_be;         // [0]=low byte (A0==0), [1]=high byte (~UBE_N)
wire [15:0] cpu_dout;       // CPU write data
reg  [15:0] cpu_din;        // read data muxed back to the core (combinational)

reg cpu_mem_read_lat, cpu_mem_write_lat;
reg hs_mem_read_lat, hs_mem_write_lat;
wire cpu_mem_read = mem_rd | cpu_mem_read_lat;
wire cpu_mem_write = mem_wr | cpu_mem_write_lat;
wire hs_mem_read = hs_read_enable | hs_mem_read_lat;
wire hs_mem_write = hs_write_enable | hs_mem_write_lat;

reg [15:0] cpu_ram_rom_data;
wire [24:0] cpu_region_addr;
wire cpu_region_writable;

wire bg_a_memrq;
wire bg_b_memrq;
wire bg_palette_memrq;
wire sprite_memrq;
wire sprite_palette_memrq;
wire sound_memrq;

reg mem_rq_active = 0;
assign sdr_cpu_mem_rq = mem_rq_active;

// A CPU SDRAM access is being decoded this cycle (mem_rq not yet asserted).
// Feeds cpu_stall so the core is frozen across the 1-clk request-detect gap.
wire cpu_new_sdr_req = ls245_en &&
    ((mem_rd & ~cpu_mem_read_lat) || (mem_wr & ~cpu_mem_write_lat));

reg b_d_dout_valid_lat, obj_pal_dout_valid_lat, sound_dout_valid_lat, sprite_dout_valid_lat;

always @(posedge CLK_32M or negedge reset_n)
begin
    if (!reset_n) begin
        b_d_dout_valid_lat <= 0;
        obj_pal_dout_valid_lat <= 0;
        sound_dout_valid_lat <= 0;
        sprite_dout_valid_lat <= 0;
    end else begin
        cpu_mem_read_lat <= mem_rd;
        cpu_mem_write_lat <= mem_wr;
        hs_mem_read_lat <= hs_read_enable;
        hs_mem_write_lat <= hs_write_enable;

        b_d_dout_valid_lat <= b_d_dout_valid;
        obj_pal_dout_valid_lat <= obj_pal_dout_valid;
        sound_dout_valid_lat <= sound_dout_valid;
        sprite_dout_valid_lat <= sprite_dout_valid;
    end
end

reg sdr_cpu_rq, sdr_cpu_ack, sdr_cpu_rq2;
reg sdr_hs_req_active;

reg [16:0] hs_address2;
always_ff @(posedge CLK_96M) begin
    sdr_cpu_req <= 0;
    if (sdr_cpu_rdy) sdr_cpu_ack <= sdr_cpu_rq;
    if (sdr_cpu_rq != sdr_cpu_rq2) begin
        sdr_cpu_req <= 1;
        sdr_cpu_rq2 <= sdr_cpu_rq;
    end
end


always_ff @(posedge CLK_32M or negedge reset_n) begin
    if (!reset_n) begin
        mem_rq_active <= 0;
    end else begin
        hs_data_ready <= 0;
        if (!mem_rq_active) begin
            if (ls245_en && ((mem_rd & ~cpu_mem_read_lat) || (mem_wr & ~cpu_mem_write_lat))) begin // sdram request
                sdr_cpu_wr_sel <= 2'b00;
                sdr_cpu_addr <= cpu_region_addr;
                sdr_cpu_code <= code_fetch;
                if (cpu_mem_write & cpu_region_writable ) begin
                    sdr_cpu_wr_sel <= cpu_be;
                    sdr_cpu_din <= cpu_dout;
                end
                sdr_cpu_rq <= ~sdr_cpu_rq;
                mem_rq_active <= 1;
              end else if ((hs_read_enable & ~hs_mem_read_lat) || (hs_write_enable & ~hs_mem_write_lat)) begin
                sdr_cpu_wr_sel <= 2'b00;
                sdr_cpu_addr <= REGION_CPU_RAM.base_addr[24:0] | hs_address[16:0];
                if (hs_mem_write) begin
                  sdr_cpu_wr_sel <= {hs_address[0], ~hs_address[0]};
                  sdr_cpu_din <= {hs_data_in, hs_data_in};
                end
                sdr_cpu_rq <= ~sdr_cpu_rq;
                hs_address2 <= hs_address;
                mem_rq_active <= 1;
                sdr_hs_req_active <= 1; 
              end
        end else if (sdr_cpu_rq == sdr_cpu_ack) begin
            if (sdr_hs_req_active) begin
              hs_data_out <= hs_address[0] ? sdr_cpu_dout[15:8] : sdr_cpu_dout[7:0];
              mem_rq_active <= 0;
              sdr_hs_req_active <= 0;
              hs_data_ready <= 1; 
            end else begin
              cpu_ram_rom_data <= sdr_cpu_dout;
              mem_rq_active <= 0;
            end

        end
    end
end

wire ls245_en, rom0_ce, rom1_ce, ram_cs2;


wire [15:0] switches = { p2_buttons, p2_joystick, p1_buttons, p1_joystick };
wire [15:0] flags = { 8'hff, TNSL, 1'b1, 1'b1 /*TEST*/, 1'b1 /*R*/, coin, start_buttons };

reg [7:0] sys_flags = 0;
wire COIN0 = sys_flags[0];
wire COIN1 = sys_flags[1];
wire SOFT_NL = ~sys_flags[2];
wire CBLK = sys_flags[3];
wire BRQ = ~m84 & ~sys_flags[4];
wire BANK = sys_flags[5];
wire NL = SOFT_NL ^ dip_sw[8];

// TODO BANK, CBLK, NL
always @(posedge CLK_32M) begin
    if (IOWR && cpu_mem_addr[7:1] == 7'h01 && cpu_be[0]) sys_flags <= cpu_dout[7:0];
end

// mux io and memory reads. The bus is word-aligned so no byte shuffling: the
// core samples cpu_din as a full 16-bit word and picks bytes via cpu_be.
always_comb begin
    bit [15:0] d16;
    bit [15:0] io16;

    if (b_d_dout_valid_lat) d16 = b_d_dout;
    else if (obj_pal_dout_valid_lat) d16 = obj_pal_dout;
    else if (sound_dout_valid_lat) d16 = sound_dout;
    else if (sprite_dout_valid_lat) d16 = sprite_dout;
    else if (cpu_mem_addr[19:16] == 4'hb) d16 = cpu_shared_ram_dout;
    else d16 = cpu_ram_rom_data;

    case (cpu_mem_addr[7:1])
    7'h00: io16 = switches;
    7'h01: io16 = flags;
    7'h02: io16 = dip_sw;
    default: io16 = 16'hffff;
    endcase

    cpu_din = (IORD | IOWR) ? io16 : d16;
end

v30_bus v30(
    .clk(CLK_32M),
    .ce(ce_cpu),
    .ce_half(ce_cpu_half),
    .reset(~reset_n),

    .cpu_addr(cpu_mem_addr),
    .cpu_be(cpu_be),
    .cpu_dout(cpu_dout),
    .cpu_din(cpu_din),

    .mem_rd(mem_rd),
    .io_rd(io_rd),
    .mem_wr(mem_wr),
    .io_wr(io_wr),
    .code_fetch(code_fetch),

    .int_req(int_req),
    .int_vector(int_vector),
    .int_ack(int_ack),

    .dbg_regs(dbg_v30_regs)
);

// DDR trace CPU taps are pruned with the VHDL export unit. TODO: reconstruct
// cs/ip/opcode from dbg_v30_regs if the on-FPGA DDR tracer is ever revived.
assign ddr_debug_data.cpu_cs = 16'd0;
assign ddr_debug_data.cpu_ip = 16'd0;
assign ddr_debug_data.cpu_opcode = 8'd0;

wire m_io = MRD | MWR;
wire sprite_dma;
wire [1:0] iset;
wire [15:0] iset_data;
wire snd_latch1_wr, snd_latch2_wr;

address_translator address_translator(
    .A(m_io ? cpu_mem_addr : {12'h000, cpu_mem_addr[7:0]}),
    .data(cpu_dout),
    .bytesel(cpu_be),
    .rd(m_io ? MRD : IORD),
    .wr(m_io ? MWR : IOWR),
    .M_IO(m_io),
    .DBEN(DBEN),
    .board_cfg(board_cfg),
    .ls245_en(ls245_en),
    .sdr_addr(cpu_region_addr),
    .writable(cpu_region_writable),
    .bg_a_memrq(bg_a_memrq),
    .bg_b_memrq(bg_b_memrq),
    .bg_palette_memrq(bg_palette_memrq),
    .sprite_memrq(sprite_memrq),
    .sprite_palette_memrq(sprite_palette_memrq),
    .sound_memrq(sound_memrq),

    .sprite_dma(sprite_dma),
    .iset(iset),
    .iset_data(iset_data),

    .snd_latch1_wr(snd_latch1_wr),
    .snd_latch2_wr(snd_latch2_wr)
);

wire int_req, int_ack;
wire [7:0] int_vector;

m72_pic m72_pic(
    .clk(CLK_32M),
    .ce(ce_cpu),
    .reset(~reset_n),

    .cs((IORD | IOWR) & ~cpu_mem_addr[7] & cpu_mem_addr[6]), // 0x40-0x43
    .wr(IOWR & cpu_be[0]),
    .rd(0),
    .a0(cpu_mem_addr[1]),

    .din(cpu_dout[7:0]),

    .int_req(int_req),
    .int_vector(int_vector),
    .int_ack(int_ack),

    .intp({5'd0, HINT, 1'b0, VBLK})
);

wire [8:0] VE, V;
wire [9:0] HE, H;
wire HBLK, VBLK, HS, VS;
wire HINT;

assign HSync = HS;
assign HBlank = HBLK;
assign VSync = VS;
assign VBlank = VBLK;

kna70h015 kna70h015(
    .CLK_32M(CLK_32M),

    .CE_PIX(ce_pix),
    .iset(iset),
    .iset_data(iset_data),
    .NL(NL),
    .S24H(0),

    .CLD(),
    .CPBLK(),

    .VE(VE),
    .V(V),
    .HE(HE),
    .H(H),

    .HBLK(HBLK),
    .VBLK(VBLK),
    .HINT(HINT),

    .HS(HS),
    .VS(VS),

    .video_50hz(video_timing == VIDEO_50HZ)
);

wire [15:0] b_d_dout;
wire b_d_dout_valid;

wire [4:0] char_r, char_g, char_b;
wire P1L;

board_b_d board_b_d(
    .CLK_32M(CLK_32M),
    .CLK_96M(CLK_96M),

    .CE_PIX(ce_pix),

    .DOUT(b_d_dout),
    .DOUT_VALID(b_d_dout_valid),

    .DIN(cpu_dout),
    .A(cpu_mem_addr),

    .IO_DIN(cpu_dout),
    .IO_A(cpu_mem_addr[7:0]),
    .IO_BE(cpu_be),

    .MRD(MRD),
    .MWR(MWR),
    .IORD(IORD),
    .IOWR(IOWR),

    .a_memrq(bg_a_memrq),
    .b_memrq(bg_b_memrq),
    .palette_memrq(bg_palette_memrq),
    
    .NL(NL),

    .VE(VE),
    .HE({HE[9], HE[7:0]}),


    .RED(char_r),
    .GREEN(char_g),
    .BLUE(char_b),
    .P1L(P1L),

    .sdr_data(sdr_bg_dout),
    .sdr_addr(sdr_bg_addr),
    .sdr_req(sdr_bg_req),
    .sdr_rdy(sdr_bg_rdy),

    .paused(paused),

    .en_layer_a(en_layer_a),
    .en_layer_b(en_layer_b),
    .en_palette(en_layer_palette),

    .m84(m84)
);


wire [15:0] sound_dout;
wire sound_dout_valid;

wire [7:0] snd_io_addr;
wire [7:0] snd_io_data;
wire snd_io_req;

wire [15:0] ym_audio = ym_audio_raw; // { ym_audio_raw[15], ym_audio_raw[15], ym_audio_raw[15:2] };
wire [15:0] ym_audio_raw;

sound sound(
    .reset(~reset_n),
    .CLK_32M(CLK_32M),
    .DIN(cpu_dout),
    .DOUT(sound_dout),
    .DOUT_VALID(sound_dout_valid),

    .A(cpu_mem_addr),
    .BE(cpu_be),

    .IO_DIN(cpu_dout[7:0]),

    .SDBEN(sound_memrq & BRQ),
    .SND(snd_latch1_wr),
    .BRQ(BRQ),
    .MRD(MRD),
    .MWR(MWR),
    .SND2(snd_latch2_wr),

    .sample_inc(z80_sample_inc),
    .sample_addr(z80_sample_addr),
    .sample_addr_wr(z80_sample_addr_wr),
    .sample_out(z80_sample_out),
    .sample_in(sample_rom_data),

    .ym_audio_l(),
    .ym_audio_r(ym_audio_raw),

    .snd_io_addr(snd_io_addr),
    .snd_io_data(snd_io_data),
    .snd_io_req(snd_io_req),

    .pause(paused),

    .m84(m84),

    .video_timing(video_timing),

    .clk_bram(clk_bram),
    .bram_wr(bram_wr),
    .bram_data(bram_data),
    .bram_addr(bram_addr),
    .bram_cs(bram_cs[4])
);

// Temp A-C board palette
wire [15:0] obj_pal_dout;
wire obj_pal_dout_valid;


wire [4:0] obj_pal_r, obj_pal_g, obj_pal_b;
kna91h014 obj_pal(
    .CLK_32M(CLK_32M),

    .G(sprite_palette_memrq),
    .SELECT(0),
    .CA(obj_pix),
    .CB(obj_pix),

    .E1_N(), // TODO
    .E2_N(), // TODO
    
    .MWR(MWR),
    .MRD(MRD),

    .DIN(cpu_dout),
    .DOUT(obj_pal_dout),
    .DOUT_VALID(obj_pal_dout_valid),
    .A(cpu_mem_addr),

    .RED(obj_pal_r),
    .GRN(obj_pal_g),
    .BLU(obj_pal_b)
);

wire [4:0] obj_r = en_sprite_palette ? obj_pal_r : { obj_pix[3:0], 1'b0 };
wire [4:0] obj_g = en_sprite_palette ? obj_pal_g : { obj_pix[3:0], 1'b0 };
wire [4:0] obj_b = en_sprite_palette ? obj_pal_b : { obj_pix[3:0], 1'b0 };

wire P0L = (|obj_pix[3:0]) && en_sprites;

assign R = ~CBLK ? ( (P0L & P1L) ? {obj_r[4:0], obj_r[4:2]} : {char_r[4:0], char_r[4:2]} ) : 8'h00;
assign G = ~CBLK ? ( (P0L & P1L) ? {obj_g[4:0], obj_g[4:2]} : {char_g[4:0], char_g[4:2]} ) : 8'h00;
assign B = ~CBLK ? ( (P0L & P1L) ? {obj_b[4:0], obj_b[4:2]} : {char_b[4:0], char_b[4:2]} ) : 8'h00;

wire [15:0] sprite_dout;
wire sprite_dout_valid;

wire [7:0] obj_pix;

sprite sprite(
    .CLK_32M(CLK_32M),
    .CLK_96M(CLK_96M),
    .CE_PIX(ce_pix),

    .DIN(cpu_dout),
    .DOUT(sprite_dout),
    .DOUT_VALID(sprite_dout_valid),

    .A(cpu_mem_addr),

    .BUFDBEN(sprite_memrq),
    .MRD(MRD),
    .MWR(MWR),

    .VE(VE),
    .NL(NL),
    .HBLK(HBLK),
    .pix_test(obj_pix),

    .TNSL(TNSL),
    .DMA_ON(sprite_dma & ~sprite_freeze),

    .sdr_data(sdr_sprite_dout),
    .sdr_addr(sdr_sprite_addr),
    .sdr_req(sdr_sprite_req),
    .sdr_rdy(sdr_sprite_rdy)
);


wire [15:0] cpu_shared_ram_dout;
wire [11:0] mcu_ram_addr;
wire [7:0] mcu_ram_din;
wire [7:0] mcu_ram_dout;
wire mcu_ram_we;
wire mcu_ram_int;
wire mcu_ram_cs;
wire [7:0] mcu_sample_out;

dualport_mailbox_2kx16 mcu_shared_ram(
    .reset(~reset_n),
    .clk_l(CLK_32M),
    .addr_l(cpu_mem_addr[11:1]),
    .cs_l(1'b1),
    .din_l(cpu_dout),
    .dout_l(cpu_shared_ram_dout),
    .we_l((cpu_mem_addr[19:16] == 4'hb && MWR) ? cpu_be : 2'b00),
    .int_l(),

    .clk_r(CLK_32M),
    .cs_r(mcu_ram_cs),
    .addr_r(mcu_ram_addr[11:0]),
    .din_r(mcu_ram_dout),
    .dout_r(mcu_ram_din),
    .we_r(mcu_ram_we),
    .int_r(mcu_ram_int)
);

wire [7:0] mculatch_data = board_cfg.main_mculatch ? cpu_dout[7:0] : snd_io_data;
wire mculatch_en = board_cfg.main_mculatch ? ( IOWR && cpu_mem_addr[7:1] == 7'h60 && cpu_be[0] ) : ( snd_io_req && snd_io_addr == 8'h82 );

mcu mcu(
    .CLK_32M(CLK_32M),
    .ce_8m(ce_mcu),
    .reset(~reset_n),

    .ext_ram_addr(mcu_ram_addr),
    .ext_ram_din(mcu_ram_din),
    .ext_ram_dout(mcu_ram_dout),
    .ext_ram_cs(mcu_ram_cs),
    .ext_ram_we(mcu_ram_we),
    .ext_ram_int(mcu_ram_int),

    .z80_din(mculatch_data),
    .z80_latch_en(mculatch_en),

    .sample_out(mcu_sample_out),

    .sample_addr_wr(mcu_sample_addr_wr),
    .sample_addr(mcu_sample_addr),
    .sample_inc(mcu_sample_inc),
    .sample_rom_data(sample_rom_data),


    .clk_bram(clk_bram),
    .bram_wr(bram_wr),
    .bram_data(bram_data),
    .bram_addr(bram_addr),
    .bram_prom_cs(bram_cs[0]),
    .bram_offsets_cs(bram_cs[2]),
    .bram_protect_cs(bram_cs[3]),

    .dbg_rom_addr(mcu_dbg_rom_addr)
);

wire [1:0] z80_sample_addr_wr, mcu_sample_addr_wr;
wire [15:0] z80_sample_addr, mcu_sample_addr;
wire [7:0] sample_rom_data;
wire [7:0] z80_sample_out;
wire z80_sample_inc, mcu_sample_inc;

sample_rom sample_rom(
    .clk(CLK_32M),
    .sample_addr_in(m84 ? z80_sample_addr : mcu_sample_addr),
    .sample_addr_wr(m84 ? z80_sample_addr_wr : mcu_sample_addr_wr),

    .sample_data(sample_rom_data),
    .sample_inc(m84 ? z80_sample_inc : mcu_sample_inc),
    
    .clk_bram(clk_bram),
    .bram_wr(bram_wr),
    .bram_data(bram_data),
    .bram_addr(bram_addr),
    .bram_cs(bram_cs[1])
);

wire [7:0] signed_mcu_sample = ( m84 ? z80_sample_out : mcu_sample_out ) - 8'h80;
reg [2:0] ce_filter_counter = 0;
wire ce_filter = &ce_filter_counter;
reg [15:0] filtered_mcu_sample;
reg [15:0] filtered_ym_audio;

// 3.5Khz 2nd order low pass filter with additional 10dB attenuation
IIR_filter #( .use_params(1), .stereo(0), .coeff_x(0.00004185087102461337 * 0.31622776601), .coeff_x0(2), .coeff_x1(1), .coeff_x2(0), .coeff_y0(-1.99222499379830120247), .coeff_y1(0.99225510233860669818), .coeff_y2(0)) samples_lpf (
	.clk(CLK_32M),
	.reset(~reset_n),

	.ce(ce_filter),
	.sample_ce(1),

	.cx(),
	.cx0(),
	.cx1(),
	.cx2(),
	.cy0(),
	.cy1(),
	.cy2(),

	.input_l({signed_mcu_sample[7:0], 8'd0}),
    .input_r(),
	.output_l(filtered_mcu_sample),
    .output_r()
);


// 9khz 1st order, 10khz 2nd order
IIR_filter #( .use_params(1), .stereo(0), .coeff_x(0.00000476166826258131), .coeff_x0(3), .coeff_x1(3), .coeff_x2(1), .coeff_y0(-2.96374831301152275032), .coeff_y1(2.92805248787211569450), .coeff_y2(-0.96430074919997255112)) music_lpf (
	.clk(CLK_32M),
	.reset(~reset_n),

	.ce(ce_filter),
	.sample_ce(1),

	.cx(),
	.cx0(),
	.cx1(),
	.cx2(),
	.cy0(),
	.cy1(),
	.cy2(),

	.input_l(ym_audio),
    .input_r(),
	.output_l(filtered_ym_audio),
    .output_r()
);

reg [16:0] audio_out;

assign AUDIO_L = audio_out[16:1];
assign AUDIO_R = audio_out[16:1];

always @(posedge CLK_32M) begin
    ce_filter_counter <= ce_filter_counter + 3'd1;
  
    if (en_audio_filters)
        audio_out <= {filtered_ym_audio[15], filtered_ym_audio[15:0]} + {filtered_mcu_sample[15], filtered_mcu_sample[15:0]};
    else
        audio_out <= {ym_audio[15], ym_audio[15:0]} + {{signed_mcu_sample[7], signed_mcu_sample[7:0], 8'd0}};
end

















reg [11:0] dbg_cpu_ext_addr;
reg [15:0] dbg_cpu_ext_data;
reg [1:0]  dbg_cpu_ext_we;

assign ddr_debug_data.cpu_ext_addr = dbg_cpu_ext_addr;
assign ddr_debug_data.cpu_ext_data = dbg_cpu_ext_data;
assign ddr_debug_data.cpu_ext_we = dbg_cpu_ext_we;

// CPU debug
always @(posedge CLK_32M) begin
    reg cs;
    reg [11:0] addr;
    reg [15:0] data;
    reg [1:0] we;

    cs <= (cpu_mem_addr[19:16] == 4'hb) && ( MWR || MRD );
    addr <= cpu_mem_addr[11:0];
    data <= cpu_dout;
    we <= MWR ? cpu_be : 2'b00;

    if (cs & ~((cpu_mem_addr[19:16] == 4'hb) && ( MWR || MRD ))) begin
        dbg_cpu_ext_addr <= addr;
        dbg_cpu_ext_we <= we;
        if (we != 2'b00) dbg_cpu_ext_data <= data;
        else dbg_cpu_ext_data <= cpu_shared_ram_dout;
    end
end

reg [11:0] dbg_mcu_ext_addr;
reg [7:0] dbg_mcu_ext_data;
reg dbg_mcu_ext_we;

assign ddr_debug_data.mcu_ext_addr = dbg_mcu_ext_addr;
assign ddr_debug_data.mcu_ext_data = dbg_mcu_ext_data;
assign ddr_debug_data.mcu_ext_we = dbg_mcu_ext_we;

// MCU debug
always @(posedge CLK_32M) begin
    reg cs;
    reg [11:0] addr;
    reg [7:0] data;
    reg we;

    cs <= mcu_ram_cs;
    addr <= mcu_ram_addr;
    data <= mcu_ram_dout;
    we <= mcu_ram_we;

    if (cs & ~mcu_ram_cs) begin
        dbg_mcu_ext_addr <= addr;
        dbg_mcu_ext_we <= we;
        if (we) dbg_mcu_ext_data <= data;
        else dbg_mcu_ext_data <= mcu_ram_din;
    end
end


wire [15:0] mcu_dbg_rom_addr;
reg [15:0] latched_mcu_dbg_rom_addr;
assign ddr_debug_data.mcu_rom_addr = latched_mcu_dbg_rom_addr; 
always @(posedge CLK_32M) if (ce_cpu) latched_mcu_dbg_rom_addr <= mcu_dbg_rom_addr;

endmodule
