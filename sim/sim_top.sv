//============================================================================
//  Irem M72 for MiSTer FPGA - Verilator simulation top
//
//  Replaces the MiSTer emu/sys_top wrapper for simulation: instantiates the
//  m72 core and rom_loader directly, exposes the three SDRAM channels to the
//  C++ harness (SimSDRAM models the controller), and mirrors the essential
//  glue from Arcade-IremM72.sv (ch3 mux, input negation, hiscore tie-off).
//============================================================================

import m72_pkg::*;

// Sim savestate build stamp (ASCII, mirrors MiSTer's `BUILD_DATE).  The sim
// has no build_id.v; the Makefile may inject a date via -DSIM_SS_VERSION,
// else the "SIM000" sentinel is used.
`ifndef SIM_SS_VERSION
`define SIM_SS_VERSION 64'("SIM000")
`endif

module sim_top(
    input             clk_32m,
    input             clk_96m,
    input             reset,          // active high

    // video (straight off the core)
    output            ce_pixel,
    output            hsync,
    output            hblank,
    output            vsync,
    output            vblank,
    output      [7:0] red,
    output      [7:0] green,
    output      [7:0] blue,

    // inputs, active-high from C++ (negated here, mirroring Arcade-IremM72.sv)
    input       [3:0] p1_joystick,    // up, down, left, right
    input       [3:0] p2_joystick,
    input       [3:0] p1_buttons,     // a, b, x, y
    input       [3:0] p2_buttons,
    input       [1:0] start,
    input       [1:0] coin,
    input             service_btn,
    input      [15:0] dipswitch,      // {dip_sw[1], dip_sw[0]}, active-high

    // SDRAM ch1: background, 32-bit read
    output     [24:0] sdr_bg_addr,
    input      [31:0] sdr_bg_dout,
    output            sdr_bg_req,
    input             sdr_bg_rdy,

    // SDRAM ch2: sprites, 64-bit read
    output     [24:0] sdr_sprite_addr,
    input      [63:0] sdr_sprite_dout,
    output            sdr_sprite_req,
    input             sdr_sprite_rdy,

    // SDRAM ch3: cpu r/w, muxed with rom download, 16-bit
    output     [24:0] sdr_ch3_addr,
    output     [15:0] sdr_ch3_din,
    input      [15:0] sdr_ch3_dout,
    output      [1:0] sdr_ch3_be,
    output            sdr_ch3_rnw,
    output            sdr_ch3_req,
    input             sdr_ch3_rdy,

    // DDR interface (savestate slot window, serviced by SimDDR)
    output            ddr_acquire,
    output     [31:0] ddr_addr,
    output     [63:0] ddr_wdata,
    input      [63:0] ddr_rdata,
    output            ddr_read,
    output            ddr_write,
    output      [7:0] ddr_burstcnt,
    output      [7:0] ddr_byteenable,
    input             ddr_busy,
    input             ddr_read_complete,

    // savestate handshake
    input             ss_do_save,
    input             ss_do_restore,
    input       [1:0] ss_index,
    output      [3:0] ss_state_out,

    // IOCTL (index 0 = ROM stream)
    input             ioctl_download,
    input       [7:0] ioctl_index,
    input             ioctl_wr,
    input       [7:0] ioctl_dout,
    output            ioctl_wait,

    output     [15:0] audio_l,
    output     [15:0] audio_r,

    // debug taps
    output     [15:0] dbg_cpu_cs,
    output     [15:0] dbg_cpu_ip,
    output      [7:0] dbg_cpu_opcode,
    output    [223:0] dbg_cpu_regs,      // full V30 register file (V30_BACKDOOR)
    output            dbg_sdr_cpu_code,  // last CPU SDRAM access was a prefetch

    // debug/config toggles (mirrors the OSD debug page, all default-on)
    input             en_layer_a,
    input             en_layer_b,
    input             en_sprites,
    input             en_layer_palette,
    input             en_sprite_palette,
    input             en_audio_filters,
    input             sprite_freeze,
    input       [1:0] video_timing_in,

    input             pause
);

///////////////////////////////////////////////////////////////////////
// ROM loading (mirrors Arcade-IremM72.sv lines 328-398)
///////////////////////////////////////////////////////////////////////

wire [19:0] bram_addr;
wire [7:0] bram_data;
wire [4:0] bram_cs;
wire bram_wr;

board_cfg_t board_cfg;

wire [24:0] sdr_rom_addr;
wire [15:0] sdr_rom_data;
wire [1:0] sdr_rom_be;
wire sdr_rom_req;

wire sdr_rom_write = ioctl_download && (ioctl_index == 0);

wire [15:0] sdr_cpu_dout, sdr_cpu_din;
wire [24:0] sdr_cpu_addr;
wire sdr_cpu_req;
wire [1:0] sdr_cpu_wr_sel;
wire sdr_cpu_mem_rq;

assign sdr_ch3_addr = sdr_rom_write ? sdr_rom_addr : sdr_cpu_addr;
assign sdr_ch3_din = sdr_rom_write ? sdr_rom_data : sdr_cpu_din;
assign sdr_ch3_be = sdr_rom_write ? sdr_rom_be : sdr_cpu_wr_sel;
assign sdr_ch3_rnw = sdr_rom_write ? 1'b0 : ~{|sdr_cpu_wr_sel};
assign sdr_ch3_req = sdr_rom_write ? sdr_rom_req : sdr_cpu_req;
assign sdr_cpu_dout = sdr_ch3_dout;
wire sdr_cpu_rdy = sdr_ch3_rdy;
wire sdr_rom_rdy = sdr_ch3_rdy;

rom_loader rom_loader(
    .sys_clk(clk_32m),
    .ram_clk(clk_96m),

    .ioctl_wr(ioctl_wr && !ioctl_index),
    .ioctl_data(ioctl_dout[7:0]),

    .ioctl_wait(ioctl_wait),

    .sdr_addr(sdr_rom_addr),
    .sdr_data(sdr_rom_data),
    .sdr_be(sdr_rom_be),
    .sdr_req(sdr_rom_req),
    .sdr_rdy(sdr_rom_rdy),

    .bram_addr(bram_addr),
    .bram_data(bram_data),
    .bram_cs(bram_cs),
    .bram_wr(bram_wr),

    .board_cfg(board_cfg)
);

///////////////////////////////////////////////////////////////////////
// DDR bridge (savestates are the only DDR client in the sim)
///////////////////////////////////////////////////////////////////////

ddr_if ddr_host();

assign ddr_acquire = ddr_host.acquire;
assign ddr_addr = ddr_host.addr;
assign ddr_byteenable = ddr_host.byteenable;
assign ddr_write = ddr_host.write;
assign ddr_read = ddr_host.read;
assign ddr_wdata = ddr_host.wdata;
assign ddr_burstcnt = ddr_host.burstcnt;
assign ddr_host.rdata = ddr_rdata;
assign ddr_host.rdata_ready = ddr_read_complete;
assign ddr_host.busy = ddr_busy;

///////////////////////////////////////////////////////////////////////
// Core
///////////////////////////////////////////////////////////////////////

wire [223:0] dbg_v30_regs;
wire         dbg_sdr_cpu_code_w;

// dbg_v30_regs packing: {psw,ip,ds,ss,cs,es,di,si,bp,sp,bx,dx,cx,ax}
assign dbg_cpu_regs = dbg_v30_regs;
assign dbg_cpu_cs = dbg_v30_regs[159:144];
assign dbg_cpu_ip = dbg_v30_regs[207:192];
assign dbg_cpu_opcode = 8'd0;              // opcode export dropped with the VHDL core
assign dbg_sdr_cpu_code = dbg_sdr_cpu_code_w;

m72 #(.SS_VERSION(`SIM_SS_VERSION)) m72_inst(
    .CLK_32M(clk_32m),
    .CLK_96M(clk_96m),
    .ce_pix(ce_pixel),
    .reset_n(~reset),
    .z80_reset_n(~reset),
    .HBlank(hblank),
    .VBlank(vblank),
    .HSync(hsync),
    .VSync(vsync),
    .R(red),
    .G(green),
    .B(blue),
    .AUDIO_L(audio_l),
    .AUDIO_R(audio_r),

    .board_cfg(board_cfg),

    .coin(~coin),
    .start_buttons(~start),

    .p1_joystick(~p1_joystick),
    .p2_joystick(~p2_joystick),
    .p1_buttons(~p1_buttons),
    .p2_buttons(~p2_buttons),
    .service_button(~service_btn),

    .dip_sw(~dipswitch),

    .sdr_sprite_addr(sdr_sprite_addr),
    .sdr_sprite_dout(sdr_sprite_dout),
    .sdr_sprite_req(sdr_sprite_req),
    .sdr_sprite_rdy(sdr_sprite_rdy),

    .sdr_bg_addr(sdr_bg_addr),
    .sdr_bg_dout(sdr_bg_dout),
    .sdr_bg_req(sdr_bg_req),
    .sdr_bg_rdy(sdr_bg_rdy),

    .sdr_cpu_dout(sdr_cpu_dout),
    .sdr_cpu_din(sdr_cpu_din),
    .sdr_cpu_addr(sdr_cpu_addr),
    .sdr_cpu_req(sdr_cpu_req),
    .sdr_cpu_rdy(sdr_cpu_rdy),
    .sdr_cpu_wr_sel(sdr_cpu_wr_sel),
    .sdr_cpu_mem_rq(sdr_cpu_mem_rq),

    // hiscore unused in sim
    .hs_address(17'd0),
    .hs_data_out(),
    .hs_data_in(8'd0),
    .hs_read_enable(1'b0),
    .hs_write_enable(1'b0),
    .hs_data_ready(),

    .clk_bram(clk_32m),
    .bram_addr(bram_addr),
    .bram_data(bram_data),
    .bram_cs(bram_cs),
    .bram_wr(bram_wr),

    .pause_rq(pause),

    .ddr(ddr_host),
    .ss_index(ss_index),
    .ss_do_save(ss_do_save),
    .ss_do_restore(ss_do_restore),
    .ss_state_out(ss_state_out),

    .dbg_v30_regs(dbg_v30_regs),
    .sdr_cpu_code(dbg_sdr_cpu_code_w),

    .en_layer_a(en_layer_a),
    .en_layer_b(en_layer_b),
    .en_sprites(en_sprites),
    .en_layer_palette(en_layer_palette),
    .en_sprite_palette(en_sprite_palette),
    .en_audio_filters(en_audio_filters),

    .sprite_freeze(sprite_freeze),

    .video_timing(video_timing_t'(video_timing_in))
);

endmodule
