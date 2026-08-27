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

module m72 #(
    parameter [63:0] SS_VERSION = 64'd0
) (
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

    // savestates: DDR slot window + host handshake
    ddr_if.to_host ddr,
    input [1:0] ss_index,
    input ss_do_save,
    input ss_do_restore,
    output [3:0] ss_state_out,

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

    // Full V30 register file for the sim CPU window (zeros unless V30_BACKDOOR).
    output [223:0] dbg_v30_regs,
    // code_fetch flag latched with the last CPU SDRAM request (sim DebugLink).
    output reg sdr_cpu_code
);

// Divide 32Mhz clock by 4 for pixel clock
reg paused /* verilator public_flat_rd */ = 0;
reg [8:0] paused_v;
reg [9:0] paused_h;

// The GLOBAL savestate section (see below) covers registers owned by three
// different always blocks. Synthesis will not resolve a register driven from
// more than one block, so the section decodes its write here and every owner
// applies the restore itself. Assigned from the ssbus after ssb is declared.
reg        ss_glb_wr;
reg [2:0]  ss_glb_addr;
reg [63:0] ss_glb_data;

// Pause acquisition. A savestate pause additionally requires the V30 BIU to
// be bus-quiet (no cycle latent in the prefetch/EU pipeline) and the sprite
// DMA idle (TNSL) so every RAM port the savestate hijacks is inert.
wire pause_rq_any = pause_rq | ss_pause;
wire ss_quiesced = v30_ss_quiet & TNSL;
// Once the ucore exposes a bus-quiet boundary during a savestate request,
// stop issuing CPU phases immediately.  Otherwise its prefetcher can launch
// another cycle before `paused` is registered, and tight code-fetch loops may
// never leave SS_BUS_QUIET asserted long enough to acquire the pause.
wire ss_cpu_quiesce = ss_pause & v30_ss_quiet;

always @(posedge CLK_32M) begin
    if (pause_rq_any & ~paused) begin
        if (~ls245_en & ~DBEN & ~mem_rq_active & (~ss_pause | ss_quiesced)) begin
            paused <= 1;
            paused_v <= V;
            paused_h <= H;
        end
    end else if (~pause_rq_any & paused) begin
        paused <= ~(V == paused_v && H == paused_h);
    end

    // GLOBAL savestate restore of the pause resume point.
    if (ss_glb_wr) begin
        case (ss_glb_addr)
        3'd3: paused_v <= ss_glb_data[8:0];
        3'd4, 3'd5, 3'd6, 3'd7: paused_h <= ss_glb_data[9:0];
        default: ;
        endcase
    end
end

reg ce_cpu, ce_mcu;

// CE-freeze stall: withhold the CPU clock train while an SDRAM access is
// outstanding. The new-request term covers the 1-clk gap between a request
// being decoded and mem_rq_active going high, so the core never advances a
// T-state mid-request. The CE train free-runs during reset (the v30_core
// needs a running clock to execute its boot/reset-vector sequence); the core
// itself is held in reset via reset_n.
wire cpu_stall = mem_rq_active | cpu_new_sdr_req;

// Steady-rate CPU clock with catch-up (ce_steady mechanism, as in the IGSPGM
// core): a free-running reference counts the target 8MHz CPU clocks; the CPU
// counter chases it whenever it is behind. SDRAM stalls therefore defer CPU
// cycles instead of losing them, and the average CPU rate stays at exactly
// 8MHz - the real board has no such stalls. The reference pauses with
// `paused` so no backlog accumulates across the pause feature.
//
// THE COUNTER STILL ADVANCES TWO PER CPU CYCLE, and `ce_cpu` is still only
// its EVEN phase, even though the de-muxed v30_core takes ONE clock enable
// and CE_HALF is gone (2026-08-14).  The odd slot is now an idle fabric clock
// rather than a second strobe, and it is deliberately kept: it is what holds
// two `ce_cpu` pulses at least TWO fabric clocks apart, which is the premise
// the ucore CE multicycle in Arcade-IremM72.sdc is derived from.  Collapsing
// it to one count per CPU cycle would make the catch-up burst 32MHz and that
// exception a lie.
reg [1:0] ce_steady_div;
reg [9:0] ce_steady_count;
reg [10:0] ce_cpu_count; // [0] selects the CE (0) / CE_HALF (1) phase

always @(posedge CLK_32M) begin
    ce_cpu <= 0;
    ce_mcu <= 0;

    if (~paused && ~ss_cpu_quiesce) begin
        ce_steady_div <= ce_steady_div + 2'd1;
        if (&ce_steady_div) ce_steady_count <= ce_steady_count + 10'd1;

        if (~cpu_stall && ce_cpu_count[10:1] != ce_steady_count) begin
            ce_cpu      <= ~ce_cpu_count[0];
            // Tie the MCU CE to the V30 CE phase: the MCU advances (and stalls)
            // in lockstep with the bursty catch-up V30, holding the shared-RAM
            // protection handshake at a constant V30<->MCU phase relationship
            // (the effect MAME gets from synchronize() on every MCU dpram write).
            // NOT gated by reset_n: the nu8051 has a SYNCHRONOUS reset (min 24 CE
            // ticks with RESET asserted), so ce_mcu must keep ticking during reset
            // or the MCU never processes it. On power-up RESET is held past reset_n
            // (via ~valid_rom during ROM load) so it reset anyway, but a reset-button
            // reset asserts RESET only while reset_n is low - exactly when ce_mcu
            // used to be 0 - leaving the MCU un-reset and protection games wedged.
            ce_mcu      <= ~ce_cpu_count[0];
            ce_cpu_count <= ce_cpu_count + 11'd1;
        end
    end

    // GLOBAL savestate restore of the CE train. Only reachable while
    // ss-paused, so it can never collide with the ~paused arm above.
    if (ss_glb_wr) begin
        case (ss_glb_addr)
        3'd1: ce_steady_count <= ss_glb_data[9:0];
        3'd2: ce_cpu_count <= ss_glb_data[10:0];
        3'd3: ce_steady_div <= ss_glb_data[10:9];
        default: ;
        endcase
    end
end

wire ce_pix_half;
jtframe_frac_cen #(2) pixel_cen
(
    .clk(CLK_32M),
    .n(10'd1),
    .m(10'd4),
    .cen({ce_pix_half, ce_pix})
);

// The nu8051 takes one CE per oscillator period (12 CE ticks per machine cycle
// internally), so ce_mcu is the true 8MHz oscillator rate.  Rather than free-run
// it from its own frac_cen, ce_mcu is tied to the V30 CE phase above so the MCU
// bursts/stalls in lockstep with the (bursty catch-up) V30 - this holds the
// shared-RAM protection handshake at a constant V30<->MCU phase, which a static
// offset could not (the phase drifted with the V30's SDRAM catch-up).

wire clock = CLK_32M;

/* Global signals from schematics. Strobes come straight from the v30_bus
 * adapter now: reads are levels (T1-half..T4), writes are held during T3. The
 * bus is word-aligned (cpu_mem_addr[0]==0) with per-byte enables in cpu_be, so
 * the old word_shuffle / cpu_word_* realignment machinery is gone. */
wire mem_rd, mem_wr_pending, mem_wr, io_rd, io_wr, code_fetch;

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
wire work_ram_memrq;

// V30 READY wait-states (authentic Tw, orthogonal to the SDRAM cpu_stall path).
// Sprite: stall any CPU access to the buffer while DMA runs (~TNSL). Tile: the
// per-layer bg_ready (from board_b_d) is low while a pending CPU tile write
// awaits its SH window. mem_rd/mem_wr and the region decodes are stable across
// the whole (now possibly Tw-extended) bus cycle, so the READY level is stable.
wire bg_ready;
wire sprite_wait = sprite_memrq & ~TNSL & (mem_rd | mem_wr_pending);
wire v30_ready   = ~sprite_wait & bg_ready;

reg mem_rq_active = 0;
assign sdr_cpu_mem_rq = mem_rq_active;

// A CPU SDRAM access is being decoded this cycle (mem_rq not yet asserted).
// Feeds cpu_stall so the core is frozen across the 1-clk request-detect gap.
wire cpu_new_sdr_req = ls245_en &&
    ((mem_rd & ~cpu_mem_read_lat) || (mem_wr & ~cpu_mem_write_lat));

reg b_d_dout_valid_lat, obj_pal_dout_valid_lat, sound_dout_valid_lat, sprite_dout_valid_lat;
reg work_ram_dout_valid_lat;
wire work_ram_dout_valid = MRD & work_ram_memrq;

always @(posedge CLK_32M or negedge reset_n)
begin
    if (!reset_n) begin
        b_d_dout_valid_lat <= 0;
        obj_pal_dout_valid_lat <= 0;
        sound_dout_valid_lat <= 0;
        sprite_dout_valid_lat <= 0;
        work_ram_dout_valid_lat <= 0;
    end else begin
        cpu_mem_read_lat <= mem_rd;
        cpu_mem_write_lat <= mem_wr;
        hs_mem_read_lat <= hs_read_enable;
        hs_mem_write_lat <= hs_write_enable;

        b_d_dout_valid_lat <= b_d_dout_valid;
        obj_pal_dout_valid_lat <= obj_pal_dout_valid;
        sound_dout_valid_lat <= sound_dout_valid;
        sprite_dout_valid_lat <= sprite_dout_valid;
        work_ram_dout_valid_lat <= work_ram_dout_valid;
    end
end

reg sdr_cpu_rq, sdr_cpu_ack, sdr_cpu_rq2;

always_ff @(posedge CLK_96M) begin
    sdr_cpu_req <= 0;
    if (sdr_cpu_rdy) sdr_cpu_ack <= sdr_cpu_rq;
    if (sdr_cpu_rq != sdr_cpu_rq2) begin
        sdr_cpu_req <= 1;
        sdr_cpu_rq2 <= sdr_cpu_rq;
    end
end

// SDRAM ch3 serves CPU ROM fetches only; work RAM (and the hiscore path
// into it) lives in block RAM below.
always_ff @(posedge CLK_32M or negedge reset_n) begin
    if (!reset_n) begin
        mem_rq_active <= 0;
    end else begin
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
              end
        end else if (sdr_cpu_rq == sdr_cpu_ack) begin
            cpu_ram_rom_data <= sdr_cpu_dout;
            mem_rq_active <= 0;
        end
    end
end

// CPU work RAM: 64Kx16 block RAM covering the full 128KB window every memory
// map decodes (rtype uses 0x40000-0x43fff of the 0x40000-0x5ffff window).
// Port A is the CPU; port B serves the hiscore module (only active while the
// core is paused) behind the savestate adaptor (only active while savestate-
// quiesced), so the two secondary masters never contend.
wire [15:0] work_ram_dout;
wire [15:0] work_ram_q_b;

wire        work_ram_wren_b;
wire [1:0]  work_ram_be_b;
wire [15:0] work_ram_addr_b;
wire [15:0] work_ram_data_b;

ram_be_ss_adaptor #(.WIDTHAD(16), .SS_IDX(SSIDX_WORK_RAM)) work_ram_ss(
    .clk(CLK_32M),

    .wren_in(hs_write_enable),
    .byteena_in({hs_address[0], ~hs_address[0]}),
    .addr_in(hs_address[16:1]),
    .data_in({hs_data_in, hs_data_in}),

    .wren_out(work_ram_wren_b),
    .byteena_out(work_ram_be_b),
    .addr_out(work_ram_addr_b),
    .data_out(work_ram_data_b),

    .q(work_ram_q_b),

    .ssbus(ssb[SSIDX_WORK_RAM])
);

dualport_ram_be #(.BYTES(2), .WIDTHAD(16)) work_ram(
    .clock_a(CLK_32M),
    .wren_a(MWR & work_ram_memrq),
    .byteena_a(cpu_be),
    .address_a(cpu_mem_addr[16:1]),
    .data_a(cpu_dout),
    .q_a(work_ram_dout),

    .clock_b(CLK_32M),
    .wren_b(work_ram_wren_b),
    .byteena_b(work_ram_be_b),
    .address_b(work_ram_addr_b),
    .data_b(work_ram_data_b),
    .q_b(work_ram_q_b)
);

// Hiscore access completion: the BRAM read is valid one clock after the
// address latch; answer (and ack writes) two clocks after the enable edge.
// The hiscore module holds its enables until hs_data_ready pulses.
reg [1:0] hs_pipe;
always_ff @(posedge CLK_32M) begin
    hs_data_ready <= 0;
    hs_pipe <= {hs_pipe[0], 1'b0};
    if ((hs_read_enable & ~hs_mem_read_lat) || (hs_write_enable & ~hs_mem_write_lat))
        hs_pipe[0] <= 1;
    if (hs_pipe[1]) begin
        hs_data_out <= hs_address[0] ? work_ram_q_b[15:8] : work_ram_q_b[7:0];
        hs_data_ready <= 1;
    end
end

//////////////////////////////////
//// SAVESTATES
//
// ssbus fabric: memory_stream (in save_state_data) gathers/scatters chunk
// data between the DDR slot window and the per-section slaves below.  The
// controller FSM quiesces the core via the pause mechanism (plus V30
// bus-quiet and sprite-DMA-idle terms), then hands the streamer the bus.

reg ss_write /* verilator public_flat_rd */ = 0;
reg ss_read /* verilator public_flat_rd */ = 0;
wire ss_busy;
reg [63:0] ss_restored_version /* verilator public_flat */ = 0;

ssbus_if ssbus();
ssbus_if ssb[SSIDX_COUNT]();

ssbus_mux #(.COUNT(SSIDX_COUNT)) ssmux(
    .clk(CLK_32M),
    .slave(ssbus),
    .masters(ssb)
);

// GLOBAL section write decode, consumed by the blocks that own each register.
always_comb begin
    ss_glb_wr   = ssb[SSIDX_GLOBAL].access(SSIDX_GLOBAL) & ssb[SSIDX_GLOBAL].write;
    ss_glb_addr = ssb[SSIDX_GLOBAL].addr[2:0];
    ss_glb_data = ssb[SSIDX_GLOBAL].data;
end

save_state_data save_state_data(
    .clk(CLK_32M),
    .reset(0),

    .ddr(ddr),

    .index(ss_index),
    .read_start(ss_read),
    .write_start(ss_write),
    .busy(ss_busy),

    .ssbus(ssbus)
);

typedef enum bit [3:0] {
    SST_IDLE               = 4'd0,
    SST_SAVE_WAIT_PAUSE    = 4'd1,
    SST_SAVE_SETTLE        = 4'd2,
    SST_SAVE_WAIT_WRITE    = 4'd3,
    SST_RESTORE_WAIT_PAUSE = 4'd6,
    SST_RESTORE_WAIT_READ  = 4'd7,
    SST_RESTORE_DRAIN      = 4'd8
} ss_state_t;

ss_state_t ss_state = SST_IDLE;
assign ss_state_out = ss_state;

logic ss_pause /* verilator public_flat_rd */;
logic ss_restore_active;
reg ss_restore_done;   // 1-clk pulse late in the drain: adapters reset to idle
reg ss_restore_hold;   // restore start -> full unpause: gates layer replay
reg [2:0] ss_counter;

wire v30_ss_quiet /* verilator public_flat_rd */;
wire ss_paused = ss_pause & paused;

always_comb begin
    ss_pause = 1;
    ss_restore_active = 0;

    case (ss_state)
        SST_IDLE: ss_pause = 0;

        SST_RESTORE_WAIT_PAUSE,
        SST_RESTORE_WAIT_READ,
        SST_RESTORE_DRAIN: ss_restore_active = 1;

        default: begin end
    endcase
end

always_ff @(posedge CLK_32M) begin
    ss_restore_done <= 0;

    // Hold the restore gate from restore start until the core fully resumes
    // (the beam-match unpause happens after the FSM returns to IDLE).
    if (ss_restore_active) ss_restore_hold <= 1;
    else if (~paused) ss_restore_hold <= 0;

    case (ss_state)
        SST_IDLE: begin
            if (ss_do_save) ss_state <= SST_SAVE_WAIT_PAUSE;
            if (ss_do_restore) ss_state <= SST_RESTORE_WAIT_PAUSE;
        end

        SST_SAVE_WAIT_PAUSE: begin
            ss_counter <= 0;
            if (ss_paused) ss_state <= SST_SAVE_SETTLE;
        end

        // Let the *_dout_valid_lat / hs pipes clear before the streamer
        // starts hijacking RAM ports.
        SST_SAVE_SETTLE: begin
            ss_counter <= ss_counter + 3'd1;
            if (&ss_counter) begin
                ss_write <= 1;
                ss_state <= SST_SAVE_WAIT_WRITE;
            end
        end

        SST_SAVE_WAIT_WRITE: begin
            if (ss_busy & ss_write) begin
                ss_write <= 0;
            end else if (~ss_busy & ~ss_write) begin
                ss_state <= SST_IDLE;
            end
        end

        SST_RESTORE_WAIT_PAUSE: begin
            ss_counter <= 0;
            if (ss_paused) begin
                ss_read <= 1;
                ss_state <= SST_RESTORE_WAIT_READ;
            end
        end

        SST_RESTORE_WAIT_READ: begin
            if (ss_busy & ss_read) begin
                ss_read <= 0;
            end else if (~ss_busy & ~ss_read) begin
                ss_state <= SST_RESTORE_DRAIN;
            end
        end

        // Drain the V30 SS write staging (and ssbus mux latency) before CE
        // can resume, and reset the bus adapters to idle.
        SST_RESTORE_DRAIN: begin
            ss_counter <= ss_counter + 3'd1;
            if (ss_counter == 3'd5) ss_restore_done <= 1;
            if (&ss_counter) ss_state <= SST_IDLE;
        end

        default: ss_state <= SST_IDLE;
    endcase
end

// GLOBAL section: sys_flags, the CE-train counters and the pause resume
// point.  All owner blocks are provably quiescent while ss-paused, so the
// restore writes here cannot race them.
always_ff @(posedge CLK_32M) begin
    ssb[SSIDX_GLOBAL].setup(SSIDX_GLOBAL, 5, 1);
    if (ssb[SSIDX_GLOBAL].access(SSIDX_GLOBAL)) begin
        if (ssb[SSIDX_GLOBAL].read) begin
            case (ssb[SSIDX_GLOBAL].addr[2:0])
            3'd0: ssb[SSIDX_GLOBAL].read_response(SSIDX_GLOBAL, {56'd0, sys_flags});
            3'd1: ssb[SSIDX_GLOBAL].read_response(SSIDX_GLOBAL, {54'd0, ce_steady_count});
            3'd2: ssb[SSIDX_GLOBAL].read_response(SSIDX_GLOBAL, {53'd0, ce_cpu_count});
            3'd3: ssb[SSIDX_GLOBAL].read_response(SSIDX_GLOBAL, {53'd0, ce_steady_div, paused_v});
            default: ssb[SSIDX_GLOBAL].read_response(SSIDX_GLOBAL, {54'd0, paused_h});
            endcase
        end else if (ssb[SSIDX_GLOBAL].write) begin
            // The registers themselves are written by their owning blocks
            // from the ss_glb_* decode; this block only acks.
            ssb[SSIDX_GLOBAL].write_ack(SSIDX_GLOBAL);
        end
    end
end

// VERSION section: build stamp round-trips through the state file so the sim
// can report which build produced a restored state.
always_ff @(posedge CLK_32M) begin
    ssb[SSIDX_VERSION].setup(SSIDX_VERSION, 1, 3); // 1 x 64-bit value (ASCII)
    if (ssb[SSIDX_VERSION].access(SSIDX_VERSION)) begin
        if (ssb[SSIDX_VERSION].read) begin
            ssb[SSIDX_VERSION].read_response(SSIDX_VERSION, SS_VERSION);
        end else if (ssb[SSIDX_VERSION].write) begin
            ss_restored_version <= ssb[SSIDX_VERSION].data[63:0];
            ssb[SSIDX_VERSION].write_ack(SSIDX_VERSION);
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
    else if (ss_glb_wr && ss_glb_addr == 3'd0) sys_flags <= ss_glb_data[7:0];
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
    else if (work_ram_dout_valid_lat) d16 = work_ram_dout;
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

v30_bus #(.SS_IDX(SSIDX_V30)) v30(
    .clk(CLK_32M),
    .ce(ce_cpu),
    .reset(~reset_n),
    .ready(v30_ready),

    .ssbus(ssb[SSIDX_V30]),
    .ss_restore_done(ss_restore_done),
    .ss_quiet(v30_ss_quiet),

    .cpu_addr(cpu_mem_addr),
    .cpu_be(cpu_be),
    .cpu_dout(cpu_dout),
    .cpu_din(cpu_din),

    .mem_rd(mem_rd),
    .io_rd(io_rd),
    .mem_wr_pending(mem_wr_pending),
    .mem_wr(mem_wr),
    .io_wr(io_wr),
    .code_fetch(code_fetch),

    .int_req(int_req),
    .int_vector(int_vector),
    .int_ack(int_ack),

    .dbg_regs(dbg_v30_regs)
);

// The write address is valid before MWR reaches T3.  Include the pending
// memory-write phase in the memory/IO address selection so the PAL can decode
// a wait-generating tile/sprite target in time for the ucore's READY sample;
// the translator's rd/wr inputs remain gated by the real bus strobes, so no
// device commits a write during this early decode phase.
wire m_io = MRD | MWR | mem_wr_pending;
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
    .work_ram_memrq(work_ram_memrq),

    .sprite_dma(sprite_dma),
    .iset(iset),
    .iset_data(iset_data),

    .snd_latch1_wr(snd_latch1_wr),
    .snd_latch2_wr(snd_latch2_wr)
);

wire int_req, int_ack;
wire [7:0] int_vector;

m72_pic #(.SS_IDX(SSIDX_PIC)) m72_pic(
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

    .intp({5'd0, HINT, 1'b0, VBLK}),

    .ssbus(ssb[SSIDX_PIC])
);

wire [8:0] VE, V;
wire [9:0] HE, H;
wire HBLK, VBLK, HS, VS;
wire HINT;

assign HSync = HS;
assign HBlank = HBLK;
assign VSync = VS;
assign VBlank = VBLK;

kna70h015 #(.SS_IDX(SSIDX_CRTC)) kna70h015(
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

    .video_50hz(video_timing == VIDEO_50HZ),

    .ssbus(ssb[SSIDX_CRTC])
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
    .bg_ready(bg_ready),

    .DIN(cpu_dout),
    .A(cpu_mem_addr),

    .IO_DIN(cpu_dout),
    .IO_A(cpu_mem_addr[7:0]),
    .IO_BE(cpu_be),

    .MRD(MRD),
    .MWR(MWR),
    .MWR_WAIT(mem_wr_pending),
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

    .m84(m84),

    .ssbus_a_ram0(ssb[SSIDX_LAYER_A_RAM0 + 0]),
    .ssbus_a_ram1(ssb[SSIDX_LAYER_A_RAM0 + 1]),
    .ssbus_a_ram2(ssb[SSIDX_LAYER_A_RAM0 + 2]),
    .ssbus_a_ram3(ssb[SSIDX_LAYER_A_RAM0 + 3]),
    .ssbus_a_regs(ssb[SSIDX_LAYER_A_REGS]),
    .ssbus_b_ram0(ssb[SSIDX_LAYER_B_RAM0 + 0]),
    .ssbus_b_ram1(ssb[SSIDX_LAYER_B_RAM0 + 1]),
    .ssbus_b_ram2(ssb[SSIDX_LAYER_B_RAM0 + 2]),
    .ssbus_b_ram3(ssb[SSIDX_LAYER_B_RAM0 + 3]),
    .ssbus_b_regs(ssb[SSIDX_LAYER_B_REGS]),
    .ssbus_palette(ssb[SSIDX_PAL_BG]),
    .ss_restore(ss_restore_hold)
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
    .bram_cs(bram_cs[4]),

    .ssbus_ram(ssb[SSIDX_SOUND_RAM]),
    .ssbus_regs(ssb[SSIDX_SOUND_REGS]),
    .ssbus_z80(ssb[SSIDX_Z80]),
    .ssbus_jt51(ssb[SSIDX_JT51]),
    .ss_restore_active(ss_restore_active)
);

// Temp A-C board palette
wire [15:0] obj_pal_dout;
wire obj_pal_dout_valid;


wire [4:0] obj_pal_r, obj_pal_g, obj_pal_b;
kna91h014 #(.SS_IDX(SSIDX_PAL_OBJ)) obj_pal(
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
    .BLU(obj_pal_b),

    .ssbus(ssb[SSIDX_PAL_OBJ])
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
    .sdr_rdy(sdr_sprite_rdy),

    .ssbus_ram_l(ssb[SSIDX_SPRITE_RAM_L]),
    .ssbus_ram_h(ssb[SSIDX_SPRITE_RAM_H]),
    .ssbus_objram(ssb[SSIDX_SPRITE_OBJRAM]),
    .ssbus_regs(ssb[SSIDX_SPRITE_REGS])
);


wire [15:0] cpu_shared_ram_dout;
wire [11:0] mcu_ram_addr;
wire [7:0] mcu_ram_din;
wire [7:0] mcu_ram_dout;
wire mcu_ram_we;
wire mcu_ram_int;
wire mcu_ram_cs;
wire [7:0] mcu_sample_out;

dualport_mailbox_2kx16 #(.SS_IDX(SSIDX_MCU_MAILBOX)) mcu_shared_ram(
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
    .int_r(mcu_ram_int),

    .ssbus(ssb[SSIDX_MCU_MAILBOX])
);

`ifdef VERILATOR
//============================================================================
// SIM-ONLY debug instrumentation: V30 <-> MCU mailbox handshake and the
// i8751's own interrupt state.  Verilator only; synthesis never sees it.
//
// Written while chasing Ninja Spirit's sampled audio dying a few seconds into
// play.  It turned out the mailbox is innocent - the V30 rings byte 0xffe once
// per frame, forever, and the MCU acks every one - and the MCU was killing
// itself: a lost timer-0 overflow (nu8051_seq.sv, the bit-destination write-
// back) stretched its watchdog period from 18.3 ms to 98.8 ms, six V30
// doorbells piled up inside one window, and the firmware took its fatal-halt
// path.  The taps below are what made that visible, and are worth keeping:
// doorbell/ack rates, the mailbox handshake bits, the firmware's own IRAM
// watchdog bytes, and an mcu_events.log of every relevant edge.
//============================================================================
reg [31:0] dbg_tick /* verilator public_flat */ = 0;
always @(posedge CLK_32M) dbg_tick <= dbg_tick + 1;

wire dbg_ring_lvl = (cpu_mem_addr[19:16] == 4'hb) && MWR && cpu_be[0] &&
                    (cpu_mem_addr[11:1] == 11'h7ff);
wire dbg_bwr_lvl  = (cpu_mem_addr[19:16] == 4'hb) && MWR;
wire dbg_ack_lvl  = mcu_ram_cs && ~mcu_ram_we && (mcu_ram_addr[11:1] == 11'h7ff);

reg dbg_ring_q = 0, dbg_bwr_q = 0, dbg_ack_q = 0;
always @(posedge CLK_32M) begin
    dbg_ring_q <= dbg_ring_lvl;
    dbg_bwr_q  <= dbg_bwr_lvl;
    dbg_ack_q  <= dbg_ack_lvl;
end
wire dbg_ring_edge = dbg_ring_lvl & ~dbg_ring_q;
wire dbg_bwr_edge  = dbg_bwr_lvl  & ~dbg_bwr_q;

// counters
reg [31:0] dbg_ring_cnt      /* verilator public_flat */ = 0;  // V30 doorbell writes
reg [31:0] dbg_ring_lost_cnt /* verilator public_flat */ = 0;  // ...that found int_r pending
reg [31:0] dbg_ack_cnt       /* verilator public_flat */ = 0;  // MCU reads of word 0xffe
reg [31:0] dbg_mcu_wr_cnt    /* verilator public_flat */ = 0;  // MCU writes to shared RAM
reg [31:0] dbg_mcu_rd_cnt    /* verilator public_flat */ = 0;  // MCU reads of shared RAM
reg [31:0] dbg_v30_bwr_cnt   /* verilator public_flat */ = 0;  // V30 writes to 0xb0000 page
reg [31:0] dbg_smpinc_cnt    /* verilator public_flat */ = 0;  // sample bytes fetched
reg [31:0] dbg_mculatch_cnt  /* verilator public_flat */ = 0;  // z80 -> MCU latch writes
reg [31:0] dbg_last_ring_t   /* verilator public_flat */ = 0;  // dbg_tick of last doorbell
reg [31:0] dbg_last_ack_t    /* verilator public_flat */ = 0;  // dbg_tick of last MCU ack
reg [31:0] dbg_max_ring_gap  /* verilator public_flat */ = 0;  // worst doorbell interval
reg [31:0] dbg_max_ack_gap   /* verilator public_flat */ = 0;  // worst ack interval
reg [15:0] dbg_last_ring_dat /* verilator public_flat */ = 0;  // data of last doorbell write
reg  [7:0] dbg_int_r_state   /* verilator public_flat */ = 0;  // {rq,ack} snapshot
reg [31:0] dbg_int_r_hi_cnt  /* verilator public_flat */ = 0;  // cycles int_r asserted

// Every counter below is zeroed when a save state finishes restoring, so a
// scripted run measures only what happened after the state load.
wire dbg_rst = ss_restore_done;

always @(posedge CLK_32M) begin
    dbg_int_r_state <= { 6'd0, mcu_shared_ram.int_r_rq, mcu_shared_ram.int_r_ack };
    if (mcu_ram_int) dbg_int_r_hi_cnt <= dbg_int_r_hi_cnt + 1;

    if (dbg_ring_edge) begin
        dbg_ring_cnt      <= dbg_ring_cnt + 1;
        dbg_last_ring_dat <= cpu_dout;
        dbg_last_ring_t   <= dbg_tick;
        if (dbg_tick - dbg_last_ring_t > dbg_max_ring_gap)
            dbg_max_ring_gap <= dbg_tick - dbg_last_ring_t;
        if (mcu_ram_int) dbg_ring_lost_cnt <= dbg_ring_lost_cnt + 1;
    end
    if (dbg_bwr_edge) dbg_v30_bwr_cnt <= dbg_v30_bwr_cnt + 1;
    if (dbg_ack_lvl) begin
        dbg_ack_cnt    <= dbg_ack_cnt + 1;
        dbg_last_ack_t <= dbg_tick;
        if (dbg_tick - dbg_last_ack_t > dbg_max_ack_gap)
            dbg_max_ack_gap <= dbg_tick - dbg_last_ack_t;
    end
    if (mcu_ram_cs &&  mcu_ram_we) dbg_mcu_wr_cnt <= dbg_mcu_wr_cnt + 1;
    if (mcu_ram_cs && ~mcu_ram_we) dbg_mcu_rd_cnt <= dbg_mcu_rd_cnt + 1;
    if (mcu_sample_inc)            dbg_smpinc_cnt <= dbg_smpinc_cnt + 1;
    if (mculatch_en)               dbg_mculatch_cnt <= dbg_mculatch_cnt + 1;

    if (dbg_rst) begin
        dbg_ring_cnt <= 0; dbg_ring_lost_cnt <= 0; dbg_ack_cnt <= 0;
        dbg_mcu_wr_cnt <= 0; dbg_mcu_rd_cnt <= 0; dbg_v30_bwr_cnt <= 0;
        dbg_smpinc_cnt <= 0; dbg_mculatch_cnt <= 0; dbg_int_r_hi_cnt <= 0;
        dbg_last_ring_t <= dbg_tick; dbg_last_ack_t <= dbg_tick;
        dbg_max_ring_gap <= 0; dbg_max_ack_gap <= 0;
    end
end

// Firmware landmark counters, so ISR entries can be compared against the
// doorbells that are supposed to cause them.
reg [15:0] dbg_mcu_pc_q = 0;
reg [31:0] dbg_int0_cnt   /* verilator public_flat */ = 0;  // 0x04e9 INT0 ISR
reg [31:0] dbg_int1_cnt   /* verilator public_flat */ = 0;  // 0x0563 INT1 ISR
reg [31:0] dbg_t0_cnt     /* verilator public_flat */ = 0;  // 0x0501 timer-0 ISR
reg [31:0] dbg_t0_early   /* verilator public_flat */ = 0;  // 0x055c early exit (5C!=0)
reg [31:0] dbg_t0_clr_cnt /* verilator public_flat */ = 0;  // 0x054d MOV 5Dh,#0
reg [31:0] dbg_t0_kill_cnt/* verilator public_flat */ = 0;  // 0x054a MOV 44h,#80
reg [31:0] dbg_t0_inc_cnt /* verilator public_flat */ = 0;  // 0x0531 INC 44h
reg [31:0] dbg_halt_cnt2  /* verilator public_flat */ = 0;  // 0x0316 fatal entry

always @(posedge CLK_32M) begin
    if (ce_mcu) begin
        dbg_mcu_pc_q <= dbg_mcu_pc;
        if (dbg_mcu_pc != dbg_mcu_pc_q) begin
            case (dbg_mcu_pc)
            16'h04e9: dbg_int0_cnt    <= dbg_int0_cnt + 1;
            16'h0563: dbg_int1_cnt    <= dbg_int1_cnt + 1;
            16'h0501: dbg_t0_cnt      <= dbg_t0_cnt + 1;
            16'h055c: dbg_t0_early    <= dbg_t0_early + 1;
            16'h054d: dbg_t0_clr_cnt  <= dbg_t0_clr_cnt + 1;
            16'h054a: dbg_t0_kill_cnt <= dbg_t0_kill_cnt + 1;
            16'h0531: dbg_t0_inc_cnt  <= dbg_t0_inc_cnt + 1;
            16'h0316: dbg_halt_cnt2   <= dbg_halt_cnt2 + 1;
            default: ;
            endcase
        end
    end
    if (dbg_rst) begin
        dbg_int0_cnt <= 0; dbg_int1_cnt <= 0; dbg_t0_cnt <= 0; dbg_t0_early <= 0;
        dbg_t0_clr_cnt <= 0; dbg_t0_kill_cnt <= 0; dbg_t0_inc_cnt <= 0;
        dbg_halt_cnt2 <= 0;
    end
end

// ------------------------------------------------------------------
// Event log.  Verilator writes mcu_events.log next to the sim binary; the
// scripts in sim/ parse it.  Everything that matters to the V30 <-> MCU
// handshake gets a line, timestamped in CLK_32M ticks.
// ------------------------------------------------------------------
wire       dbg_iram_we    = mcu.nu8051.u_iram.p_en & mcu.nu8051.u_iram.p_we;
wire [7:0] dbg_iram_wa    = mcu.nu8051.u_iram.p_addr;
wire [7:0] dbg_iram_wd    = mcu.nu8051.u_iram.p_wdata;
reg        dbg_int_r_q    = 0;
reg        dbg_mcu_int_q  = 0;
// nu8051 interrupt/timer taps
wire       dbg_irq_ack  = mcu.nu8051.irq_ack;
wire [2:0] dbg_irq_src  = mcu.nu8051.irq_ack_src;
wire       dbg_irq_prio = mcu.nu8051.irq_ack_prio;
wire [7:0] dbg_tcon /* verilator public_flat_rd */ = mcu.nu8051.tcon_q;
wire [7:0] dbg_tl0  /* verilator public_flat_rd */ = mcu.nu8051.u_timer.tl0_r;
wire [7:0] dbg_th0  /* verilator public_flat_rd */ = mcu.nu8051.u_timer.th0_r;
wire [7:0] dbg_ie   /* verilator public_flat_rd */ = mcu.nu8051.u_irq.ie_r;
reg  [7:0] dbg_tcon_q = 0;
reg  [7:0] dbg_ie_q   = 0;

integer dbg_log = 0;
initial dbg_log = $fopen("mcu_events.log", "w");
// A wedged MCU spins on MOVX @DPTR,A at ~30k writes a frame, so the log is
// capped rather than left to eat the disk.  DBG_LOG_MAX lines is minutes of
// healthy play and still catches the wedge that follows a failure.
localparam int DBG_LOG_MAX = 400000;
reg [31:0] dbg_log_lines = 0;
wire dbg_logging = (dbg_log != 0) && (dbg_log_lines < DBG_LOG_MAX);
wire dbg_evt = dbg_rst | dbg_ring_edge | (dbg_ack_lvl & ~dbg_ack_q)
             | (mcu_ram_cs & mcu_ram_we & (mcu_ram_addr[11:2] == 10'h3ff))
             | (mcu_ram_int != dbg_int_r_q) | dbg_irq_ack
             | (dbg_tcon != dbg_tcon_q) | (dbg_ie != dbg_ie_q)
             | (dbg_iram_we & ((dbg_iram_wa == 8'h44) | (dbg_iram_wa == 8'h45) |
                               (dbg_iram_wa == 8'h5b) | (dbg_iram_wa == 8'h5c) |
                               (dbg_iram_wa == 8'h5d)));

always @(posedge CLK_32M) begin
    dbg_int_r_q <= mcu_ram_int;
    dbg_tcon_q  <= dbg_tcon;
    dbg_ie_q    <= dbg_ie;
    if (dbg_logging) begin
        if (dbg_evt) dbg_log_lines <= dbg_log_lines + 1;
        if (dbg_rst)
            $fwrite(dbg_log, "%0d RESTORE\n", dbg_tick);
        if (dbg_ring_edge)
            $fwrite(dbg_log, "%0d RING data=%04x be=%b v30pc=%05x\n",
                    dbg_tick, cpu_dout, cpu_be, dbg_v30_pc);
        if (dbg_ack_lvl & ~dbg_ack_q)
            $fwrite(dbg_log, "%0d MCURD addr=%03x\n", dbg_tick, mcu_ram_addr);
        if (mcu_ram_cs & mcu_ram_we & (mcu_ram_addr[11:2] == 10'h3ff))
            $fwrite(dbg_log, "%0d MCUWR addr=%03x data=%02x\n",
                    dbg_tick, mcu_ram_addr, mcu_ram_dout);
        if (mcu_ram_int != dbg_int_r_q)
            $fwrite(dbg_log, "%0d INT0=%0d\n", dbg_tick, mcu_ram_int);
        if (dbg_irq_ack)
            $fwrite(dbg_log, "%0d IRQACK src=%0d prio=%0d tcon=%02x ie=%02x\n",
                    dbg_tick, dbg_irq_src, dbg_irq_prio, dbg_tcon, dbg_ie);
        if (dbg_tcon != dbg_tcon_q)
            $fwrite(dbg_log, "%0d TCON %02x->%02x tl0=%02x th0=%02x mcupc=%04x\n",
                    dbg_tick, dbg_tcon_q, dbg_tcon, dbg_tl0, dbg_th0, dbg_mcu_pc);
        if (dbg_ie != dbg_ie_q)
            $fwrite(dbg_log, "%0d IE %02x->%02x mcupc=%04x\n",
                    dbg_tick, dbg_ie_q, dbg_ie, dbg_mcu_pc);
        if (dbg_iram_we && (dbg_iram_wa == 8'h44 || dbg_iram_wa == 8'h45 ||
                            dbg_iram_wa == 8'h5b || dbg_iram_wa == 8'h5c ||
                            dbg_iram_wa == 8'h5d))
            $fwrite(dbg_log, "%0d IRAM[%02x]=%02x mcupc=%04x\n",
                    dbg_tick, dbg_iram_wa, dbg_iram_wd, dbg_mcu_pc);
    end
end

// The i8751 firmware's watchdog, straight out of the PROM:
//   INT0 ISR  (0x04e9): reads mailbox bytes CFFE/CFFF, then INC 5Dh
//   timer-0   (0x0526): 5D==1 -> MOV 44h,#00; 5D==0 or 2 -> INC 44h;
//                       5D>=3 -> MOV 44h,#80h
//   main loop (0x030f): 44h >= 0x1e -> fatal halt at 0x0316 (TR0 off, EA off,
//                       AJMP 0x31d forever) - no more samples, no protection.
// So three doorbells inside one timer-0 period kill the MCU instantly.  Latch
// everything about the first time 5D reaches 3.
wire [7:0] dbg_iram_5d /* verilator public_flat_rd */ = mcu.nu8051.u_iram.mem['h5d];
wire [7:0] dbg_iram_44 /* verilator public_flat_rd */ = mcu.nu8051.u_iram.mem['h44];
reg  [7:0] dbg_5d_max  /* verilator public_flat */ = 0;
reg        dbg_trip    /* verilator public_flat */ = 0;
reg [31:0] dbg_trip_tick /* verilator public_flat */ = 0;
reg [31:0] dbg_trip_ring_cnt /* verilator public_flat */ = 0;

// last four doorbell rings and MCU acks, frozen once the trip fires
reg [31:0] dbg_ring_t0 /* verilator public_flat */ = 0;
reg [31:0] dbg_ring_t1 /* verilator public_flat */ = 0;
reg [31:0] dbg_ring_t2 /* verilator public_flat */ = 0;
reg [31:0] dbg_ring_t3 /* verilator public_flat */ = 0;
reg [31:0] dbg_ring_pc0 /* verilator public_flat */ = 0;
reg [31:0] dbg_ring_pc1 /* verilator public_flat */ = 0;
reg [31:0] dbg_ring_pc2 /* verilator public_flat */ = 0;
reg [31:0] dbg_ring_pc3 /* verilator public_flat */ = 0;
reg [31:0] dbg_ack_t0  /* verilator public_flat */ = 0;
reg [31:0] dbg_ack_t1  /* verilator public_flat */ = 0;
reg [31:0] dbg_ack_t2  /* verilator public_flat */ = 0;
reg [31:0] dbg_ack_t3  /* verilator public_flat */ = 0;

wire [31:0] dbg_v30_pc = { 4'd0, dbg_v30_regs[159:144] } * 32'd16 +
                         { 16'd0, dbg_v30_regs[207:192] };

always @(posedge CLK_32M) begin
    if (dbg_iram_5d > dbg_5d_max) dbg_5d_max <= dbg_iram_5d;
    if (!dbg_trip) begin
        if (dbg_ring_edge) begin
            dbg_ring_t3 <= dbg_ring_t2; dbg_ring_t2 <= dbg_ring_t1;
            dbg_ring_t1 <= dbg_ring_t0; dbg_ring_t0 <= dbg_tick;
            dbg_ring_pc3 <= dbg_ring_pc2; dbg_ring_pc2 <= dbg_ring_pc1;
            dbg_ring_pc1 <= dbg_ring_pc0; dbg_ring_pc0 <= dbg_v30_pc;
        end
        if (dbg_ack_lvl && !dbg_ack_q) begin
            dbg_ack_t3 <= dbg_ack_t2; dbg_ack_t2 <= dbg_ack_t1;
            dbg_ack_t1 <= dbg_ack_t0; dbg_ack_t0 <= dbg_tick;
        end
        if (dbg_iram_5d >= 8'd3) begin
            dbg_trip          <= 1'b1;
            dbg_trip_tick     <= dbg_tick;
            dbg_trip_ring_cnt <= dbg_ring_cnt;
        end
    end
    if (dbg_rst) begin
        dbg_trip <= 0; dbg_trip_tick <= 0; dbg_trip_ring_cnt <= 0; dbg_5d_max <= 0;
        dbg_ring_t0 <= 0; dbg_ring_t1 <= 0; dbg_ring_t2 <= 0; dbg_ring_t3 <= 0;
        dbg_ring_pc0 <= 0; dbg_ring_pc1 <= 0; dbg_ring_pc2 <= 0; dbg_ring_pc3 <= 0;
        dbg_ack_t0 <= 0; dbg_ack_t1 <= 0; dbg_ack_t2 <= 0; dbg_ack_t3 <= 0;
    end
end

// MCU program counter, and a sticky record of the fatal-halt window.
wire [15:0] dbg_mcu_pc /* verilator public_flat_rd */ = mcu.nu8051.u_seq.pc;
reg  [15:0] dbg_mcu_pc_min /* verilator public_flat */ = 16'hffff;
reg  [15:0] dbg_mcu_pc_max /* verilator public_flat */ = 0;
reg [31:0] dbg_mcu_halt_cnt /* verilator public_flat */ = 0;
always @(posedge CLK_32M) begin
    if (ce_mcu) begin
        if (dbg_mcu_pc < dbg_mcu_pc_min) dbg_mcu_pc_min <= dbg_mcu_pc;
        if (dbg_mcu_pc > dbg_mcu_pc_max) dbg_mcu_pc_max <= dbg_mcu_pc;
        if (dbg_mcu_pc >= 16'h031d && dbg_mcu_pc <= 16'h0320)
            dbg_mcu_halt_cnt <= dbg_mcu_halt_cnt + 1;
    end
    if (dbg_rst) begin
        dbg_mcu_pc_min <= 16'hffff; dbg_mcu_pc_max <= 0; dbg_mcu_halt_cnt <= 0;
    end
end
`endif

wire [7:0] mculatch_data = board_cfg.main_mculatch ? cpu_dout[7:0] : snd_io_data;
wire mculatch_en = board_cfg.main_mculatch ? ( IOWR && cpu_mem_addr[7:1] == 7'h60 && cpu_be[0] ) : ( snd_io_req && snd_io_addr == 8'h82 );

mcu #(.SS_IDX(SSIDX_MCU_CPU), .SS_IDX_EMU(SSIDX_MCU_EMU)) mcu(
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

    .ssbus(ssb[SSIDX_MCU_CPU]),
    .ssbus_emu(ssb[SSIDX_MCU_EMU])
);

wire [1:0] z80_sample_addr_wr, mcu_sample_addr_wr;
wire [15:0] z80_sample_addr, mcu_sample_addr;
wire [7:0] sample_rom_data;
wire [7:0] z80_sample_out;
wire z80_sample_inc, mcu_sample_inc;

sample_rom #(.SS_IDX(SSIDX_SAMPLE)) sample_rom(
    .clk(CLK_32M),
    .sample_addr_in(m84 ? z80_sample_addr : mcu_sample_addr),
    .sample_addr_wr(m84 ? z80_sample_addr_wr : mcu_sample_addr_wr),

    .sample_data(sample_rom_data),
    .sample_inc(m84 ? z80_sample_inc : mcu_sample_inc),
    
    .clk_bram(clk_bram),
    .bram_wr(bram_wr),
    .bram_data(bram_data),
    .bram_addr(bram_addr),
    .bram_cs(bram_cs[1]),

    .ssbus(ssb[SSIDX_SAMPLE])
);

// Sample DAC.  The MCU (port 1) / M84 Z80 sample port is an unsigned code
// centred on 8'h80, so `- 8'h80` makes it signed.  Boards with no sample DAC
// fitted -- R-Type is the only supported set -- load no sample region, yet
// those ports still idle at a non-silent code: the 8051's port 1 resets to
// 8'hFF, which becomes a full-scale +127 and lands on the output as a constant
// DC through samples_lpf.  Hold the DAC at the silence code unless a sample
// region was actually downloaded.  (`samples_present` is a load-time constant
// written in the clk_bram domain and read here, exactly like `mcu_emulator`'s
// `active`; it settles long before the core leaves reset.  Like `rom.sv`'s own
// `stage = BOARD_CFG`, it relies on power-on initialisation - MiSTer reloads
// the bitstream when a different MRA is selected, so each ROM load starts from
// a fresh configuration.)
reg samples_present = 0;
always @(posedge clk_bram) if (bram_wr & bram_cs[1]) samples_present <= 1;

wire [7:0] sample_dac = samples_present ? ( m84 ? z80_sample_out : mcu_sample_out )
                                        : 8'h80;
wire [7:0] signed_mcu_sample = sample_dac - 8'h80;
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

endmodule
