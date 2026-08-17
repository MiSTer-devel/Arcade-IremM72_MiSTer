//============================================================================
//  Irem M72 for MiSTer FPGA - 8051 protection and sample playback MCU
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
//
//  PHASE-7 REWRITE (nu8051 PLAN §6.4, core_design.md §8 conformance walk).
//  Drop-in replacement for the Oregano-core mcu.sv: same module name, same
//  port list, same board semantics - a different CPU inside and one fewer
//  lie about time.
//
//  WHAT WENT AWAY
//
//  * `delayed_ce_count` / `delayed_ce`.  The old glue stalled CE for 14
//    ticks after every shared-RAM MOVX write to fake a real i8751's speed,
//    because the Oregano core retires MOVX in a handful of clocks instead of
//    the two machine cycles the part takes.  nu8051 is machine-cycle
//    accurate: a MOVX is 24 CE ticks = 3.0 us at 8 MHz, intrinsically.  CE
//    is plain `ce_8m` now (CD §8, first table row).
//  * The 128-byte `dpramv_cen #(.widthad_a(7)) internal_ram` and its five
//    wires.  IRAM lives inside the core (and reaches a save state through
//    the core's SS window, D1.2) instead of hanging off a port.
//  * The registered `ext_src` read-mux state machine.  The read mux is
//    combinational off `MEM_ADDR` now, which is stable from S4P2 of the
//    MOVX's first cycle - about nine CE ticks before the core samples - so
//    the autoincrement can no longer race the byte it is meant to deliver
//    (CD §8, "Required glue rework").
//
//  WHAT THE STROBES DO NOW - the C4.8 §8 requirement
//
//  The Oregano core pulsed `memx_o` for one CE tick.  This core holds
//  RD#/WR# low for SIX (S1P1..S3P2 of the MOVX's second cycle, timing §4),
//  because that is what the part does.  A level-triggered `if (ext_cs)`
//  block would therefore fire `sample_inc`, the Z80 ack and the shared-RAM
//  write SIX TIMES per MOVX.  Every side effect below is consequently
//  EDGE-qualified, and the two directions deliberately take different
//  edges:
//
//    wr_stb = MEM_WR_N rising edge (end S3P2 C2) - address and MEM_DOUT are
//             valid until S4P1 C2, three ticks later, so the one-shot write
//             still sees both.
//    rd_stb = MEM_RD_N rising edge - i.e. AFTER the core has sampled
//             MEM_DIN at that same edge, so `sample_inc` bumps the pointer
//             behind the read rather than under it.
//
//  Both are single-CLK_32M-cycle pulses.  That matters beyond tidiness:
//  `sample_rom.sv` increments its 18-bit pointer once per clock while
//  `sample_inc` is high, and `dualport_mailbox_2kx16` re-evaluates its
//  interrupt request on every clock `cs_r` is asserted.
//
//  THE THREE CORRECTED FACTS (PLAN §6.4, QUESTION-P56-6; RTL outranks the
//  prose in docs/mcu.txt in all three)
//
//   (a) INT1's clear is the WRITE to 0x0002, not a read.  mcu.txt's register
//       line ("write to acknowledge") is right and its interrupt-summary
//       line ("cleared on read") is wrong.
//   (b) INT0's clear is the MCU reading the MB8421 mailbox WORD - bytes
//       0xCFFE/0xCFFF (dualport_mailbox.sv:75, `addr_r[11:1] == 'h7ff`).
//       0xC7FE is only a protocol rendezvous byte the HLE polls, and reading
//       it clears nothing.  The clear is not implemented here at all: it is
//       the mailbox's, and it happens because this wrapper presents the read
//       as a proper `cs_r & ~we_r` strobe at the right address.
//   (c) The sample pointer is `{high, low} << 5` held as TWO INDEPENDENT
//       FIELDS - 0x0000 writes the low field, 0x0001 the high field, in
//       either order (sample_rom.sv:39-40).  This wrapper stores the two
//       bytes and lets `sample_rom.sv` do the shift, exactly as before.
//
//  SAVE STATES (savestate_design.md §8.3)
//
//  The core's SS bus is TIED OFF here, because the M72 core has no
//  save-state infrastructure to connect it to and adding ports would break
//  the drop-in.  When it gains one, three wrapper registers must be
//  snapshotted alongside the core's 353-word stream - they are named
//  `ss_glue_*` below so a future hook can find them:
//      sample_addr[15:0]  the two pointer setup fields
//      z80_latch[7:0]     the Z80 sound-latch byte
//      z80_latch_int      its INT1 level
//  Everything else here is either combinational or a one-shot strobe that is
//  quiescent at any instant the platform would freeze at.  The live 18-bit
//  playback pointer is `sample_rom.sv`'s, not this module's, and belongs to
//  the platform's own snapshot - as does the mailbox's `int_r` level.
//
//  INTEGRATION NOTE - what the drop-in must re-attach.  The old mcu.sv also
//  instantiated `mcu_emulator` (the HLE fallback for sets with no protection
//  ROM dump) and muxed eight of its outputs against the real MCU's on
//  `emulator_active`.  That mux is platform policy, unchanged by this
//  rewrite and deliberately absent from this file so the wrapper is testable
//  standalone in the nu8051 repo; re-add it verbatim around this module's
//  outputs (old mcu.sv lines 78-96 and 228-266) when dropping in.
//
//============================================================================

`timescale 1ns/1ps

module mcu #(
    parameter SS_IDX     = -1,   // SSIDX_MCU_CPU  (nu8051 core + wrapper glue)
    parameter SS_IDX_EMU = -1    // SSIDX_MCU_EMU  (forwarded to mcu_emulator)
) (
    input             CLK_32M,
    input             ce_8m,
    input             reset,

    // shared ram
    output     [11:0] ext_ram_addr,
    input       [7:0] ext_ram_din,
    output      [7:0] ext_ram_dout,
    output            ext_ram_cs,
    output            ext_ram_we,
    input             ext_ram_int,

    // z80 latch
    input       [7:0] z80_din,
    input             z80_latch_en,

    // sample output, 8-bit unsigned
    output      [7:0] sample_out,

    output      [1:0] sample_addr_wr,
    output     [15:0] sample_addr,
    output            sample_inc,
    input       [7:0] sample_rom_data,

    // ioctl
    input             clk_bram,
    input             bram_wr,
    input       [7:0] bram_data,
    input      [19:0] bram_addr,
    input             bram_prom_cs,
    input             bram_samples_cs,
    input             bram_offsets_cs,
    input             bram_protect_cs,

    // savestate: MCU_CPU slave (nu8051 core linear window + wrapper glue) and
    // the MCU_EMU slave forwarded straight through to the mcu_emulator below.
    ssbus_if.slave    ssbus,
    ssbus_if.slave    ssbus_emu
);

    // ------------------------------------------------------------------
    // the core
    // ------------------------------------------------------------------
    // P_8052 = 0: the M72 MCU is an i8751H - a plain 8051.  No timer 2, no
    // T2CON/RCAP2, 128 bytes of IRAM, five interrupt sources.  Proven as a
    // separate elaboration by the C5.5 8051-config regression and measured
    // as its own synthesis build in Phase 7 (`syn/build.tcl tieoff8051`).
    // ROM_AW = 13: the 8 KB protection PROM.
    localparam bit P_8052_M72 = 1'b0;
    localparam int ROM_AW_M72 = 13;

    wire [15:0] mem_addr;
    wire  [7:0] mem_dout;
    wire        mem_rd_n, mem_wr_n;
    wire [ROM_AW_M72-1:0] rom_addr;
    wire  [7:0] rom_data;
    wire  [7:0] p1_out;

    // ------------------------------------------------------------------
    // XDATA decode  (declared ahead of the core so `ext_din` is in scope)
    // ------------------------------------------------------------------
    //   0x0000  R = sample byte at the pointer, then pointer++
    //           W = pointer LOW field
    //   0x0001  W = pointer HIGH field      (no read arm on the board)
    //   0x0002  R = Z80 sound latch
    //           W = acknowledge -> drops INT1              [corrected fact a]
    //   0xC000-0xCFFF  the 4 KB MB8421 shared with the V30; the mailbox word
    //           0xCFFE/0xCFFF carries INT0's clear-on-read [corrected fact b]
    //           and 0xCFFC/0xCFFD is the MCU's answer, which interrupts the
    //           V30.  Firmware reaches this page with MOVX @Ri and P2 = 0xC7
    //           etc., which works because MEM_ADDR[15:8] is the P2 latch on
    //           the @Ri form (TQ10).
    wire sel_sample = (mem_addr == 16'h0000);
    wire sel_ptr_hi = (mem_addr == 16'h0001);
    wire sel_z80    = (mem_addr == 16'h0002);
    wire sel_ram    = (mem_addr[15:12] == 4'hC);

    // the three save-state glue registers (savestate_design §8.3)
    reg  [15:0] ss_glue_sample_addr = 16'd0;   // two independent byte fields
    reg   [7:0] ss_glue_z80_latch   = 8'd0;
    reg         ss_glue_z80_int     = 1'b0;

    wire [7:0] z80_latch     = ss_glue_z80_latch;
    wire       z80_latch_int = ss_glue_z80_int;

    // The read mux, combinational off MEM_ADDR (CD §8): stable from S4P2 of
    // the MOVX's first cycle, ~9 CE ticks before the core samples MEM_DIN at
    // the RD# rising edge.
    wire [7:0] ext_din = sel_sample ? sample_rom_data
                       : sel_z80    ? z80_latch
                                    : ext_ram_din;

    // The PROM is loaded over ioctl at run time; until the first byte lands
    // the core would execute an array of zeros, so reset is held (unchanged
    // from the old wrapper).  It also satisfies the core's >= 24 CE tick
    // minimum reset width trivially.
    reg valid_rom = 1'b0;
    always @(posedge clk_bram) if (bram_prom_cs & bram_wr) valid_rom <= 1'b1;

    // ------------------------------------------------------------------
    // MCU_CPU savestate slave (SSIDX_MCU_CPU)
    // ------------------------------------------------------------------
    // The linear 0x000-0x2FF core SS window (768 words) is streamed straight
    // through: the core returns real data for mapped addresses and 0 for the
    // gaps, and on restore writes to unmapped addresses are no-ops (only the
    // tag@0x000 self-checks and round-trips).  Three wrapper glue words follow
    // it (768 sample_addr, 769 z80_latch, 770 z80_int).  The read path uses
    // the V30 slave's 2-clk SS_ADDR->SS_RDATA staging (rtl/v30/v30_bus.sv);
    // writes pulse SS_WE for one CLK.
    localparam int SS_CORE_WORDS = 768;   // 0x000-0x2FF linear
    wire        ss_is_core   = ssbus.addr < SS_CORE_WORDS;
    wire [15:0] ss_core_rdata;
    wire        ss_core_err /* verilator public_flat */;
    reg  [1:0]  ss_rd_delay = 2'd0;
    reg         ss_wr_done  = 1'b0;
    // The core is held in RESET whenever no MCU PROM was loaded (games with no
    // i8751, e.g. R-Type: valid_rom stays 0).  Its SS park-contract (A-SS2)
    // forbids SS_WE during RESET, and restoring the all-reset core state into
    // an already-reset core is a no-op, so SS core writes are gated off then.
    wire        core_in_reset = reset | ~valid_rom;
    wire        ss_core_we  = ssbus.access(SS_IDX) & ssbus.write & ss_is_core
                              & ~ss_wr_done & ~core_in_reset;

    /* verilator lint_off PINCONNECTEMPTY */
    nu8051_core #(.P_8052(P_8052_M72), .ROM_AW(ROM_AW_M72)) nu8051 (
        .CLK        (CLK_32M),
        .CE         (ce_8m),            // no delayed_ce; the core keeps time
        .RESET      (reset | ~valid_rom),

        // de-muxed external bus.  M72 wires XDATA here and leaves the muxed
        // P0/P2 surface alone entirely (P0..P3 are not board signals on this
        // MCU except P1), so `nu8051_pins` is not in this path.
        .MEM_ADDR   (mem_addr),
        .MEM_DOUT   (mem_dout),
        .MEM_DIN    (ext_din),
        .MEM_ALE    (),
        .MEM_PSEN_N (),
        .MEM_RD_N   (mem_rd_n),
        .MEM_WR_N   (mem_wr_n),
        .EA_N       (1'b1),             // all code from the internal PROM

        .ROM_ADDR   (rom_addr),
        .ROM_DATA   (rom_data),

        // P1 is the 8-bit unsigned PCM port to the DAC.  Everything else is
        // unbonded on the board: inputs read 1, outputs go nowhere.
        .P0_IN      (8'hFF), .P1_IN (8'hFF), .P2_IN (8'hFF), .P3_IN (8'hFF),
        .P0_OUT     (),      .P1_OUT(p1_out), .P2_OUT(), .P3_OUT(),
        .P0_OEN     (),      .P1_OEN(),       .P2_OEN(), .P3_OEN(),

        // INT0 = the V30 mailbox, INT1 = the Z80 sound latch.  Both are
        // LEVEL sources presented active-low (firmware runs IT0 = IT1 = 0);
        // IEx is the inverted pin re-sampled at every S5P2, so an ack drops
        // the request within the same machine cycle that performs it.
        .INT0_N     (~ext_ram_int),
        .INT1_N     (~z80_latch_int),
        .T0         (1'b1),
        .T1         (1'b1),
        .T2         (1'b1),
        .T2EX       (1'b1),
        .RXD_IN     (1'b1),
        .RXD_OUT    (),
        .RXD_OE     (),
        .TXD        (),

        // save state: driven by the MCU_CPU slave below.  SS_ADDR is the raw
        // linear ssbus address (no dense<->sparse translation); SS_WE is gated
        // to the 0x000-0x2FF window so glue-word writes never reach the core.
        .SS_ADDR    (ssbus.addr[9:0]),
        .SS_WDATA   (ssbus.data[15:0]),
        .SS_WE      (ss_core_we),
        .SS_RDATA   (ss_core_rdata),
        .SS_ERR     (ss_core_err)
`ifdef NU8051_BACKDOOR
        // Not the verification path - the core TB drives `nu8051_core`
        // directly - but `check_core.py` discovers its compilation units by
        // globbing rtl/*.sv, so this instance must still elaborate in a
        // -DNU8051_BACKDOOR build.  Tied off: no boundary force, no byte
        // port, no way for the wrapper to perturb a vector run.
        , .bkd_load  (1'b0)
        , .bkd_pc    (16'd0)
        , .bkd_addr  (9'd0)
        , .bkd_wdata (8'd0)
        , .bkd_we    (1'b0)
        /* verilator lint_off PINCONNECTEMPTY */
        , .bkd_rdata ()
        /* verilator lint_on PINCONNECTEMPTY */
`endif
    );
    /* verilator lint_on PINCONNECTEMPTY */

    wire [7:0] mcu_sample_out = p1_out;

    // ------------------------------------------------------------------
    // program ROM - the §1.4 ROM-1 contract, verbatim
    // ------------------------------------------------------------------
    // "ROM_ADDR captured at a posedge CLK where CE==1; ROM_DATA valid before
    // the next CE-enabled posedge" IS `dpramv_cen` with `cen_a = ce_8m`.
    // ROM-3 gives it three CE ticks of slack it does not need.
    dpramv_cen #(.widthad_a(13)) prom (
        .clock_a  (CLK_32M),
        .address_a(rom_addr),
        .q_a      (rom_data),
        .wren_a   (1'b0),
        .data_a   (8'd0),
        .cen_a    (ce_8m),

        .clock_b  (clk_bram),
        .address_b(bram_addr[12:0]),
        .data_b   (bram_data),
        .wren_b   (bram_prom_cs & bram_wr),
        /* verilator lint_off PINCONNECTEMPTY */
        .q_b      (),
        /* verilator lint_on PINCONNECTEMPTY */
        .cen_b    (1'b1)
    );

    // ------------------------------------------------------------------
    // the two strobe edges
    // ------------------------------------------------------------------
    // Sampled on CLK_32M, not on ce_8m: the core's strobe outputs only move
    // on enabled edges anyway, so a plain edge detector yields exactly one
    // CLK_32M-wide pulse per MOVX, one cycle after the strobe's own edge.
    // The address and (for writes) the data are still valid there - both are
    // held to S4P1 of the second cycle, twelve CLK_32M cycles later.
    reg rd_n_q = 1'b1, wr_n_q = 1'b1;
    always @(posedge CLK_32M) begin
        rd_n_q <= mem_rd_n;
        wr_n_q <= mem_wr_n;
    end
    wire rd_stb = mem_rd_n & ~rd_n_q;   // read complete: core has its byte
    wire wr_stb = mem_wr_n & ~wr_n_q;   // write capture instant

    // ------------------------------------------------------------------
    // shared RAM
    // ------------------------------------------------------------------
    // Address and data are combinational so the mailbox's free-running
    // registered read port has the whole MOVX cycle to produce `ext_ram_din`
    // (CD §8: valid by end S3P2 C2 - met with ~20 CLK_32M to spare).  Only
    // `cs`/`we` are pulsed, which is what the mailbox's interrupt logic
    // wants: it evaluates on every clock `cs_r` is high.
    wire [11:0] mcu_ext_ram_addr = mem_addr[11:0];
    wire  [7:0] mcu_ext_ram_dout = mem_dout;
    wire        mcu_ext_ram_cs   = sel_ram & (rd_stb | wr_stb);
    wire        mcu_ext_ram_we   = sel_ram & wr_stb;

    // ------------------------------------------------------------------
    // sample port + Z80 latch  (the ss_glue_* registers declared above)
    // ------------------------------------------------------------------
    reg   [1:0] sample_addr_wr_r    = 2'b00;
    reg         sample_inc_r        = 1'b0;

    wire [15:0] mcu_sample_addr    = ss_glue_sample_addr;
    wire  [1:0] mcu_sample_addr_wr = sample_addr_wr_r;
    wire        mcu_sample_inc     = sample_inc_r;

    always @(posedge CLK_32M) begin
        // one-shots: default low, raised for exactly one cycle below
        sample_addr_wr_r <= 2'b00;
        sample_inc_r     <= 1'b0;

        // The Z80 and the V30 do not share the MCU's clock enable, so the
        // board side runs on the raw clock.
        if (z80_latch_en) begin
            ss_glue_z80_latch <= z80_din;
            ss_glue_z80_int   <= 1'b1;
        end

        if (wr_stb) begin
            // [corrected fact c] two independent fields; sample_rom.sv does
            // the <<5.  Either setup order is legal, which is what the
            // s-pointer-setup / s-pointer-setup-low-first pair proves.
            if (sel_sample) begin
                ss_glue_sample_addr[7:0] <= mem_dout;
                sample_addr_wr_r         <= 2'b01;
            end
            if (sel_ptr_hi) begin
                ss_glue_sample_addr[15:8] <= mem_dout;
                sample_addr_wr_r          <= 2'b10;
            end
            // [corrected fact a] the WRITE is the acknowledge.  Ordered
            // after the `z80_latch_en` arm above, so a latch write and its
            // acknowledge landing in the same CLK_32M cycle resolve as the
            // acknowledge - the priority the shipping wrapper and the
            // verified TB environment model both have.
            if (sel_z80) ss_glue_z80_int <= 1'b0;
        end

        if (rd_stb) begin
            // "Read, sample byte and increment sample pointer": the byte the
            // core just took is the PRE-increment one, because this pulse is
            // a cycle behind the RD# rising edge it triggers on.
            if (sel_sample) sample_inc_r <= 1'b1;
        end

        if (reset) begin
            ss_glue_z80_int  <= 1'b0;
            sample_addr_wr_r <= 2'b00;
            sample_inc_r     <= 1'b0;
        end

        // Savestate restore of the wrapper glue registers (this block is
        // frozen under pause, so it never fights a live strobe).  Highest
        // priority so a restore always wins.
        if (ssbus.access(SS_IDX) & ssbus.write) begin
            case (ssbus.addr)
            32'd768: ss_glue_sample_addr <= ssbus.data[15:0];
            32'd769: ss_glue_z80_latch   <= ssbus.data[7:0];
            32'd770: ss_glue_z80_int      <= ssbus.data[0];
            default: ;
            endcase
        end
    end

    // ------------------------------------------------------------------
    // MCU_CPU slave protocol: setup / enumeration / reads / acks.  The glue
    // register restore-writes live in the main block above (single driver);
    // this block only pulses ack and gates the 2-clk core read staging.
    // ------------------------------------------------------------------
    always @(posedge CLK_32M) begin
        ssbus.setup(SS_IDX, SS_CORE_WORDS + 3, 1);   // 771 x 16-bit

        if (ssbus.access(SS_IDX)) begin
            if (ssbus.write) begin
                ss_wr_done <= 1'b1;
                ssbus.write_ack(SS_IDX);
            end else if (ssbus.read) begin
                ss_rd_delay <= { ss_rd_delay[0], 1'b1 };
                if (ss_is_core) begin
                    // core SS_RDATA valid 2 CLKs after SS_ADDR presents
                    if (ss_rd_delay[1])
                        ssbus.read_response(SS_IDX, { 48'd0, ss_core_rdata });
                end else begin
                    case (ssbus.addr)
                    32'd768: ssbus.read_response(SS_IDX, { 48'd0, ss_glue_sample_addr });
                    32'd769: ssbus.read_response(SS_IDX, { 56'd0, ss_glue_z80_latch });
                    32'd770: ssbus.read_response(SS_IDX, { 63'd0, ss_glue_z80_int });
                    default: ssbus.read_response(SS_IDX, 64'd0);
                    endcase
                end
            end
        end else begin
            ss_rd_delay <= 2'd0;
            ss_wr_done  <= 1'b0;
        end
    end

    // ------------------------------------------------------------------
    // emulator (HLE) fallback + output mux  (re-added from old mcu.sv, the
    // drop-in header's "INTEGRATION NOTE").  For sets with no protection ROM
    // dump the mcu_emulator drives the board outputs instead of the real
    // core; `emulator_active` is high exactly when a sample-offset table was
    // loaded over ioctl.  When `emulator_active==0` the mux is byte-identical
    // to the real-MCU path.
    // ------------------------------------------------------------------
    wire        emulator_active;
    wire [15:0] emulator_sample_addr;
    wire  [1:0] emulator_sample_addr_wr;
    wire        emulator_sample_inc;
    wire  [7:0] emulator_sample_out;

    wire [11:0] emulator_ext_ram_addr;
    wire  [7:0] emulator_ext_ram_dout;
    wire        emulator_ext_ram_cs;
    wire        emulator_ext_ram_we;

    assign ext_ram_addr   = emulator_active ? emulator_ext_ram_addr   : mcu_ext_ram_addr;
    assign ext_ram_dout   = emulator_active ? emulator_ext_ram_dout   : mcu_ext_ram_dout;
    assign ext_ram_cs     = emulator_active ? emulator_ext_ram_cs     : mcu_ext_ram_cs;
    assign ext_ram_we     = emulator_active ? emulator_ext_ram_we     : mcu_ext_ram_we;
    assign sample_out     = emulator_active ? emulator_sample_out     : mcu_sample_out;
    assign sample_addr    = emulator_active ? emulator_sample_addr    : mcu_sample_addr;
    assign sample_addr_wr = emulator_active ? emulator_sample_addr_wr : mcu_sample_addr_wr;
    assign sample_inc     = emulator_active ? emulator_sample_inc     : mcu_sample_inc;

    mcu_emulator #(.SS_IDX(SS_IDX_EMU)) mcu_emulator(
        .CLK_32M(CLK_32M),
        .ce_8m(ce_8m),
        .reset(reset),

        .ssbus(ssbus_emu),

        .active(emulator_active),

        // shared ram
        .ext_ram_addr(emulator_ext_ram_addr),
        .ext_ram_din(ext_ram_din),
        .ext_ram_dout(emulator_ext_ram_dout),
        .ext_ram_cs(emulator_ext_ram_cs),
        .ext_ram_we(emulator_ext_ram_we),
        .ext_ram_int(ext_ram_int),

        .z80_din(z80_din),
        .z80_latch_en(z80_latch_en),

        .sample_addr(emulator_sample_addr),
        .sample_addr_wr(emulator_sample_addr_wr),
        .sample_inc(emulator_sample_inc),
        .sample_in(sample_rom_data),
        .sample_out(emulator_sample_out),

        // ioctl
        .clk_bram(clk_bram),
        .bram_wr(bram_wr),
        .bram_data(bram_data),
        .bram_addr(bram_addr),
        .bram_offsets_cs(bram_offsets_cs),
        .bram_protect_cs(bram_protect_cs)
    );

    // bram_samples_cs is consumed by the platform's sample ROM, not by this
    // module; ss_core_err is surfaced for logging (public_flat) but has no
    // functional consumer here.  Both sunk so no UNUSED lint fires.
    wire _unused_ok = &{1'b0, bram_samples_cs, ss_core_err, 1'b0};

endmodule
