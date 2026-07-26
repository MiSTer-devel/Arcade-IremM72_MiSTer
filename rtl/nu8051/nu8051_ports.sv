//============================================================================
// Imported from nu8051 (machine-cycle-accurate Intel 8051/8052 core, SystemVerilog)
// Source: rtl/nu8051_ports.sv, commit e2c97de0fabb3b3baa1eb98a15284bc74ef92f4e
// Do not hand-edit; re-import from upstream. The M72 integration wrapper is
// rtl/mcu.sv (distilled from nu8051 rtl/m72/mcu.sv), NOT a raw re-import.
//============================================================================
//============================================================================
//
//  nu8051_ports - P0..P3 latches, pin-sample registers, output folds
//                 (core_design.md §1.5, §5.4 row 2, §7.2)
//
//  * Per port bit: latch flop (the SFR), pin-sample register (Px_IN latched
//    at the S5P1 edge, TQ2) and the driven value Px_OUT = latch & alt_out
//    (periph §3.5).  Because the latch only ever changes at an S6P2 commit
//    edge, "the output buffer samples the latch during Phase 1" is exactly
//    reproduced by driving Px_OUT from the latch: a value committed at S6P2
//    is first *observable* in the S1P1 record of the next cycle, which is
//    what CAD-11 checks.
//  * Read path: read-modify-write instructions read the LATCH (d_rmw, timing
//    §5 list); every other read returns the S5P1 pin sample (periph §3.1).
//    The sample is `Px_IN & latch` - the RESOLVED pin - because a latch bit of
//    0 pulls its pin low regardless of the external driver (periph §3.2/§3.3;
//    the reference model's `latch & port_in`).  QUESTION-P47-3.
//  * P0 is open-drain GPIO: P0_OEN = ~latch (periph §3.3).  P1..P3 are
//    quasi-bidirectional, OEN constant FFH (§1.2).
//  * TQ4: the P0 latch is written FFH at the S6P2 edge of EVERY machine cycle
//    that contained external bus activity (external fetch slot or MOVX).
//
//============================================================================

`timescale 1ns/1ps

module nu8051_ports (
    input  logic        CLK,
    input  logic        CE,
    input  logic        rst_hold,

    input  logic        tk_s5p1,          // pin-sample tick (TQ2)
    input  logic        tk_s6p2,          // commit tick
    input  logic        bus_active,       // this cycle drove the external bus

    // SFR bus
    input  logic  [7:0] sfr_addr,
    input  logic        sfr_rmw,
    output logic  [7:0] sfr_rdata,
    output logic        sfr_hit,
    input  logic        sfr_we,
    input  logic  [7:0] sfr_wdata,

    // pins
    input  logic  [7:0] P0_IN, P1_IN, P2_IN, P3_IN,
    output logic  [7:0] P0_OUT, P1_OUT, P2_OUT, P3_OUT,
    output logic  [7:0] P0_OEN, P1_OEN, P2_OEN, P3_OEN,

    // alternate-function folds (§1.5); all inactive-high in Phase 3
    input  logic        alt_rd_n,         // -> P3.7
    input  logic        alt_wr_n,         // -> P3.6
    input  logic        alt_txd,          // -> P3.1
    input  logic        alt_rxd_oe,
    input  logic        alt_rxd_out,      // -> P3.0 when alt_rxd_oe

    output logic  [7:0] p2_latch_q,       // MOVX @Ri high address byte (TQ10)

    // save state (savestate_design §5.1/§5.2, map 0x060-0x067).  Eight
    // symbols, eight flops: the four latches and the four S5P1 pin samples.
    // The four `Px_OUT` symbols D1.2 originally carried were deleted by edit
    // E-3 (QUESTION-P6-2) - Px_OUT is a continuous assignment from the latch
    // (P3 folding the mapped UART state), never a register.
    input  logic  [9:0] ss_addr,
    input  logic [15:0] ss_wdata,
    input  logic        ss_we,
    output logic [15:0] ss_rdata

`ifdef NU8051_BACKDOOR
    ,
    input  logic        sfr_bkd_we
`endif
);

`include "nu8051_defs.svh"
    import nu8051_ss_pkg::*;

    logic [7:0] latch [0:3];
    logic [7:0] pin_s [0:3];

    wire [7:0] pin_in [0:3];
    assign pin_in[0] = P0_IN;
    assign pin_in[1] = P1_IN;
    assign pin_in[2] = P2_IN;
    assign pin_in[3] = P3_IN;

    assign p2_latch_q = latch[2];

    // ---- alternate-output fold (periph §3.5) ------------------------------
    wire [7:0] alt3 = {alt_rd_n, alt_wr_n, 1'b1, 1'b1, 1'b1, 1'b1,
                       alt_txd, (alt_rxd_oe ? alt_rxd_out : 1'b1)};

    assign P0_OUT = latch[0];
    assign P1_OUT = latch[1];
    assign P2_OUT = latch[2];
    assign P3_OUT = latch[3] & alt3;

    assign P0_OEN = ~latch[0];            // open drain (periph §3.3)
    assign P1_OEN = 8'hFF;
    assign P2_OEN = 8'hFF;
    assign P3_OEN = 8'hFF;

    // ---- SFR decode -------------------------------------------------------
    function automatic logic [1:0] port_of(input logic [7:0] a);
        case (a)
            SFR_P0:  port_of = 2'd0;
            SFR_P1:  port_of = 2'd1;
            SFR_P2:  port_of = 2'd2;
            default: port_of = 2'd3;
        endcase
    endfunction

    wire is_port = (sfr_addr == SFR_P0) || (sfr_addr == SFR_P1)
                || (sfr_addr == SFR_P2) || (sfr_addr == SFR_P3);
    wire [1:0] pidx = port_of(sfr_addr);

    assign sfr_hit   = is_port;
    // QUESTION-P51-1: the pin-sample register is TRANSPARENT during its own
    // sampling phase.  `tk_s5p1` is the S5P1 phase level, and the seq's R3
    // late slot captures the SFR bus at the edge ENDING S5P1 (core_design
    // §2.2/§5.1) - the very edge at which `pin_s` takes the new sample.  A
    // plain register read there would hand the reader the PREVIOUS cycle's
    // sample, i.e. one machine cycle of read-back latency that neither the
    // manual nor core_design §9 ("every read-pin data path uses the LATEST
    // sample") allows: a latch written at S6P2 is on the pin from the next
    // S1P1 (timing §5) and that pin is what the next cycle's S5P1 sample -
    // and hence an instruction executing in that cycle - must see.  During
    // S5P1 the read therefore returns the live resolved node, which is bit
    // for bit the value the register is capturing at the end of the phase;
    // outside S5P1 (the R1/R2 slots at S2P1/S2P2) it returns the register,
    // which is then correctly the previous cycle's sample.
    // Falsifier that fired: tests/directed_rtl/c5_1 `b-late-*-sep0` (4 of
    // 59) - `MOV P1,#00H` then `MOV A,P1` read FFH.
    wire [7:0] pin_rd = tk_s5p1 ? (pin_in[pidx] & latch[pidx]) : pin_s[pidx];
    // RMW reads take the latch, everything else the S5P1 pin sample (§5.5)
    assign sfr_rdata = sfr_rmw ? latch[pidx] : pin_rd;

    // ---- save-state read mux (registered, savestate_design §5.1) ----------
    always_ff @(posedge CLK) begin
        case (ss_addr)
            SSA_P_P0_LATCH: ss_rdata <= {8'b0, latch[0]};
            SSA_P_P1_LATCH: ss_rdata <= {8'b0, latch[1]};
            SSA_P_P2_LATCH: ss_rdata <= {8'b0, latch[2]};
            SSA_P_P3_LATCH: ss_rdata <= {8'b0, latch[3]};
            SSA_P_P0_IN:    ss_rdata <= {8'b0, pin_s[0]};
            SSA_P_P1_IN:    ss_rdata <= {8'b0, pin_s[1]};
            SSA_P_P2_IN:    ss_rdata <= {8'b0, pin_s[2]};
            SSA_P_P3_IN:    ss_rdata <= {8'b0, pin_s[3]};
            default:        ss_rdata <= 16'h0000;
        endcase
    end

    always_ff @(posedge CLK) begin
        // ---- save-state write decode (§5.2), restore-priority position ----
        if (ss_we) begin
            case (ss_addr)
                SSA_P_P0_LATCH: latch[0] <= ss_wdata[7:0];
                SSA_P_P1_LATCH: latch[1] <= ss_wdata[7:0];
                SSA_P_P2_LATCH: latch[2] <= ss_wdata[7:0];
                SSA_P_P3_LATCH: latch[3] <= ss_wdata[7:0];
                SSA_P_P0_IN:    pin_s[0] <= ss_wdata[7:0];
                SSA_P_P1_IN:    pin_s[1] <= ss_wdata[7:0];
                SSA_P_P2_IN:    pin_s[2] <= ss_wdata[7:0];
                SSA_P_P3_IN:    pin_s[3] <= ss_wdata[7:0];
                default: ;
            endcase
        end else if (CE) begin
            if (rst_hold) begin
                for (int i = 0; i < 4; i++) begin
                    latch[i] <= 8'hFF;         // §7.2
                    pin_s[i] <= pin_in[i];
                end
            end else begin
                // The sampled PIN is the resolved node, not the external
                // driver alone: a latch bit of 0 turns the output nFET on and
                // pulls the pin low whatever the outside world drives
                // (periph §3.2 quasi-bidirectional, §3.3 P0 open-drain), which
                // is exactly the reference model's `latch & port_in` read
                // (periph §3.1 / i8051.cpp:282-321).  Without the AND the core
                // can read a 1 out of a pin its own latch is holding down.
                // QUESTION-P47-3: found by the C4.7 seq corpus (seq52
                // `seq-0491`: `CPL P2.3` then `MOV A,P2` - the first program in
                // 1000 to write a port latch low and later read that pin).  No
                // existing vector changes: every one of them drives pins that
                // are already a subset of the latch (QUESTION-T22-3).
                if (tk_s5p1)
                    for (int i = 0; i < 4; i++)
                        pin_s[i] <= pin_in[i] & latch[i];
                if (tk_s6p2) begin
                    if (sfr_we && is_port) latch[pidx] <= sfr_wdata;
                    // TQ4 wins over an explicit write in the same cycle: the
                    // bus really does drive FFH into the P0 latch.
                    if (bus_active) latch[0] <= 8'hFF;
                end
            end
        end
`ifdef NU8051_BACKDOOR
        else if (sfr_bkd_we && is_port) begin
            latch[pidx] <= sfr_wdata;
        end
`endif
    end

    initial begin
        for (int i = 0; i < 4; i++) begin
            latch[i] = 8'hFF;
            pin_s[i] = 8'hFF;
        end
        ss_rdata = 16'h0000;
    end

endmodule
