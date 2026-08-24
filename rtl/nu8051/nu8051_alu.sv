//============================================================================
// Imported from nu8051 (machine-cycle-accurate Intel 8051/8052 core, SystemVerilog)
// Source: rtl/nu8051_alu.sv, commit ad28e1b655ec7a38054174beaed157f013ea5643
// Do not hand-edit; re-import from upstream. The M72 integration wrapper is
// rtl/mcu.sv (distilled from nu8051 rtl/m72/mcu.sv), NOT a raw re-import.
//============================================================================
//============================================================================
//
//  nu8051_alu - ALU: combinational op set (core_design.md §4.1, §4.3) plus
//               the MUL/DIV serial engine (§4.2 [D-04])
//
//  Inputs a, b (8) + cin; outputs y (8), the B-side result `b_out` and the
//  C/AC/OV candidates.  Flag *writes* are masked by d_flagw and commit at S6P2
//  (TQ6) - this block only produces candidates.
//
//  SCOPE: the whole combinational op set is here (it is small and
//  self-contained, and the Phase-4 chunks need it verbatim).  Instantiated by
//  nu8051_seq, which owns every operand mux, from C4.1 on.
//
//  C4.3 adds the ONLY state this block holds: the §4.2 serial MUL/DIV unit,
//  three registers that are exactly the D1.2 save-state allocation for this
//  module (SSA_A_MD_ACC 16b / SSA_A_MD_CNT 4b / SSA_A_MD_BUSY 1b, map
//  0x040-0x042; savestate_design §2.2).  Both engines are 1 bit per CE tick
//  and share the register file:
//
//    MUL AB  md_acc = {product_hi, multiplier/product_lo}; 8 shift-add steps
//            (add `b` into the high half when the low half's LSB is 1, then
//            shift the 17-bit {carry,acc} right one).  After 8 steps md_acc IS
//            the 16-bit product: [7:0] -> A, [15:8] -> B, OV = |[15:8]
//            (IQ5 manual rule: OV = 1 iff the product exceeds 0FFH).
//    DIV AB  md_acc = {remainder, quotient}; 8 restoring-division steps
//            (shift {rem,quo} left one, subtract `b` when it fits, quotient bit
//            = did-fit).  After 8 steps [7:0] = quotient -> A, [15:8] =
//            remainder -> B, OV = 0.  B == 0: the engine never starts (IQ2 /
//            §4.2 "skip iteration"), A and B are echoed back unchanged and
//            OV = 1.
//
//  Both set C = 0 unconditionally (§4.1).  The operands are the live ACC/B
//  taps: nothing can write either register between the start tick and the
//  S6P2(C4) commit, which is why the map needs no operand copy.
//
//  `md_flush` drops any iteration in flight - it is the sequencer's reset hold
//  (CE-gated) or a `bkd_load` boundary force.  The two arrive on DIFFERENT
//  ports: `md_flush` is honoured inside the `if (ce)` arm, `md_park_flush`
//  (the boundary force alone) inside the parked arm.  A level-driven parked
//  clear would be a free-running clear - savestate_design §6 class C-1 - and
//  would silently undo an SS restore of 0x040-0x042 whenever the restored
//  `seqstate` is RESET_HOLD (QUESTION-P6-3).
//
//  C4.5 adds the six bit ops (§4.1's "bit ops" row).  They add NO state and no
//  port: the sequencer presents the containing byte on `a` and a MASK (bit
//  destinations) or the already-complemented source bit (C destinations) on
//  `b`, so both halves of every bit op fall out of the existing 8-bit datapath.
//
//  ALU_XCH has no arm: its A-side result IS the operand (the default `y = b`);
//  the operand-side half of an exchange is formed in the sequencer, which is
//  where both the old ACC and the destination address live.
//
//  Notation follows instr §3: c3/c6/c7 = carry out of adder bit 3/6/7;
//  b3/b6/b7 = borrow out of the corresponding subtractor stages.
//
//============================================================================

`timescale 1ns/1ps

module nu8051_alu (
    input  logic        CLK,
    input  logic        CE,
    input  logic        md_flush,    // reset hold / bkd_load: drop any iteration
    // The PARKED flush (CE == 0): the sim-only §2.8 boundary force ONLY.
    // Never `rst_hold` - see QUESTION-P6-3 / savestate_design §6 class C-1.
    input  logic        md_park_flush,
    input  logic        md_start,    // tick ending S2P1(C1) of a MUL/DIV (§4.2)

    input  logic  [4:0] op,          // ALU_* (rtl/nu8051_decode.svh encoding)
    input  logic  [7:0] a,
    input  logic  [7:0] b,
    input  logic        cin,         // PSW.CY
    input  logic        acin,        // PSW.AC (DA A step 1)

    output logic  [7:0] y,
    output logic  [7:0] b_out,       // B-side result (MUL/DIV only)
    output logic        c_out,
    output logic        ac_out,
    output logic        ov_out,

    // serial-unit state, exported for the §1.9 state fold (SS map 0x040-0x042)
    output logic [15:0] md_acc_o,
    output logic  [3:0] md_cnt_o,
    output logic        md_busy_o,

    // save state (savestate_design §5.1/§5.2): the staged command, fanned in
    // from the core through nu8051_seq (this block is instantiated there).
    input  logic  [9:0] ss_addr,
    input  logic [15:0] ss_wdata,
    input  logic        ss_we,
    output logic [15:0] ss_rdata
);

`include "nu8051_decode_ops.svh"

    import nu8051_ss_pkg::*;

    // ---- adder / subtractor with the documented intermediate carries -------
    // (each partial adder is sized to its own operands: a 9-bit context around
    // a 5-bit concat is a WIDTHEXPAND warning, and the warning bar is zero)
    wire       addc_in = (op == ALU_ADDC) ? cin : 1'b0;
    wire [4:0] sum_lo  = {1'b0, a[3:0]} + {1'b0, b[3:0]} + 5'(addc_in);
    wire       c3      = sum_lo[4];
    wire [7:0] sum_67  = {1'b0, a[6:0]} + {1'b0, b[6:0]} + 8'(addc_in);
    wire       c6      = sum_67[7];
    wire [8:0] sum     = {1'b0, a} + {1'b0, b} + 9'(addc_in);
    wire       c7      = sum[8];

    wire       sub_cin = (op == ALU_SUBB) ? cin : 1'b0;
    wire [4:0] dif_lo  = {1'b0, a[3:0]} - {1'b0, b[3:0]} - 5'(sub_cin);
    wire       bw3     = dif_lo[4];
    wire [7:0] dif_67  = {1'b0, a[6:0]} - {1'b0, b[6:0]} - 8'(sub_cin);
    wire       bw6     = dif_67[7];
    wire [8:0] dif     = {1'b0, a} - {1'b0, b} - 9'(sub_cin);
    wire       bw7     = dif[8];

    // ---- DA A, two-step rule (§4.3) ---------------------------------------
    wire [8:0] da_t   = {1'b0, a} + ((acin || (a[3:0] > 4'd9)) ? 9'h006 : 9'h000);
    wire       da_hi  = cin || da_t[8] || (da_t[7:4] > 4'd9);
    wire [8:0] da_y   = da_t + (da_hi ? 9'h060 : 9'h000);

    // ---- MUL/DIV serial unit (§4.2 [D-04]) --------------------------------
    // Three registers, exactly the D1.2 allocation for this module.
    logic [15:0] md_acc;
    logic  [3:0] md_cnt;
    logic        md_busy;

    assign md_acc_o  = md_acc;
    assign md_cnt_o  = md_cnt;
    assign md_busy_o = md_busy;

    wire md_is_div = (op == ALU_DIV);
    wire md_div0   = md_is_div && (b == 8'h00);

    // MUL step: the low half is the multiplier, consumed LSB-first, and becomes
    // the low product byte as the accumulator shifts down over it.
    wire  [8:0] mul_sum  = {1'b0, md_acc[15:8]} + (md_acc[0] ? {1'b0, b} : 9'd0);
    wire [15:0] mul_next = {mul_sum, md_acc[7:1]};

    // DIV step: restoring division, MSB-first.  The invariant rem < b keeps
    // {rem, quo[7]} inside 9 bits and the difference inside 8, so the 8-bit
    // subtract below is exact whenever it is selected.
    wire  [8:0] div_sh   = {md_acc[15:8], md_acc[7]};
    wire        div_ge   = (div_sh >= {1'b0, b});
    wire  [7:0] div_sub  = div_sh[7:0] - b;
    wire  [7:0] div_rem  = div_ge ? div_sub : div_sh[7:0];
    wire [15:0] div_next = {div_rem, md_acc[6:0], div_ge};

    // ---- save-state read mux (registered, savestate_design §5.1) ----------
    always_ff @(posedge CLK) begin
        case (ss_addr)
            SSA_A_MD_ACC:  ss_rdata <= md_acc;
            SSA_A_MD_CNT:  ss_rdata <= {12'b0, md_cnt};
            SSA_A_MD_BUSY: ss_rdata <= {15'b0, md_busy};
            default:       ss_rdata <= 16'h0000;
        endcase
    end

    always_ff @(posedge CLK) begin
        // ---- save-state write decode (§5.2): above the reset/flush arms,
        // restore-priority position.  Legal by the park contract: CE == 0
        // whenever ss_we is high, so the branches this swallows are no-ops.
        if (ss_we) begin
            case (ss_addr)
                SSA_A_MD_ACC:  md_acc  <= ss_wdata;
                SSA_A_MD_CNT:  md_cnt  <= ss_wdata[3:0];
                SSA_A_MD_BUSY: md_busy <= ss_wdata[0];
                default: ;
            endcase
        end else if (CE) begin
            if (md_flush) begin
                md_acc <= 16'h0000; md_cnt <= 4'd0; md_busy <= 1'b0;
            end else if (md_start && !md_div0) begin
                // both engines start from {0, A}: MUL's multiplier, DIV's
                // dividend with a zero partial remainder
                md_acc  <= {8'h00, a};
                md_cnt  <= 4'd0;
                md_busy <= 1'b1;
            end else if (md_busy) begin
                md_acc <= md_is_div ? div_next : mul_next;
                md_cnt <= md_cnt + 4'd1;
                if (md_cnt == 4'd7) md_busy <= 1'b0;
            end
        end else if (md_park_flush) begin
            // CE parked: only the sim-only boundary force may move these
            // registers, and only on its own pulse (QUESTION-P6-3).
            md_acc <= 16'h0000; md_cnt <= 4'd0; md_busy <= 1'b0;
        end
    end

    always_comb begin
        y      = b;
        b_out  = b;
        c_out  = cin;
        ac_out = acin;
        ov_out = 1'b0;
        unique case (op)
            ALU_ADD, ALU_ADDC: begin
                y = sum[7:0]; c_out = c7; ac_out = c3; ov_out = c6 ^ c7;
            end
            ALU_SUBB: begin
                y = dif[7:0]; c_out = bw7; ac_out = bw3; ov_out = bw6 ^ bw7;
            end
            ALU_INC:  y = a + 8'd1;
            ALU_DEC:  y = a - 8'd1;
            ALU_ANL:  y = a & b;
            ALU_ORL:  y = a | b;
            ALU_XRL:  y = a ^ b;
            ALU_CLRA: y = 8'h00;
            ALU_CPLA: y = ~a;
            ALU_SWAP: y = {a[3:0], a[7:4]};
            ALU_RL:   y = {a[6:0], a[7]};
            ALU_RR:   y = {a[0], a[7:1]};
            ALU_RLC:  begin y = {a[6:0], cin}; c_out = a[7]; end
            ALU_RRC:  begin y = {cin, a[7:1]}; c_out = a[0]; end
            ALU_DA:   begin
                y = da_y[7:0];
                // DA A SETS C only, never clears it (§4.3 step 3)
                c_out = cin | da_t[8] | da_y[8];
            end
            // §4.2: results are read out of the serial register file at the
            // S6P2(C4) commit; C is 0 for both, always.
            ALU_MUL: begin
                y = md_acc[7:0]; b_out = md_acc[15:8];
                c_out = 1'b0; ov_out = |md_acc[15:8];
            end
            ALU_DIV: begin
                // B == 0 (IQ2, mirror-MAME): A/B echoed back unchanged, OV = 1
                y     = md_div0 ? a : md_acc[7:0];
                b_out = md_div0 ? b : md_acc[15:8];
                c_out = 1'b0; ov_out = md_div0;
            end
            ALU_CJNE: begin
                y = a;                        // result discarded
                c_out = (a < b);              // 1 iff op1 < op2 unsigned
            end
            ALU_XCHD: y = {a[7:4], b[3:0]};
            // ---- C4.5 bit family (§4.1 "bit ops" row) --------------------
            // Two results, one arm: `y` is the CONTAINING BYTE with the
            // addressed bit replaced (the §5.3 read-modify-write half) and
            // `c_out` is the C-destination half.  The sequencer's bit-address
            // unit picks what the B port carries - the containing-byte MASK
            // when the destination is a bit, {7'b0, source bit} when it is C -
            // so this block never sees a bit index or an address, and the
            // complement of the `/bit` forms has already been applied to that
            // source bit (d_bitn).  Only one of the two halves is ever
            // committed: d_dst selects the byte write, d_flagw the C write.
            ALU_BITSET: begin y = a |  b;              c_out = 1'b1;        end
            ALU_BITCLR: begin y = a & ~b;              c_out = 1'b0;        end
            ALU_BITCPL: begin y = a ^  b;              c_out = ~cin;        end
            ALU_BITMOV: begin y = cin ? (a | b) : (a & ~b);
                                                       c_out = b[0];        end
            ALU_BITANL: begin y = a;                   c_out = cin & b[0];  end
            ALU_BITORL: begin y = a;                   c_out = cin | b[0];  end
            default:  y = b;                  // ALU_PASS / ALU_NOP / mov forms
        endcase
    end

    initial begin
        md_acc = 16'h0000; md_cnt = 4'd0; md_busy = 1'b0;
        ss_rdata = 16'h0000;
    end

endmodule
