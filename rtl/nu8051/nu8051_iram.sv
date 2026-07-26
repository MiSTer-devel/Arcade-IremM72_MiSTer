//============================================================================
// Imported from nu8051 (machine-cycle-accurate Intel 8051/8052 core, SystemVerilog)
// Source: rtl/nu8051_iram.sv, commit e2c97de0fabb3b3baa1eb98a15284bc74ef92f4e
// Do not hand-edit; re-import from upstream. The M72 integration wrapper is
// rtl/mcu.sv (distilled from nu8051 rtl/m72/mcu.sv), NOT a raw re-import.
//============================================================================
//============================================================================
//
//  nu8051_iram - 256-byte internal RAM, single synchronous port [D-05]
//                (core_design.md §5.1)
//
//  * One physical array, 256 bytes ALWAYS (sized for the 8052 superset,
//    PLAN §6.1); on P_8052==0 the upper half is architecturally unreachable
//    (§5.2) and simply unused - the array itself is never masked, so the
//    backdoor's "full 256B raw array access" (§1.9) works in both configs.
//  * Single port, one access per CE tick: the address presented DURING a
//    phase is captured at the CE edge ending that phase, and the data is
//    valid during the NEXT phase, where the sequencer captures it into
//    rd1_data / rd2_data (arrival-edge capture rule, D1.2 F-1 / INV-IRD).
//    Nothing outside those captures may reference `rdata`.
//  * Writes take effect at the write tick (S6P1 / S6P2 commit edges).
//  * Save-state / backdoor window: while CE is parked the SS/backdoor engine
//    owns the port (savestate_design §5.4).  Borrowing the port clobbers the
//    output register - which is exactly why the F-1 staging flops exist.
//
//============================================================================

`timescale 1ns/1ps

module nu8051_iram (
    input  logic        CLK,
    input  logic        CE,

    input  logic  [7:0] addr,        // presented during the phase before use
    output logic  [7:0] rdata,       // valid during the following phase only
    input  logic        we,
    input  logic  [7:0] wdata,

    // save-state window (savestate_design §5.4): the THIRD master on this
    // single port, selected by `ss_sel` = "CE parked AND the staged SS
    // address is inside 0x200-0x2FF".  The select is a pure mux, never an
    // arbiter - INV-MUX: the core side issues no access on a CE-low edge and
    // the SS side none on a CE-high edge, so the two can never collide.
    // Window traffic clobbers `rdata`; that is INV-IRD, and it is why the
    // sequencer's F-1 staging flops (rd1_data/rd2_data) exist.
    input  logic        ss_sel,
    input  logic  [7:0] ss_addr,
    input  logic        ss_we,
    input  logic  [7:0] ss_wdata

`ifdef NU8051_BACKDOOR
    ,
    // parked-CE raw byte port (§1.9): one access per CLK, no masking
    input  logic        bkd_we,
    input  logic  [7:0] bkd_addr,
    input  logic  [7:0] bkd_wdata,
    output logic  [7:0] bkd_rdata
`endif
);

    logic [7:0] mem [0:255];

    // ---- the port mux (savestate_design §5.4) -----------------------------
    // ONE array access per CLK edge, from one of two masters.  Written as a
    // mux on the port inputs (not two `mem[...]` expressions in two branches)
    // so the single-port BRAM inference the design relies on is unaffected -
    // the mux sits on address/we/wdata, which is inference-safe.
    // PROVEN, not asserted: the Phase-7 variant matrix (QUESTION-P7-1)
    // removed this mux and the RAM was STILL uninferred, which is how the
    // window was cleared of a charge it had been carrying since §7.7 wrote
    // the criterion.  The real culprit was the conditional read below.
    wire       p_en    = CE || ss_sel;
    wire [7:0] p_addr  = ss_sel ? ss_addr  : addr;
    wire       p_we    = ss_sel ? ss_we    : we;
    wire [7:0] p_wdata = ss_sel ? ss_wdata : wdata;

    // ---- array + write-first bypass (QUESTION-P7-1) -----------------------
    // `rdata` is unchanged as a FUNCTION: the byte just written on a write
    // tick, the stored byte otherwise, held while the port is disabled.  What
    // changed in Phase 7 is only its SHAPE.  The original form
    //
    //     rdata <= p_we ? p_wdata : mem[p_addr];
    //
    // makes the array read CONDITIONAL, and Quartus 17.1 refuses to infer a
    // RAM from a conditional read - "RAM logic ... is uninferred due to
    // asynchronous read logic" (Info 276007).  It built the 256x8 array out
    // of 2,048 registers plus a 256:1 read mux instead, which is what G6's
    // "IRAM still inferred as block RAM" criterion exists to catch.
    //
    // Measured, not guessed (QUESTION-P7-1 carries the variant matrix): the
    // savestate window mux is INNOCENT - the same failure reproduces with the
    // mux and the `initial` block both removed, and disappears the moment the
    // read becomes unconditional.  The fix is therefore a read-first array
    // (the one shape 17.1 recognises) plus a registered bypass at the output
    // that restores write-first at the pin.  Same value, same instants, same
    // hold-while-disabled behaviour; `+9` exempt flops, `-2048` mapped-array
    // flops, and the array lands in block memory.
    //
    // All three registers inherit `iram_rdata`'s §2.9 exemption verbatim:
    // they ARE the port output path, clobbered by window traffic by
    // construction (INV-IRD), consumed only at the arrival edge that follows
    // their own address issue.  No SS symbol, width or address changes - the
    // frozen map is untouched, and `sw/ss_lint.py` does not audit this file's
    // flop inventory (nu8051_iram owns no `SSA_*` symbol; the window is
    // range-decoded, §7.6).
    logic [7:0] ram_q;        // the array's own output register
    logic [7:0] byp_data;     // the write-first bypass value ...
    logic       byp_sel;      // ... and its select

    always_ff @(posedge CLK) begin
        if (p_en) begin
            if (p_we) mem[p_addr] <= p_wdata;
            ram_q    <= mem[p_addr];
            byp_sel  <= p_we;
            byp_data <= p_wdata;
        end
`ifdef NU8051_BACKDOOR
        else begin
            if (bkd_we) mem[bkd_addr] <= bkd_wdata;
            bkd_rdata <= mem[bkd_addr];
        end
`endif
    end

    assign rdata = byp_sel ? byp_data : ram_q;

    initial begin
        for (int i = 0; i < 256; i++) mem[i] = 8'h00;
        ram_q    = 8'h00;
        byp_data = 8'h00;
        byp_sel  = 1'b0;
`ifdef NU8051_BACKDOOR
        bkd_rdata = 8'h00;
`endif
    end

endmodule
