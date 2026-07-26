//============================================================================
// Imported from nu8051 (machine-cycle-accurate Intel 8051/8052 core, SystemVerilog)
// Source: rtl/nu8051_sfr.sv, commit e2c97de0fabb3b3baa1eb98a15284bc74ef92f4e
// Do not hand-edit; re-import from upstream. The M72 integration wrapper is
// rtl/mcu.sv (distilled from nu8051 rtl/m72/mcu.sv), NOT a raw re-import.
//============================================================================
//============================================================================
//
//  nu8051_sfr - core SFR block: SP, DPL, DPH, PCON, PSW, ACC, B
//               (core_design.md §5.4 ownership table, row 1)
//
//  * Reads are a combinational mux (SFRs are flops, not BRAM - §5.4), so
//    `sfr_rdata` is valid in the same tick the address is presented.
//  * Writes commit at the S6P2 edge with everything else (TQ6): the caller
//    holds `sfr_we` high through S6P2 and the CE edge applies it.
//  * PSW.P is **combinational** from ACC (odd parity => 1, core_design §4.1 /
//    peripheral_model §1.4) - never stored, so it cannot go stale.  PSW.1 is
//    plain storage.  Writes to PSW therefore drop bit 0 on the floor.
//  * Reserved bits inside implemented SFRs are plain storage (§5.4).
//
//============================================================================

`timescale 1ns/1ps

module nu8051_sfr (
    input  logic        CLK,
    input  logic        CE,
    input  logic        rst_hold,        // RESET_HOLD: apply reset values

    // SFR bus (§5.4)
    input  logic  [7:0] sfr_addr,
    output logic  [7:0] sfr_rdata,
    output logic        sfr_hit,
    input  logic        sfr_we,
    input  logic  [7:0] sfr_wdata,

    // direct register taps for the datapath (bank select, stack, dptr)
    output logic  [7:0] acc_q,
    output logic  [7:0] b_q,
    output logic  [7:0] psw_q,           // bit 0 already carries live parity
    output logic  [7:0] sp_q,
    output logic [15:0] dptr_q,
    output logic        pcon_smod,       // PCON.7 -> UART baud doubler (§5.4)
    // C5.6: the two power-control bits.  They are ordinary storage here - the
    // clock-gating behaviour they select lives in the sequencer (core_design
    // §6.5, periph §7) - but the sequencer needs the POST-COMMIT value at the
    // same S6P2 edge that writes them, so it re-derives that from its own
    // `sfr_we`/`sfr_wdata` and only needs the held bits from this side.
    output logic        pcon_idl,        // PCON.0
    output logic        pcon_pd,         // PCON.1
    // "Any enabled interrupt ... hardware clears PCON.0, terminating Idle
    // mode" (periph §7).  The sequencer pulses this at the S6P2 edge of the
    // idle cycle whose poll consummates, and it OUTRANKS a write of the same
    // edge (there can be no such write: nothing executes in idle).
    input  logic        pcon_idl_clr,

    // flag/side writes from the sequencer (masked by d_flagw), applied at the
    // same commit edge; an explicit destination write to PSW wins [D-03]
    input  logic        flag_we_c,
    input  logic        flag_c,
    input  logic        flag_we_ac,
    input  logic        flag_ac,
    input  logic        flag_we_ov,
    input  logic        flag_ov,
    input  logic        acc_we,
    input  logic  [7:0] acc_wdata,
    input  logic        b_we,
    input  logic  [7:0] b_wdata,
    input  logic        sp_we,
    input  logic  [7:0] sp_wdata,
    input  logic        dptr_we,
    input  logic [15:0] dptr_wdata,

    // save state (savestate_design §5.1/§5.2, map 0x050-0x056)
    input  logic  [9:0] ss_addr,
    input  logic [15:0] ss_wdata,
    input  logic        ss_we,
    output logic [15:0] ss_rdata

`ifdef NU8051_BACKDOOR
    ,
    input  logic        sfr_bkd_we       // storage write while CE is parked
`endif
);

`include "nu8051_defs.svh"
    import nu8051_ss_pkg::*;

    logic [7:0] sp_r, dpl_r, dph_r, pcon_r, psw_r, acc_r, b_r;

    // ---- live parity (combinational, never stored) ------------------------
    wire parity = ^acc_r;

    assign acc_q  = acc_r;
    assign b_q    = b_r;
    assign psw_q  = {psw_r[7:1], parity};
    assign sp_q   = sp_r;
    assign dptr_q = {dph_r, dpl_r};
    // PCON.7 = SMOD, the serial port's baud doubler (periph §5.4/§7).  A
    // write commits at S6P2 like every other SFR write, so a `MOV PCON,#80H`
    // moves the divider from the NEXT machine cycle on.
    assign pcon_smod = pcon_r[7];
    // C5.6: PCON.0/PCON.1 (periph §7).  HMOS-vs-CHMOS bit subsetting is not
    // modelled - all eight PCON bits are storage (core_design §6.5).
    assign pcon_idl  = pcon_r[0];
    assign pcon_pd   = pcon_r[1];

    // ---- combinational read mux -------------------------------------------
    always_comb begin
        sfr_hit   = 1'b1;
        case (sfr_addr)
            SFR_SP:   sfr_rdata = sp_r;
            SFR_DPL:  sfr_rdata = dpl_r;
            SFR_DPH:  sfr_rdata = dph_r;
            SFR_PCON: sfr_rdata = pcon_r;
            SFR_PSW:  sfr_rdata = {psw_r[7:1], parity};
            SFR_ACC:  sfr_rdata = acc_r;
            SFR_B:    sfr_rdata = b_r;
            default:  begin sfr_rdata = 8'h00; sfr_hit = 1'b0; end
        endcase
    end

    wire hit_sp   = (sfr_addr == SFR_SP);
    wire hit_dpl  = (sfr_addr == SFR_DPL);
    wire hit_dph  = (sfr_addr == SFR_DPH);
    wire hit_pcon = (sfr_addr == SFR_PCON);
    wire hit_psw  = (sfr_addr == SFR_PSW);
    wire hit_acc  = (sfr_addr == SFR_ACC);
    wire hit_b    = (sfr_addr == SFR_B);

    // PSW next value: flag side-writes first, destination write wins [D-03]
    logic [7:0] psw_nxt;
    always_comb begin
        psw_nxt = psw_r;
        if (flag_we_c)  psw_nxt[PSW_CY] = flag_c;
        if (flag_we_ac) psw_nxt[PSW_AC] = flag_ac;
        if (flag_we_ov) psw_nxt[PSW_OV] = flag_ov;
        if (sfr_we && hit_psw) psw_nxt = sfr_wdata;
    end

    // ---- save-state read mux (registered, savestate_design §5.1) ----------
    // PSW is the map's single derived-bit field: the word read here is
    // {psw_r[7:1], parity} - bit 0 is combinational from ACC and is not
    // storage, which is what ss_sweep_mask(SSA_F_PSW) = 16'h00FE encodes.
    always_ff @(posedge CLK) begin
        case (ss_addr)
            SSA_F_ACC:  ss_rdata <= {8'b0, acc_r};
            SSA_F_B:    ss_rdata <= {8'b0, b_r};
            SSA_F_PSW:  ss_rdata <= {8'b0, psw_r[7:1], parity};
            SSA_F_SP:   ss_rdata <= {8'b0, sp_r};
            SSA_F_DPL:  ss_rdata <= {8'b0, dpl_r};
            SSA_F_DPH:  ss_rdata <= {8'b0, dph_r};
            SSA_F_PCON: ss_rdata <= {8'b0, pcon_r};
            default:    ss_rdata <= 16'h0000;
        endcase
    end

    always_ff @(posedge CLK) begin
        // ---- save-state write decode (§5.2), restore-priority position ----
        if (ss_we) begin
            case (ss_addr)
                SSA_F_ACC:  acc_r       <= ss_wdata[7:0];
                SSA_F_B:    b_r         <= ss_wdata[7:0];
                SSA_F_PSW:  psw_r[7:1]  <= ss_wdata[7:1];   // bit 0 not storage
                SSA_F_SP:   sp_r        <= ss_wdata[7:0];
                SSA_F_DPL:  dpl_r       <= ss_wdata[7:0];
                SSA_F_DPH:  dph_r       <= ss_wdata[7:0];
                SSA_F_PCON: pcon_r      <= ss_wdata[7:0];
                default: ;
            endcase
        end else if (CE) begin
            if (rst_hold) begin
                // §7.2 reset state table
                sp_r   <= 8'h07;
                dpl_r  <= 8'h00;
                dph_r  <= 8'h00;
                pcon_r <= 8'h00;
                psw_r  <= 8'h00;
                acc_r  <= 8'h00;
                b_r    <= 8'h00;
            end else begin
                psw_r <= psw_nxt;
                if (acc_we)                    acc_r  <= acc_wdata;
                else if (sfr_we && hit_acc)    acc_r  <= sfr_wdata;
                if (b_we)                      b_r    <= b_wdata;
                else if (sfr_we && hit_b)      b_r    <= sfr_wdata;
                if (sp_we)                     sp_r   <= sp_wdata;
                else if (sfr_we && hit_sp)     sp_r   <= sfr_wdata;
                if (dptr_we)                   {dph_r, dpl_r} <= dptr_wdata;
                else begin
                    if (sfr_we && hit_dpl)     dpl_r  <= sfr_wdata;
                    if (sfr_we && hit_dph)     dph_r  <= sfr_wdata;
                end
                if (sfr_we && hit_pcon)        pcon_r <= sfr_wdata;
                // C5.6 / periph §7: the hardware clear of IDL.  Written after
                // the destination write so it wins if both land on one edge;
                // in practice they cannot (nothing executes while idle).
                if (pcon_idl_clr)              pcon_r[0] <= 1'b0;
            end
        end
`ifdef NU8051_BACKDOOR
        else if (sfr_bkd_we) begin
            // §1.9: raw storage write, no side effects.  PSW bit 0 is not
            // storage (live parity), so it is dropped here too.
            if (hit_sp)   sp_r   <= sfr_wdata;
            if (hit_dpl)  dpl_r  <= sfr_wdata;
            if (hit_dph)  dph_r  <= sfr_wdata;
            if (hit_pcon) pcon_r <= sfr_wdata;
            if (hit_psw)  psw_r  <= sfr_wdata;
            if (hit_acc)  acc_r  <= sfr_wdata;
            if (hit_b)    b_r    <= sfr_wdata;
        end
`endif
    end

    initial begin
        sp_r = 8'h07; dpl_r = 8'h00; dph_r = 8'h00; pcon_r = 8'h00;
        psw_r = 8'h00; acc_r = 8'h00; b_r = 8'h00;
        ss_rdata = 16'h0000;
    end

endmodule
