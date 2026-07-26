//============================================================================
// Imported from nu8051 (machine-cycle-accurate Intel 8051/8052 core, SystemVerilog)
// Source: rtl/nu8051_defs.svh, commit e2c97de0fabb3b3baa1eb98a15284bc74ef92f4e
// Do not hand-edit; re-import from upstream. The M72 integration wrapper is
// rtl/mcu.sv (distilled from nu8051 rtl/m72/mcu.sv), NOT a raw re-import.
//============================================================================
//============================================================================
//
//  nu8051_defs.svh - shared constants for the nu8051 core (Phase 3)
//
//  Phase indices (core_design.md §0), SFR direct addresses
//  (peripheral_model.md §1.1/§1.2 via core_design §5.4) and the
//  NU8051_BACKDOOR port-group macros.
//
//  Decode field ENCODINGS are NOT here: they are generated into
//  rtl/nu8051_decode.svh by sw/gen_decode.py (single source, core_design §3
//  contract point 4).
//
//============================================================================

// NOTE: intentionally NO include guard.  This header declares localparams at
// MODULE scope and is included once per module; Verilator shares `define state
// across the whole compilation unit, so a guard would silently starve every
// module after the first.

// verilator lint_off UNUSEDPARAM

// ---- §0 phase index: ph = 0..11 = S1P1..S6P2 -------------------------------
localparam logic [3:0] PH_S1P1 = 4'd0;
localparam logic [3:0] PH_S1P2 = 4'd1;
localparam logic [3:0] PH_S2P1 = 4'd2;
localparam logic [3:0] PH_S2P2 = 4'd3;
localparam logic [3:0] PH_S3P1 = 4'd4;
localparam logic [3:0] PH_S3P2 = 4'd5;
localparam logic [3:0] PH_S4P1 = 4'd6;
localparam logic [3:0] PH_S4P2 = 4'd7;
localparam logic [3:0] PH_S5P1 = 4'd8;
localparam logic [3:0] PH_S5P2 = 4'd9;
localparam logic [3:0] PH_S6P1 = 4'd10;
localparam logic [3:0] PH_S6P2 = 4'd11;

// ---- SFR direct addresses (core_design §5.4 ownership table) ---------------
localparam logic [7:0] SFR_P0     = 8'h80;
localparam logic [7:0] SFR_SP     = 8'h81;
localparam logic [7:0] SFR_DPL    = 8'h82;
localparam logic [7:0] SFR_DPH    = 8'h83;
localparam logic [7:0] SFR_PCON   = 8'h87;
localparam logic [7:0] SFR_TCON   = 8'h88;
localparam logic [7:0] SFR_TMOD   = 8'h89;
localparam logic [7:0] SFR_TL0    = 8'h8A;
localparam logic [7:0] SFR_TL1    = 8'h8B;
localparam logic [7:0] SFR_TH0    = 8'h8C;
localparam logic [7:0] SFR_TH1    = 8'h8D;
localparam logic [7:0] SFR_P1     = 8'h90;
localparam logic [7:0] SFR_SCON   = 8'h98;
localparam logic [7:0] SFR_SBUF   = 8'h99;
localparam logic [7:0] SFR_P2     = 8'hA0;
localparam logic [7:0] SFR_IE     = 8'hA8;
localparam logic [7:0] SFR_P3     = 8'hB0;
localparam logic [7:0] SFR_IP     = 8'hB8;
localparam logic [7:0] SFR_T2CON  = 8'hC8;
localparam logic [7:0] SFR_RCAP2L = 8'hCA;
localparam logic [7:0] SFR_RCAP2H = 8'hCB;
localparam logic [7:0] SFR_TL2    = 8'hCC;
localparam logic [7:0] SFR_TH2    = 8'hCD;
localparam logic [7:0] SFR_PSW    = 8'hD0;
localparam logic [7:0] SFR_ACC    = 8'hE0;
localparam logic [7:0] SFR_B      = 8'hF0;

// ---- PSW bit positions (peripheral_model §1.4) -----------------------------
localparam int PSW_P   = 0;
localparam int PSW_UD  = 1;
localparam int PSW_OV  = 2;
localparam int PSW_RS0 = 3;
localparam int PSW_RS1 = 4;
localparam int PSW_F0  = 5;
localparam int PSW_AC  = 6;
localparam int PSW_CY  = 7;

// ---- TCON / TMOD bit positions (peripheral_model §4) -----------------------
localparam int TCON_IT0 = 0;
localparam int TCON_IE0 = 1;
localparam int TCON_IT1 = 2;
localparam int TCON_IE1 = 3;
localparam int TCON_TR0 = 4;
localparam int TCON_TF0 = 5;
localparam int TCON_TR1 = 6;
localparam int TCON_TF1 = 7;

// ---- T2CON bit positions (peripheral_model §4.6, Figure 11) — 8052 ---------
localparam int T2CON_CPRL2 = 0;
localparam int T2CON_CT2   = 1;
localparam int T2CON_TR2   = 2;
localparam int T2CON_EXEN2 = 3;
localparam int T2CON_TCLK  = 4;
localparam int T2CON_RCLK  = 5;
localparam int T2CON_EXF2  = 6;
localparam int T2CON_TF2   = 7;

// ---- SCON bit positions (peripheral_model §5.2, Figure 14) -----------------
localparam int SCON_RI  = 0;
localparam int SCON_TI  = 1;
localparam int SCON_RB8 = 2;
localparam int SCON_TB8 = 3;
localparam int SCON_REN = 4;
localparam int SCON_SM2 = 5;
localparam int SCON_SM1 = 6;
localparam int SCON_SM0 = 7;

// verilator lint_on UNUSEDPARAM

// ---- NU8051_BACKDOOR slave hook (§1.9) -------------------------------------
// The sim-only backdoor reaches SFR *storage* directly, bypassing read/write
// side effects, one access per parked CLK.  It borrows the SFR bus itself:
// while CE is parked the core top drives sfr_addr/sfr_wdata from the backdoor
// port and pulses `sfr_bkd_we` (which every SFR owner applies on a CLK edge
// with CE low).  Only this one extra input per owner is needed, and the
// existing combinational read mux serves the backdoor read as well.
// `sfr_rmw` is forced high during backdoor access so port reads return the
// latch, which is what "SFR storage" means for a port (§1.9, §5.5 lowering).
//
// Every SFR owner therefore carries:
//     input logic sfr_bkd_we      // 1 = write storage on this CLK (CE low)
// guarded by `ifdef NU8051_BACKDOOR at the port list and the instantiation.

