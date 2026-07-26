//============================================================================
// Imported from nu8051 (machine-cycle-accurate Intel 8051/8052 core, SystemVerilog)
// Source: rtl/nu8051_ss_pkg.sv, commit e2c97de0fabb3b3baa1eb98a15284bc74ef92f4e
// Do not hand-edit; re-import from upstream. The M72 integration wrapper is
// rtl/mcu.sv (distilled from nu8051 rtl/m72/mcu.sv), NOT a raw re-import.
//============================================================================
//============================================================================
//
//  nu8051_ss_pkg - save-state address map (single source of truth)
//
//  Transcribed verbatim from docs/notes/savestate_design.md (D1.2) §3.1
//  (full address table), §3.2 (canonical boundary state), §4 (package
//  specification).  Nothing here is invented: every localparam is a row of
//  the D1.2 table.
//
//  Authored during T2.3 because verification_plan.md §8 T2.3 requires the
//  TB/checker backdoor addresses to be *imported from this package, not
//  hardcoded*.  The Phase-6 implementation (savestate_design §10) extends
//  the package only by adding rows for map appends (append-only rule, §3).
//
//  Map version: SS_VERSION = 8'h01, SS_COUNT = 353, SS_REG_COUNT = 97,
//  SS_TAG = 16'h0161 - FROZEN by the Phase-6 A0 audit (savestate_design
//  §3.4; append-only from there, countersignature of E-3/E-4 pending).
//  Edits: E-2 (QUESTION-P54-1) added `tx_bnd_pend` at 0x0D3; E-3
//  (QUESTION-P6-2) deleted the four `Px_OUT` symbols - they have no backing
//  flop, Px_OUT is a continuous assignment from the mapped latch (+ the
//  mapped UART fold for P3) - and compacted the ports region; E-4
//  (QUESTION-P6-1) appended `bus_seen` at 0x01A, the TQ4 arming bit the
//  inventory missed.  See savestate_design §3.3.
//
//  Edit E-1 (QUESTION-P48-1, savestate_design §3.3, 2026-07-25, pre-A0):
//  SSA_S_MOVX_ADDR deleted - the MOVX xdata address holder is the mapped
//  MEM_ADDR launch register (SSA_S_MEM_ADDR); seq region compacted
//  (movx_dout 0x010 ... wr_n_r 0x019), SS_SEQ_COUNT 26 -> 25.  Legal as a
//  renumber because no snapshot/version has ever shipped; the append-only
//  rule binds from the Phase-6 A0 freeze.
//
//============================================================================

`timescale 1ns/1ps

// The map package deliberately publishes every symbol of the D1.2 table,
// most of which no single consumer references.
/* verilator lint_off UNUSEDPARAM */

package nu8051_ss_pkg;

    localparam int          SS_ADDR_W  = 10;
    localparam int          SS_VERSION = 1;        // 8'h01
    localparam logic [9:0]  SSA_TAG    = 10'h000;

    // ---- region bases / counts (savestate_design §3) ----
    localparam logic [9:0] SS_SEQ_BASE   = 10'h001;  localparam int SS_SEQ_COUNT   = 26;
    localparam logic [9:0] SS_ALU_BASE   = 10'h040;  localparam int SS_ALU_COUNT   = 3;
    localparam logic [9:0] SS_SFR_BASE   = 10'h050;  localparam int SS_SFR_COUNT   = 7;
    localparam logic [9:0] SS_PORTS_BASE = 10'h060;  localparam int SS_PORTS_COUNT = 8;
    localparam logic [9:0] SS_TIMER_BASE = 10'h080;  localparam int SS_TIMER_COUNT = 23;
    localparam logic [9:0] SS_UART_BASE  = 10'h0C0;  localparam int SS_UART_COUNT  = 20;
    localparam logic [9:0] SS_IRQ_BASE   = 10'h0E0;  localparam int SS_IRQ_COUNT   = 9;
    localparam logic [9:0] SS_IRAM_BASE  = 10'h200;  localparam int SS_IRAM_COUNT  = 256;

    localparam int SS_REG_COUNT = 1 + SS_SEQ_COUNT + SS_ALU_COUNT
                                + SS_SFR_COUNT + SS_PORTS_COUNT
                                + SS_TIMER_COUNT + SS_UART_COUNT
                                + SS_IRQ_COUNT;                     // 97
    localparam int SS_COUNT     = SS_REG_COUNT + SS_IRAM_COUNT;     // 353

    localparam logic [15:0] SS_TAG = {8'(SS_VERSION), 8'(SS_REG_COUNT)}; // 16'h0161

    // ---- one localparam per §3.1 table row ----
    // sequencer (nu8051_seq)
    localparam logic [9:0] SSA_S_PH          = 10'h001;
    localparam logic [9:0] SSA_S_SEQSTATE    = 10'h002;
    localparam logic [9:0] SSA_S_MCYC        = 10'h003;
    localparam logic [9:0] SSA_S_PC          = 10'h004;
    localparam logic [9:0] SSA_S_IR          = 10'h005;
    localparam logic [9:0] SSA_S_OP1         = 10'h006;
    localparam logic [9:0] SSA_S_OP2         = 10'h007;
    localparam logic [9:0] SSA_S_EA_LATCHED  = 10'h008;
    localparam logic [9:0] SSA_S_RST_S5P2    = 10'h009;
    localparam logic [9:0] SSA_S_FP_VALID    = 10'h00A;
    localparam logic [9:0] SSA_S_FP_EXT      = 10'h00B;
    localparam logic [9:0] SSA_S_FP_CONSUME  = 10'h00C;
    localparam logic [9:0] SSA_S_FP_DST      = 10'h00D;
    localparam logic [9:0] SSA_S_RD1_DATA    = 10'h00E;
    localparam logic [9:0] SSA_S_RD2_DATA    = 10'h00F;
    // E-1: SSA_S_MOVX_ADDR deleted (no backing register; the MOVX xdata
    // address is held by SSA_S_MEM_ADDR - savestate_design §3.3)
    localparam logic [9:0] SSA_S_MOVX_DOUT   = 10'h010;
    localparam logic [9:0] SSA_S_MOVX_DIN    = 10'h011;
    localparam logic [9:0] SSA_S_TAKE_SRC    = 10'h012;
    localparam logic [9:0] SSA_S_TAKE_PRIO   = 10'h013;
    localparam logic [9:0] SSA_S_MEM_ADDR    = 10'h014;
    localparam logic [9:0] SSA_S_ROM_ADDR    = 10'h015;
    localparam logic [9:0] SSA_S_ALE         = 10'h016;
    localparam logic [9:0] SSA_S_PSEN_N      = 10'h017;
    localparam logic [9:0] SSA_S_RD_N        = 10'h018;
    localparam logic [9:0] SSA_S_WR_N        = 10'h019;
    // E-4 (QUESTION-P6-1): "this machine cycle contained external bus
    // activity" - set at every external launch edge and in both MOVX cycles,
    // consumed by the ports module's TQ4 P0-latch clobber at S6P2.
    localparam logic [9:0] SSA_S_BUS_SEEN    = 10'h01A;
    // ALU serial unit (nu8051_alu)
    localparam logic [9:0] SSA_A_MD_ACC      = 10'h040;
    localparam logic [9:0] SSA_A_MD_CNT      = 10'h041;
    localparam logic [9:0] SSA_A_MD_BUSY     = 10'h042;
    // SFR block (nu8051_sfr)
    localparam logic [9:0] SSA_F_ACC         = 10'h050;
    localparam logic [9:0] SSA_F_B           = 10'h051;
    localparam logic [9:0] SSA_F_PSW         = 10'h052;
    localparam logic [9:0] SSA_F_SP          = 10'h053;
    localparam logic [9:0] SSA_F_DPL         = 10'h054;
    localparam logic [9:0] SSA_F_DPH         = 10'h055;
    localparam logic [9:0] SSA_F_PCON        = 10'h056;
    // ports (nu8051_ports)
    localparam logic [9:0] SSA_P_P0_LATCH    = 10'h060;
    localparam logic [9:0] SSA_P_P1_LATCH    = 10'h061;
    localparam logic [9:0] SSA_P_P2_LATCH    = 10'h062;
    localparam logic [9:0] SSA_P_P3_LATCH    = 10'h063;
    // E-3 (QUESTION-P6-2): SSA_P_P0_OUT..P3_OUT deleted (0x064-0x067) - the
    // Px_OUT pin words are continuous assignments from the mapped latches
    // (P3 folding the mapped UART state), never registers; the pin-sample
    // registers compacted down into the freed run.
    localparam logic [9:0] SSA_P_P0_IN       = 10'h064;
    localparam logic [9:0] SSA_P_P1_IN       = 10'h065;
    localparam logic [9:0] SSA_P_P2_IN       = 10'h066;
    localparam logic [9:0] SSA_P_P3_IN       = 10'h067;
    // timers (nu8051_timer)
    localparam logic [9:0] SSA_T_TCON        = 10'h080;
    localparam logic [9:0] SSA_T_TMOD        = 10'h081;
    localparam logic [9:0] SSA_T_TL0         = 10'h082;
    localparam logic [9:0] SSA_T_TH0         = 10'h083;
    localparam logic [9:0] SSA_T_TL1         = 10'h084;
    localparam logic [9:0] SSA_T_TH1         = 10'h085;
    localparam logic [9:0] SSA_T_T0_SMPL     = 10'h086;
    localparam logic [9:0] SSA_T_T0_PEND     = 10'h087;
    localparam logic [9:0] SSA_T_T1_SMPL     = 10'h088;
    localparam logic [9:0] SSA_T_T1_PEND     = 10'h089;
    localparam logic [9:0] SSA_T_TF0_STAGE   = 10'h08A;
    localparam logic [9:0] SSA_T_TF1_STAGE   = 10'h08B;
    localparam logic [9:0] SSA_T_T2CON       = 10'h08C;
    localparam logic [9:0] SSA_T_RCAP2L      = 10'h08D;
    localparam logic [9:0] SSA_T_RCAP2H      = 10'h08E;
    localparam logic [9:0] SSA_T_TL2         = 10'h08F;
    localparam logic [9:0] SSA_T_TH2         = 10'h090;
    localparam logic [9:0] SSA_T_T2_SMPL     = 10'h091;
    localparam logic [9:0] SSA_T_T2_PEND     = 10'h092;
    localparam logic [9:0] SSA_T_T2EX_SMPL   = 10'h093;
    localparam logic [9:0] SSA_T_T2EX_PEND   = 10'h094;
    localparam logic [9:0] SSA_T_T2_OSC2     = 10'h095;
    localparam logic [9:0] SSA_T_TF2_STAGE   = 10'h096;
    // UART (nu8051_uart)
    localparam logic [9:0] SSA_U_SCON        = 10'h0C0;
    localparam logic [9:0] SSA_U_SBUF_TX     = 10'h0C1;
    localparam logic [9:0] SSA_U_SBUF_RX     = 10'h0C2;
    localparam logic [9:0] SSA_U_TX_SHIFT    = 10'h0C3;
    localparam logic [9:0] SSA_U_TX_BITCNT   = 10'h0C4;
    localparam logic [9:0] SSA_U_TX_SEND     = 10'h0C5;
    localparam logic [9:0] SSA_U_TX_DATAF    = 10'h0C6;
    localparam logic [9:0] SSA_U_TX_REQ      = 10'h0C7;
    localparam logic [9:0] SSA_U_TX_DIV16    = 10'h0C8;
    localparam logic [9:0] SSA_U_RX_DIV16    = 10'h0C9;
    localparam logic [9:0] SSA_U_SMOD_DIV    = 10'h0CA;
    localparam logic [9:0] SSA_U_M2_DIV2     = 10'h0CB;
    localparam logic [9:0] SSA_U_RX_SHIFT    = 10'h0CC;
    localparam logic [9:0] SSA_U_RX_BITCNT   = 10'h0CD;
    localparam logic [9:0] SSA_U_RX_RECV     = 10'h0CE;
    localparam logic [9:0] SSA_U_RX_PEND     = 10'h0CF;
    localparam logic [9:0] SSA_U_RXD_PREV    = 10'h0D0;
    localparam logic [9:0] SSA_U_RX_MAJ      = 10'h0D1;
    localparam logic [9:0] SSA_U_TXD         = 10'h0D2;
    // C5.4 / QUESTION-P54-1 (map edit E-2): the divide-by-16 rollover carried
    // to the machine-cycle boundary, so a modes-1-3 transmission commences
    // "at S1P1 of the machine cycle following the rollover" (periph §5.6).
    localparam logic [9:0] SSA_U_TX_BND      = 10'h0D3;
    // interrupt unit (nu8051_irq)
    localparam logic [9:0] SSA_I_IE          = 10'h0E0;
    localparam logic [9:0] SSA_I_IP          = 10'h0E1;
    localparam logic [9:0] SSA_I_IPL_LO      = 10'h0E2;
    localparam logic [9:0] SSA_I_IPL_HI      = 10'h0E3;
    localparam logic [9:0] SSA_I_IRQ_REQ     = 10'h0E4;
    localparam logic [9:0] SSA_I_IRQ_SRC     = 10'h0E5;
    localparam logic [9:0] SSA_I_IRQ_PRIO    = 10'h0E6;
    localparam logic [9:0] SSA_I_INT0_SMPL   = 10'h0E7;
    localparam logic [9:0] SSA_I_INT1_SMPL   = 10'h0E8;

    // ---- published state encodings that cross the SS interface (§4) ----
    localparam logic [2:0] SS_SEQST_RESET_HOLD = 3'd0;
    localparam logic [2:0] SS_SEQST_PRIME      = 3'd1;
    localparam logic [2:0] SS_SEQST_EXEC       = 3'd2;
    localparam logic [2:0] SS_SEQST_ILCALL1    = 3'd3;
    localparam logic [2:0] SS_SEQST_ILCALL2    = 3'd4;
    localparam logic [2:0] SS_SEQST_IDLE       = 3'd5;

    localparam logic [1:0] SS_FPDST_IR   = 2'd0;
    localparam logic [1:0] SS_FPDST_OP1  = 2'd1;
    localparam logic [1:0] SS_FPDST_OP2  = 2'd2;
    localparam logic [1:0] SS_FPDST_MOVC = 2'd3;

    localparam logic [2:0] SS_IRQ_IE0 = 3'd0;
    localparam logic [2:0] SS_IRQ_TF0 = 3'd1;
    localparam logic [2:0] SS_IRQ_IE1 = 3'd2;
    localparam logic [2:0] SS_IRQ_TF1 = 3'd3;
    localparam logic [2:0] SS_IRQ_SER = 3'd4;
    localparam logic [2:0] SS_IRQ_T2  = 3'd5;

    // ---- dense-iteration helper (TB/harness/wrapper): stream index -> address
    function automatic logic [9:0] ss_addr_of(input int i);
        int k;
        k = i;
        if (k == 0)                                     ss_addr_of = SSA_TAG;
        else if (k < 1 + SS_SEQ_COUNT)
            ss_addr_of = SS_SEQ_BASE   + 10'(k - 1);
        else if (k < 1 + SS_SEQ_COUNT + SS_ALU_COUNT)
            ss_addr_of = SS_ALU_BASE   + 10'(k - 1 - SS_SEQ_COUNT);
        else if (k < 1 + SS_SEQ_COUNT + SS_ALU_COUNT + SS_SFR_COUNT)
            ss_addr_of = SS_SFR_BASE   + 10'(k - 1 - SS_SEQ_COUNT - SS_ALU_COUNT);
        else if (k < 1 + SS_SEQ_COUNT + SS_ALU_COUNT + SS_SFR_COUNT + SS_PORTS_COUNT)
            ss_addr_of = SS_PORTS_BASE + 10'(k - 1 - SS_SEQ_COUNT - SS_ALU_COUNT
                                                - SS_SFR_COUNT);
        else if (k < 1 + SS_SEQ_COUNT + SS_ALU_COUNT + SS_SFR_COUNT + SS_PORTS_COUNT
                     + SS_TIMER_COUNT)
            ss_addr_of = SS_TIMER_BASE + 10'(k - 1 - SS_SEQ_COUNT - SS_ALU_COUNT
                                                - SS_SFR_COUNT - SS_PORTS_COUNT);
        else if (k < 1 + SS_SEQ_COUNT + SS_ALU_COUNT + SS_SFR_COUNT + SS_PORTS_COUNT
                     + SS_TIMER_COUNT + SS_UART_COUNT)
            ss_addr_of = SS_UART_BASE  + 10'(k - 1 - SS_SEQ_COUNT - SS_ALU_COUNT
                                                - SS_SFR_COUNT - SS_PORTS_COUNT
                                                - SS_TIMER_COUNT);
        else if (k < SS_REG_COUNT)
            ss_addr_of = SS_IRQ_BASE   + 10'(k - 1 - SS_SEQ_COUNT - SS_ALU_COUNT
                                                - SS_SFR_COUNT - SS_PORTS_COUNT
                                                - SS_TIMER_COUNT - SS_UART_COUNT);
        else
            ss_addr_of = SS_IRAM_BASE  + 10'(k - SS_REG_COUNT);
    endfunction

    // ---- field width per address (TB width sweep; 0 = unmapped/reserved) ----
    function automatic int ss_field_width(input logic [9:0] a);
        if (a >= SS_IRAM_BASE && a < SS_IRAM_BASE + 10'(SS_IRAM_COUNT))
            ss_field_width = 8;
        else begin
            case (a)
                SSA_TAG:            ss_field_width = 16;
                SSA_S_PH:           ss_field_width = 4;
                SSA_S_SEQSTATE:     ss_field_width = 3;
                SSA_S_MCYC:         ss_field_width = 2;
                SSA_S_PC:           ss_field_width = 16;
                SSA_S_IR:           ss_field_width = 8;
                SSA_S_OP1:          ss_field_width = 8;
                SSA_S_OP2:          ss_field_width = 8;
                SSA_S_EA_LATCHED:   ss_field_width = 1;
                SSA_S_RST_S5P2:     ss_field_width = 1;
                SSA_S_FP_VALID:     ss_field_width = 1;
                SSA_S_FP_EXT:       ss_field_width = 1;
                SSA_S_FP_CONSUME:   ss_field_width = 1;
                SSA_S_FP_DST:       ss_field_width = 2;
                SSA_S_RD1_DATA:     ss_field_width = 8;
                SSA_S_RD2_DATA:     ss_field_width = 8;
                SSA_S_MOVX_DOUT:    ss_field_width = 8;
                SSA_S_MOVX_DIN:     ss_field_width = 8;
                SSA_S_TAKE_SRC:     ss_field_width = 3;
                SSA_S_TAKE_PRIO:    ss_field_width = 1;
                SSA_S_MEM_ADDR:     ss_field_width = 16;
                SSA_S_ROM_ADDR:     ss_field_width = 13;
                SSA_S_ALE:          ss_field_width = 1;
                SSA_S_BUS_SEEN:     ss_field_width = 1;
                SSA_S_PSEN_N:       ss_field_width = 1;
                SSA_S_RD_N:         ss_field_width = 1;
                SSA_S_WR_N:         ss_field_width = 1;
                SSA_A_MD_ACC:       ss_field_width = 16;
                SSA_A_MD_CNT:       ss_field_width = 4;
                SSA_A_MD_BUSY:      ss_field_width = 1;
                SSA_F_ACC, SSA_F_B, SSA_F_PSW, SSA_F_SP,
                SSA_F_DPL, SSA_F_DPH, SSA_F_PCON:
                                    ss_field_width = 8;
                SSA_P_P0_LATCH, SSA_P_P1_LATCH, SSA_P_P2_LATCH, SSA_P_P3_LATCH,
                SSA_P_P0_IN, SSA_P_P1_IN, SSA_P_P2_IN, SSA_P_P3_IN:
                                    ss_field_width = 8;
                SSA_T_TCON, SSA_T_TMOD, SSA_T_TL0, SSA_T_TH0,
                SSA_T_TL1, SSA_T_TH1:
                                    ss_field_width = 8;
                SSA_T_T0_SMPL, SSA_T_T0_PEND, SSA_T_T1_SMPL, SSA_T_T1_PEND,
                SSA_T_TF0_STAGE, SSA_T_TF1_STAGE:
                                    ss_field_width = 1;
                SSA_T_T2CON, SSA_T_RCAP2L, SSA_T_RCAP2H, SSA_T_TL2, SSA_T_TH2:
                                    ss_field_width = 8;
                SSA_T_T2_SMPL, SSA_T_T2_PEND, SSA_T_T2EX_SMPL, SSA_T_T2EX_PEND,
                SSA_T_T2_OSC2, SSA_T_TF2_STAGE:
                                    ss_field_width = 1;
                SSA_U_SCON, SSA_U_SBUF_TX, SSA_U_SBUF_RX:
                                    ss_field_width = 8;
                SSA_U_TX_SHIFT:     ss_field_width = 9;
                SSA_U_TX_BITCNT:    ss_field_width = 4;
                SSA_U_TX_SEND, SSA_U_TX_DATAF, SSA_U_TX_REQ:
                                    ss_field_width = 1;
                SSA_U_TX_DIV16:     ss_field_width = 4;
                SSA_U_RX_DIV16:     ss_field_width = 4;
                SSA_U_SMOD_DIV, SSA_U_M2_DIV2:
                                    ss_field_width = 1;
                SSA_U_RX_SHIFT:     ss_field_width = 9;
                SSA_U_RX_BITCNT:    ss_field_width = 4;
                SSA_U_RX_RECV, SSA_U_RX_PEND, SSA_U_RXD_PREV:
                                    ss_field_width = 1;
                SSA_U_RX_MAJ:       ss_field_width = 2;
                SSA_U_TXD, SSA_U_TX_BND:
                                    ss_field_width = 1;
                SSA_I_IE, SSA_I_IP: ss_field_width = 8;
                SSA_I_IPL_LO, SSA_I_IPL_HI, SSA_I_IRQ_REQ:
                                    ss_field_width = 1;
                SSA_I_IRQ_SRC:      ss_field_width = 3;
                SSA_I_IRQ_PRIO, SSA_I_INT0_SMPL, SSA_I_INT1_SMPL:
                                    ss_field_width = 1;
                default:            ss_field_width = 0;
            endcase
        end
    endfunction

    // ---- width-sweep readback mask (derived-bit exceptions) ----
    function automatic logic [15:0] ss_sweep_mask(input logic [9:0] a);
        int w;
        w = ss_field_width(a);
        if (a == SSA_F_PSW)   ss_sweep_mask = 16'h00FE;  // bit0 = combinational parity
        else if (w >= 16)     ss_sweep_mask = 16'hFFFF;
        else if (w == 0)      ss_sweep_mask = 16'h0000;
        else                  ss_sweep_mask = 16'((1 << w) - 1);
    endfunction

    // ---- region-of helper for the core read mux (§1.2) ----
    function automatic logic [2:0] ss_region_of(input logic [9:0] a);
        if      (a >= SS_IRQ_BASE)   ss_region_of = 3'd6;
        else if (a >= SS_UART_BASE)  ss_region_of = 3'd5;
        else if (a >= SS_TIMER_BASE) ss_region_of = 3'd4;
        else if (a >= SS_PORTS_BASE) ss_region_of = 3'd3;
        else if (a >= SS_SFR_BASE)   ss_region_of = 3'd2;
        else if (a >= SS_ALU_BASE)   ss_region_of = 3'd1;
        else                         ss_region_of = 3'd0;
    endfunction

endpackage

/* verilator lint_on UNUSEDPARAM */
