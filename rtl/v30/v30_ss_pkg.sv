//============================================================================
// Imported from nec_test (https://github.com/.../nec_test)
// Source: hdl/rtl/core, commit 595e2c4bbf7e474e3240eb25ff94515385c807ca
// Cycle-accurate NEC V30 (uPD70116) max-mode core, verified vs golden traces.
// Do not hand-edit; re-import from upstream. No license header upstream (same author).
//============================================================================

//============================================================================
//  v30_ss_pkg - save-state ADDRESS MAP (v2, single source of truth)
//
//  Addressed register-file interface (savestate_v2_design.md, fable 2026-07-21;
//  supersedes the v1 shadow-shift-register access of savestate_design.md). One
//  address per state element; the module read-mux/write-decode touch the EXISTING
//  core flops directly - no packed-struct shadow. State inventory (295 BIU + 851
//  EU bits) carried over verbatim from savestate_design.md section 2.
//
//  AUDIT INVARIANT: every RTL state register <-> exactly ONE SSA_* symbol <-> one
//  read-mux arm <-> one write-decode arm. Each SSA_B_* appears exactly twice in
//  v30_biu.sv, each SSA_E_* exactly twice in v30_eu.sv (lint-grep, design 3).
//
//  APPEND-ONLY: new fields append at the end of their module's dense region
//  (never renumber); any map edit bumps SS_VERSION and the counts.
//
//  Quartus 17.1: localparams + helper functions only (functions elaborated by
//  TB/tooling, never by synthesis). No packed-struct aggregates.
//============================================================================
`ifndef V30_SS_PKG_SV
`define V30_SS_PKG_SV

package v30_ss_pkg;

  localparam int          SS_ADDR_W    = 9;
  localparam int          SS_VERSION   = 8'h02;   // v2 addressed scheme
  localparam logic [8:0]  SSA_TAG      = 9'h000;
  localparam logic [8:0]  SS_BIU_BASE  = 9'h001;
  localparam int          SS_BIU_COUNT = 82;
  localparam logic [8:0]  SS_EU_BASE   = 9'h100;
  localparam int          SS_EU_COUNT  = 119;
  localparam int          SS_COUNT     = 1 + SS_BIU_COUNT + SS_EU_COUNT; // 202
  localparam logic [15:0] SS_TAG       = {8'(SS_VERSION), 8'(SS_COUNT)};

  //--------------------------------------------------------------------------
  // BIU region (module v30_biu): 0x001-0x052, 82 addresses, 295 state bits
  //--------------------------------------------------------------------------
  localparam logic [8:0] SSA_B_STATE            = 9'h001;
  localparam logic [8:0] SSA_B_CUR_TYPE         = 9'h002;
  localparam logic [8:0] SSA_B_CUR_ADDR_LO      = 9'h003;
  localparam logic [8:0] SSA_B_CUR_ADDR_HI      = 9'h004;
  localparam logic [8:0] SSA_B_CUR_FETCH        = 9'h005;
  localparam logic [8:0] SSA_B_CUR_WR           = 9'h006;
  localparam logic [8:0] SSA_B_CUR_SWAP         = 9'h007;
  localparam logic [8:0] SSA_B_CUR_SPLIT1       = 9'h008;
  localparam logic [8:0] SSA_B_CUR_SPLIT2       = 9'h009;
  localparam logic [8:0] SSA_B_CUR_WRAP         = 9'h00A;
  localparam logic [8:0] SSA_B_CUR_WDATA        = 9'h00B;
  localparam logic [8:0] SSA_B_CUR_SEG          = 9'h00C;
  localparam logic [8:0] SSA_B_CUR_UBE_N        = 9'h00D;
  localparam logic [8:0] SSA_B_CUR_KIND         = 9'h00E;
  localparam logic [8:0] SSA_B_NXT_VALID        = 9'h00F;
  localparam logic [8:0] SSA_B_NXT_TYPE         = 9'h010;
  localparam logic [8:0] SSA_B_NXT_ADDR_LO      = 9'h011;
  localparam logic [8:0] SSA_B_NXT_ADDR_HI      = 9'h012;
  localparam logic [8:0] SSA_B_NXT_FETCH        = 9'h013;
  localparam logic [8:0] SSA_B_NXT_WR           = 9'h014;
  localparam logic [8:0] SSA_B_NXT_SWAP         = 9'h015;
  localparam logic [8:0] SSA_B_NXT_SPLIT1       = 9'h016;
  localparam logic [8:0] SSA_B_NXT_SPLIT2       = 9'h017;
  localparam logic [8:0] SSA_B_NXT_WRAP         = 9'h018;
  localparam logic [8:0] SSA_B_NXT_WDATA        = 9'h019;
  localparam logic [8:0] SSA_B_NXT_SEG          = 9'h01A;
  localparam logic [8:0] SSA_B_NXT_UBE_N        = 9'h01B;
  localparam logic [8:0] SSA_B_NXT_KIND         = 9'h01C;
  localparam logic [8:0] SSA_B_UBE_N            = 9'h01D;
  localparam logic [8:0] SSA_B_EU_STARTED       = 9'h01E;
  localparam logic [8:0] SSA_B_EU_HAND          = 9'h01F;
  localparam logic [8:0] SSA_B_EU_RDATA         = 9'h020;
  localparam logic [8:0] SSA_B_TW_ANY           = 9'h021;
  localparam logic [8:0] SSA_B_EVALD            = 9'h022;
  localparam logic [8:0] SSA_B_DEFER_T4         = 9'h023;
  localparam logic [8:0] SSA_B_DEFER_IDLE       = 9'h024;
  localparam logic [8:0] SSA_B_EVAL_EXT         = 9'h025;
  localparam logic [8:0] SSA_B_FLUSH_HOLD       = 9'h026;
  localparam logic [8:0] SSA_B_EXT_FLUSHED      = 9'h027;
  localparam logic [8:0] SSA_B_READY_PREV       = 9'h028;
  localparam logic [8:0] SSA_B_EU_REQ_P1        = 9'h029;
  localparam logic [8:0] SSA_B_EU_REQ_P2        = 9'h02A;
  localparam logic [8:0] SSA_B_EU_READY_P1      = 9'h02B;
  localparam logic [8:0] SSA_B_EU_READY_P2      = 9'h02C;
  localparam logic [8:0] SSA_B_TW_PAR           = 9'h02D;
  localparam logic [8:0] SSA_B_T1_HALF2         = 9'h02E;
  localparam logic [8:0] SSA_B_PH_FF            = 9'h02F;
  localparam logic [8:0] SSA_B_GPH_FF           = 9'h030;
  localparam logic [8:0] SSA_B_LOCK_ACTIVE      = 9'h031;
  localparam logic [8:0] SSA_B_LOCK_DONE        = 9'h032;
  localparam logic [8:0] SSA_B_Q_MEM0           = 9'h033;
  localparam logic [8:0] SSA_B_Q_MEM1           = 9'h034;
  localparam logic [8:0] SSA_B_Q_MEM2           = 9'h035;
  localparam logic [8:0] SSA_B_Q_MEM3           = 9'h036;
  localparam logic [8:0] SSA_B_Q_MEM4           = 9'h037;
  localparam logic [8:0] SSA_B_Q_MEM5           = 9'h038;
  localparam logic [8:0] SSA_B_Q_RD             = 9'h039;
  localparam logic [8:0] SSA_B_Q_WR             = 9'h03A;
  localparam logic [8:0] SSA_B_Q_CNT            = 9'h03B;
  localparam logic [8:0] SSA_B_Q_AVL            = 9'h03C;
  localparam logic [8:0] SSA_B_Q_AGED           = 9'h03D;
  localparam logic [8:0] SSA_B_Q_HEAD_DRY_Q     = 9'h03E;
  localparam logic [8:0] SSA_B_FETCH_DISCARD    = 9'h03F;
  localparam logic [8:0] SSA_B_FETCH_CS         = 9'h040;
  localparam logic [8:0] SSA_B_FETCH_OFF        = 9'h041;
  localparam logic [8:0] SSA_B_FETCH_DATA       = 9'h042;
  localparam logic [8:0] SSA_B_PUSH_PEND        = 9'h043;
  localparam logic [8:0] SSA_B_PUSH_PEND_HI     = 9'h044;
  localparam logic [8:0] SSA_B_E_WAIT           = 9'h045;
  localparam logic [8:0] SSA_B_HALT_T1          = 9'h046;
  localparam logic [8:0] SSA_B_HALT_DONE        = 9'h047;
  localparam logic [8:0] SSA_B_OCC34_AGE        = 9'h048;
  localparam logic [8:0] SSA_B_POP_SR           = 9'h049;
  localparam logic [8:0] SSA_B_RECENT_EVX       = 9'h04A;
  localparam logic [8:0] SSA_B_LAST_WAS_STORE   = 9'h04B;
  localparam logic [8:0] SSA_B_LAW_TW_CNT       = 9'h04C;
  localparam logic [8:0] SSA_B_LAW_DTW          = 9'h04D;
  localparam logic [8:0] SSA_B_LAW_DCNT         = 9'h04E;
  localparam logic [8:0] SSA_B_LAW_WINDOW       = 9'h04F;
  localparam logic [8:0] SSA_B_LAW_CTR          = 9'h050;
  localparam logic [8:0] SSA_B_LAW_SEL          = 9'h051;
  localparam logic [8:0] SSA_B_LAW_PROV         = 9'h052;

  //--------------------------------------------------------------------------
  // EU region (module v30_eu): 0x100-0x176, 119 addresses, 851 state bits
  //--------------------------------------------------------------------------
  localparam logic [8:0] SSA_E_RF0              = 9'h100;
  localparam logic [8:0] SSA_E_RF1              = 9'h101;
  localparam logic [8:0] SSA_E_RF2              = 9'h102;
  localparam logic [8:0] SSA_E_RF3              = 9'h103;
  localparam logic [8:0] SSA_E_RF4              = 9'h104;
  localparam logic [8:0] SSA_E_RF5              = 9'h105;
  localparam logic [8:0] SSA_E_RF6              = 9'h106;
  localparam logic [8:0] SSA_E_RF7              = 9'h107;
  localparam logic [8:0] SSA_E_SR0              = 9'h108;
  localparam logic [8:0] SSA_E_SR1              = 9'h109;
  localparam logic [8:0] SSA_E_SR2              = 9'h10A;
  localparam logic [8:0] SSA_E_SR3              = 9'h10B;
  localparam logic [8:0] SSA_E_PSW              = 9'h10C;
  localparam logic [8:0] SSA_E_PC               = 9'h10D;
  localparam logic [8:0] SSA_E_ARCH_IP          = 9'h10E;
  localparam logic [8:0] SSA_E_STATE            = 9'h10F;
  localparam logic [8:0] SSA_E_WNEXT            = 9'h110;
  localparam logic [8:0] SSA_E_DRET             = 9'h111;
  localparam logic [8:0] SSA_E_DLY              = 9'h112;
  localparam logic [8:0] SSA_E_DIV_REM_LO       = 9'h113;
  localparam logic [8:0] SSA_E_DIV_REM_HI       = 9'h114;
  localparam logic [8:0] SSA_E_DIV_QUO          = 9'h115;
  localparam logic [8:0] SSA_E_DIV_DEN          = 9'h116;
  localparam logic [8:0] SSA_E_DIV_CNT          = 9'h117;
  localparam logic [8:0] SSA_E_DIV_BUSY         = 9'h118;
  localparam logic [8:0] SSA_E_DIV_WORD         = 9'h119;
  localparam logic [8:0] SSA_E_DIV_SIGNED       = 9'h11A;
  localparam logic [8:0] SSA_E_DIV_NSIGN        = 9'h11B;
  localparam logic [8:0] SSA_E_DIV_DSIGN        = 9'h11C;
  localparam logic [8:0] SSA_E_DIV_PEND         = 9'h11D;
  localparam logic [8:0] SSA_E_DIV_LATE         = 9'h11E;
  localparam logic [8:0] SSA_E_SH_R             = 9'h11F;
  localparam logic [8:0] SSA_E_SH_X             = 9'h120;
  localparam logic [8:0] SSA_E_SH_OTH           = 9'h121;
  localparam logic [8:0] SSA_E_SH_CY            = 9'h122;
  localparam logic [8:0] SSA_E_SH_OP            = 9'h123;
  localparam logic [8:0] SSA_E_SH_WF            = 9'h124;
  localparam logic [8:0] SSA_E_SH_N             = 9'h125;
  localparam logic [8:0] SSA_E_SH_BUSY          = 9'h126;
  localparam logic [8:0] SSA_E_SH_FBASE         = 9'h127;
  localparam logic [8:0] SSA_E_SH_RES           = 9'h128;
  localparam logic [8:0] SSA_E_SH_FL            = 9'h129;
  localparam logic [8:0] SSA_E_OPC              = 9'h12A;
  localparam logic [8:0] SSA_E_OPC2             = 9'h12B;
  localparam logic [8:0] SSA_E_MRM              = 9'h12C;
  localparam logic [8:0] SSA_E_IMMB             = 9'h12D;
  localparam logic [8:0] SSA_E_DISP             = 9'h12E;
  localparam logic [8:0] SSA_E_A4_CNT           = 9'h12F;
  localparam logic [8:0] SSA_E_A4_K             = 9'h130;
  localparam logic [8:0] SSA_E_A4_SRC           = 9'h131;
  localparam logic [8:0] SSA_E_A4_CARRY         = 9'h132;
  localparam logic [8:0] SSA_E_A4_Z             = 9'h133;
  localparam logic [8:0] SSA_E_MEM_OP           = 9'h134;
  localparam logic [8:0] SSA_E_IVT_OFF          = 9'h135;
  localparam logic [8:0] SSA_E_IVT_SEG          = 9'h136;
  localparam logic [8:0] SSA_E_TRAP_PSW         = 9'h137;
  localparam logic [8:0] SSA_E_SEG_OVR_EN       = 9'h138;
  localparam logic [8:0] SSA_E_SEG_OVR          = 9'h139;
  localparam logic [8:0] SSA_E_LOCK_EN          = 9'h13A;
  localparam logic [8:0] SSA_E_REP_EN           = 9'h13B;
  localparam logic [8:0] SSA_E_REP_KIND         = 9'h13C;
  localparam logic [8:0] SSA_E_FLUSH_NOW        = 9'h13D;
  localparam logic [8:0] SSA_E_STR_WR           = 9'h13E;
  localparam logic [8:0] SSA_E_RSLOT            = 9'h13F;
  localparam logic [8:0] SSA_E_REP1_ABORT       = 9'h140;
  localparam logic [8:0] SSA_E_STR_DONE         = 9'h141;
  localparam logic [8:0] SSA_E_CMP1             = 9'h142;
  localparam logic [8:0] SSA_E_CMP_R2S          = 9'h143;
  localparam logic [8:0] SSA_E_FL_CS            = 9'h144;
  localparam logic [8:0] SSA_E_FL_IP            = 9'h145;
  localparam logic [8:0] SSA_E_EA_SAVE_LO       = 9'h146;
  localparam logic [8:0] SSA_E_EA_SAVE_HI       = 9'h147;
  localparam logic [8:0] SSA_E_EA_SAVE_SEG      = 9'h148;
  localparam logic [8:0] SSA_E_LDP2             = 9'h149;
  localparam logic [8:0] SSA_E_FRET_PH          = 9'h14A;
  localparam logic [8:0] SSA_E_FACC             = 9'h14B;
  localparam logic [8:0] SSA_E_IRET_PW          = 9'h14C;
  localparam logic [8:0] SSA_E_POPR_PEND        = 9'h14D;
  localparam logic [8:0] SSA_E_PREP_ACC         = 9'h14E;
  localparam logic [8:0] SSA_E_PRACC            = 9'h14F;
  localparam logic [8:0] SSA_E_W4SKIP           = 9'h150;
  localparam logic [8:0] SSA_E_PREP_BPD         = 9'h151;
  localparam logic [8:0] SSA_E_SHW              = 9'h152;
  localparam logic [8:0] SSA_E_POPM_HOLD        = 9'h153;
  localparam logic [8:0] SSA_E_IE_OFF           = 9'h154;
  localparam logic [8:0] SSA_E_IE_LEN           = 9'h155;
  localparam logic [8:0] SSA_E_IE_FLD           = 9'h156;
  localparam logic [8:0] SSA_E_IE_W0            = 9'h157;
  localparam logic [8:0] SSA_E_IE_MODE          = 9'h158;
  localparam logic [8:0] SSA_E_IE_PH2           = 9'h159;
  localparam logic [8:0] SSA_E_IE_DLY           = 9'h15A;
  localparam logic [8:0] SSA_E_IE_CHAIN         = 9'h15B;
  localparam logic [8:0] SSA_E_IE_RDYHOLD       = 9'h15C;
  localparam logic [8:0] SSA_E_IE_LGOT          = 9'h15D;
  localparam logic [8:0] SSA_E_INT_P            = 9'h15E;
  localparam logic [8:0] SSA_E_NMI_P            = 9'h15F;
  localparam logic [8:0] SSA_E_NMI_LATCH        = 9'h160;
  localparam logic [8:0] SSA_E_POLL_S1          = 9'h161;
  localparam logic [8:0] SSA_E_SHADOW           = 9'h162;
  localparam logic [8:0] SSA_E_IE_PEND          = 9'h163;
  localparam logic [8:0] SSA_E_IE_VAL           = 9'h164;
  localparam logic [8:0] SSA_E_PSW_OLD          = 9'h165;
  localparam logic [8:0] SSA_E_POP_PEND         = 9'h166;
  localparam logic [8:0] SSA_E_IE_P             = 9'h167;
  localparam logic [8:0] SSA_E_WAITS_SEEN       = 9'h168;
  localparam logic [8:0] SSA_E_POST_FLUSH       = 9'h169;
  localparam logic [8:0] SSA_E_INSN_IP          = 9'h16A;
  localparam logic [8:0] SSA_E_IVT_VEC          = 9'h16B;
  localparam logic [8:0] SSA_E_HWAKE_IE0        = 9'h16C;
  localparam logic [8:0] SSA_E_IRQ_DISP         = 9'h16D;
  localparam logic [8:0] SSA_E_IRQ_NMI_IVT      = 9'h16E;
  localparam logic [8:0] SSA_E_EU_WR            = 9'h16F;
  localparam logic [8:0] SSA_E_EU_WORD          = 9'h170;
  localparam logic [8:0] SSA_E_EU_ADDR_LO       = 9'h171;
  localparam logic [8:0] SSA_E_EU_ADDR_HI       = 9'h172;
  localparam logic [8:0] SSA_E_EU_SEG           = 9'h173;
  localparam logic [8:0] SSA_E_EU_WDATA         = 9'h174;
  localparam logic [8:0] SSA_E_EU_KIND          = 9'h175;
  localparam logic [8:0] SSA_E_HALT_DISP        = 9'h176;

  // dense-iteration helper (TB/harness): stream index -> address
  function automatic logic [8:0] ss_addr_of(input int i);
    if (i == 0)                 ss_addr_of = SSA_TAG;
    else if (i <= SS_BIU_COUNT) ss_addr_of = SS_BIU_BASE + 9'(i - 1);
    else                        ss_addr_of = SS_EU_BASE  + 9'(i - 1 - SS_BIU_COUNT);
  endfunction

  // field width per address (TB mode-5 round-trip check; 0 = unmapped)
  function automatic int ss_field_width(input logic [8:0] a);
    case (a)
      SSA_TAG: ss_field_width = 16;
      SSA_B_STATE:             ss_field_width = 3;
      SSA_B_CUR_TYPE:          ss_field_width = 3;
      SSA_B_CUR_ADDR_LO:       ss_field_width = 16;
      SSA_B_CUR_ADDR_HI:       ss_field_width = 4;
      SSA_B_CUR_FETCH:         ss_field_width = 1;
      SSA_B_CUR_WR:            ss_field_width = 1;
      SSA_B_CUR_SWAP:          ss_field_width = 1;
      SSA_B_CUR_SPLIT1:        ss_field_width = 1;
      SSA_B_CUR_SPLIT2:        ss_field_width = 1;
      SSA_B_CUR_WRAP:          ss_field_width = 1;
      SSA_B_CUR_WDATA:         ss_field_width = 16;
      SSA_B_CUR_SEG:           ss_field_width = 2;
      SSA_B_CUR_UBE_N:         ss_field_width = 1;
      SSA_B_CUR_KIND:          ss_field_width = 2;
      SSA_B_NXT_VALID:         ss_field_width = 1;
      SSA_B_NXT_TYPE:          ss_field_width = 3;
      SSA_B_NXT_ADDR_LO:       ss_field_width = 16;
      SSA_B_NXT_ADDR_HI:       ss_field_width = 4;
      SSA_B_NXT_FETCH:         ss_field_width = 1;
      SSA_B_NXT_WR:            ss_field_width = 1;
      SSA_B_NXT_SWAP:          ss_field_width = 1;
      SSA_B_NXT_SPLIT1:        ss_field_width = 1;
      SSA_B_NXT_SPLIT2:        ss_field_width = 1;
      SSA_B_NXT_WRAP:          ss_field_width = 1;
      SSA_B_NXT_WDATA:         ss_field_width = 16;
      SSA_B_NXT_SEG:           ss_field_width = 2;
      SSA_B_NXT_UBE_N:         ss_field_width = 1;
      SSA_B_NXT_KIND:          ss_field_width = 2;
      SSA_B_UBE_N:             ss_field_width = 1;
      SSA_B_EU_STARTED:        ss_field_width = 1;
      SSA_B_EU_HAND:           ss_field_width = 1;
      SSA_B_EU_RDATA:          ss_field_width = 16;
      SSA_B_TW_ANY:            ss_field_width = 1;
      SSA_B_EVALD:             ss_field_width = 1;
      SSA_B_DEFER_T4:          ss_field_width = 1;
      SSA_B_DEFER_IDLE:        ss_field_width = 1;
      SSA_B_EVAL_EXT:          ss_field_width = 1;
      SSA_B_FLUSH_HOLD:        ss_field_width = 1;
      SSA_B_EXT_FLUSHED:       ss_field_width = 1;
      SSA_B_READY_PREV:        ss_field_width = 1;
      SSA_B_EU_REQ_P1:         ss_field_width = 1;
      SSA_B_EU_REQ_P2:         ss_field_width = 1;
      SSA_B_EU_READY_P1:       ss_field_width = 1;
      SSA_B_EU_READY_P2:       ss_field_width = 1;
      SSA_B_TW_PAR:            ss_field_width = 1;
      SSA_B_T1_HALF2:          ss_field_width = 1;
      SSA_B_PH_FF:             ss_field_width = 1;
      SSA_B_GPH_FF:            ss_field_width = 1;
      SSA_B_LOCK_ACTIVE:       ss_field_width = 1;
      SSA_B_LOCK_DONE:         ss_field_width = 1;
      SSA_B_Q_MEM0:            ss_field_width = 8;
      SSA_B_Q_MEM1:            ss_field_width = 8;
      SSA_B_Q_MEM2:            ss_field_width = 8;
      SSA_B_Q_MEM3:            ss_field_width = 8;
      SSA_B_Q_MEM4:            ss_field_width = 8;
      SSA_B_Q_MEM5:            ss_field_width = 8;
      SSA_B_Q_RD:              ss_field_width = 3;
      SSA_B_Q_WR:              ss_field_width = 3;
      SSA_B_Q_CNT:             ss_field_width = 3;
      SSA_B_Q_AVL:             ss_field_width = 3;
      SSA_B_Q_AGED:            ss_field_width = 2;
      SSA_B_Q_HEAD_DRY_Q:      ss_field_width = 1;
      SSA_B_FETCH_DISCARD:     ss_field_width = 1;
      SSA_B_FETCH_CS:          ss_field_width = 16;
      SSA_B_FETCH_OFF:         ss_field_width = 16;
      SSA_B_FETCH_DATA:        ss_field_width = 16;
      SSA_B_PUSH_PEND:         ss_field_width = 2;
      SSA_B_PUSH_PEND_HI:      ss_field_width = 1;
      SSA_B_E_WAIT:            ss_field_width = 1;
      SSA_B_HALT_T1:           ss_field_width = 1;
      SSA_B_HALT_DONE:         ss_field_width = 1;
      SSA_B_OCC34_AGE:         ss_field_width = 4;
      SSA_B_POP_SR:            ss_field_width = 8;
      SSA_B_RECENT_EVX:        ss_field_width = 4;
      SSA_B_LAST_WAS_STORE:    ss_field_width = 1;
      SSA_B_LAW_TW_CNT:        ss_field_width = 4;
      SSA_B_LAW_DTW:           ss_field_width = 4;
      SSA_B_LAW_DCNT:          ss_field_width = 3;
      SSA_B_LAW_WINDOW:        ss_field_width = 1;
      SSA_B_LAW_CTR:           ss_field_width = 3;
      SSA_B_LAW_SEL:           ss_field_width = 3;
      SSA_B_LAW_PROV:          ss_field_width = 1;
      SSA_E_RF0:               ss_field_width = 16;
      SSA_E_RF1:               ss_field_width = 16;
      SSA_E_RF2:               ss_field_width = 16;
      SSA_E_RF3:               ss_field_width = 16;
      SSA_E_RF4:               ss_field_width = 16;
      SSA_E_RF5:               ss_field_width = 16;
      SSA_E_RF6:               ss_field_width = 16;
      SSA_E_RF7:               ss_field_width = 16;
      SSA_E_SR0:               ss_field_width = 16;
      SSA_E_SR1:               ss_field_width = 16;
      SSA_E_SR2:               ss_field_width = 16;
      SSA_E_SR3:               ss_field_width = 16;
      SSA_E_PSW:               ss_field_width = 16;
      SSA_E_PC:                ss_field_width = 16;
      SSA_E_ARCH_IP:           ss_field_width = 16;
      SSA_E_STATE:             ss_field_width = 7;
      SSA_E_WNEXT:             ss_field_width = 7;
      SSA_E_DRET:              ss_field_width = 7;
      SSA_E_DLY:               ss_field_width = 6;
      SSA_E_DIV_REM_LO:        ss_field_width = 16;
      SSA_E_DIV_REM_HI:        ss_field_width = 1;
      SSA_E_DIV_QUO:           ss_field_width = 16;
      SSA_E_DIV_DEN:           ss_field_width = 16;
      SSA_E_DIV_CNT:           ss_field_width = 6;
      SSA_E_DIV_BUSY:          ss_field_width = 1;
      SSA_E_DIV_WORD:          ss_field_width = 1;
      SSA_E_DIV_SIGNED:        ss_field_width = 1;
      SSA_E_DIV_NSIGN:         ss_field_width = 1;
      SSA_E_DIV_DSIGN:         ss_field_width = 1;
      SSA_E_DIV_PEND:          ss_field_width = 1;
      SSA_E_DIV_LATE:          ss_field_width = 1;
      SSA_E_SH_R:              ss_field_width = 16;
      SSA_E_SH_X:              ss_field_width = 8;
      SSA_E_SH_OTH:            ss_field_width = 8;
      SSA_E_SH_CY:             ss_field_width = 1;
      SSA_E_SH_OP:             ss_field_width = 3;
      SSA_E_SH_WF:             ss_field_width = 1;
      SSA_E_SH_N:              ss_field_width = 8;
      SSA_E_SH_BUSY:           ss_field_width = 1;
      SSA_E_SH_FBASE:          ss_field_width = 16;
      SSA_E_SH_RES:            ss_field_width = 16;
      SSA_E_SH_FL:             ss_field_width = 16;
      SSA_E_OPC:               ss_field_width = 8;
      SSA_E_OPC2:              ss_field_width = 8;
      SSA_E_MRM:               ss_field_width = 8;
      SSA_E_IMMB:              ss_field_width = 8;
      SSA_E_DISP:              ss_field_width = 16;
      SSA_E_A4_CNT:            ss_field_width = 8;
      SSA_E_A4_K:              ss_field_width = 8;
      SSA_E_A4_SRC:            ss_field_width = 16;
      SSA_E_A4_CARRY:          ss_field_width = 1;
      SSA_E_A4_Z:              ss_field_width = 1;
      SSA_E_MEM_OP:            ss_field_width = 16;
      SSA_E_IVT_OFF:           ss_field_width = 16;
      SSA_E_IVT_SEG:           ss_field_width = 16;
      SSA_E_TRAP_PSW:          ss_field_width = 16;
      SSA_E_SEG_OVR_EN:        ss_field_width = 1;
      SSA_E_SEG_OVR:           ss_field_width = 2;
      SSA_E_LOCK_EN:           ss_field_width = 1;
      SSA_E_REP_EN:            ss_field_width = 1;
      SSA_E_REP_KIND:          ss_field_width = 2;
      SSA_E_FLUSH_NOW:         ss_field_width = 1;
      SSA_E_STR_WR:            ss_field_width = 1;
      SSA_E_RSLOT:             ss_field_width = 6;
      SSA_E_REP1_ABORT:        ss_field_width = 1;
      SSA_E_STR_DONE:          ss_field_width = 1;
      SSA_E_CMP1:              ss_field_width = 16;
      SSA_E_CMP_R2S:           ss_field_width = 1;
      SSA_E_FL_CS:             ss_field_width = 16;
      SSA_E_FL_IP:             ss_field_width = 16;
      SSA_E_EA_SAVE_LO:        ss_field_width = 16;
      SSA_E_EA_SAVE_HI:        ss_field_width = 4;
      SSA_E_EA_SAVE_SEG:       ss_field_width = 2;
      SSA_E_LDP2:              ss_field_width = 1;
      SSA_E_FRET_PH:           ss_field_width = 2;
      SSA_E_FACC:              ss_field_width = 2;
      SSA_E_IRET_PW:           ss_field_width = 1;
      SSA_E_POPR_PEND:         ss_field_width = 1;
      SSA_E_PREP_ACC:          ss_field_width = 1;
      SSA_E_PRACC:             ss_field_width = 3;
      SSA_E_W4SKIP:            ss_field_width = 1;
      SSA_E_PREP_BPD:          ss_field_width = 1;
      SSA_E_SHW:               ss_field_width = 9;
      SSA_E_POPM_HOLD:         ss_field_width = 6;
      SSA_E_IE_OFF:            ss_field_width = 4;
      SSA_E_IE_LEN:            ss_field_width = 5;
      SSA_E_IE_FLD:            ss_field_width = 16;
      SSA_E_IE_W0:             ss_field_width = 16;
      SSA_E_IE_MODE:           ss_field_width = 2;
      SSA_E_IE_PH2:            ss_field_width = 1;
      SSA_E_IE_DLY:            ss_field_width = 12;
      SSA_E_IE_CHAIN:          ss_field_width = 1;
      SSA_E_IE_RDYHOLD:        ss_field_width = 1;
      SSA_E_IE_LGOT:           ss_field_width = 1;
      SSA_E_INT_P:             ss_field_width = 4;
      SSA_E_NMI_P:             ss_field_width = 5;
      SSA_E_NMI_LATCH:         ss_field_width = 1;
      SSA_E_POLL_S1:           ss_field_width = 1;
      SSA_E_SHADOW:            ss_field_width = 1;
      SSA_E_IE_PEND:           ss_field_width = 1;
      SSA_E_IE_VAL:            ss_field_width = 1;
      SSA_E_PSW_OLD:           ss_field_width = 16;
      SSA_E_POP_PEND:          ss_field_width = 1;
      SSA_E_IE_P:              ss_field_width = 4;
      SSA_E_WAITS_SEEN:        ss_field_width = 1;
      SSA_E_POST_FLUSH:        ss_field_width = 1;
      SSA_E_INSN_IP:           ss_field_width = 16;
      SSA_E_IVT_VEC:           ss_field_width = 8;
      SSA_E_HWAKE_IE0:         ss_field_width = 1;
      SSA_E_IRQ_DISP:          ss_field_width = 1;
      SSA_E_IRQ_NMI_IVT:       ss_field_width = 1;
      SSA_E_EU_WR:             ss_field_width = 1;
      SSA_E_EU_WORD:           ss_field_width = 1;
      SSA_E_EU_ADDR_LO:        ss_field_width = 16;
      SSA_E_EU_ADDR_HI:        ss_field_width = 4;
      SSA_E_EU_SEG:            ss_field_width = 2;
      SSA_E_EU_WDATA:          ss_field_width = 16;
      SSA_E_EU_KIND:           ss_field_width = 2;
      SSA_E_HALT_DISP:         ss_field_width = 1;
      default: ss_field_width = 0;
    endcase
  endfunction

endpackage

`endif
