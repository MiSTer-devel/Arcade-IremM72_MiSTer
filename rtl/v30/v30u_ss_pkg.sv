//============================================================================
//  v30u_ss_pkg - the ucore save-state ADDRESS MAP.
//
//  The PACKAGE NAME is deliberately `v30_ss_pkg`, the same as the FSM core's:
//  the two cores are drop-in alternatives selected by the RTL FILE LIST
//  (sw/check_core.py --core {fsm,ucore}), and hdl/tb/tb_v30_core.sv imports
//  the package by name.  Exactly one of hdl/rtl/core/v30_ss_pkg.sv and
//  hdl/rtl/ucore/v30u_ss_pkg.sv is ever compiled.
//
//  SS_VERSION is in the 0x80 FAMILY (bit 7 set) so a stream can never be
//  mistaken for the FSM core's; the FSM map (0x03 / 82 / 120 / 203) is FROZEN
//  as a reference and is not extended.
//
//  AUDIT INVARIANT (inherited): every RTL state register <-> exactly ONE SSA_*
//  symbol <-> one read-mux arm <-> one write-decode arm.  Each SSA_B_* appears
//  exactly TWICE in v30u_biu.sv, each SSA_E_* exactly twice in v30u_eu.sv --
//  and for the EU that means twice across v30u_eu.sv PLUS its `.svh` includes,
//  which is where both save arms actually live.
//
//  ---------------------------------------------------------------------------
//  THE FLOP CENSUS (stage U3 platform audit, `sw/ss_lint.py --core ucore`).
//  The audit above only sees symbols ALREADY in the map, so it cannot see a
//  flop nobody mapped.  The census runs the other way -- every architectural
//  flop in the RTL must be mapped or explicitly whitelisted:
//
//    region  file(s)                     arch flops  mapped  whitelist  UNMAPPED
//    BIU     v30u_biu.sv                        83      83          0         0
//    EU      v30u_eu.sv + 9 includes            140     116         24         0
//    TOTAL                                      223     199         24         0
//
//  Whitelist: sw/ss_flop_whitelist_ucore.txt, 24 entries, ALL of them
//  block-local working values of the EU's single clocked block (written before
//  read on every clock, nothing survives the edge) plus the write-only
//  vestigial `ending`.  They are whitelisted only because they are declared at
//  module scope; declaring them INSIDE the always block would remove both them
//  and the whitelist.
//
//  F49 (U3 open item 5) -- CLOSED IN U4.  The census's first run found FIVE
//  architectural flops absent from the map:
//    v30u_biu.sv  r_cur_odd, r_cmt_odd, r_rq_odd[0:1], r_rd_land
//    v30u_eu.sv   rst_ctr
//  `r_rd_land` drives `eu_rdata_n` -- A COMPLETED READ'S DATA, so a restore
//  without it loses a landed word -- and the three `*_odd` flops carry the
//  split access's ODD BASE, which decides the byte swap at v30u_biu.sv:1329.
//  `rst_ctr` is F25's four-clock reset march.  None was scrambled by the SS1
//  sweep or probed by mode 5, because BOTH INSTRUMENTS ONLY VISIT ADDRESSES
//  THAT EXIST: no instrument that walks the map can find a flop nobody put in
//  the map, which is the whole reason the census runs RTL -> map instead.
//  They are appended at 0x061-0x065 (BIU) and 0x173 (EU); SS_VERSION 0x81 ->
//  0x82 and the counts move with them.
//  ---------------------------------------------------------------------------
//
//  APPEND-ONLY: new fields append at the end of their module's dense region
//  (never renumber); any map edit bumps SS_VERSION and the counts.
//
//  Quartus 17.1: localparams + helper functions only.  No packed structs.
//============================================================================
`ifndef V30_SS_PKG_SV
`define V30_SS_PKG_SV

package v30_ss_pkg;

  localparam int          SS_ADDR_W    = 9;
  // F49 (U4): 0x81 -> 0x82.  The five architectural flops the census found
  // UNMAPPED are appended, which ADDS ADDRESSES, so the version moves -- a v1
  // stream has no words for them and must not be silently accepted.
  // F52 (SM3 sitting 3): 0x82 -> 0x83.  H1's recognition floor is FOUR new
  // BIU flops (`bnd_pending`, `bnd_arm`, `bnd_stamp`, `bnd_cnt`), appended at
  // 0x066-0x069 -- same rule, same reason.
  // SM3 SITTING 11: 0x83 -> 0x84.  Those same four addresses are REMOVED --
  // the floor is now one term on the EU's own IE gate and the flops are gone.
  // Removing addresses moves the version for the SAME reason adding them does:
  // a v3 stream carries four words this map has nowhere to put.  The vacated
  // codes 0x066-0x069 are NOT reused (the map never renumbers); the region's
  // count falls and the tag falls with it.
  // SM3 SITTING 21 / F56: 0x84 -> 0x85.  `pf_land` is DELETED -- M6 is
  // refuted by its own firing census and is gone from both engines -- so
  // SSA_B_PF_LAND leaves the map.  It is the FIRST address retired from the
  // MIDDLE of a dense region: 9'h038 becomes a HOLE that `ss_addr_of` steps
  // over, and NO symbol is renumbered (the append-only rule is kept in the
  // form that matters -- an address never means two different things).  A v4
  // stream carries one word this map has nowhere to put, which is why the
  // version moves.
  // SM3 SITTING 25 / §86: 0x85 -> 0x86.  The BRK/TF single-step arm APPENDS
  // ONE address, `SSA_E_BRK` at 0x174 -- seven bits carrying `brk_p[3:0]`, the
  // arm, its sample-instant pulse and the trap's kind bit.  Same rule, same
  // reason as every append before it: a v5 stream has no word for them and
  // must not be silently accepted.
  // SM3 SITTING 26 / §87.A: 0x86 -> 0x87.  The illegal-form stall APPENDS ONE
  // address, `SSA_E_OPR_LOADED` at 0x175 -- one bit, the OPR-valid interlock
  // that decides whether an `F` row sourcing OPR has anything to wait for.
  // Same rule, same reason as every append before it: a v6 stream has no word
  // for it and must not be silently accepted.
  // WRFUZZ H3: 0x87 -> 0x88.  The two request-phase bits packed in
  // `SSA_B_RQ_LATE` are appended at 0x06A; 0x066-0x069 remain retired.
  // WRFUZZ 8F ghost: 0x88 -> 0x89.  `ghost_rd_discard` is appended at 0x176;
  // it follows the discarded read across the instruction boundary until the
  // BIU returns it, so a stream frozen in that interval must carry the bit.
  // WRFUZZ LEA residue: 0x89 -> 0x8A.  `ea_residue` is appended at 0x177;
  // it is the retained EA-adder lane exposed by undocumented 8D / mod=3.
  // WRFUZZ LEA pair rail: 0x8A -> 0x8B.  The retained RHS and its two-register
  // select are appended at 0x178-0x179.
  // WRFUZZ 8F ghost feed: 0x8B -> 0x8D.  `ghost_rd_feed` and
  // `ghost_rd_ready` are appended at 0x17A-0x17B: the first distinguishes an
  // idle-phase ghost that feeds the overlapping pre-read, and the second says
  // the ghost completion matured before successor T1 and permits its early F.
  // WRFUZZ PF_LOST decoder hold: 0x8D -> 0x8E.  One valid bit and one byte
  // append the observed successor ModR/M latch at 0x17C-0x17D.
  localparam int          SS_VERSION   = 8'h8E;   // ucore map v14 (PF_LOST ModR/M hold)
  localparam logic [8:0]  SSA_TAG      = 9'h000;
  localparam logic [8:0]  SS_BIU_BASE  = 9'h001;
  localparam int          SS_BIU_COUNT = 101;  // U4 F49 (+5); s11 (-4); s21 (-1); H3 (+1)
  localparam logic [8:0]  SS_EU_BASE   = 9'h100;
  localparam int          SS_EU_COUNT  = 126;  // U2 p5 (+2 recog); U4 F49 (+1);
                                              // SM3 s25 / §86 (+1, the BRK arm);
                                              // SM3 s26 / §87.A (+1, opr_loaded);
                                              // WRFUZZ 8F (+1, discarded read);
                                              // WRFUZZ LEA (+1, EA residue);
                                              // WRFUZZ 8F (+2, feed + maturity);
                                              // WRFUZZ PF_LOST (+2, ModR/M hold)
  localparam int          SS_COUNT     = 1 + SS_BIU_COUNT + SS_EU_COUNT;
  localparam logic [15:0] SS_TAG       = {8'(SS_VERSION), 8'(SS_COUNT)};

  //--------------------------------------------------------------------------
  // BIU region (module v30u_biu): 0x001-0x06A, with retired holes
  //--------------------------------------------------------------------------
  localparam logic [8:0] SSA_B_T1_HALF2         = 9'h001;
  localparam logic [8:0] SSA_B_RUN              = 9'h002;
  localparam logic [8:0] SSA_B_TS               = 9'h003;
  localparam logic [8:0] SSA_B_CUR_BS           = 9'h004;
  localparam logic [8:0] SSA_B_CUR_ADDR_LO      = 9'h005;
  localparam logic [8:0] SSA_B_CUR_ADDR_HI      = 9'h006;
  localparam logic [8:0] SSA_B_CUR_DATA         = 9'h007;
  localparam logic [8:0] SSA_B_CUR_UBE_N        = 9'h008;
  localparam logic [8:0] SSA_B_CUR_SEG          = 9'h009;
  localparam logic [8:0] SSA_B_CUR_FETCH        = 9'h00A;
  localparam logic [8:0] SSA_B_CUR_HALT         = 9'h00B;
  localparam logic [8:0] SSA_B_CUR_NOADDR       = 9'h00C;
  localparam logic [8:0] SSA_B_CUR_WR           = 9'h00D;
  localparam logic [8:0] SSA_B_CUR_NEED         = 9'h00E;
  localparam logic [8:0] SSA_B_CUR_RDLAST       = 9'h00F;
  localparam logic [8:0] SSA_B_CUR_PN           = 9'h010;
  localparam logic [8:0] SSA_B_CUR_LATET1       = 9'h011;
  localparam logic [8:0] SSA_B_EVALD            = 9'h012;
  localparam logic [8:0] SSA_B_SEV              = 9'h013;
  localparam logic [8:0] SSA_B_DAGE             = 9'h014;
  localparam logic [8:0] SSA_B_CMT_VALID        = 9'h015;
  localparam logic [8:0] SSA_B_CMT_BS           = 9'h016;
  localparam logic [8:0] SSA_B_CMT_ADDR_LO      = 9'h017;
  localparam logic [8:0] SSA_B_CMT_ADDR_HI      = 9'h018;
  localparam logic [8:0] SSA_B_CMT_DATA         = 9'h019;
  localparam logic [8:0] SSA_B_CMT_UBE_N        = 9'h01A;
  localparam logic [8:0] SSA_B_CMT_SEG          = 9'h01B;
  localparam logic [8:0] SSA_B_CMT_FETCH        = 9'h01C;
  localparam logic [8:0] SSA_B_CMT_HALT         = 9'h01D;
  localparam logic [8:0] SSA_B_CMT_NOADDR       = 9'h01E;
  localparam logic [8:0] SSA_B_CMT_WR           = 9'h01F;
  localparam logic [8:0] SSA_B_CMT_NEED         = 9'h020;
  localparam logic [8:0] SSA_B_CMT_RDLAST       = 9'h021;
  localparam logic [8:0] SSA_B_CMT_PN           = 9'h022;
  localparam logic [8:0] SSA_B_CDAGE            = 9'h023;
  localparam logic [8:0] SSA_B_CMT_PREV_FP      = 9'h024;
  localparam logic [8:0] SSA_B_CMT_WAS_OWED     = 9'h025;
  localparam logic [8:0] SSA_B_LAST_FADDR       = 9'h026;
  localparam logic [8:0] SSA_B_Q0               = 9'h027;
  localparam logic [8:0] SSA_B_Q1               = 9'h028;
  localparam logic [8:0] SSA_B_Q2               = 9'h029;
  localparam logic [8:0] SSA_B_Q3               = 9'h02A;
  localparam logic [8:0] SSA_B_Q4               = 9'h02B;
  localparam logic [8:0] SSA_B_Q5               = 9'h02C;
  localparam logic [8:0] SSA_B_Q_HEAD           = 9'h02D;
  localparam logic [8:0] SSA_B_Q_CNT            = 9'h02E;
  localparam logic [8:0] SSA_B_GRN_N            = 9'h02F;
  localparam logic [8:0] SSA_B_GRN_TTL          = 9'h030;
  localparam logic [8:0] SSA_B_FETCH_PTR        = 9'h031;
  localparam logic [8:0] SSA_B_CS               = 9'h032;
  localparam logic [8:0] SSA_B_SUSPENDED        = 9'h033;
  localparam logic [8:0] SSA_B_HALTED           = 9'h034;
  localparam logic [8:0] SSA_B_HALT_PEND        = 9'h035;
  localparam logic [8:0] SSA_B_PF_OWED          = 9'h036;
  localparam logic [8:0] SSA_B_PF_ARM           = 9'h037;
  // 9'h038 -- RETIRED, NOT REUSED (SM3 s21 / F56).  It held `pf_land`, M6's
  // queue-landing block.  `ss_addr_of` steps over it; `ss_field_width` has no
  // arm for it and returns 0, which no instrument ever asks, because the dense
  // iterator never visits it.
  localparam logic [8:0] SSA_B_INFL_TTL         = 9'h039;
  localparam logic [8:0] SSA_B_INFL_N           = 9'h03A;
  localparam logic [8:0] SSA_B_ABSORB_TTL       = 9'h03B;
  localparam logic [8:0] SSA_B_NO_EVAL          = 9'h03C;
  localparam logic [8:0] SSA_B_FLUSH_EVAL       = 9'h03D;
  localparam logic [8:0] SSA_B_E_PEND           = 9'h03E;
  localparam logic [8:0] SSA_B_RQ_N             = 9'h03F;
  localparam logic [8:0] SSA_B_RQ0_BS           = 9'h040;
  localparam logic [8:0] SSA_B_RQ0_ADDR_LO      = 9'h041;
  localparam logic [8:0] SSA_B_RQ0_ADDR_HI      = 9'h042;
  localparam logic [8:0] SSA_B_RQ0_DATA         = 9'h043;
  localparam logic [8:0] SSA_B_RQ0_UBE          = 9'h044;
  localparam logic [8:0] SSA_B_RQ0_SEG          = 9'h045;
  localparam logic [8:0] SSA_B_RQ0_NOADDR       = 9'h046;
  localparam logic [8:0] SSA_B_RQ0_WR           = 9'h047;
  localparam logic [8:0] SSA_B_RQ0_NEED         = 9'h048;
  localparam logic [8:0] SSA_B_RQ0_LAST         = 9'h049;
  localparam logic [8:0] SSA_B_RQ1_BS           = 9'h04A;
  localparam logic [8:0] SSA_B_RQ1_ADDR_LO      = 9'h04B;
  localparam logic [8:0] SSA_B_RQ1_ADDR_HI      = 9'h04C;
  localparam logic [8:0] SSA_B_RQ1_DATA         = 9'h04D;
  localparam logic [8:0] SSA_B_RQ1_UBE          = 9'h04E;
  localparam logic [8:0] SSA_B_RQ1_SEG          = 9'h04F;
  localparam logic [8:0] SSA_B_RQ1_NOADDR       = 9'h050;
  localparam logic [8:0] SSA_B_RQ1_WR           = 9'h051;
  localparam logic [8:0] SSA_B_RQ1_NEED         = 9'h052;
  localparam logic [8:0] SSA_B_RQ1_LAST         = 9'h053;
  localparam logic [8:0] SSA_B_SLOT_BUSY        = 9'h054;
  localparam logic [8:0] SSA_B_SLOT_ACC         = 9'h055;
  localparam logic [8:0] SSA_B_OPR_HELD         = 9'h056;
  localparam logic [8:0] SSA_B_DONE_CTR         = 9'h057;
  localparam logic [8:0] SSA_B_DONE_WR          = 9'h058;
  localparam logic [8:0] SSA_B_RD_DONE_P        = 9'h059;
  localparam logic [8:0] SSA_B_WR_DONE_P        = 9'h05A;
  localparam logic [8:0] SSA_B_OPR_FREE_P       = 9'h05B;
  localparam logic [8:0] SSA_B_RD_VAL           = 9'h05C;
  localparam logic [8:0] SSA_B_READY_PREV       = 9'h05D;
  localparam logic [8:0] SSA_B_LAST_UBE         = 9'h05E;

  // dense-iteration helper (TB/harness): stream index -> address
  // SM3 s21 / F56: the BIU region carries ONE RETIRED CODE, 9'h038, and the
  // dense stream steps over it.  The `+ 1` is not a renumbering -- every
  // surviving symbol keeps the address it has always had; what changes is the
  // stream INDEX it appears at, and the SS_VERSION bump is what makes that
  // safe.  A second retired code would need a second term, and that is the
  // signal to re-think the region rather than add one.
  function automatic logic [8:0] ss_addr_of(input int i);
    logic [8:0] a;
    if (i == 0)                 ss_addr_of = SSA_TAG;
    else if (i <= SS_BIU_COUNT) begin
      a = SS_BIU_BASE + 9'(i - 1);
      if (a >= 9'h038) a = a + 9'd1;
      if (a >= 9'h066) a = a + 9'd4;
      ss_addr_of = a;
    end
    else                        ss_addr_of = SS_EU_BASE  + 9'(i - 1 - SS_BIU_COUNT);
  endfunction

  // field width per address (TB mode-5 round-trip check; 0 = unmapped)


  localparam logic [8:0] SSA_B_RD_FIRST_HI      = 9'h05F;
  localparam logic [8:0] SSA_B_RD_WAS_SPLIT     = 9'h060;

  // ---------------------------------------------------------------------------
  // F49 (U4) -- THE FIVE FLOPS THE CENSUS FOUND, APPENDED.
  // The census exists to run RTL -> map, which is the only direction that can
  // see a flop nobody put in the map; sec.44.3's "211 symbols, each exactly
  // twice" and mode 5's width sweep both only visit addresses that EXIST.  It
  // found five on its first run and this is the fix, appended at the end of the
  // BIU's dense region per the APPEND-ONLY rule above.
  //   `r_rd_land` drives `eu_rdata_n` -- A COMPLETED READ'S DATA.  A restore
  //   that loses it is a real defect, not a bookkeeping one.
  //   the three `*_odd` flops carry the split access's ODD BASE, which decides
  //   the byte swap at v30u_biu.sv:1329.
  // ---------------------------------------------------------------------------
  localparam logic [8:0] SSA_B_CUR_ODD          = 9'h061;
  localparam logic [8:0] SSA_B_CMT_ODD          = 9'h062;
  localparam logic [8:0] SSA_B_RQ0_ODD          = 9'h063;
  localparam logic [8:0] SSA_B_RQ1_ODD          = 9'h064;
  localparam logic [8:0] SSA_B_RD_LAND          = 9'h065;

  // ---------------------------------------------------------------------------
  // SM3 SITTING 11 -- 0x066-0x069 ARE RETIRED, NOT REUSED.  They held H1's
  // recognition floor (`bnd_pending` / `bnd_arm` / `bnd_stamp` / `bnd_cnt`,
  // F52).  The floor is now one term on the EU's IE gate and carries no state
  // of its own, so the flops and their addresses are gone and SS_VERSION moved
  // 0x83 -> 0x84.  The codes stay vacant so a v3 stream can never be read as a
  // v4 one by accident.
  // ---------------------------------------------------------------------------

  // WRFUZZ H3 -- request arrival phase.  Bits [1:0] correspond to the two
  // backing-store entries and say that a write reservation arrived no earlier
  // than the preceding read's T3.  Packed into one appended stream word.
  localparam logic [8:0] SSA_B_RQ_LATE          = 9'h06A;

  //--------------------------------------------------------------------------
  // EU region (module v30u_eu): 0x100-0x177
  //--------------------------------------------------------------------------
  localparam logic [8:0] SSA_E_AX                   = 9'h100;
  localparam logic [8:0] SSA_E_CX                   = 9'h101;
  localparam logic [8:0] SSA_E_DX                   = 9'h102;
  localparam logic [8:0] SSA_E_BX                   = 9'h103;
  localparam logic [8:0] SSA_E_SP                   = 9'h104;
  localparam logic [8:0] SSA_E_BP                   = 9'h105;
  localparam logic [8:0] SSA_E_IX                   = 9'h106;
  localparam logic [8:0] SSA_E_IY                   = 9'h107;
  localparam logic [8:0] SSA_E_ES                   = 9'h108;
  localparam logic [8:0] SSA_E_CS                   = 9'h109;
  localparam logic [8:0] SSA_E_SS                   = 9'h10A;
  localparam logic [8:0] SSA_E_DS                   = 9'h10B;
  localparam logic [8:0] SSA_E_PC                   = 9'h10C;
  localparam logic [8:0] SSA_E_PSW                  = 9'h10D;
  localparam logic [8:0] SSA_E_TMPA                 = 9'h10E;
  localparam logic [8:0] SSA_E_TMPB                 = 9'h10F;
  localparam logic [8:0] SSA_E_TMPC                 = 9'h110;
  localparam logic [8:0] SSA_E_OPR                  = 9'h111;
  localparam logic [8:0] SSA_E_IND                  = 9'h112;
  localparam logic [8:0] SSA_E_COUNT                = 9'h113;
  localparam logic [8:0] SSA_E_PFXCNT               = 9'h114;
  localparam logic [8:0] SSA_E_STAT                 = 9'h115;
  localparam logic [8:0] SSA_E_SIGN_NEG             = 9'h116;
  localparam logic [8:0] SSA_E_BIT_N                = 9'h117;
  localparam logic [8:0] SSA_E_AL_OP                = 9'h118;
  localparam logic [8:0] SSA_E_AL_TMP               = 9'h119;
  localparam logic [8:0] SSA_E_AL_BYTE              = 9'h11A;
  localparam logic [8:0] SSA_E_AL_EACONST           = 9'h11B;
  localparam logic [8:0] SSA_E_AL_EAVAL             = 9'h11C;
  localparam logic [8:0] SSA_E_AL_ADJUST            = 9'h11D;
  localparam logic [8:0] SSA_E_AL_ADJTMP            = 9'h11E;
  localparam logic [8:0] SSA_E_AL_BITARM            = 9'h11F;
  localparam logic [8:0] SSA_E_AL_BITN              = 9'h120;
  localparam logic [8:0] SSA_E_AL_SPENT             = 9'h121;
  localparam logic [8:0] SSA_E_UPC_PAGE             = 9'h122;
  localparam logic [8:0] SSA_E_UPC_OPC              = 9'h123;
  localparam logic [8:0] SSA_E_UPC_LOC              = 9'h124;
  localparam logic [8:0] SSA_E_SEG_OVR_EN           = 9'h125;
  localparam logic [8:0] SSA_E_SEG_OVR              = 9'h126;
  localparam logic [8:0] SSA_E_REP_KIND             = 9'h127;
  localparam logic [8:0] SSA_E_LOCK                 = 9'h128;
  localparam logic [8:0] SSA_E_OPC_REG              = 9'h129;
  localparam logic [8:0] SSA_E_OP8                  = 9'h12A;
  localparam logic [8:0] SSA_E_IMM8                 = 9'h12B;
  localparam logic [8:0] SSA_E_OPC_BASE             = 9'h12C;
  localparam logic [8:0] SSA_E_OPC_FROM_RM          = 9'h12D;
  localparam logic [8:0] SSA_E_MODRM_REG            = 9'h12E;
  localparam logic [8:0] SSA_E_XOP                  = 9'h12F;
  localparam logic [8:0] SSA_E_REP_TEST             = 9'h130;
  localparam logic [8:0] SSA_E_REP_POL              = 9'h131;
  localparam logic [8:0] SSA_E_BUS_WORD             = 9'h132;
  localparam logic [8:0] SSA_E_OPC8080              = 9'h133;
  localparam logic [8:0] SSA_E_MODE8080             = 9'h134;
  localparam logic [8:0] SSA_E_INTR_PEND            = 9'h135;
  localparam logic [8:0] SSA_E_HALTED               = 9'h136;
  localparam logic [8:0] SSA_E_M_KIND               = 9'h137;
  localparam logic [8:0] SSA_E_M_IDX                = 9'h138;
  localparam logic [8:0] SSA_E_M_EA                 = 9'h139;
  localparam logic [8:0] SSA_E_M_SEG                = 9'h13A;
  localparam logic [8:0] SSA_E_M_BYTE               = 9'h13B;
  localparam logic [8:0] SSA_E_R_KIND               = 9'h13C;
  localparam logic [8:0] SSA_E_R_IDX                = 9'h13D;
  localparam logic [8:0] SSA_E_R_EA                 = 9'h13E;
  localparam logic [8:0] SSA_E_R_SEG                = 9'h13F;
  localparam logic [8:0] SSA_E_R_BYTE               = 9'h140;
  localparam logic [8:0] SSA_E_WB_KIND              = 9'h141;
  localparam logic [8:0] SSA_E_WB_IDX               = 9'h142;
  localparam logic [8:0] SSA_E_WB_EA                = 9'h143;
  localparam logic [8:0] SSA_E_WB_SEG               = 9'h144;
  localparam logic [8:0] SSA_E_WB_BYTE              = 9'h145;
  localparam logic [8:0] SSA_E_PEND_ACT             = 9'h146;
  localparam logic [8:0] SSA_E_PEND_OFF             = 9'h147;
  localparam logic [8:0] SSA_E_PEND_SEG             = 9'h148;
  localparam logic [8:0] SSA_E_PEND_BYTE            = 9'h149;
  localparam logic [8:0] SSA_E_PEND_IO              = 9'h14A;
  localparam logic [8:0] SSA_E_OPR_FRESH            = 9'h14B;
  localparam logic [8:0] SSA_E_RDQ0                 = 9'h14C;
  localparam logic [8:0] SSA_E_RDQ1                 = 9'h14D;
  localparam logic [8:0] SSA_E_RDQ_N                = 9'h14E;
  localparam logic [8:0] SSA_E_RD_PENDING           = 9'h14F;
  localparam logic [8:0] SSA_E_RD_DONE_CNT          = 9'h150;
  localparam logic [8:0] SSA_E_RD_AGE0              = 9'h151;
  localparam logic [8:0] SSA_E_WR_OUT               = 9'h152;
  localparam logic [8:0] SSA_E_OPC_VALID            = 9'h154;
  localparam logic [8:0] SSA_E_OPC_BYTE             = 9'h155;
  localparam logic [8:0] SSA_E_POP_IS_FIRST         = 9'h156;
  localparam logic [8:0] SSA_E_LD_B                 = 9'h157;
  localparam logic [8:0] SSA_E_LD_PLA               = 9'h158;
  localparam logic [8:0] SSA_E_LD_EXT               = 9'h159;
  localparam logic [8:0] SSA_E_LD_PAGE              = 9'h15A;
  localparam logic [8:0] SSA_E_LD_HASRM             = 9'h15B;
  localparam logic [8:0] SSA_E_LD_RM                = 9'h15C;
  localparam logic [8:0] SSA_E_LD_DISP              = 9'h15D;
  localparam logic [8:0] SSA_E_LD_DLO               = 9'h15E;
  localparam logic [8:0] SSA_E_LD_GRPD              = 9'h15F;
  localparam logic [8:0] SSA_E_LD_BYTE              = 9'h160;
  localparam logic [8:0] SSA_E_LD_PRERD             = 9'h161;
  localparam logic [8:0] SSA_E_LD_RIPE_PREV         = 9'h162;
  localparam logic [8:0] SSA_E_ST                   = 9'h163;
  localparam logic [8:0] SSA_E_CHG                  = 9'h164;
  localparam logic [8:0] SSA_E_POSTE                = 9'h165;
  localparam logic [8:0] SSA_E_ROWQ                 = 9'h166;
  localparam logic [8:0] SSA_E_ROW_POSTED           = 9'h167;
  localparam logic [8:0] SSA_E_ROW_PAIRED           = 9'h168;
  localparam logic [8:0] SSA_E_RLOOP_N              = 9'h169;
  localparam logic [8:0] SSA_E_SUPPRESS             = 9'h16A;
  localparam logic [8:0] SSA_E_FIRST_POP            = 9'h16B;
  localparam logic [8:0] SSA_E_ROWB0                = 9'h16C;
  localparam logic [8:0] SSA_E_ROWB1                = 9'h16D;
  localparam logic [8:0] SSA_E_POLL_PIPE            = 9'h16E;
  // F8's post-`E` DEBT is EU state: the row's own opcode context travels with
  // it (F23 / D1 / F22 -- eighteen bits) and `iend_owed` says the successor's
  // latch reset is still owed.  `SSA_E_POSTE` was already in the map, so a
  // freeze at `poste=1` was representable and NOT restorable; these close it.
  localparam logic [8:0] SSA_E_PE_OPC_REG           = 9'h16F;
  localparam logic [8:0] SSA_E_PE_PFXCNT            = 9'h170;
  localparam logic [8:0] SSA_E_PE_FLAGS             = 9'h153;
  // U2 pass 5 -- THE RECOGNITION IS EU STATE TOO.  The pin pipelines are
  // flops, so a freeze that lands three clocks after an assert and does not
  // carry them restores a part that has forgotten the pin.  Two words: the
  // three pipelines packed (INT 4, NMI 5, IE 4 = 13 bits) and the latches.
  localparam logic [8:0] SSA_E_PIN_PIPE             = 9'h171;
  // ...and `SSA_E_IRQ_LATCH` is the recognition's LATCH word: nmi_latch,
  // irq_shadow, bnd_armed, irq_sel_nmi, unhalt_pend, (U2 pass 6) the REP
  // boundary's anchor selector `rep_chain`, and first-INTA collision
  // provenance, and bit 7 records that the selected interrupt woke HALT.
  // Bits 5:7 of a word that was already in the map -- NO address
  // is added and NO count changes, so SS_VERSION does NOT move; a v1 stream
  // restores them as 0, their sequence-start values.

  localparam logic [8:0] SSA_E_IRQ_LATCH            = 9'h172;

  // F49 (U4): the census's fifth flop.  `rst_ctr` is F25's four-clock reset
  // march -- the EU comes out of RESET running the ROM's own sequence at page 7
  // opcode 0x03, and this counter is where in that march it stands.
  localparam logic [8:0] SSA_E_RST_CTR              = 9'h173;

  // §86 (SM3 sitting 25) -- THE BRK/TF SINGLE-STEP ARM, one word, appended at
  // the end of the EU's dense region exactly as the append-only rule says.
  // Seven bits: `brk_p[3:0]` (the TF pipeline), `brk_arm` (the arm itself),
  // `brk_smp` (its sample instant, one clock past the boundary's F pop) and
  // `irq_sel_brk` (which of the three doors `S_IRQ_D` is walking through).
  // A freeze between the sample and the take that did not carry these would
  // restore a part that has forgotten a trap it had already decided to take,
  // which is exactly what U2 pass 5 said about the pin pipelines.
  localparam logic [8:0] SSA_E_BRK                  = 9'h174;

  // §87.A (SM3 sitting 26) -- THE OPR-VALID INTERLOCK, one bit, appended at the
  // end of the EU's dense region.  `opr_loaded` says whether ANYTHING has put a
  // value into OPR since the decode started -- the decoder's operand pre-read,
  // a completed micro-row read, or a `-> OPR` transfer.  It is what makes the
  // `F` interlock's wait TERMINATING: a row that sources OPR with this clear
  // and nothing outstanding is waiting for an access that will never be posted,
  // and the EU parks there forever (the illegal-form stall).  A freeze taken
  // inside a parked machine that did not carry this bit would restore a part
  // that resumes an instruction silicon never finishes.
  localparam logic [8:0] SSA_E_OPR_LOADED           = 9'h175;

  // The undocumented 8F mod3 stack read completes after the instruction has
  // retired.  This bit follows the resulting one-place displacement through
  // an overlapping read chain, so the unmatched tail completion is dropped.
  localparam logic [8:0] SSA_E_GHOST_DISCARD        = 9'h176;

  // The retained EA-adder lane normally follows tmpa.  A ModR/M address
  // calculation writes its pre-displacement base here without disturbing the
  // live microcode temp; undocumented 8D / mod=3 is its only consumer.
  localparam logic [8:0] SSA_E_EA_RESIDUE           = 9'h177;

  // Retained RHS selection from the last two-register ModR/M EA.  The valid
  // bit is cleared by every unary EA, whose undocumented LEA path uses tmpb.
  localparam logic [8:0] SSA_E_EA_PAIR_RHS          = 9'h178;
  localparam logic [8:0] SSA_E_EA_PAIR_VALID        = 9'h179;

  // Only an idle-phase ghost can become the untagged head consumed by a
  // younger pre-read.  Full-phase ghosts retain ordinary discard semantics.
  localparam logic [8:0] SSA_E_GHOST_FEED           = 9'h17A;
  localparam logic [8:0] SSA_E_GHOST_READY          = 9'h17B;
  localparam logic [8:0] SSA_E_OPC_RM_VALID         = 9'h17C;
  localparam logic [8:0] SSA_E_OPC_RM_BYTE          = 9'h17D;

  function automatic int ss_field_width(input logic [8:0] a);
    case (a)
      SSA_TAG: ss_field_width = 16;
      SSA_B_T1_HALF2:        ss_field_width = 1;
      SSA_B_RUN:             ss_field_width = 1;
      SSA_B_TS:              ss_field_width = 3;
      SSA_B_CUR_BS:          ss_field_width = 3;
      SSA_B_CUR_ADDR_LO:     ss_field_width = 16;
      SSA_B_CUR_ADDR_HI:     ss_field_width = 4;
      SSA_B_CUR_DATA:        ss_field_width = 16;
      SSA_B_CUR_UBE_N:       ss_field_width = 1;
      SSA_B_CUR_SEG:         ss_field_width = 2;
      SSA_B_CUR_FETCH:       ss_field_width = 1;
      SSA_B_CUR_HALT:        ss_field_width = 1;
      SSA_B_CUR_NOADDR:      ss_field_width = 1;
      SSA_B_CUR_WR:          ss_field_width = 1;
      SSA_B_CUR_NEED:        ss_field_width = 1;
      SSA_B_CUR_RDLAST:      ss_field_width = 1;
      SSA_B_CUR_PN:          ss_field_width = 2;
      SSA_B_CUR_LATET1:      ss_field_width = 1;
      SSA_B_EVALD:           ss_field_width = 1;
      SSA_B_SEV:             ss_field_width = 2;
      SSA_B_DAGE:            ss_field_width = 3;
      SSA_B_CMT_VALID:       ss_field_width = 1;
      SSA_B_CMT_BS:          ss_field_width = 3;
      SSA_B_CMT_ADDR_LO:     ss_field_width = 16;
      SSA_B_CMT_ADDR_HI:     ss_field_width = 4;
      SSA_B_CMT_DATA:        ss_field_width = 16;
      SSA_B_CMT_UBE_N:       ss_field_width = 1;
      SSA_B_CMT_SEG:         ss_field_width = 2;
      SSA_B_CMT_FETCH:       ss_field_width = 1;
      SSA_B_CMT_HALT:        ss_field_width = 1;
      SSA_B_CMT_NOADDR:      ss_field_width = 1;
      SSA_B_CMT_WR:          ss_field_width = 1;
      SSA_B_CMT_NEED:        ss_field_width = 1;
      SSA_B_CMT_RDLAST:      ss_field_width = 1;
      SSA_B_CMT_PN:          ss_field_width = 2;
      SSA_B_CDAGE:           ss_field_width = 3;
      SSA_B_CMT_PREV_FP:     ss_field_width = 16;
      SSA_B_CMT_WAS_OWED:    ss_field_width = 1;
      SSA_B_LAST_FADDR:      ss_field_width = 16;
      SSA_B_Q0:              ss_field_width = 8;
      SSA_B_Q1:              ss_field_width = 8;
      SSA_B_Q2:              ss_field_width = 8;
      SSA_B_Q3:              ss_field_width = 8;
      SSA_B_Q4:              ss_field_width = 8;
      SSA_B_Q5:              ss_field_width = 8;
      SSA_B_Q_HEAD:          ss_field_width = 3;
      SSA_B_Q_CNT:           ss_field_width = 4;
      SSA_B_GRN_N:           ss_field_width = 2;
      SSA_B_GRN_TTL:         ss_field_width = 2;
      SSA_B_FETCH_PTR:       ss_field_width = 16;
      SSA_B_CS:              ss_field_width = 16;
      SSA_B_SUSPENDED:       ss_field_width = 1;
      SSA_B_HALTED:          ss_field_width = 1;
      SSA_B_HALT_PEND:       ss_field_width = 1;
      SSA_B_PF_OWED:         ss_field_width = 1;
      SSA_B_PF_ARM:          ss_field_width = 1;
      SSA_B_INFL_TTL:        ss_field_width = 2;
      SSA_B_INFL_N:          ss_field_width = 2;
      SSA_B_ABSORB_TTL:      ss_field_width = 2;
      SSA_B_NO_EVAL:         ss_field_width = 1;
      SSA_B_FLUSH_EVAL:      ss_field_width = 1;
      SSA_B_E_PEND:          ss_field_width = 1;
      SSA_B_RQ_N:            ss_field_width = 2;
      SSA_B_RQ0_BS:          ss_field_width = 3;
      SSA_B_RQ0_ADDR_LO:     ss_field_width = 16;
      SSA_B_RQ0_ADDR_HI:     ss_field_width = 4;
      SSA_B_RQ0_DATA:        ss_field_width = 16;
      SSA_B_RQ0_UBE:         ss_field_width = 1;
      SSA_B_RQ0_SEG:         ss_field_width = 2;
      SSA_B_RQ0_NOADDR:      ss_field_width = 1;
      SSA_B_RQ0_WR:          ss_field_width = 1;
      SSA_B_RQ0_NEED:        ss_field_width = 1;
      SSA_B_RQ0_LAST:        ss_field_width = 1;
      SSA_B_RQ1_BS:          ss_field_width = 3;
      SSA_B_RQ1_ADDR_LO:     ss_field_width = 16;
      SSA_B_RQ1_ADDR_HI:     ss_field_width = 4;
      SSA_B_RQ1_DATA:        ss_field_width = 16;
      SSA_B_RQ1_UBE:         ss_field_width = 1;
      SSA_B_RQ1_SEG:         ss_field_width = 2;
      SSA_B_RQ1_NOADDR:      ss_field_width = 1;
      SSA_B_RQ1_WR:          ss_field_width = 1;
      SSA_B_RQ1_NEED:        ss_field_width = 1;
      SSA_B_RQ1_LAST:        ss_field_width = 1;
      SSA_B_SLOT_BUSY:       ss_field_width = 1;
      SSA_B_SLOT_ACC:        ss_field_width = 1;
      // U2 pass 6: MEASURED by mode 5's first EU-enabled run -- `opr_held` is a
      // 2-bit COUNTER (M13/11.4), and the hand-written table said 1.
      SSA_B_OPR_HELD:        ss_field_width = 2;
      SSA_B_DONE_CTR:        ss_field_width = 2;
      SSA_B_DONE_WR:         ss_field_width = 1;
      SSA_B_RD_DONE_P:       ss_field_width = 1;
      SSA_B_WR_DONE_P:       ss_field_width = 1;
      SSA_B_OPR_FREE_P:      ss_field_width = 1;
      SSA_B_RD_VAL:          ss_field_width = 16;
      SSA_B_READY_PREV:      ss_field_width = 1;
      SSA_B_LAST_UBE:        ss_field_width = 1;
      // U2 pass 6 -- the two BIU fields declared AFTER this function used to
      // fall through to `default: 0` for the same reason the whole EU region
      // did: the function was placed before their localparams.  It now sits
      // at the END of the package, so every symbol is in scope.
      SSA_B_RD_FIRST_HI:     ss_field_width = 8;
      SSA_B_RD_WAS_SPLIT:    ss_field_width = 1;
      SSA_B_RQ_LATE:         ss_field_width = 2;
      SSA_B_CUR_ODD:         ss_field_width = 1;   // F49
      SSA_B_CMT_ODD:         ss_field_width = 1;   // F49
      SSA_B_RQ0_ODD:         ss_field_width = 1;   // F49
      SSA_B_RQ1_ODD:         ss_field_width = 1;   // F49
      SSA_B_RD_LAND:         ss_field_width = 16;  // F49
      //---------------------------------------------------------------------
      // EU region (v30u_eu.sv).  One entry per SSA_E_* symbol; the width is
      // the read mux's own slice, which the write decode matches field for
      // field (checked mechanically against both arms).  Mode 5 (--ss-mode 5)
      // is what proves that pairing dynamically, per address.
      //---------------------------------------------------------------------
      SSA_E_AX:                  ss_field_width = 16;
      SSA_E_CX:                  ss_field_width = 16;
      SSA_E_DX:                  ss_field_width = 16;
      SSA_E_BX:                  ss_field_width = 16;
      SSA_E_SP:                  ss_field_width = 16;
      SSA_E_BP:                  ss_field_width = 16;
      SSA_E_IX:                  ss_field_width = 16;
      SSA_E_IY:                  ss_field_width = 16;
      SSA_E_ES:                  ss_field_width = 16;
      SSA_E_CS:                  ss_field_width = 16;
      SSA_E_SS:                  ss_field_width = 16;
      SSA_E_DS:                  ss_field_width = 16;
      SSA_E_PC:                  ss_field_width = 16;
      SSA_E_PSW:                 ss_field_width = 16;
      SSA_E_TMPA:                ss_field_width = 16;
      SSA_E_TMPB:                ss_field_width = 16;
      SSA_E_TMPC:                ss_field_width = 16;
      SSA_E_OPR:                 ss_field_width = 16;
      SSA_E_IND:                 ss_field_width = 16;
      SSA_E_COUNT:               ss_field_width = 16;
      SSA_E_PFXCNT:              ss_field_width = 8;
      SSA_E_STAT:                ss_field_width = 16;
      SSA_E_SIGN_NEG:            ss_field_width = 1;
      SSA_E_BIT_N:               ss_field_width = 4;
      SSA_E_AL_OP:               ss_field_width = 5;
      SSA_E_AL_TMP:              ss_field_width = 2;
      SSA_E_AL_BYTE:             ss_field_width = 1;
      SSA_E_AL_EACONST:          ss_field_width = 1;
      SSA_E_AL_EAVAL:            ss_field_width = 16;
      SSA_E_AL_ADJUST:           ss_field_width = 2;
      SSA_E_AL_ADJTMP:           ss_field_width = 2;
      SSA_E_AL_BITARM:           ss_field_width = 1;
      SSA_E_AL_BITN:             ss_field_width = 4;
      SSA_E_AL_SPENT:            ss_field_width = 1;
      SSA_E_UPC_PAGE:            ss_field_width = 3;
      SSA_E_UPC_OPC:             ss_field_width = 8;
      SSA_E_UPC_LOC:             ss_field_width = 4;
      SSA_E_SEG_OVR_EN:          ss_field_width = 1;
      SSA_E_SEG_OVR:             ss_field_width = 2;
      SSA_E_REP_KIND:            ss_field_width = 3;
      SSA_E_LOCK:                ss_field_width = 1;
      SSA_E_OPC_REG:             ss_field_width = 8;
      SSA_E_OP8:                 ss_field_width = 1;
      SSA_E_IMM8:                ss_field_width = 1;
      SSA_E_OPC_BASE:            ss_field_width = 5;
      SSA_E_OPC_FROM_RM:         ss_field_width = 1;
      SSA_E_MODRM_REG:           ss_field_width = 3;
      SSA_E_XOP:                 ss_field_width = 4;
      SSA_E_REP_TEST:            ss_field_width = 2;
      SSA_E_REP_POL:             ss_field_width = 1;
      SSA_E_BUS_WORD:            ss_field_width = 1;
      SSA_E_OPC8080:             ss_field_width = 1;
      SSA_E_MODE8080:            ss_field_width = 1;
      SSA_E_INTR_PEND:           ss_field_width = 1;
      SSA_E_HALTED:              ss_field_width = 1;
      SSA_E_M_KIND:              ss_field_width = 2;
      SSA_E_M_IDX:               ss_field_width = 3;
      SSA_E_M_EA:                ss_field_width = 16;
      SSA_E_M_SEG:               ss_field_width = 3;
      SSA_E_M_BYTE:              ss_field_width = 1;
      SSA_E_R_KIND:              ss_field_width = 2;
      SSA_E_R_IDX:               ss_field_width = 3;
      SSA_E_R_EA:                ss_field_width = 16;
      SSA_E_R_SEG:               ss_field_width = 3;
      SSA_E_R_BYTE:              ss_field_width = 1;
      SSA_E_WB_KIND:             ss_field_width = 2;
      SSA_E_WB_IDX:              ss_field_width = 3;
      SSA_E_WB_EA:               ss_field_width = 16;
      SSA_E_WB_SEG:              ss_field_width = 3;
      SSA_E_WB_BYTE:             ss_field_width = 1;
      SSA_E_PEND_ACT:            ss_field_width = 1;
      SSA_E_PEND_OFF:            ss_field_width = 16;
      SSA_E_PEND_SEG:            ss_field_width = 3;
      SSA_E_PEND_BYTE:           ss_field_width = 1;
      SSA_E_PEND_IO:             ss_field_width = 1;
      SSA_E_OPR_FRESH:           ss_field_width = 1;
      SSA_E_RDQ0:                ss_field_width = 16;
      SSA_E_RDQ1:                ss_field_width = 16;
      SSA_E_RDQ_N:               ss_field_width = 2;
      SSA_E_RD_PENDING:          ss_field_width = 2;
      SSA_E_RD_DONE_CNT:         ss_field_width = 2;
      SSA_E_RD_AGE0:             ss_field_width = 1;
      SSA_E_WR_OUT:              ss_field_width = 2;
      SSA_E_OPC_VALID:           ss_field_width = 1;
      SSA_E_OPC_BYTE:            ss_field_width = 8;
      SSA_E_POP_IS_FIRST:        ss_field_width = 1;
      SSA_E_LD_B:                ss_field_width = 8;
      SSA_E_LD_PLA:              ss_field_width = 14;
      SSA_E_LD_EXT:              ss_field_width = 1;
      SSA_E_LD_PAGE:             ss_field_width = 3;
      SSA_E_LD_HASRM:            ss_field_width = 1;
      SSA_E_LD_RM:               ss_field_width = 8;
      SSA_E_LD_DISP:             ss_field_width = 16;
      SSA_E_LD_DLO:              ss_field_width = 8;
      SSA_E_LD_GRPD:             ss_field_width = 1;
      SSA_E_LD_BYTE:             ss_field_width = 1;
      SSA_E_LD_PRERD:            ss_field_width = 1;
      SSA_E_LD_RIPE_PREV:        ss_field_width = 1;
      SSA_E_ST:                  ss_field_width = 6;
      SSA_E_CHG:                 ss_field_width = 2;
      SSA_E_POSTE:               ss_field_width = 1;
      SSA_E_ROWQ:                ss_field_width = 2;
      SSA_E_ROW_POSTED:          ss_field_width = 1;
      SSA_E_ROW_PAIRED:          ss_field_width = 1;
      SSA_E_RLOOP_N:             ss_field_width = 16;
      SSA_E_SUPPRESS:            ss_field_width = 1;
      SSA_E_FIRST_POP:           ss_field_width = 1;
      SSA_E_ROWB0:               ss_field_width = 8;
      SSA_E_ROWB1:               ss_field_width = 8;
      SSA_E_POLL_PIPE:           ss_field_width = 3;
      SSA_E_PE_OPC_REG:          ss_field_width = 8;
      SSA_E_PE_PFXCNT:           ss_field_width = 8;
      SSA_E_PE_FLAGS:            ss_field_width = 3;
      SSA_E_PIN_PIPE:            ss_field_width = 13;
      SSA_E_IRQ_LATCH:           ss_field_width = 8;
      SSA_E_BRK:                 ss_field_width = 7;
      SSA_E_OPR_LOADED:          ss_field_width = 1;   // §87.A
      SSA_E_GHOST_DISCARD:       ss_field_width = 1;
      SSA_E_EA_RESIDUE:          ss_field_width = 16;
      SSA_E_EA_PAIR_RHS:         ss_field_width = 16;
      SSA_E_EA_PAIR_VALID:       ss_field_width = 1;
      SSA_E_GHOST_FEED:          ss_field_width = 1;
      SSA_E_GHOST_READY:         ss_field_width = 1;
      SSA_E_OPC_RM_VALID:        ss_field_width = 1;
      SSA_E_OPC_RM_BYTE:         ss_field_width = 8;
      SSA_E_RST_CTR:             ss_field_width = 3;   // F49
      default: ss_field_width = 0;
    endcase
  endfunction

endpackage

`endif
