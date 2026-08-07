//============================================================================
//
//  v30u_eu - the ucore execution unit: micro-sequencer, loader march,
//            datapath and ALU.
//
//  THIS MODULE IS A TRANSLITERATION OF sim/exec_impl.h + sim/loader_impl.h +
//  sim/alu.cpp.  The governance rule (hdl/rtl/ucore/README.md) is that the
//  model is the SPEC: an RTL-vs-sim divergence is a bug HERE.
//
//  --- THE INVERSION (campaign risk #1) -------------------------------------
//
//  The model PUMPS the clock: the interpreter calls `charge(n)` / `pop()` /
//  `wait_read()` and the BIU ticks underneath it.  The RTL cannot do that, so
//  the interpreter is re-expressed as a state machine whose state is
//  "the step of the model's program that occupies THIS clock":
//
//    * every port the BIU samples (`q_pop`, `eu_post`, `eu_pair`, `q_flush`,
//      `eu_susp`, `eu_halt`) is a COMBINATIONAL function of EU REGISTERS ONLY
//      -- the state, the micro-row word standing on `upc`, and the datapath.
//      The model makes those acts "at clk_ == c, before tick(c)"; the RTL BIU
//      consumes them in its step (b) at the edge ending c.  Same instant.
//    * `charge(n)` becomes "this state occupies n clocks".
//    * every `while (...) tick()` becomes a STALL: the state is re-entered
//      and its acts are withheld.  There are five, and they are named:
//
//        stall_q       a queue byte is demanded and not ripe        (M8)
//        stall_opr     the F / OPR interlock                        (11.4/M13)
//        stall_slot    the EU's ONE bus-request slot is busy        (M10)
//        stall_retire  the retire deadline: a store owes its data   (7.7)
//        stall_pin     the 1BL retire lead / a halted part          (S9a)
//
//    * a model step that charges NOTHING (consuming a pre-popped opcode, the
//      pure-decode logic) is a ZERO-COST state: it is executed inside the same
//      edge that computed it, by the bounded chain loop below.  The loop is
//      how "several model steps ride one clock" is expressed.
//
//  --- THE SEQUENCER --------------------------------------------------------
//
//  upc = {page[2:0], opc[7:0], loc[3:0]}, 15 bits (U0 finding F1), and it is
//  the architectural SS-mapped flop: the two tables stand combinationally on
//  it (see v30u_ucrom.sv), so a restore has no hidden BRAM-address state.
//  `loc` is a 4-bit counter inside the opcode's 16-row block and CARRIES into
//  the opcode byte (ledger: IMUL 00B3 -> 0218).
//
//  --- CE DISCIPLINE --------------------------------------------------------
//
//  Nothing clocked runs unless `srst` or `ce`; reset is ungated so `bkd_load`
//  fires regardless of CE.  There is no negedge process in the EU.
//
//  U4 pass 3 -- WHERE `ce` IS.  The module is a NEXT-STATE FUNCTION
//  (`always @*`, producing `<reg>_n`) plus a REGISTER BANK
//  (`always_ff @(posedge clk) if (ss_we || srst || ce)`).  `ce` appears in the
//  commit and NOWHERE ELSE, which is the whole reason for the shape: written
//  the other way -- one clocked block whose third arm was `else if (ce)` --
//  Quartus extracted no clock enable, threaded `ce` through the EU's 61-level
//  cone to every flip-flop's DATA input, clocked the registers every SYS clock
//  and made the multicycle exception the divided CPU clock earns FALSE
//  (ledger sec.51.7).  The behaviour is identical either way and the full ladder
//  was re-scored to say so; the NETLIST is not.
//
//============================================================================

module v30u_eu (
    input             clk,
    input             ce,
    input             srst,

    // queue port
    input       [7:0] q_byte,
    input             q_ripe,
    input             q_ripe_lead_n, // ...and it will be ripe NEXT clock
    input       [3:0] q_cnt,
    output            q_pop,
    output            q_first,
    output            q_flush,
    output            flush_pre,
    output            flush_rep,
    output            flush_stage,
    output            flush_pend,
    output            flush_nmi,
    output            flush_int_live,
    output     [15:0] flush_cs,
    output     [15:0] flush_cs_old,
    output            flush_cs_we,
    output     [15:0] flush_ip,

    // bus requests
    output            eu_post,
    output            eu_post_hold,
    output            eu_ghost_preview,
    output            eu_halt_irq,
    output            eu_vector_post,
    output      [2:0] eu_bs,
    output     [19:0] eu_addr,
    output     [19:0] eu_addr2,     // split: the second cycle's own address
    output            eu_split,
    output      [1:0] eu_seg,
    output      [1:0] eu_seg2,
    output            eu_word,
    input             eu_slot_busy,
    input             eu_slot_busy_n,
    input             eu_access_active,
    input             eu_direct_fetch,
    input             eu_fetch_tail,
    input             eu_ghost_full,
    input             eu_ghost_idle,
    input             eu_ghost_stack_first,
    input             eu_rd_wait,
    output            eu_pair,
    output            eu_pair2,     // the pairing fills TWO reserved cycles
    output     [15:0] eu_wdata,
    input      [15:0] eu_rdata_n,
    input             eu_rd_done_n,
    input             eu_rd_edge,   // the read's DATA EDGE (T3/Tw -> T4)
    input      [15:0] eu_rd_edge_d,
    input             eu_wr_done_n,
    input             eu_wr_eval,
    input             eu_opr_free,   // F31: the LEVEL `opr_held == 0`

    // prefetch control
    output            eu_susp,
    output            eu_resume,
    output            eu_halt,
    output            eu_unhalt,
    // F43 (SM3 sitting 6): the SAME wake, tapped ONE STAGE further down the
    // SAME pin pipeline, for the reader whose decision edge leads its own
    // clock -- the BIU's HALT display.  No new state: `int_p[1]` against
    // `int_p[2]`, exactly as `irq_rep_chn` reads a chained REP boundary
    // against `irq_rep_1st`.
    output            eu_unhalt_disp,
    input             halted,

    // SM3 sitting 11: what is left of H1's recognition floor.  The floor
    // itself is now one term on this module's own IE gate and holds no state;
    // these two lines carry the PREFETCHER SUSPEND that rides it.
    output            eu_bnd_take,  // a recognition boundary IS taken this clock
    output            eu_bnd_post,  // ...it is a retire, and the IE gate HELD it

    // machine state the pins carry
    output            psw_ie,
    output            md8080,

    // pins
    input             pin_int,
    input             pin_nmi,
    input             pin_poll_n,

    // backdoor
    input             bkd_load,
    input     [223:0] bkd_regs,
    output    [223:0] dbg_regs,
    output            dbg_first_pop,
    output            dbg_pend,

    // save-state
    input       [8:0] ss_addr,
    input      [15:0] ss_wdata,
    input             ss_we,
    output reg [15:0] ss_rdata
);

import v30_ss_pkg::*;

`include "pla3_tables.svh"

//============================================================================
// ENCODINGS
//============================================================================

// rows.h bus-status encodings -- the comparator stack's own
localparam bit [2:0] BS_INTA = 3'd0, BS_IOR = 3'd1, BS_IOW = 3'd2,
                     BS_MEMR = 3'd5, BS_MEMW = 3'd6, BS_PASV = 3'd7;

// sim/state.h register indices == the microcode Source1/Dest1 field values
localparam bit [2:0] R_AW = 3'd0, R_CW = 3'd1, R_DW = 3'd2, R_BW = 3'd3,
                     R_SP = 3'd4, R_BP = 3'd5, R_IX = 3'd6, R_IY = 3'd7;
localparam bit [1:0] SR_ES = 2'd0, SR_CS = 2'd1, SR_SS = 2'd2, SR_DS = 2'd3;
localparam bit [2:0] SEG_ZERO = 3'd4;   // physical == offset (vector fetch)

// OperandRef::Kind
localparam bit [1:0] OK_NONE = 2'd0, OK_REG = 2'd1, OK_SREG = 2'd2,
                     OK_MEM = 2'd3;

// sim/state.h PSW bits
localparam int FCY = 0, FP = 2, FAC = 4, FZ = 6, FS = 7,
               FBRK = 8, FIE = 9, FDIR = 10, FV = 11;
localparam bit [15:0] PSW_WRITABLE = 16'h0FD5;
localparam bit [15:0] PSW_FORCED   = 16'hF002;
localparam bit [15:0] ARITH_MASK   = 16'h0000 | (16'd1 << FCY) | (16'd1 << FP)
                                   | (16'd1 << FAC) | (16'd1 << FZ)
                                   | (16'd1 << FS) | (16'd1 << FV);

// sim/state.h AluOp
localparam bit [4:0] A_ADD=5'h00, A_OR=5'h01, A_ADC=5'h02, A_SBB=5'h03,
                     A_AND=5'h04, A_SUB=5'h05, A_XOR=5'h06, A_CMP=5'h07,
                     A_ROL=5'h08, A_ROR=5'h09, A_RCL=5'h0A, A_RCR=5'h0B,
                     A_SHL=5'h0C, A_SHR=5'h0D, A_SHL6=5'h0E, A_SAR=5'h0F,
                     A_ROL12=5'h10, A_DIV=5'h12, A_MUL=5'h13, A_ADJD=5'h14,
                     A_ADJA=5'h15, A_OPC=5'h16, A_BIT=5'h17,
                     A_INC=5'h18, A_DEC=5'h19, A_NOT=5'h1A, A_NEG=5'h1B,
                     A_INC2=5'h1C, A_DEC2=5'h1D, A_ABS=5'h1E, A_PASS=5'h1F;

// ucrom.h MicroType
localparam bit [1:0] TY_ALU = 2'd0, TY_JMP = 2'd1, TY_CTL = 2'd2;

// exec_impl.h Cond
localparam bit [3:0] C_C=4'd0, C_NC=4'd1, C_Z=4'd2, C_NZ=4'd3, C_OP8B=4'd4,
                     C_CNTZ=4'd5, C_L=4'd6, C_OP8=4'd7, C_ALWAYS=4'd8,
                     C_SIGN=4'd9, C_O=4'd10, C_NS=4'd11, C_REP=4'd12,
                     C_BUSY=4'd13, C_INTR=4'd14, C_OPC=4'd15;

// exec_impl.h Ictl / Ectl
localparam bit [3:0] I_ENDEM=4'd0, I_CITF=4'd1, I_MFC=4'd2, I_MFS=4'd3,
                     I_BCDINIT=4'd4, I_CLRCYV=4'd6, I_SETCYV=4'd7, I_SUSP=4'd8,
                     I_FLUSH=4'd9, I_SIGNTGL=4'd12, I_BCDNZ=4'd13,
                     I_FARJMP=4'd14;
localparam bit [2:0] E_MEMR=3'd1, E_MEMW=3'd2, E_INTATAIL=3'd3, E_INTA=3'd5,
                     E_WRITEBACK=3'd6;

// RepKind / RepTest
localparam bit [2:0] REP_NONE=3'd0, REP_E=3'd1, REP_NE=3'd2, REP_C=3'd3,
                     REP_NC=3'd4;
localparam bit [1:0] TEST_NONE=2'd0, TEST_Z=2'd1, TEST_CY=2'd2;

//============================================================================
// STATE
//============================================================================

// --- the architectural file ------------------------------------------------
reg [15:0] gpr [0:7];
reg [15:0] sreg [0:3];
reg [15:0] pc;
reg [15:0] psw;
reg [15:0] tmpa, tmpb, tmpc;
// The EA adder has a retained output beside the microcode tmp registers.
// Ordinary tmpa writes refresh it; a ModR/M EA calculation instead leaves the
// pre-displacement base here.  Only undocumented register-form LEA observes
// the distinction.
reg [15:0] ea_residue;
// A two-register EA also leaves its index input selected on the adder's other
// rail.  Unary EA forms do not select that rail, so their later undocumented
// LEA observation continues to come from the ordinary tmpb scratch path.
reg [15:0] ea_pair_rhs;
reg        ea_pair_valid;
reg [15:0] opr;
reg [15:0] ind;
reg [15:0] count;
reg  [7:0] pfxcnt;
reg [15:0] stat;
reg        sign_neg;
reg  [3:0] bit_n;

// --- the latched micro-ALU (sim/state.h::AluLatch) -------------------------
reg  [4:0] al_op;
reg  [1:0] al_tmp;
reg        al_byte;
reg        al_eaconst;
reg [15:0] al_eaval;
reg  [1:0] al_adjust;      // 0 none, 1 ADJD, 2 ADJA
reg  [1:0] al_adjtmp;
reg        al_bitarm;
reg  [3:0] al_bitn;
reg        al_spent;

// --- the micro-PC ----------------------------------------------------------
reg  [2:0] upc_page;
reg  [7:0] upc_opc;
reg  [3:0] upc_loc;

// --- prefix / pre-decode latches -------------------------------------------
reg        seg_override;
reg  [1:0] seg_ovr;
reg  [2:0] rep_kind;
reg        lock_pfx;
reg  [7:0] opc_reg;
reg        op8, imm8;
reg  [4:0] opc_base;
reg        opc_from_modrm;
reg  [2:0] modrm_reg;
reg  [3:0] xop;
reg  [1:0] rep_test;
reg        rep_pol;
reg        bus_word;
reg        opc8080;
reg        mode8080;
reg        intr_pending;
reg        eu_halted;

// --- the interrupt recognition (U2 pass 5; docs/facts/interrupt_model.md) ---
// The pins reach the decision through flops, and THAT is the whole mechanism:
// the ucore does NOT render `timed_runner.cpp`'s `D = max(B, A + pipe)`, which
// the third Codex review (ledger C5) showed to be an artefact of the REPLAY
// driver -- nothing in hardware can hold a boundary for a pin that has not
// asserted yet.  What is left is causal and small: pipeline the INT LEVEL,
// edge-latch NMI, and test the MATURED event at each boundary.
reg  [3:0] int_p;          // pin_int:  int_p[k] is the level of clock c-1-k
reg  [4:0] nmi_p;          // pin_nmi, one deeper (the latch is an EDGE)
reg        nmi_latch;      // set at edge+3, so it reads true from edge+4
reg  [3:0] ie_p;           // psw[FIE] through the SAME three flops
reg        rep_chain;      // ...this REP boundary is a CHAINED one (>= 2)
reg        irq_shadow;     // a segment-register write skips ONE boundary
reg        bnd_armed;      // this `S_OPC_POP` is an instruction boundary
reg        irq_sel_nmi;    // the kind, latched AT the boundary
reg        irq_sel_brk;    // ...and the THIRD kind: the single-step trap
reg        unhalt_pend;    // the NMI wake's `unhalt()`, owed to the entry clock
reg        irq_fast_inta; // first INTA may consume a standing CODE collision
reg        irq_halt_entry;// HALT wake owes one IRQ-dispatch handoff clock

// --- the BRK/TF single-step trap (§86; the law is §84.3 / §85.2b) ----------
// THE ARM DOES NOT SEE A `TF` THAT ROSE TOO RECENTLY, and it is an ARM, not a
// gate: a bit SAMPLED at one instruction boundary and TAKEN at the NEXT.
//
//   "At every instruction boundary the machine first TAKES the trap if the arm
//    bit is set (and the entry clears it), and then SAMPLES `TF` into the arm
//    bit.  A PREFIX BYTE ENDS AN INSTRUCTION BOUNDARY."   -- §84.1, verbatim
//
// MEASURED against silicon on a directed board cell built for it (§85.2b):
// the floor is **3 clocks**, 121,890 rows at 0 row-diffs, with no other value
// in [1,7] within four orders of magnitude.  §84.7's PURE GATE -- "take at the
// first boundary at which TF has been up for at least N clocks" -- is REFUTED
// by that cell's two SATURATED controls (`popfmemr`/`popfmul`, whose first
// boundary sits 13 and 25 clocks past the rise): the chip STILL waits one more
// boundary there, so the fitting `N` sets are {5,6,7} against {26,27,28} and
// no `N` exists.  That is an arm bit, and it is measured.
//
// NOTHING HERE NAMES AN OPCODE.  §84.1 wrote the POPF/IRET asymmetry as an
// opcode rule; §84.3 re-read it as ONE FLOOR (007A and 01EA are both
// `OPR -> FLAGS  F E` rows with identical geometry -- what differs is CLOCKS,
// because `9D` retires AT its own flag write and `CF` flushed the queue at
// 01E8), and the cell's `iret`/`popfnone` pair MEASURES the consequence:
// phase 2 against phase 3 at both wait levels, which only a clock floor gets.
// THE DEPTH IS **4**, AND IT IS THE MEASURED FLOOR OF **3** -- the difference
// is a COORDINATE, and it is the one `ie_p` already pays.  The model stamps the
// rise in `sample_ie()`, called at the TOP of `tick()`, so a PSW bit that is up
// during clock `c` is stamped `rise = c`; block (a) here freezes `brk_now` from
// the same register at the same instant and it reaches `brk_p[0]` one flop
// later.  MEASURED, engine vs engine on the cell's own sleds:
// `rise_sim = rise_rtl + 1` on 8 of 8 first rises (`popfnone`/`popfclc`/
// `popfmul`/`iret` x w0, w3), so the model's `c >= rise + 3` IS this module's
// `c >= rise + 4`.  It is the same offset that makes the INT gate read
// `ie_p[2]` -- three flops -- where the model's `kIeFloor` is 2.
//
// AND SILICON SAYS SO INDEPENDENTLY.  §85's cell re-scored with this core as
// the engine, per clock over its 30 retained captures, at every candidate
// depth in [1,7]:
//     1: 71,304   2: 71,304   3: 33,941   **4: 0**   5: 14,630
//     6: 43,762   7: 61,033          (121,860 rows each)
// One value is EXACT and nothing else is within four orders of magnitude --
// the same shape, on the same captures, that gave the model its 3.
//
// `V30_BRK_FLOOR` is the INSTRUMENT KNOB that made that table possible and
// nothing else: a compile-time define with the landed value as its default, no
// gate sets it, and the synthesised design carries exactly `BRK_FLOOR` flops.
// (The model's is `V30SIM_BRKFLOOR`; this is the same instrument, RTL side.)
//   *Falsifier*: any capture on which a depth other than 4 scores fewer
//   row-diffs against silicon than 4 does.
`ifndef V30_BRK_FLOOR
 `define V30_BRK_FLOOR 4
`endif
localparam int BRK_FLOOR = `V30_BRK_FLOOR;
reg [BRK_FLOOR-1:0] brk_p;  // psw[FBRK] through the SAME flops as `ie_p`
reg        brk_arm;        // THE ARM.  One flop, and it crosses boundaries.
reg        brk_smp;        // ...a boundary's F pop rode the PREVIOUS clock

// operand latches M / R / WB
reg  [1:0] m_kind, r_kind, wb_kind;
reg  [2:0] m_idx,  r_idx,  wb_idx;
reg [15:0] m_ea,   r_ea,   wb_ea;
reg  [2:0] m_seg,  r_seg,  wb_seg;
reg        m_byte, r_byte, wb_byte;

// --- the write-data pairing latch (exec_impl.h::Pending) --------------------
reg        pend_active;
reg [15:0] pend_off;
reg  [2:0] pend_seg;
reg        pend_byte;
reg        pend_io;
reg        opr_fresh;
// §87.A -- THE OPR-VALID INTERLOCK.  Has ANYTHING put a value into `opr` since
// this decode started?  The decoder's operand pre-read (`ld_preread`), a
// completed micro-row read delivered out of `rdq`, or a `-> OPR` transfer.
// It is NOT `opr_fresh`, which is the WRITE-PAIRING latch ("OPR carries a word
// a store has not taken yet") and is cleared by the pairing; conflating the two
// moves every paired store.  See `opr_starved` beside `f_wait`.
reg        opr_loaded;

// --- the bus interlock bookkeeping (the EU's half) -------------------------
reg [15:0] rdq0, rdq1;     // completed reads awaiting OPR delivery
reg  [1:0] rdq_n;
reg  [1:0] rd_pending;     // posted reads (rd_last) not yet completed
reg        ghost_rd_discard; // displaced tail completion after a mod3 POP read
reg        ghost_rd_feed;  // idle ghost may feed an overlapping successor read
reg        ghost_rd_ready; // ghost completion matured before successor T1
reg  [1:0] rd_done_cnt;    // completed, not yet consumed by an F row
reg        rd_age0;        // the oldest completion pulsed on THIS clock
reg        iend_owed;      // F22: the post-`E` row owes the successor its reset
reg  [2:0] rst_ctr;        // F25: the reset dispatch's four clocks
reg [15:0] tsel;           // F29: the LIVE tmp an arming row reads
reg  [7:0] pe_opc_reg;     // F23: the opcode context the post-`E` row runs on
reg        pe_opc8080;
reg        pe_op8;         // ...INCLUDING the operand width (pass-3 review D1)
reg  [7:0] pe_pfxcnt;      // ...and the prefix count (F22's registered residue)
reg  [1:0] wr_out;         // posted write CYCLES not yet done

// --- the opcode latch ------------------------------------------------------
reg        opc_valid;
reg  [7:0] opc_byte;
// PF_LOST can let the already-prefetched successor opcode take its one
// ModR/M byte before the overlapping operand read releases the loader.  This
// is the decoder's second input latch, not another prefetch-queue slot.
reg        opc_rm_valid;
reg  [7:0] opc_rm_byte;
reg        pop_is_first;

// --- the loader's working set ----------------------------------------------
reg  [7:0] ld_b;
reg [13:0] ld_pla;
reg        ld_ext;
reg  [2:0] ld_page;
reg        ld_hasrm;
reg  [7:0] ld_rm;
reg [15:0] ld_disp;
reg  [7:0] ld_dlo;
reg        ld_grpd;
reg        ld_byte;
reg        ld_preread;
reg        ld_ripe_prev;   // M8's `pen`: was the byte poppable BEFORE demand?

// --- the sequencer ---------------------------------------------------------
reg  [5:0] st;
reg  [1:0] chg;            // remaining charge clocks of the current step
reg        ending;         // the E row has run; the post-E row is next
reg  [1:0] rowq;           // queue bytes this row has already taken
reg        row_posted;     // this row's bus cycle has been posted
reg        row_paired;     // ...and its data handed over
reg [15:0] rloop_n;        // R-loop iterations still owed
reg        suppress_commit;
reg        first_pop_seen;
reg  [7:0] rowb0, rowb1;   // the queue bytes this row has taken
reg        poste;          // the post-E row's work is owed to the NEXT clock

//----------------------------------------------------------------------------
// STATES.  A state occupies one clock unless marked ZERO-COST, in which case
// it is executed inside the edge that reached it (the chain loop).
//----------------------------------------------------------------------------
localparam bit [5:0]
    S_OPC_POP   = 6'd0,   // pop the instruction's first byte (an F)
    S_TAKE_OPC  = 6'd1,   // ZERO-COST: consume the pre-popped opcode
    S_DECODE    = 6'd2,   // ZERO-COST: the prefix test
    S_DECODE2   = 6'd3,   // ZERO-COST: width / 1BL / ModR/M fan-out
    S_PFX_CHG   = 6'd4,   // a prefix's own second clock
    S_EXT_CHG1  = 6'd5,   // 0F: charge(1) before the second byte
    S_EXT_POP   = 6'd6,   // 0F: the real opcode, popped as an S
    S_1BL_LEAD  = 6'd7,   // wait_retire_lead ALONE (the write is at S_DECODE2)
    S_MODRM     = 6'd8,   // pop the ModR/M byte (opcode+1)
    S_D8_A      = 6'd9,   // opcode+2: no byte demanded
    S_D8_B      = 6'd10,  // opcode+3: the disp8 (M8's `pen`)
    S_D16_LO    = 6'd11,  // opcode+2
    S_D16_A     = 6'd12,  // opcode+3: no byte demanded
    S_D16_HI    = 6'd13,  // opcode+4: the high byte (M8's `pen`)
    S_EA_CHG    = 6'd14,  // mod != 3, no displacement: the EA-compute clock
    S_EA_CALC   = 6'd15,  // ZERO-COST: the address adder
    S_NORM_CHG  = 6'd16,  // no ModR/M: the opcode+1 clock
    S_BIND      = 6'd17,  // ZERO-COST: group dispatch, OPC select, binding
    S_PRERD     = 6'd18,  // the pre-decode operand read + wait_opr
    S_GRPD_CHG  = 6'd19,  // the second-byte group's extra clock
    S_ENTER     = 6'd20,  // ZERO-COST: enter the micro-sequence
    S_ROW       = 6'd22,  // a micro-row's own clock
    S_ROW_CHG   = 6'd23,  // the taken-JMP / FARJMP redirect bubble
    S_RLOOP     = 6'd24,  // one iterative step per clock
    S_EPOP      = 6'd25,  // the E row's successor pop, deferred
    S_TAIL      = 6'd26,  // ZERO-COST: run_micro's tail
    S_TAIL_W    = 6'd27,  // ...its F interlock + emit_pending
    S_TAIL_POP  = 6'd28,  // ...and its deferred opcode pre-pop
    S_INSTR_END = 6'd29,  // ZERO-COST: step() returns
    S_HALTED    = 6'd30,  // the part is parked
    S_RESET     = 6'd31,  // F25: the internal reset dispatch, 4 clocks
    S_IRQ_D     = 6'd32,  // the recognised boundary's ONE decision clock
    S_1BL_CHG   = 6'd33;  // ...and the 1BL form's own trailing charge(1)

// THE NINE STATES A ZERO-COST HAND-OVER CAN LEAVE THE CHAIN STANDING IN.
// Every OTHER arm's predecessors set `stop`, so no other state can occupy a
// chain position >= 1 -- the property v30u_eu_step.svh's 24 `chain == 4'd0`
// guards rest on, and the one this fails safe against.
//
// A FUNCTION AND NOT A WIRE, and that is F11b's trap for the fourth time: `st`
// is written with BLOCKING assignments inside the chain, so a `wire` off it
// carries the PRE-EDGE state and the test would ask about position 0's state
// at position 1.  Measured as written: 392/4164.
function automatic logic st_zero_ok(input [5:0] s);
    st_zero_ok = (s == S_TAKE_OPC) || (s == S_DECODE) || (s == S_DECODE2)
              || (s == S_EA_CALC)  || (s == S_BIND)   || (s == S_ENTER)
              || (s == S_TAIL)     || (s == S_TAIL_POP)
              || (s == S_INSTR_END);
endfunction

//============================================================================
// THE COMPOSITION (campaign risk #1, second half) -- F7, see the ledger
//============================================================================
// The BIU publishes TWO NAMED VIEWS of every quantity this module consumes
// (v30u_biu's "THE EU CONTRACT" block), and WHICH ONE to read is decided by
// WHERE the value is used, not by what a simulator happens to schedule:
//
//   * the COMBINATIONAL ACT DECODE below (`q_pop`, `eu_post`, `eu_pair`,
//     `q_flush`, `eu_susp`, `eu_halt`) reads the REGISTERED view -- `q_ripe`,
//     `q_byte`, `eu_slot_busy`, `eu_wr_done`, `eu_opr_free`.  An act is
//     consumed by the BIU on the clock that names it, so it must be decided
//     from the BIU's level DURING that clock;
//
//   * THE CLOCKED STEP reads the `_n` view -- `eu_slot_busy_n`,
//     `eu_rd_done_n`, `eu_wr_done_n`, `eu_rdata_n`,
//     `q_ripe_lead_n`, and the `*_n` stall wires derived from them.  That step
//     produces the EU's state for the clock this edge OPENS and must therefore
//     see the BIU as it will be then.
//
//   * `q_ripe` / `q_byte` are read by the clocked step in the REGISTERED view
//     and that is not an exception: the byte the step consumes is the one the
//     BIU handed over on the clock that is ending, which is the clock the pop
//     rode.  Everything else the step consults is the BIU it is stepping into.
//
// Nothing here depends on process order, and no `_n` signal reaches a
// combinational output of this module, so the EU->BIU direction stays a
// registered boundary and closes no loop.  REFUTED, and kept refuted: LATCHING
// the levels in this module -- that puts every pop one clock late (B8 500->123)
// because a latch delays the whole view instead of naming the two.

//============================================================================
// THE MICRO-ROW STANDING ON `upc`
//============================================================================
wire [12:0] dec_addr = {upc_page, upc_opc, upc_loc[3:2]};
wire        dec_valid;
wire  [8:0] dec_bank;
wire [28:0] row;

v30u_ucrom u_ucrom (
    .dec_addr (dec_addr),
    .dec_valid(dec_valid),
    .dec_bank (dec_bank),
    .rom_addr ({dec_bank, upc_loc[1:0]}),
    .rom_word (row)
);

// sw/ucore_tables.py::MicroOp -- the field positions, verbatim
wire [4:0] r_s1   = row[28:24];
wire [4:0] r_d1   = row[23:19];
wire [3:0] r_s2   = row[18:15];
wire [1:0] r_d2   = row[14:13];
wire       r_f    = ~row[12];
wire       r_w    = ~row[11];
wire       r_e    = ~row[10];
wire [1:0] r_type = row[9] ? TY_CTL : (row[8] ? TY_JMP : TY_ALU);
wire [4:0] r_aluop= row[7:3];
wire [1:0] r_alutmp = row[2:1];
wire       r_r    = row[0] && (r_type == TY_ALU);
wire [3:0] r_cond = row[7:4];
wire [3:0] r_loc  = row[3:0];
wire [3:0] r_ictl = row[9:6] & 4'hF;   // {row[9]=1 marks CTL}: ictl = row[8:5]
wire [2:0] r_ectl = row[4:2];
wire [1:0] r_sr   = row[1:0];

// The CTL field split, taken literally from the reference parse:
//     ictl = (bits >> 5) & 15 ; ectl = (bits >> 2) & 7 ; sr = bits & 3
wire [3:0] c_ictl = row[8:5];
wire [2:0] c_ectl = row[4:2];
wire [1:0] c_sr   = row[1:0];

wire       r_farjmp = (r_type == TY_CTL) && (c_ictl == I_FARJMP);
wire [4:0] r_farloc = {c_ectl, c_sr};
wire       r_nopmove = (r_s1 == 5'h1F) && (r_d1 == 5'h1F);
wire       r_hasconst = (r_s1 == 5'h17);
wire [5:0] r_constval = {r_s2, r_d2};
// A FARJMP row aliases Ext:SR as its target, so it has no bus cycle.
wire [2:0] r_ect = r_farjmp ? 3'd7 : c_ectl;

// unmapped micro-address: the model substitutes a NOP CTL row
wire       row_nop = !dec_valid;
wire [4:0] e_s1    = row_nop ? 5'h1F : r_s1;
wire [4:0] e_d1    = row_nop ? 5'h1F : r_d1;
wire [3:0] e_s2    = row_nop ? 4'hF  : r_s2;
wire [1:0] e_d2    = row_nop ? 2'd3  : r_d2;
wire       e_f     = row_nop ? 1'b0  : r_f;
wire       e_w     = row_nop ? 1'b0  : r_w;
wire       e_e     = row_nop ? 1'b0  : r_e;
wire       e_r     = row_nop ? 1'b0  : r_r;
wire [1:0] e_type  = row_nop ? TY_CTL : r_type;
wire [3:0] e_ictl  = row_nop ? 4'hF  : c_ictl;
wire [2:0] e_ectl  = row_nop ? 3'd7  : r_ect;
wire [1:0] e_sr    = row_nop ? 2'd3  : c_sr;
wire       e_farjmp= row_nop ? 1'b0  : r_farjmp;
wire       e_nopmv = row_nop ? 1'b1  : r_nopmove;
wire       e_hasc  = row_nop ? 1'b0  : r_hasconst;

wire e_is_rloop = (e_type == TY_ALU) && e_r;
wire e_have1 = !e_nopmv;
wire e_have2 = !e_hasc && ((e_s2 != 4'd15) || (e_d2 != 2'd3));

// The 0F21 negative/no-carry result rail advances the E hand-over to row 5 and inhibits
// the stale architectural writeback that would otherwise ride that shortened
// path.  A non-negative 0F21 capture and four 0F23 captures all retain the
// ordinary row-6 hand-over, so the condition is the ALU sign rail, not an
// opcode-wide timing shortcut.
wire ext4s_early_e = (upc_page == 3'd4) && (upc_opc == 8'h21) &&
                     (upc_loc == 4'd5) && stat[FS] && !stat[FCY];
wire ext4s_early_wblock = (upc_page == 3'd4) && (upc_opc == 8'h21) &&
                          (upc_loc == 4'd3) && sig_flags[FS] && !sig_flags[FCY];
// On the shortened path row 6 is the post-E overlap.  On the ordinary path it
// is the E row itself and `poste` is still clear, which makes this distinction
// state-free.
wire ext4s_early_post = poste && (upc_page == 3'd4) &&
                        (upc_opc == 8'h21) && (upc_loc == 4'd6);
wire ext4s_arch_d1 = (e_d1 <= 5'd4) || (e_d1 == 5'd15) ||
                     (e_d1 == 5'd18) || (e_d1 == 5'd19) ||
                     (e_d1 >= 5'd24);

//============================================================================
// SEGMENT / WIDTH HELPERS
//============================================================================
// sim/biu_timed.cpp::seg_code -- sim Sreg ES,CS,SS,DS = 0,1,2,3; the S4:S3 pin
// code is ES,SS,CS,DS = 0,1,2,3.  kSegZero and I/O drive the "no segment"
// code, which is the same encoding as CS.
function automatic [1:0] seg_code(input [2:0] s);
    case (s)
        3'd0: seg_code = 2'd0;   // ES
        3'd1: seg_code = 2'd2;   // CS
        3'd2: seg_code = 2'd1;   // SS
        3'd3: seg_code = 2'd3;   // DS
        default: seg_code = 2'd2;
    endcase
endfunction

// exec_impl.h::sr_is_io -- SR == IO selects the I/O space only for the port /
// block-I/O opcode classes.
wire row_io  = (e_sr == 2'd1) && ((xop == 4'hF) || (xop == 4'h6));
// exec_impl.h::sr_segment
wire [2:0] row_seg = (e_sr == 2'd0) ? 3'd0
                   : (e_sr == 2'd1) ? SEG_ZERO
                   : (e_sr == 2'd2) ? 3'd2
                   : (m_kind == OK_MEM) ? m_seg
                   : (r_kind == OK_MEM) ? r_seg
                   : (seg_override ? {1'b0, seg_ovr} : 3'd3);
// Bus width follows OP8b unless Ext [-03-] has forced WORD (ledger A37).
wire row_bbyte = op8 && !bus_word;

wire [15:0] seg_val = (row_seg == SEG_ZERO) ? 16'h0000 : sreg[row_seg[1:0]];
wire [19:0] row_phys = {seg_val, 4'd0} + {4'd0, ind};

//============================================================================
// STALLS (the five named wires)
//============================================================================
// A row's F interlock: which HALF applies is the direction of the row's own
// touch (exec_impl.h::deliver_read).  A row that READS OPR waits for the read
// that fills it; a row that only LOADS OPR waits only for a store to give OPR
// back.
wire row_reads_opr = (e_s1 == 5'd6);

// wait_next_read(extra): the next outstanding EU read, in order.
//
// `|| eu_rd_done_n` is F11a's rule on the READ side, and it was missing.  The
// WRITE side already reads its completion as REGISTER-ONLY LOOKAHEAD
// (`retire_ok_n`'s `eu_wr_done_n`, published from `r_done_ctr`/`r_done_wr`),
// because `wait_bus`'s deadline is a clock, not a flag: the row runs ON the
// completion clock.  `wait_next_read` is the same deadline -- the model waits
// `while (clk_ < rd_done_q_.front())` and runs AT that clock -- so the read
// side must read the same lookahead.  Without it the EU's own `rd_done_cnt`
// only rises on the clock AFTER the completion and every `F` row that waits on
// a read runs one clock late (`58 idx 0`: the successor's F pop at row 14
// where the golden has it at row 13; the whole POP/RET/direct-address family).
// The consumer sees the count already incremented because block (a) runs
// BEFORE the chain in the same edge.
wire nr_have   = (rd_done_cnt != 2'd0) || eu_rd_done_n;
wire nr_wait   = !nr_have && (rd_pending != 2'd0);
// ...and `extra` (wait_opr's +1) bites only when the completion is on THIS
// clock -- otherwise the deadline is already past.
wire nr_extra_block = nr_have && rd_age0;

// wait_opr_free: the store lets go of OPR at its own T2+1 (11.4, fixed index).
// F7: the same term in the two views -- the plain name for the act decode, the
// `_n` name for the clocked step.  ONE expression each, differing only in which
// BIU view it reads.
// F31 -- THE HOLD IS THE BIU'S COUNT, AND THERE IS ONLY ONE VIEW OF IT.
//
// The EU used to keep its OWN `opr_owned` counter of the stores it had paired
// and test it against the BIU's release PULSE.  That is F11's named error --
// one event, two reconstructions -- and it was wrong in both of its parts:
//
//  * the pulse view.  `opr_free_now_n` read `eu_opr_free_n`, the release
//    computed DURING the T2 clock, so the `F` row completed at the end of T2.
//    The model's release instant is `opr_free_clk_ = T2 + 1` and the row runs
//    ON that clock, i.e. T3.  Every non-OPR-reading `F` row therefore ran one
//    clock early, which is the whole multi-cycle-push family: `60`'s ROM
//    alternates `SIGMA -> IND CTL MEMW SS` with `<reg> -> OPR F CTL`, and the
//    idle `Ti` the golden shows between consecutive stores IS that clock.
//    (`60` was green on push #1 only, and only by accident: the FARJMP bubble
//    after 023B put the clock back.)
//  * the count.  The model's `++opr_held_` is CONDITIONAL --
//    `if (!(r == &cur_ && run_ && ci_ > 1))`, a fact about the BIU's own
//    running cycle -- and the EU cannot see it, so an EU-side count can never
//    be faithful.  `opr_owned += (pend_split ? 2 : 1)` also over-counted a
//    split against the pulse test: ALL 96 odd-SP `60` cases failed and ALL 104
//    even-SP cases passed.
//
// So the counter is the BIU's `opr_held` (which IS `BiuTimed::opr_held_`) and
// the published fact is the model's own predicate off the REGISTER.  One
// expression, both views -- the act decode and the clocked step cannot drift.
//
// C4 (THE THIRD CODEX REVIEW) -- ...AND M13's 8080 ARM WAS NEVER RENDERED.
// `wait_opr_free()` opens with `if (md8080_ && *md8080_) { wait_bus(); return; }`
// -- "in 8080 EMULATION MODE the store does not let go until it has RETIRED.
// In native mode the store hands its word to the AD output latch at T2 and OPR
// is free from T3 (11.4, the FIXED index 2 that does NOT stretch).  With MD set
// the release is the store's own eu_done -- the completion eval + 2, the
// deadline `wait_bus()` already carries -- so it STRETCHES with the eval like
// every other eval-keyed quantity."  The EU had NO mode term at all, in this
// rendering OR in the pulse rendering F31 replaced, so this is PRE-EXISTING and
// not F31's -- but F31 made this the one place it can live.  `wait_bus`'s
// deadline is `retire_ok_n`, so the arm is the SPEC's own line, verbatim.
//
// UNREACHABLE ON THE CURRENT STIMULUS and therefore UNVERIFIED BY IT: `mode8080`
// is set only by an `MFC` row, and the 8080 loader / BRKEM path is ledger R4,
// unimplemented.  The census proves the no-op (G3 164,787 either way, zero forms
// moved), which is D1's shape -- a latent correctness fix whose score does not
// move.  *Falsifier*: the first 8080-mode store the ucore executes, which is
// R4's own gate; that is where this arm gets its verification, not here.
// wait_bus (the retire deadline): every posted store, then its e+2.  ONE view
// only -- `eu_wr_done_n` is registered logic (see the BIU's `done_fire`), so
// the act decode may read it, and act and step MUST read the same thing: they
// are the same event seen from two sides (F11).
wire retire_ok_n = (wr_out == 2'd0) ||
                   ((wr_out == 2'd1) && eu_wr_done_n);

// W7 -- THE MFS FRAME STORE RELEASES AT ITS COMPLETION EVAL.  The consumer is
// the semantic row `CS -> OPR, SIGMA -> IND, F, JMP CNTZ`: it immediately
// follows the `MFS + MEMW SS` row, then the ROM runs MFC, 01F8 and the next
// MEMW issue.  Letting the fixed T3 release win erased those ROM clocks when
// the write stretched; waiting through retire added two invented clocks.  The
// completion pulse leaves the three real intervening rows to set the gap.
// This names the row's controls, not its ROM address or an opcode.
wire mfs_frame_release = (st == S_ROW) && e_f &&
                         (e_type == TY_JMP) && (r_cond == C_CNTZ) &&
                         e_have1 && (e_s1 == 5'd1) && (e_d1 == 5'd6);
wire eval_ok_n = (wr_out == 2'd0) ||
                 ((wr_out == 2'd1) && eu_wr_eval);
wire opr_free_now = mode8080          ? retire_ok_n
                  : mfs_frame_release ? eval_ok_n
                                      : eu_opr_free;

// §87.A -- THE ILLEGAL-FORM STALL.  `nr_wait` waits for the next OUTSTANDING
// read; when nothing is outstanding it is 0 and the row runs.  But the `F`
// interlock is a WAIT, and a wait needs something to wait for: a row that
// SOURCES OPR is waiting for the read that FILLS OPR, and if nothing has filled
// it since this decode began (`opr_loaded`) AND nothing is outstanding or
// completed, that wait has no terminator.  The EU parks on the row forever.
//
// It is NOT a halt: no `HLT` row ran, `eu_halted` stays clear, the HALT status
// is never driven and the PREFETCHER IS NOT FROZEN -- the BIU goes on fetching
// until the queue is full and then sits `PASV` with `qs = 0`, which is exactly
// the 957-3,906-row idle tail silicon shows (`ucore_provenance.md` §86.F).
//
// The rule NAMES NO OPCODE.  Swept over the whole opcode space -- native and
// `0F`, every ModR/M `reg`, `mod == 3` and memory, 8,192 forms -- it fires on
// `62` CHKIND, `C4` LES, `C5` LDS and the `FE`/`FF` group at `/3` and `/5`, all
// at `mod == 3`, and on NOTHING else: those are the forms whose first row takes
// the pre-read word out of OPR while a second word is still being fetched, and
// `mod == 3` is exactly the case where the decoder issues no pre-read at all.
// `FF /7` at `mod == 3` (2,477 `v20suite` goldens) does NOT fire -- its row
// `M -> OPR` WRITES the register -- and neither does `8D` LEA, which is why the
// archived FSM core's `S_HALT` wedge was right on `62`/`C4`/`C5` and a BUG on
// LEA (`tests/v30/mod3_illegal/metadata.json`).
//
// *Falsifier*: any golden case, in any suite, that reaches this wire (a golden
// is a captured chip record, and a stalled chip records no case); or a form
// outside the seven that stalls on silicon; or one of the seven that does not.
wire opr_starved = row_reads_opr && !opr_loaded && !nr_have &&
                   (rd_pending == 2'd0) && (rdq_n == 2'd0);

// ONE view: the `_n` twin is gone with the pulse it read (F31).  `nr_wait` is
// register-only lookahead (F18/F11a) and is legal in the act decode.
wire f_wait = row_reads_opr ? (nr_wait || opr_starved || !opr_free_now)
                            : !opr_free_now;

// BUSY is the 9B POLL_N pin, sampled through the same 3-deep pin pipeline the
// INT level goes through (biu_timed.h::poll_busy, "AND IT IS THE SAME 3-DEEP
// PIN PIPELINE").  U2 pass 5: the pipeline is REAL now, and its RESET VALUE is
// the pin -- a shift register that has been clocked since power-on holds the
// level it has been seeing, and `poll_busy()` reads a STATICALLY LOW POLL_N as
// "not busy" on clock 0 (`if (!(ev_pins_ & 4)) return false;`).  Reset to all
// ones instead made the first three clocks read BUSY, which is exactly the
// `POLL.LO` half that failed at row 3.
reg [2:0] poll_pipe;
wire poll_busy = poll_pipe[2];

//----------------------------------------------------------------------------
// THE RECOGNITION (interrupt_model.md's two laws, and nothing else)
//----------------------------------------------------------------------------
// "the boundary decision runs during the would-pop cycle B and sees the pin
// level of cycle B-3" -- so the INT LEVEL and the IE GATE go through the same
// three flops, which is also why there is no separate EI shadow flag.
// "NMI latch: set 3 cycles after the pin edge; latest catching edge = B-4".
//
// EVERY read of these registers from inside the clocked step goes through
// these wires: block (a) shifts the pipelines with BLOCKING assignments at the
// top of the edge, so the REGISTERS already hold the NEXT clock's view while
// the wires still hold THIS clock's (F11b's trap, and the whole module's
// convention).
wire irq_pin_int = int_p[2];                       // the pin at c-3
// SM3 SITTING 11 -- THE GATE READS IE TWICE, AND THE SECOND READ IS THE LIVE
// ONE.  `ie_p[2]` alone says only "IE was up three clocks ago"; a `CLI` since
// then does not take it back, so the recognition fires at a boundary where the
// architectural IE is CLEAR and pushes a PSW with IE = 0.  MEASURED on the
// socket (`sm3_s11_prereg_2026-08-04.md` §4, S3/S4/S5): on `CLI;POPF;NOP;NOP`
// and `CLI;STI;NOP;NOP` the chip takes the boundary after the CLI **0 times in
// 24 delays at every wait level**, and on `CLI;POPF` -- where that boundary is
// the only one IE ever reaches -- it takes NO ENTRY AT ALL in 24 of 24, while
// the same sled on the **NMI** pin acknowledges 24 of 24.  This core scored
// 3 / 5 / 24 of those, all of them at the forbidden boundary.
//   *Falsifier*: a maskable acknowledge on silicon whose PUSHED PSW has IE = 0.
// A request sampled on the same clock IE rises is retained in the existing
// `intr_pending` latch (block (a)) until this floor matures.  The latch has a
// second, older use for REP withdrawal, so only a non-REP context feeds it
// into this ordinary-boundary path; REP consumes it through C_INTR instead.
wire irq_int_lvl = (int_p[2] ||
                    (intr_pending && (rep_kind == REP_NONE))) &&
                   ie_p[2] && psw[FIE];
wire irq_nmi_lvl = nmi_latch;
wire irq_any     = irq_nmi_lvl || irq_int_lvl;
// A REP iteration boundary samples at the SAME depth below its own decision
// EDGE (interrupt_model.md, "REP abort": pin@edge-4) and reads the LIVE IE,
// not the pipelined one -- but its edge is NOT the loop row's clock, and the
// two anchors the SPEC records are one clock apart:
//
//   "the boundary-1 decision edge sits at a fixed opcode-pop+7 ... its flush
//    is invariant at pop+16 = edge+9.  Chained boundaries (>= 2) are
//    write-accept-anchored: decision at the accept edge, flush at accept+9."
//
// The `JMP REP` row stands at opcode-pop+6, so the FIRST boundary's edge is
// the row's clock + 1 (tap c-3) and a CHAINED one's is + 2 (tap c-2) -- the
// chained element's own store is still PENDING at the loop row, so its accept
// is one clock further out.  MEASURED over the 56 `INT.F3AA` mid-string
// aborts: the golden's flush is at pop+16 in ALL 35 one-element aborts and at
// the last write's T1 + 8 (= accept + 9) in ALL 21 chained ones.
//
// This is why a SINGLE tap depth has no clean fit (U2 pass 5 recorded the scan
// as a negative result: [0] 174, [1] 178, [2] 179, [3] 175) -- the two
// boundaries want DIFFERENT depths, because they are anchored to different
// edges.  Nothing here is fitted: both taps are `edge - 4`.
wire irq_rep_1st = irq_nmi_lvl || (int_p[2] && psw[FIE]);   // edge = c+1
wire irq_rep_chn = irq_nmi_lvl || (int_p[1] && psw[FIE]);   // edge = c+2

//============================================================================
// THE ROW'S DEMANDS
//============================================================================
// A queue byte: Source1 [-07-] (Q) and Source2 [-05-] (Q).
wire row_q1 = e_have1 && (e_s1 == 5'd7);
wire row_q2 = e_have2 && (e_s2 == 4'd5);
wire [1:0] row_qn = {1'b0, row_q1} + {1'b0, row_q2};
wire row_need_q  = (st == S_ROW) && ({1'b0, rowq} < row_qn);

// The bus rows.  [-06-] commits OPR to the r/m operand only when that operand
// is memory (the 8F mod==3 ghost).
// F28 -- A ROW SUPPRESSES ITS OWN WRITE-BACK.  `suppress_commit` is set INSIDE
// v30u_eu_row.svh, on the very row whose `[-06-]` it cancels (CMP: the ALU does
// not drive the result bus), so the act decode must reconstruct it rather than
// read the register a clock behind.  Without it `38`/`39`/`80.7` posted a store
// that never happened, `pend_after` deferred the successor's pop, and the F
// landed one clock late.
wire suppress_now = (e_s1 == 5'd20) && !sig_commits &&
                    (e_d1 == 5'd19) && (m_kind == OK_MEM);
wire row_wb_mem  = (wb_kind == OK_MEM) && !suppress_commit && !suppress_now;
wire row_is_read = (e_type == TY_CTL) && (e_ectl == E_MEMR);
wire row_is_wr   = (e_type == TY_CTL) && (e_ectl == E_MEMW);
wire row_is_wb   = (e_type == TY_CTL) && (e_ectl == E_WRITEBACK) && row_wb_mem;
wire row_is_inta = (e_type == TY_CTL) && (e_ectl == E_INTA);
wire row_bus     = row_is_read || row_is_wr || row_is_wb || row_is_inta;

// A register-bound POP has no ModR/M address to compute, but its discarded
// stack-read row still reaches the bus.  Row 0058 copies the standing SIGMA
// value into tmpa; using that retained scratch value reproduces the measured
// ghost address for the closed histories, while the still-divergent histories
// remain evidence that the die's stale-address source is not yet universalized.
// The same row geometry is shared by ordinary POP-register paths, so the
// undocumented 8F page and its absent memory operand are both part of the
// select.
wire ghost_read_stale_alu = (upc_page == 3'd0) && (upc_opc == 8'h8f) &&
                            row_is_read && (row_seg == 3'd2) &&
                            (m_kind == OK_REG) && (wb_kind == OK_REG) &&
                            e_have1 && (e_s1 == 5'd28) && (e_d1 == 5'd5) &&
                            e_have2 && (e_s2 == 4'd12) && (e_d2 == 2'd1);
// On the one measured successor overlap, the completed 8F sequence still
// drives the microaddress while the next loader pre-read stands.  The ghost
// completion's existing discard bit is its phase/cadence provenance.  The
// separate feed bit below decides only whether its data may fill the younger
// untagged read slot; full-phase ghosts still need this tail's bus geometry.
wire ghost_preread_tail = (st == S_PRERD) && (upc_page == 3'd0) &&
                           (upc_opc == 8'h8f) && (upc_loc == 4'd4) &&
                           ghost_rd_discard;
// Its T1 is also the successor-opcode pop point: use the ordinary pre-pop
// latch below, so the later E row sees an already-held byte rather than
// popping it a second time.
wire ghost_preread_epop = (st == S_PRERD) && ghost_rd_ready &&
                           !opc_valid && q_ripe &&
                           (row_posted ||
                            (ghost_rd_discard && eu_rd_wait)) &&
                           ((ghost_rd_discard &&
                             (ghost_rd_feed || eu_access_active)) ||
                            (!ghost_rd_discard &&
                             (eu_rd_edge || eu_rd_wait)));
wire ghost_preread_edge_lag = eu_rd_edge && ghost_preread_tail && opc_valid;
wire ghost_preread_late = (ghost_preread_epop && !ghost_rd_discard &&
                           eu_rd_edge) || ghost_preread_edge_lag;
// Once the PF_LOST opcode is held, the same overlap admits exactly its
// ModR/M byte.  The ordinary decoder consumes this latch in S_MODRM; all
// displacement and immediate bytes still come directly from the BIU queue.
wire ghost_rm_pop = ghost_rd_ready && opc_valid && !opc_rm_valid && q_ripe &&
                    pla3_has_modrm(pla3_native(opc_byte));
// The prefixed E4 witness reaches the same untagged hand-off one stage before
// its micro-row read posts: the ghost completes on the normal-decode charge.
// That completion is the read store's head; E4's later physical result is the
// unmatched tail.  This is also the phase that loses E4's I/O and byte rails.
wire ghost_row_tail = ghost_rd_discard && (pe_opc_reg == 8'h8f) &&
                      (st == S_NORM_CHG) && (ld_page == 3'd1) &&
                      (ld_b == 8'he4);

// the access this row asks for.  Its OFFSET is `ind_now`, which is the row's
// OWN IND write when it has one -- see "THE ROW'S OWN TRANSFERS" below, next
// to the source muxes it needs.
wire [2:0] acc_seg   = row_is_wb ? wb_seg : row_seg;
// PF_LOST reaches the successor's bus-space select as well as its stale
// address.  On the measured REP-prefixed E4 successor the I/O rail is absent,
// so the otherwise identical read is a word memory cycle at offset 0042.
wire       ghost_lost_io = row_io && (upc_page == 3'd1) &&
                           (upc_opc == 8'he4) && (pe_opc_reg == 8'h8f);
wire       acc_byte  = row_is_wb ? wb_byte : (row_bbyte && !ghost_lost_io);
wire       acc_io    = row_is_wb ? 1'b0 : (row_io && !ghost_lost_io);
wire [15:0] acc_segv = (acc_seg == SEG_ZERO) ? 16'h0000 : sreg[acc_seg[1:0]];

// The PRE-DECODE operand read (loader_impl.h): the operand the sequence
// READS, and only that one.
wire        pr_use_m = (m_kind == OK_MEM);
wire  [2:0] pr_seg   = pr_use_m ? m_seg : r_seg;
wire [15:0] pr_ea    = pr_use_m ? m_ea   : r_ea;
wire        pr_byte  = pr_use_m ? m_byte : r_byte;
wire [19:0] pr_phys  = {sreg[pr_seg[1:0]], 4'd0} + {4'd0, pr_ea};
wire        pr_split = !pr_byte && pr_phys[0];
// A PF_LOST overlap on an odd pre-read leaves the normal segment rail on its
// first byte and the default data-segment rail on its second.  The BIU already
// stores each split half independently; publish both rails with the two
// addresses rather than adding any transaction history.
wire  [2:0] pr_seg2  = (ghost_preread_tail && pr_split) ? 3'd3 : pr_seg;
wire [19:0] pr_phys2 = {sreg[pr_seg2[1:0]], 4'd0} + {4'd0, pr_ea + 16'd1};

// The staged write in the pairing latch (exec_impl.h::Pending).
wire [15:0] pend_segv = (pend_seg == SEG_ZERO) ? 16'h0000 : sreg[pend_seg[1:0]];
wire [19:0] pend_phys = pend_io ? {4'd0, pend_off}
                                : ({pend_segv, 4'd0} + {4'd0, pend_off});
wire       pend_split = !pend_byte && pend_phys[0];

//============================================================================
// THE ALU
//============================================================================
function automatic [15:0] szp(input [16:0] r, input bbyte);
    logic [15:0] f;
    logic p;
    begin
        f = 16'd0;
        if (bbyte ? (r[7:0] == 8'd0) : (r[15:0] == 16'd0)) f[FZ] = 1'b1;
        if (bbyte ? r[7] : r[15]) f[FS] = 1'b1;
        p = ^r[7:0];
        if (!p) f[FP] = 1'b1;              // even parity -> P = 1
        szp = f;
    end
endfunction

// alu_opc_select
//
// F23 -- ...AND THE OPCODE LATCH THE `ALU OPC` MUX READS.  `opc_reg` is not
// RESET by the successor's decode, it is OVERWRITTEN (S_DECODE2, `opc_reg =
// ld_b`), so F22's deferral cannot reach it: by the post-`E` row's clock the
// EU already held the SUCCESSOR's opcode and every `ALU OPC` row resolved
// against it.  The whole accumulator-immediate block picked the operation the
// injected `90` selects -- index 2, ADC -- which is why `14` (ADC AL,imm8) was
// the ONE form of its group that passed and `24` (AND) came out as an add.
//
// The decoder's opcode latch loads at the END of its own clock; the post-`E`
// row reads it BEFORE that.  So the row's own opcode context travels with F8's
// debt: 9 bits, captured where `poste` is raised.
// D1 (SECOND CODEX REVIEW, §32) -- and `op8` travels with them.  F23 shadowed
// nine bits and claimed that was the row's whole opcode context; it was not.
// S_DECODE2 OVERWRITES `op8` on edge `c` exactly as it overwrites `opc_reg`
// (`v30u_eu_step.svh`'s `op8 = ld_byte`), and the post-`E` row reads it in
// three places: `al_byte = op8` when it latches an ALU op, `SIGNTGL`'s
// `tmpb[7]` vs `tmpb[15]`, and `dir*sz` as a Source1.  Ten bits, not nine.
// F22 SETTLED (pass 4) -- ...and `pfxcnt` travels with them, for the SAME
// reason, which is the reason the residue existed at all.
//
// F22 and F23/D1 are ONE mechanism seen from two sides -- "the post-`E` row
// runs on the machine it belongs to" -- and which RENDERING a field takes is
// decided by exactly one question: DOES THE SUCCESSOR'S DECODE CHAIN WRITE
// THIS FIELD ON EDGE `c`?
//
//   no  -> DEFER the successor's reset past the discharge (`iend_late`).  The
//          register still holds the predecessor's value when the row reads it,
//          so no copy is needed.
//   yes -> the value must TRAVEL, because the register has already moved on.
//
// `pfxcnt` is the ONE field that is in both sets: `loader_decode`'s prologue
// RESETS it and S_DECODE's prefix arm WRITES it (`pfxcnt = pfxcnt + 1`).
// Deferring its reset was therefore wrong in both directions at once --
//   * the post-`E` row read `pfxcnt + 1` (the successor prefix's increment
//     stacked on the predecessor's count), and
//   * the deferred reset then landed on edge `c+1` and DESTROYED that
//     increment, so the successor ran its whole instruction with `pfxcnt = 0`.
// The second is the one that matters: `pfxcnt` is read on exactly ONE ROM row
// in the whole part -- `0225 PFXCNT -> tmpa`, the REPX withdrawal's
// `PC := PC - PFXCNT - 1` rewind -- and a `REP` string is BY CONSTRUCTION
// reached through a prefix.  Under whole-program replay every mid-string
// interrupt would have resumed at the opcode instead of at the first prefix,
// i.e. the 8086 lost-prefix bug the V30's ROM exists to avoid.
//
// So `pfxcnt` leaves the deferred set and joins the debt: the reset is back in
// S_INSTR_END's immediate block (the successor's prologue, where the model puts
// it) and the post-`E` row reads the copy.  The debt is EIGHTEEN bits.
// *Falsifier*: any other ROM row that reads PFXCNT, or any edge-`c` decode-chain
// write to one of `iend_late`'s remaining fields.
wire [7:0] opc_reg_eff = poste ? pe_opc_reg : opc_reg;
wire       opc8080_eff = poste ? pe_opc8080 : opc8080;
wire       op8_eff     = poste ? pe_op8     : op8;
wire [7:0] pfxcnt_eff  = poste ? pe_pfxcnt  : pfxcnt;
wire [2:0] opc_sel = opc_from_modrm ? modrm_reg : opc_reg_eff[5:3];
function automatic [4:0] opc8080_map(input [2:0] s);
    case (s)
        3'd0: opc8080_map = A_ADD;
        3'd1: opc8080_map = A_ADC;
        3'd2: opc8080_map = A_SUB;
        3'd3: opc8080_map = A_SBB;
        3'd4: opc8080_map = A_AND;
        3'd5: opc8080_map = A_XOR;
        3'd6: opc8080_map = A_OR;
        default: opc8080_map = A_CMP;
    endcase
endfunction
wire [4:0] alu_opc_sel = opc8080_eff ? opc8080_map(opc_sel)
                                     : (opc_base + {2'b0, opc_sel});

wire [4:0] eff_op = (al_op == A_OPC) ? alu_opc_sel : al_op;
// the operation the row NOW standing latches (exec_impl.h's `new_op`)
wire [4:0] nxt_op = (r_aluop == A_OPC) ? alu_opc_sel : r_aluop;
wire is_iter = ((eff_op >= A_ROL) && (eff_op <= A_SAR)) ||
               (eff_op == A_ROL12) || (eff_op == A_DIV) || (eff_op == A_MUL);

wire [15:0] tmps [0:3];
assign tmps[0] = tmpa;
assign tmps[1] = tmpb;
assign tmps[2] = tmpc;
assign tmps[3] = 16'd0;
// the same three, read by the row that LATCHES BIT / ABS
wire [15:0] tmps_lat [0:3];
assign tmps_lat[0] = tmpa;
assign tmps_lat[1] = tmpb;
assign tmps_lat[2] = tmpc;
assign tmps_lat[3] = 16'd0;

// The datapath is 16 bits ALWAYS for the ADD-class ops; `byte` selects only
// where the flag taps sit (ledger, "ALU width").
wire ev_byte = al_byte && (eff_op != A_INC2) && (eff_op != A_DEC2);
wire [15:0] port_a = tmpb;
wire [15:0] port_b_raw = tmps[al_tmp];
wire [15:0] port_b = al_bitarm ? (port_b_raw & (16'd1 << al_bitn))
                               : port_b_raw;
wire [15:0] a_m = ev_byte ? {8'd0, port_a[7:0]} : port_a;
wire [15:0] b_m = ev_byte ? {8'd0, port_b[7:0]} : port_b;
wire        cin = psw[FCY];

// --- the BCD adjust unit ---------------------------------------------------
wire [15:0] adj_src = tmps[al_adjtmp];
wire [7:0]  adj_al  = adj_src[7:0];
wire adj_lo  = (adj_al[3:0] > 4'd9) || psw[FAC];
wire adj_hi  = (al_adjust == 2'd1) &&
               (((adj_al[7:4] > 4'd9) || psw[FCY]) ||
                ((adj_al[7:4] == 4'd9) && (adj_al[3:0] > 4'd9) && !psw[FAC]));
wire [7:0] adj_corr = (adj_lo ? 8'h06 : 8'h00) + (adj_hi ? 8'h60 : 8'h00);
wire adj_sub = (eff_op == A_SUB);
wire [8:0] adj_sum = adj_sub ? ({1'b0, adj_al} - {1'b0, adj_corr})
                             : ({1'b0, adj_al} + {1'b0, adj_corr});
wire [7:0] adj_val8 = (al_adjust == 2'd2) ? {4'd0, adj_sum[3:0]} : adj_sum[7:0];
wire [7:0] adj_ahi = adj_src[15:8];
wire [7:0] adj_bhi = port_b[15:8];
wire [7:0] adj_rhi = adj_sub ? (adj_ahi - adj_bhi - {7'd0, adj_sum[8]})
                             : (adj_ahi + adj_bhi + {7'd0, adj_sum[8]});
wire [15:0] adj_flags = szp({9'd0, adj_sum[7:0]}, 1'b1)
                      | (16'd1 << FCY) & {16{(al_adjust == 2'd2) ? adj_lo : adj_hi}}
                      | (16'd1 << FAC) & {16{adj_lo}}
                      | (16'd1 << FV) & {16{adj_sub
                          ? (((adj_al ^ adj_corr) & (adj_al ^ adj_sum[7:0])) & 8'h80) != 8'h00
                          : ((~(adj_al ^ adj_corr) & (adj_al ^ adj_sum[7:0])) & 8'h80) != 8'h00}};

// --- the combinational ops -------------------------------------------------
reg [16:0] ar_full;      // 17-bit so the carry-out is visible
reg [16:0] ar_m;
reg [15:0] ev_val;
reg [15:0] ev_flags;
reg [15:0] ev_mask;
reg        ev_commits;

wire [16:0] add_m = {1'b0, a_m} + {1'b0, b_m} +
                    {16'd0, ((eff_op == A_ADC) ? cin : 1'b0)};
wire [16:0] sub_m = {1'b0, a_m} - {1'b0, b_m} -
                    {16'd0, ((eff_op == A_SBB) ? cin : 1'b0)};
wire [15:0] add_f = port_a + port_b + {15'd0, ((eff_op == A_ADC) ? cin : 1'b0)};
wire [15:0] sub_f = port_a - port_b - {15'd0, ((eff_op == A_SBB) ? cin : 1'b0)};
wire add_cy = ev_byte ? add_m[8] : add_m[16];
wire sub_cy = ev_byte ? sub_m[8] : sub_m[16];
wire add_ac = (a_m[4] ^ b_m[4] ^ add_m[4]);
wire sub_ac = (a_m[4] ^ b_m[4] ^ sub_m[4]);
wire add_ov = ev_byte ? ((~(a_m[7] ^ b_m[7])) & (a_m[7] ^ add_m[7]))
                      : ((~(a_m[15] ^ b_m[15])) & (a_m[15] ^ add_m[15]));
wire sub_ov = ev_byte ? ((a_m[7] ^ b_m[7]) & (a_m[7] ^ sub_m[7]))
                      : ((a_m[15] ^ b_m[15]) & (a_m[15] ^ sub_m[15]));
wire [15:0] inc_m = b_m + 16'd1;
wire [15:0] dec_m = b_m - 16'd1;
wire inc_ac = (b_m[4] ^ 1'b0 ^ inc_m[4]) | (b_m[3:0] == 4'hF);
wire dec_ac = (b_m[3:0] == 4'h0);
wire inc_ov = ev_byte ? (b_m[7:0] == 8'h7F) : (b_m == 16'h7FFF);
wire dec_ov = ev_byte ? (b_m[7:0] == 8'h80) : (b_m == 16'h8000);
wire [15:0] neg_m = 16'd0 - b_m;

always @* begin
    ar_full = 17'd0;
    ar_m    = 17'd0;
    ev_val  = 16'd0;
    ev_flags= 16'd0;
    ev_mask = 16'd0;
    ev_commits = 1'b1;
    case (eff_op)
        A_ADD, A_ADC: begin
            ar_m = add_m; ev_val = add_f; ev_mask = ARITH_MASK;
            ev_flags = szp({1'b0, add_m[15:0]}, ev_byte);
            ev_flags[FCY] = add_cy;
            ev_flags[FAC] = add_ac;
            ev_flags[FV]  = add_ov;
        end
        A_SUB, A_SBB, A_CMP: begin
            ar_m = sub_m; ev_val = sub_f; ev_mask = ARITH_MASK;
            ev_flags = szp({1'b0, sub_m[15:0]}, ev_byte);
            ev_flags[FCY] = sub_cy;
            ev_flags[FAC] = sub_ac;
            ev_flags[FV]  = sub_ov;
            if (eff_op == A_CMP) ev_commits = 1'b0;
        end
        A_AND: begin
            ev_val = port_a & port_b; ev_mask = ARITH_MASK;
            ev_flags = szp({1'b0, a_m & b_m}, ev_byte);
        end
        A_OR: begin
            ev_val = port_a | port_b; ev_mask = ARITH_MASK;
            ev_flags = szp({1'b0, a_m | b_m}, ev_byte);
        end
        A_XOR: begin
            ev_val = port_a ^ port_b; ev_mask = ARITH_MASK;
            ev_flags = szp({1'b0, a_m ^ b_m}, ev_byte);
        end
        A_INC: begin
            ev_val = port_b + 16'd1;
            ev_mask = (16'd1<<FP)|(16'd1<<FAC)|(16'd1<<FZ)|(16'd1<<FS)|(16'd1<<FV);
            ev_flags = szp({1'b0, inc_m}, ev_byte);
            ev_flags[FAC] = inc_ac;
            ev_flags[FV]  = inc_ov;
        end
        A_DEC: begin
            ev_val = port_b - 16'd1;
            ev_mask = (16'd1<<FP)|(16'd1<<FAC)|(16'd1<<FZ)|(16'd1<<FS)|(16'd1<<FV);
            ev_flags = szp({1'b0, dec_m}, ev_byte);
            ev_flags[FAC] = dec_ac;
            ev_flags[FV]  = dec_ov;
        end
        A_INC2: begin ev_val = port_b + 16'd2; ev_mask = 16'd0; end
        A_DEC2: begin ev_val = port_b - 16'd2; ev_mask = 16'd0; end
        A_NOT:  begin ev_val = ~port_b;        ev_mask = 16'd0; end
        A_NEG:  begin
            ev_val = 16'd0 - port_b; ev_mask = ARITH_MASK;
            ev_flags = szp({1'b0, neg_m}, ev_byte);
            ev_flags[FCY] = (ev_byte ? (neg_m[7:0] != 8'd0) : (neg_m != 16'd0));
            ev_flags[FAC] = (b_m[4] ^ neg_m[4]);
            ev_flags[FV]  = ev_byte ? (b_m[7:0] == 8'h80) : (b_m == 16'h8000);
        end
        A_ABS: begin
            // [-1E-]: magnitude.  At byte width only the low lane is replaced.
            if (ev_byte) begin
                ev_val = port_b[7] ? {port_b[15:8], (8'd0 - port_b[7:0])}
                                   : port_b;
            end else begin
                ev_val = port_b[15] ? (16'd0 - port_b) : port_b;
            end
            ev_mask = 16'd0;
        end
        A_ADJD, A_ADJA, A_BIT: begin ev_val = port_b; ev_mask = 16'd0; end
        default: begin   // PASS and everything unlisted
            ev_val = port_b; ev_mask = ARITH_MASK;
            ev_flags = szp({1'b0, b_m}, ev_byte);
        end
    endcase
    // F30 -- THE ADJUST UNIT WAS COMPUTED AND DISCARDED.  `alu_eval` takes the
    // ARMED path BEFORE the ordinary op table (`sim/alu.cpp:127`): an ADJD/ADJA
    // armed by the previous row REPLACES the ADD/SUB pass with one pass of the
    // decimal corrector.  The EU built `adj_lo`/`adj_hi`/`adj_corr`/`adj_sum`/
    // `adj_val8`/`adj_rhi`/`adj_flags` faithfully -- and then never selected
    // them, so all three outputs sat in this module's `_unused_eu` sink and
    // `27`/`2F`/`37`/`3F` came out as the raw `tmpb + ONES` (`27 idx 3`:
    // exp e369, got e368 -- the correction, exactly, missing).
    //
    // The corrector sits on the LOW LANE of port B only; the HIGH lane goes
    // through raw and the ONE carry chain runs the whole 16 bits, which is what
    // `adj_rhi` already computes.
    if ((al_adjust != 2'd0) && ((eff_op == A_ADD) || (eff_op == A_SUB))) begin
        ev_val   = {adj_rhi, adj_val8};
        ev_flags = adj_flags;
        ev_mask  = ARITH_MASK;
        ev_commits = 1'b1;
    end
end

// --- the ONE shared iterative stepper --------------------------------------
// MUL (shift-add), DIV (restoring), ROL12 and the shift/rotate family are the
// SAME unit stepped once per clock; no lpm_divide, no per-op datapath.
wire        it_byte = al_byte;
wire [15:0] it_a    = it_byte ? {8'd0, tmpb[7:0]} : tmpb;
wire [15:0] it_bop  = tmps[al_tmp];
wire [15:0] it_mask = it_byte ? 16'h00FF : 16'hFFFF;
wire        it_msb  = it_byte ? it_a[7] : it_a[15];
wire        it_lsb  = it_a[0];
reg  [15:0] it_val;      // the SIGMA the step presents
reg  [15:0] it_tmpa;     // the multiplier / quotient register after the step
reg  [15:0] it_flags;
reg  [15:0] it_fmask;
reg         it_writes_tmpa;

// MUL
wire [15:0] mul_mplier = it_byte ? {8'd0, tmpa[7:0]} : tmpa;
wire [15:0] mul_mcand  = it_byte ? {8'd0, it_bop[7:0]} : it_bop;
wire [16:0] mul_sum    = {1'b0, it_a} +
                         (mul_mplier[0] ? {1'b0, mul_mcand} : 17'd0);
wire        mul_cout   = it_byte ? mul_sum[8] : mul_sum[16];
wire [15:0] mul_hi     = it_byte
                       ? {8'd0, {mul_cout, mul_sum[7:1]}}
                       : {mul_cout, mul_sum[15:1]};
wire [15:0] mul_lo     = it_byte
                       ? {8'd0, {mul_sum[0], mul_mplier[7:1]}}
                       : {mul_sum[0], mul_mplier[15:1]};
// DIV (restoring)
wire [15:0] div_divisor = it_byte ? {8'd0, it_bop[7:0]} : it_bop;
wire [15:0] div_lo0     = it_byte ? {8'd0, tmpa[7:0]} : tmpa;
// F32 -- THE RESTORING DIVIDER'S COMPARE IS ONE BIT WIDER THAN ITS OPERANDS.
// `alu.cpp::kDiv` computes `hi = (a << 1) | (lo >> (w-1))` in a uint32_t and
// does NOT mask it before `if (hi >= divisor)`: the bit shifted OUT of the high
// half is exactly what decides the subtract, and it is the whole point of a
// restoring step.  This shifted `it_a[w-1]` away, so every dividend whose high
// half reached the top bit took the wrong branch.  MEASURED: `F6.6 idx 2`
// (0x9151 / 179) `exp ax=94cf got 5100`.
wire [16:0] div_hi0     = it_byte
                        ? {8'd0, it_a[7:0], div_lo0[7]}
                        : {it_a[15:0], div_lo0[15]};
wire [15:0] div_lo1     = (div_lo0 << 1) & it_mask;
wire        div_fits    = (div_hi0 >= {1'b0, div_divisor});
wire [15:0] div_hi1     = div_fits ? (div_hi0[15:0] - div_divisor)
                                   : div_hi0[15:0];
wire [15:0] div_lo2     = div_fits ? (div_lo1 | 16'd1) : div_lo1;
// shifts / rotates
reg  [15:0] sh_r;
reg         sh_cy;
always @* begin
    sh_cy = 1'b0;
    sh_r  = 16'd0;
    case (eff_op)
        A_ROL: begin sh_cy = it_msb; sh_r = ((it_a << 1) | {15'd0, it_msb}) & it_mask; end
        A_ROR: begin sh_cy = it_lsb; sh_r = ((it_a >> 1) | (it_lsb ? (it_byte ? 16'h0080 : 16'h8000) : 16'd0)) & it_mask; end
        A_RCL: begin sh_cy = it_msb; sh_r = ((it_a << 1) | {15'd0, cin}) & it_mask; end
        A_RCR: begin sh_cy = it_lsb; sh_r = ((it_a >> 1) | (cin ? (it_byte ? 16'h0080 : 16'h8000) : 16'd0)) & it_mask; end
        A_SHL, A_SHL6: begin sh_cy = it_msb; sh_r = (it_a << 1) & it_mask; end
        A_SHR: begin sh_cy = it_lsb; sh_r = (it_a >> 1) & it_mask; end
        A_SAR: begin sh_cy = it_lsb; sh_r = ((it_a >> 1) | (it_msb ? (it_byte ? 16'h0080 : 16'h8000) : 16'd0)) & it_mask; end
        default: ;
    endcase
end
wire sh_left = (eff_op == A_ROL) || (eff_op == A_RCL) ||
               (eff_op == A_SHL) || (eff_op == A_SHL6);
wire sh_rmsb = it_byte ? sh_r[7] : sh_r[15];
wire sh_rmsb1= it_byte ? sh_r[6] : sh_r[14];
wire sh_v    = sh_left ? (sh_rmsb != sh_cy) : (sh_rmsb != sh_rmsb1);
// THE SHIFTER IS TWO 8-BIT LANES FED BY ONE FEEDBACK BIT (alu.cpp).
wire sh_wrap = (eff_op == A_ROR) || (eff_op == A_RCR);
wire sh_fb   = sh_wrap ? sh_r[7] : 1'b0;
wire [7:0] sh_hi = sh_left ? {tmpb[14:8], tmpb[7]}
                           : {sh_fb, tmpb[15:9]};

always @* begin
    it_val   = 16'd0;
    it_tmpa  = tmpa;
    it_flags = 16'd0;
    it_fmask = 16'd0;
    it_writes_tmpa = 1'b0;
    case (eff_op)
        A_MUL: begin
            it_val  = it_byte ? {tmpb[15:8], mul_hi[7:0]} : mul_hi;
            it_tmpa = it_byte ? {tmpa[15:8], mul_lo[7:0]} : mul_lo;
            it_writes_tmpa = 1'b1;
        end
        A_DIV: begin
            it_val  = it_byte ? {tmpb[15:8], div_hi1[7:0]} : div_hi1;
            it_tmpa = it_byte ? {tmpa[15:8], div_lo2[7:0]} : div_lo2;
            it_writes_tmpa = 1'b1;
        end
        A_ROL12: begin
            it_val = {tmpb[14:0], tmpb[11]};
        end
        default: begin
            it_flags[FCY] = sh_cy;
            it_flags[FV]  = sh_v;
            if ((eff_op == A_SHL) || (eff_op == A_SHR) ||
                (eff_op == A_SAR) || (eff_op == A_SHL6)) begin
                it_flags = it_flags | szp({1'b0, sh_r}, it_byte);
                it_fmask = ARITH_MASK;
            end else begin
                it_fmask = (16'd1 << FCY) | (16'd1 << FV);
            end
            it_val = it_byte ? {sh_hi, sh_r[7:0]} : sh_r;
        end
    endcase
end

// SIGMA: what the row's Source1 [-14-] presents.
wire [15:0] sigma = al_eaconst ? al_eaval
                  : is_iter ? (al_spent ? tmpb : it_val)
                            : ev_val;
wire [15:0] sig_flags = is_iter ? it_flags : ev_flags;
wire [15:0] sig_mask  = al_eaconst ? 16'd0
                      : is_iter ? (al_spent ? 16'd0 : it_fmask)
                                : ev_mask;
wire        sig_commits = al_eaconst ? 1'b1 : (is_iter ? 1'b1 : ev_commits);

//============================================================================
// SOURCE / DESTINATION MUXES
//============================================================================
wire [15:0] flags_rd = (psw & PSW_WRITABLE) | PSW_FORCED;

// state.h::rb16 -- the byte-register read is 16 bits wide
function automatic [15:0] rb16(input [2:0] code, input [15:0] pair);
    rb16 = code[2] ? {pair[7:0], pair[15:8]} : pair;
endfunction

// A byte-coded FE/FF-group register feeding the E-tagged stack-write row
// supplies its unswapped parent word on silicon.  This is distinct from an
// ordinary byte-register read: the row is M -> OPR while issuing MEMW SS.
// The current group microcode page excludes immediate and fixed-register PUSH
// rows, whose unused M decode metadata can otherwise have the same values.
wire m_modrm_stack_word = (upc_page == 3'd3) &&
                          (m_kind == OK_REG) && m_byte && e_e &&
                          row_is_wr && (row_seg == 3'd2) &&
                          e_have1 && (e_s1 == 5'd19) && (e_d1 == 5'd6);
// The FE group's undocumented register-direct jump rows have the same
// physical exception.  Although the ModR/M metadata is byte-coded, the row
// is `M -> PC; FLUSH` and silicon redirects to the whole parent register
// (FE E6 with DW=7940 goes to 7940, not the rb16 view 4079).
wire m_modrm_pc_word = (upc_page == 3'd3) &&
                       (m_kind == OK_REG) && m_byte &&
                       e_have1 && (e_s1 == 5'd19) && (e_d1 == 5'd4) &&
                       (e_ictl == I_FLUSH);
wire m_modrm_parent_word = m_modrm_stack_word || m_modrm_pc_word;
wire [15:0] m_reg_rd = !m_byte              ? gpr[m_idx]
                       : m_modrm_parent_word ? gpr[m_idx[1:0]]
                                             : rb16(m_idx, gpr[m_idx[1:0]]);
wire [15:0] m_rd = (m_kind == OK_REG)  ? m_reg_rd
                 : (m_kind == OK_SREG) ? sreg[m_idx[1:0]]
                 : (m_kind == OK_MEM)  ? opr : 16'd0;
wire [15:0] r_rd = (r_kind == OK_REG)  ? (r_byte ? rb16(r_idx, gpr[r_idx[1:0]])
                                                 : gpr[r_idx])
                 : (r_kind == OK_SREG) ? sreg[r_idx[1:0]]
                 : (r_kind == OK_MEM)  ? opr : 16'd0;

wire [15:0] dirsz = (op8_eff ? (psw[FDIR] ? 16'hFFFF : 16'h0001)
                             : (psw[FDIR] ? 16'hFFFE : 16'h0002));

reg  [15:0] s1_val;
reg         s1_byte;
always @* begin
    s1_byte = 1'b0;
    case (e_s1)
        5'd0,5'd1,5'd2,5'd3: s1_val = sreg[e_s1[1:0]];
        5'd4:  s1_val = pc;
        5'd6:  s1_val = opr;
        5'd7:  begin s1_val = {8'd0, q_byte}; s1_byte = 1'b1; end
        5'd8:  s1_val = dirsz;
        5'd9:  s1_val = 16'd0;
        5'd10: s1_val = {8'd0, pfxcnt_eff};        // F22
        5'd12: s1_val = tmpa;
        5'd13: s1_val = tmpb;
        5'd14: s1_val = tmpc;
        5'd15: s1_val = flags_rd;
        5'd16: s1_val = {gpr[R_AW][7:0], gpr[R_AW][15:8]};
        5'd17: s1_val = count;
        5'd18: s1_val = r_rd;
        5'd19: s1_val = m_rd;
        5'd20: s1_val = sigma;
        5'd21: s1_val = 16'hFFFF;
        5'd22: s1_val = {8'd0, opc_reg & 8'h38};
        5'd23: begin s1_val = {10'd0, r_constval}; s1_byte = 1'b1; end
        default: s1_val = (e_s1 >= 5'd24) ? gpr[e_s1[2:0]] : 16'd0;
    endcase
end

reg [15:0] s2_val;
always @* begin
    case (e_s2)
        4'd0: s2_val = 16'hFFFF;
        4'd4: s2_val = sigma;
        4'd5: s2_val = {8'd0, q_byte};
        4'd6: s2_val = 16'd0;
        4'd7: s2_val = r_rd;
        default: s2_val = (e_s2 >= 4'd8) ? gpr[e_s2[2:0]] : 16'd0;
    endcase
end

//============================================================================
// THE ROW'S OWN TRANSFERS  (family A, U2 pass 3)
//============================================================================
// exec_impl.h::run_micro does the two parallel transfers FIRST and issues the
// row's bus cycle AFTERWARDS, off `m_.ind`.  So the address a row posts is the
// IND THE ROW ITSELF JUST WROTE whenever the row writes IND -- which is how
// every non-ModR/M address in the ROM is formed (`50`: `SIGMA -> SP  SIGMA ->
// IND  E  CTL MEMW SS`; the ModR/M forms get IND from the loader, which is why
// they were already green and this was invisible for two passes).
//
// The EU already had exactly this rule for the WRITE DATA (`opr_now`, "the
// value this row is about to put there").  It is the same rule; it was applied
// to one destination only.  `s1_now` / `s2_now` are the two transfer values,
// and dest2 is written after dest1, so dest2 wins if both name IND.
// ...and the ACT decode needs the same live OPR (F21).  It is combinational,
// so it does not get the step's blocking write for free: it reconstructs the
// `F` delivery the row is about to make.  ONE expression, both sides (F11).
// ...and when the completion is the LOOKAHEAD itself (`nr_have`'s
// `eu_rd_done_n`) the word is not in the store yet -- block (a) puts it there
// in the same edge the step then pops it, so the act decode must read
// `eu_rdata_n` directly.  `C3`'s `00F3 OPR -> PC  F E  CTL FLUSH` redirects
// off exactly that word, and read the STALE PC without this.
wire [15:0] opr_live = (e_f && (rdq_n != 2'd0)) ? rdq0
                     : (e_f && eu_rd_done_n)    ? eu_rdata_n
                     : opr;
wire [15:0] s1_now = (e_s1 == 5'd7) ? {8'd0, q_byte}
                   : (e_s1 == 5'd6) ? opr_live
                   : s1_val;
wire [15:0] s2_now = (e_s2 == 4'd5) ? {8'd0, q_byte} : s2_val;
// The CMP suppression: a SIGMA source that does not commit drives nothing.
wire wr_ind1 = e_have1 && (e_d1 == 5'd5) &&
               !((e_s1 == 5'd20) && !sig_commits);
wire wr_ind2 = e_have2 && (e_d2 == 2'd2) &&
               !((e_s2 == 4'd4) && !sig_commits);
wire [15:0] ind_now = wr_ind2 ? s2_now : wr_ind1 ? s1_now : ind;

// F14, the same rule for the REDIRECT.  `exec_impl.h`'s CTL block calls
// `biu_.flush(m_.sreg[kCS], m_.pc)` AFTER the transfers, so a row that writes
// PC (or CS) and flushes on the same row redirects to the value IT JUST WROTE
// -- which is every taken jump in the ROM (`EB`: `015B  SIGMA -> PC  E  CTL
// FLUSH`).  Reading the register gave the PRE-jump PC, so the refill fetched
// from `target - disp`.  The queue pops a row makes also advance PC, and they
// happen before the transfer, exactly as in `rd_src1`.
wire [15:0] pc_after_q = pc + {15'd0, row_q1} + {15'd0, row_q2};
wire wr_pc1 = e_have1 && (e_d1 == 5'd4) &&
              !((e_s1 == 5'd20) && !sig_commits);
wire wr_cs1 = e_have1 &&
              ((e_d1 == {3'd0, SR_CS}) ||
               ((e_d1 == 5'd18) && (r_kind == OK_SREG) &&
                (r_idx[1:0] == SR_CS)) ||
               ((e_d1 == 5'd19) && (m_kind == OK_SREG) &&
                (m_idx[1:0] == SR_CS))) &&
              !((e_s1 == 5'd20) && !sig_commits);
wire [15:0] pc_now = wr_pc1 ? s1_now : pc_after_q;
wire [15:0] cs_now = wr_cs1 ? s1_now : sreg[SR_CS];

wire        ghost_uses_ea = (ea_residue != tmpa);
wire [13:0] ghost_prev_pla = pla3_native(pe_opc_reg);
// Immediate IMUL is the one native PLA class (0104, shared by 69/6B) that
// leaves a second result word on OPR after retiring the low product.  Its two
// result rails contend for a non-negative immediate; the negative-immediate
// rail leaves TMPA intact (mc2/1379 versus the mc2/1292 control).
wire        ghost_uses_mul_hi = (ghost_prev_pla == 14'h0104) && !tmpc[15];
// The retained EA rail is a word-address rail.  A scratch/SIGMA residue keeps
// its low bit; a retained ModR/M address normally does not.  MOV-to-segment
// retains the EA rail itself, including its measured low bit (t30-raw/13).
wire [15:0] ghost_ea_off = (pe_opc_reg == 8'h8e) ? ea_residue
                                                  : {ea_residue[15:1], 1'b0};
wire [15:0] ghost_off = ghost_uses_ea ? ghost_ea_off : tmpa;
wire [13:0] ghost_next_pla = pla3_native(q_byte);
wire ghost_next_byte = q_ripe &&
                       (pla3_byte_only(ghost_next_pla) ||
                        (pla3_w_from_bit0(ghost_next_pla) && !q_byte[0]));
// At a completely idle fetch hand-off, the two byte-slice boundaries have
// different owners.  The predecessor's width leaves the untouched high lane
// charged; the successor's width leaves the low byte boundary charged.
// Current-socket value-only mutations expose the two independent terms:
// predecessor byte C000 else 8000, successor byte 0080 else 0000.
wire [15:0] ghost_relax = eu_ghost_full ? 16'hFFFF
                         : eu_ghost_idle ? ((pe_op8 ? 16'hC000 : 16'h8000) |
                                            (ghost_next_byte ? 16'h0080 : 16'h0000))
                         : 16'h0000;
wire [15:0] ghost_bus_off = ghost_uses_mul_hi ? (tmpa & opr)
                            : (eu_ghost_idle && !q_ripe) ? gpr[R_SP]
                            : (ghost_off & (gpr[R_SP] | ghost_relax));
wire [15:0] acc_off  = ghost_read_stale_alu ? ghost_bus_off
                       : row_is_wb          ? wb_ea : ind_now;
wire [19:0] acc_phys_base = acc_io ? {4'd0, acc_off}
                                   : ({acc_segv, 4'd0} + {4'd0, acc_off});
wire [19:0] ghost_stack_phys = {acc_segv, 4'd0} + {4'd0, ind_now};
// An odd stack POP has already launched its first byte from SS:SP when the
// stale address reaches the second half.  Even-stack ghosts launch directly
// from the stale value.  The real stack address, not the stale pin value,
// decides whether the word is split.
wire [19:0] acc_phys = (ghost_read_stale_alu && ghost_stack_phys[0] &&
                        eu_ghost_stack_first)
                     ? ghost_stack_phys : acc_phys_base;
// On an idle odd-stack hand-off the first byte sees the contended stale
// address, while the second byte sees the un-contended scratch rail.  This is
// the same split already selected from the real stack address above; no extra
// history is needed.
wire [19:0] acc_phys2= (ghost_read_stale_alu && eu_ghost_idle &&
                        !ghost_uses_ea && ghost_stack_phys[0])
                      ? (({acc_segv, 4'd0} + {4'd0, ghost_off}) + 20'd1)
                      : ghost_read_stale_alu ? (acc_phys_base + 20'd1)
                     : acc_io ? {4'd0, acc_off + 16'd1}
                              : ({acc_segv, 4'd0} + {4'd0, acc_off + 16'd1});
wire       acc_split = !acc_byte &&
                       (ghost_read_stale_alu
                        ? ((ghost_uses_ea || ghost_uses_mul_hi)
                           ? acc_phys_base[0] : ghost_stack_phys[0])
                                             : acc_phys[0]);

//============================================================================
// COMBINATIONAL OUTPUTS -- what the BIU samples during THIS clock
//============================================================================
// The F interlock has to be clear before any of the row's acts happen.
wire row_blocked = (st == S_ROW) && e_f && f_wait;

//----------------------------------------------------------------------------
// SM3 SITTING 12 (R7') -- THE READ'S DATA EDGE, AND WHY IT IS A `D`-PIN MUX
// AND NOT A CHAIN INPUT.
//----------------------------------------------------------------------------
// `eu_rd_edge` is the ONLY thing in this module that carries the LIVE `READY`
// pin.  (By elimination, `sm3_s12_prereg_2026-08-04.md` §2: READY reaches
// `v30u_biu`'s next-state in three places; `ready_prev` goes straight to a
// flop; the `ts` advance reaches the EU only through `eu_slot_busy_n`, which
// `S_PRERD` consumes inside an arm that sets `stop` on every branch, so its
// cone ends at `row_posted_n`/`rd_pending_n`; everything else the EU reads --
// `q_ripe_lead_n`, `eu_rd_done_n`, `eu_wr_done_n`, `eu_rdata_n`,
// `eu_rd_edge_d` -- is REGISTER-ONLY.)
//
// It used to be applied inside block (a), i.e. it SEEDED `psw_n` at the head of
// the twelve-position chain, so one AND gate off the READY pin selected the
// input of the deepest combinational structure in the core.  MEASURED:
// `system_large|c_ready_q` -> `v30u_eu|opc_base[3]` at **62-63 logic levels,
// 51.2 ns against 31.25**, on 20,000 of 20,000 failing paths, Fmax 19.42 MHz
// (`docs/notes/sm3_s12_r7p/CTRL_cone.txt`).  The identical RTL closed at
// 45.67 MHz one sitting earlier: the fitter's physical synthesis happens to
// break that cone or happens not to (§70.5).  Same cone, same fix, as §52.3's
// `srst`.  With the load moved here the control is **42.37 MHz, 0 failing
// paths, worst `c_ready_q` path 19 levels**.
//
// THE RULE ITSELF IS UNCHANGED and it is `interrupt_model.md`'s:
//   "POP PSW consumes the popped image at its read's data edge -- the new IE
//    shows in the PS bits during the read's own T4."
// Only WHERE it is applied moves.
//
// `row_blocked` IS PART OF THE TAKE, AND IT IS THERE BECAUSE THE FORM WITHOUT
// IT WAS BUILT AND REFUTED (`sm3_s12b_prereg_2026-08-04.md` §1, seed
// `mc2/2788`).  Without it the two forms differ on exactly one shape: a SECOND
// read outstanding while an EARLIER one already sits in the completed-read
// store (`rd_pending=2`, `rd_done_cnt=1`, `rdq_n=1`), so `nr_have` holds,
// `nr_wait` is 0, the row is NOT blocked -- and the `OPR -> FLAGS` row RUNS on
// the same clock this data edge fires, writing `opr_live` (the EARLIER word)
// where the data edge wants `eu_rd_edge_d` (the CURRENT one).  Different
// values; the order decides.  The term is REGISTER-ONLY (`st`, `e_f`,
// `f_wait`), so it costs the cone nothing.
//
// WHY THE MOVE IS EXACT, in two cases that partition the clock:
//  * `row_blocked` HOLDS -- `S_ROW`'s `chain == 0` arm sets `stop` and assigns
//    nothing and positions 1-11 are skipped, so `psw_n` reaches the commit
//    equal to its preload `psw`, PROVIDED `poste`/`iend_owed` are not owed
//    (they run between the old write site and the chain and read-modify-write
//    `psw_n`).  That proviso is FALSIFIER (A) in the clocked observer.
//  * `row_blocked` DOES NOT HOLD -- the row is a pure register transfer
//    (`row_bus` false, `e_s1 == 6` not 7), so `row_acts_ok` holds, the step
//    performs `dest1 = FLAGS` and OVERWRITES `psw_n` entirely with
//    `s1_now = opr_live`.  Whatever block (a) put there was DEAD, so deleting
//    it changes nothing.  That is FALSIFIER (B).
//
// Both falsifiers are on the RAW take -- without `row_blocked` -- so they see
// every clock the old form would have written on, and they STAY in the tree.
wire        rd_edge_take_raw  = eu_rd_edge && (st == S_ROW) && e_f &&
                                (e_s1 == 5'd6) && (e_d1 == 5'd15);
wire        rd_edge_psw_take  = rd_edge_take_raw && row_blocked;
wire [15:0] rd_edge_psw       = (eu_rd_edge_d & PSW_WRITABLE) | PSW_FORCED;

// The row's own queue demand
wire q_demand_row = row_need_q && !row_blocked;

// The interrupt entry's first IVT read is reserved while row 7.10.0 is
// standing, two micro-rows before 7.10.2 accounts for the read.  This is the
// address-preparation half of the same fixed vector path: NMI and single-step
// supply their fixed vector numbers, while software and maskable interrupts
// leave the acquired vector in OPR.  Posting here lets ordinary BIU
// arbitration decide whether the status follows an idle clock, a direct
// redirect, or the final no-wait landing phase; no vector-specific delay or
// bus-state table is needed.
wire vector_fixed = irq_sel_nmi || irq_sel_brk;
// A waited landing overlaps only when NMI was taken at the cold opcode-pop
// boundary.  `irq_fast_inta` already records that boundary for the hardware-
// acknowledge collision; retaining it through the fixed-vector prologue also
// distinguishes this one waited overlap from ordinary row-boundary entries.
// No-wait landings remain the common NMI/BRK path.
wire vector_tail = eu_fetch_tail && vector_fixed &&
                   (!q_ripe || (irq_sel_nmi && irq_fast_inta));
wire vector_overlap = (st == S_ROW) &&
                      ((eu_direct_fetch && !q_ripe) || vector_tail);
wire vector_early = !row_blocked && vector_overlap &&
                    (upc_page == 3'd7) && (upc_opc == 8'h10) &&
                    (upc_loc == 4'd0);
wire vector_first = (st == S_ROW) &&
                    (upc_page == 3'd7) && (upc_opc == 8'h10) &&
                    (upc_loc == 4'd2);
wire [15:0] vector_number = irq_sel_nmi ? 16'd2
                          : irq_sel_brk ? 16'd1 : opr;
wire [19:0] vector_phys_early = {2'b0, vector_number, 2'b0};
// When the reservation is still short of T1, the BIU's ordinary slot latch is
// its provenance at 7.10.2.  All non-overlapped entries arrive here with the
// slot free and post in the normal way.
wire vector_reserved = vector_first && (eu_slot_busy || eu_access_active);

// The E row's successor pop (max-of-two-deadlines: the E row's own clock and
// the retire deadline).  A staged write defers it to the sequence tail.
//
// F11: THE DEMAND AND THE TAKE ARE ONE EVENT.  This wire is what puts the pop
// on the bus; v30u_eu_row.svh's cadence block is what consumes the byte.  They
// must be true on exactly the same clocks, so every term the step applies is a
// term here -- the row must RUN this clock (all four stalls), and the retire
// deadline must count THE WRITE THIS ROW IS ABOUT TO POST, which the step sees
// (it reads `wr_out` after its own increment) and the pre-edge wire did not.
// 7.10.2 owns the read reserved by 7.10.0, so an occupied slot there is its
// own access, not a reason to post or wait for a second copy.
wire       row_slot_wait = row_bus && !row_posted &&
                           eu_slot_busy &&
                           !vector_reserved;
wire [2:0] row_wr_add    = (row_is_wr || row_is_wb)
                           ? (acc_split ? 3'd2 : 3'd1) : 3'd0;
wire [2:0] wr_after      = {1'b0, wr_out} + row_wr_add;
wire       retire_ok_e   = (wr_after == 3'd0) ||
                           ((wr_after == 3'd1) && eu_wr_done_n);
// ...and the E row's OWN pairing latch.  `exec_impl.h`'s
// `if (pend_.active && opr_fresh_) emit_pending();` runs IMMEDIATELY BEFORE the
// cadence, so the cadence sees the latch CLEARED -- reading the pre-edge
// `pend_active` here would take a byte the bus was never asked for.  Same two
// terms as `eu_pair` below, which is the act that does the clearing.
wire pend_new   = pend_active || (row_is_wr || row_is_wb);
wire pend_after = pend_new && !(opr_fresh || row_wr_opr);
// F24 -- A ROW THAT FLUSHES CANNOT POP.  `exec_impl.h`'s CTL block calls
// `biu_.flush()` BEFORE the cadence block reaches `opcode_prefetch`, so on a
// redirect row the queue the pop would take from has already been emptied and
// the pop waits for the refill.  The EU had the demand and the flush as two
// independent acts of the same clock and took the byte the flush was about to
// discard: `EB idx 0` popped its successor at row 5, where the golden shows
// PASV and the flush's own `E` on row 6.  The whole taken-JMP family (Jcc,
// E9/E8/EB/EA, FF.2/.4, RET, INT) is this one term.
wire row_flush = (e_type == TY_CTL) && !e_farjmp && (e_ictl == I_FLUSH);
wire row_epop = (st == S_ROW) && (e_e || ext4s_early_e) &&
                !pend_after && !opc_valid &&
                !row_blocked && (rowq >= row_qn) && !row_pre_wait &&
                !row_slot_wait && retire_ok_e && !row_flush && !bnd_fire;

//----------------------------------------------------------------------------
// THE BOUNDARY, AND WHY IT IS NOT THE POP
//----------------------------------------------------------------------------
// `BiuTimed::boundary_no_pop()` is `if (opc_valid_) return clk_; wait_bus();`
// -- the RETIRE deadline and NOTHING ELSE.  The recognition decision does not
// need the byte (it is the decision NOT to take one), so a recognised boundary
// does not slide when the queue is dry, and it is NOT cancelled by a flush
// (F24 is a fact about a POP; there is no pop here).  MEASURED in the SPEC:
// `INT.90` 200/200 with the retire deadline, 177 with the pop deadline, and
// every one of the 23 failures an odd-address dry-queue case.
//
// So `bnd_row` is `row_epop` with the two POP-ONLY terms removed (`q_ripe`,
// which lives in the step, and `!row_flush`).
//
// ...AND THE BOUNDARY IS A WINDOW, NOT A CLOCK.  `boundary_no_pop()` returns
// the retire deadline, but the model then takes its decision at
// `max(B, A + pipe)` -- and C5 says that `max` cannot be hardware.  What CAN
// be, and is what the frozen FSM does, is the plain reading of
// `pop_want = (state == S_FIRST && !irq_take)`: the part SITS at the pop point
// from the retire deadline until the byte arrives, and `irq_take` is a LEVEL
// sampled on every clock of that wait.  The two agree on every replayed case
// because the replay had already chosen a boundary that fires -- but only the
// window is causal, and it is what the goldens show.
// MEASURED: `INT.90 idx 14`, retire met on row 3 with a dry queue and the pin
// maturing on row 4; the chip's row-4 pop is SUPPRESSED.  A one-clock boundary
// declines it and pops.
wire bnd_row  = (st == S_ROW) && (e_e || ext4s_early_e) &&
                !pend_after && !opc_valid &&
                !row_blocked && (rowq >= row_qn) && !row_pre_wait &&
                !row_slot_wait && retire_ok_e;
// ...and `tailw_go` is a pop point too, so it is a boundary point too: the
// model's tail is `{deliver; emit;} ... if (at_fire_boundary()) ...
// opcode_prefetch()`, one check covering both.  MEASURED, `INT.F3AA idx 2`:
// `irq_take` was already true on the `E` row (which deferred, a store being
// staged) and the tail popped one clock later with no boundary evaluated.
wire bnd_epop = ((st == S_EPOP) || (st == S_TAIL_POP) || tailw_go) &&
                retire_ok_n;
// ...and the boundary of a PRE-DECODE-EXECUTED form, which has no `E` row at
// all: `step()` checks it after `loader_decode` returns (S9b), i.e. on the
// clock the successor's cold pop would ride.  `bnd_armed` is what separates
// that `S_OPC_POP` from the one a PREFIX hands over -- the model's prefix loop
// is INSIDE `loader_decode`, so there is no boundary between a prefix and its
// instruction, which is the measured "no sample between 26 and 8B".
wire bnd_opc  = (st == S_OPC_POP) && bnd_armed;
wire at_bnd   = bnd_row || bnd_epop || bnd_opc;

// THE SHADOW IS A DECODE-TIME CLASS, NOT A WRITE.  "Recognition-deferring
// instructions: every segment-register load -- measured on `MOV SS,AW` AND
// `MOV DS0,AW`" ... "8C sreg-STORE shadows recognition too".  It was tempting
// to derive it from the sreg WRITE itself, and that is REFUTED by the row
// order: `8E`'s write is on the POST-`E` row (`0.8e.1`), which the model runs
// AFTER the cadence -- so at the boundary no write has happened yet, and the
// golden still skips the sample (`INT.8ED0 idx 16` row 4: the chip pops, the
// write-derived rendering vectored).  The class is what the PLA already says
// (`pla3_sreg_mov`, the same bit that makes the ModR/M `reg` field an SREG),
// and it is set where the loader latches it.
// REGISTERED RESIDUE: the far-CALL / far-JMP `CS` write is documented to
// shadow too and is NOT in this class; no golden combines the two.
// SM3 SITTING 11 -- AND H1's SEPARATE FLOOR IS GONE.  It used to sit in the
// BIU (`bnd_hold`, five flops, armed by an INTA) and hold this boundary open
// for two clocks.  The gate above now demands IE up NOW **and** up three
// clocks ago, so a recognition cannot act until a rising IE has crossed the
// floor -- and `bnd_hold` was MEASURED INERT against the whole 3,242-seed
// bank (REGISTERED 1,490 / EVT 910 / COMBINED 2,400, to the seed, with it
// forced to zero).  It is DELETED rather than left standing.
//
// WHAT THE BIU STILL NEEDS FROM HERE is the prefetcher SUSPEND: the
// recognition that PAYS the floor grants the slot between the floor and its
// own request to NOTHING (the census's two idle clocks).  `eu_bnd_post`
// carries that condition now: `!ie_p[3]` is exactly "the IE gate is what held
// this boundary", since the take needs `ie_p[2]` and IE was still low four
// clocks back.  This includes an INT remembered on IE's rise; a REP withdrawal
// remains on C_INTR's separate route while `rep_kind != REP_NONE`.  MEASURED:
// forcing the suspend unconditional costs EVT 910 -> 897.
//
// AND `!irq_nmi_lvl`, for the SAME reason the floor itself is maskable-only: a
// non-maskable recognition is not IE-gated, never paid the floor, and must not
// carry the suspend that goes with it.  MEASURED: without this term the entry
// after an NMI suspends a fetch the chip runs, and `NMI.B8` falls 200 -> 188
// (12 cases, all `row 6 busstat: exp CODE got PASV`).
wire irq_take = irq_any && !irq_shadow;
// §86 -- AND THE TRAP RIDES THE SAME BOUNDARY.  `at_fire_boundary()` in the
// model is `ext_fire() || brk_take_`: ONE recognition path with one more term
// on it, not a second path.  The floor is `ie_p`'s sentence one bit over --
// "TF up NOW **and** up three clocks ago" -- and the take is the ARM, which
// was sampled at the PREVIOUS boundary (see `brk_smp` below).
wire brk_seen = psw[FBRK] && brk_p[BRK_FLOOR-1];
// W3.1 -- **...AND THE TRAP RIDES THE SHADOW TOO.**  §86.A booked its absence
// as a divergence with a falsifier written down (*"the trap is not shadowed
// behind a segment-register load ... silicon is documented to, this tree has
// no cell"*).  The cell is the `wr1` corpus and the falsifier FIRED.
//
// MEASURED, chip-side and engine-free (`sw/w31_shadow.py`), over 380 captures
// / 3,411 chip vector-1 entries / 1,363 consecutive trap pairs, where `grace`
// is instruction boundaries the part ran PAST between two traps and §84/§85
// measured the storm cadence at 0 on 1,742 + 90 pairs:
//
//     MOV sreg  8C 8E    g>=1 on 69, g0 on     0      the class already here
//     POP sreg  07 17    g>=1 on  6, g0 on     0      NEW -- no PLA column
//     PUSH sreg 06 16    g>=1 on  0, g0 on     5      NOT the class
//     LES/LDS   C4 C5    g>=1 on  0, g0 on    11      NOT the class
//     all other opcodes  g>=1 on  0, g0 on 1,277      195 distinct opcodes
//
// Not one exception in either direction.  The single grace-2 pair is two
// consecutive `8E`s -- the shadow COMPOSING, which this shape gives for free
// because the arm is untouched and only the TAKE is gated.
//
// THE CLASS IS TWO MICROCODE ENTRIES, which is why `8C` -- a segment-register
// READ -- is in it and `PUSH` sreg is not:
//     00?.100011?0.00   8C 8E          `R -> M`     the MOV-sreg entry
//     00?.000??111.00   07 0F 17 1F    `OPR -> R`   the POP-sreg entry
//     00?.000??110.00   06 0E 16 1E    `R -> OPR`   PUSH sreg -- a DIFFERENT
//                                                   entry, and it does not
//                                                   shadow
// *Falsifier*: any capture in which `PUSH` sreg or `LES`/`LDS` shows a grace
// >= 1, or a member of the two entries shows a grace of 0.
wire brk_take = brk_arm && !irq_shadow;
wire bnd_take = irq_take || brk_take;
wire bnd_fire = at_bnd && bnd_take;
// ...and the boundary that fires is the one that CLEARS the arm and suspends
// the prefetcher.  `at_bnd` implies `!opc_valid` on all three of its arms,
// which is `boundary_no_pop()`'s own early return ("already latched: this IS
// the pop clock") -- a pre-popped successor never reaches the floor at all.
assign eu_bnd_take = bnd_fire;
// §86 -- ...AND THE SUSPEND BELONGS TO THE RECOGNITION THAT PAID THE IE
// FLOOR, WHICH THE TRAP NEVER DOES.  `irq_take` is the added term.  It is
// INERT on every path that existed before the trap, because this wire is only
// ever consumed as `eu_bnd_take && eu_bnd_post` and the old `eu_bnd_take`
// (`at_bnd && irq_take`) already implied it.  The trap is not IE-gated, for
// the same reason a non-maskable recognition is not -- and `!irq_nmi_lvl` is
// already here saying exactly that about NMI.  In the model the suspend is
// inside `if (post_redirect && live)` with `live = maskable() && ...`, and
// `maskable()` is `ev_pin_ == 0`: a run with no pin directive never suspends,
// which is every seed a BRK trap fires in.
assign eu_bnd_post = irq_take && !ie_p[3] && !irq_nmi_lvl;

// §35.4 -- ...AND F11's RULE APPLIES TO THE TAIL'S OWN FALL-THROUGH.  With
// `S_TAIL_W` made zero-cost the tail's POP now happens inside the delivery's
// edge, so the act decode has to reconstruct the hand-over the step is about
// to make -- otherwise the EU eats a byte the BIU was never asked for (the
// first attempt did exactly that: the three string forms' ARCH fell from
// 500 to their cycle counts while nothing timed moved).
wire tailw_go = (st == S_TAIL_W) && !opc_valid &&
                (opr_fresh || !(nr_wait || !opr_free_now));

wire q_demand = ((st == S_OPC_POP) && !bnd_fire) ||
                (st == S_EXT_POP) || ((st == S_MODRM) && !opc_rm_valid) ||
                (st == S_D16_LO) ||
                ((st == S_D8_B)   && (!ld_ripe_prev ? (chg == 2'd1) : 1'b1)) ||
                ((st == S_D16_HI) && (!ld_ripe_prev ? (chg == 2'd1) : 1'b1)) ||
                q_demand_row || row_epop || ghost_preread_epop || ghost_rm_pop ||
                // F11 again: both deferred-pop states TAKE the byte only past
                // the retire deadline, so neither may DEMAND it before.
                (((st == S_EPOP) || (st == S_TAIL_POP) || tailw_go) &&
                 retire_ok_n && !bnd_fire);

assign q_pop   = q_demand;
assign q_first = (st == S_OPC_POP) ? pop_is_first
               : (st == S_EPOP) || (st == S_TAIL_POP) || tailw_go ||
                 row_epop || ghost_preread_epop ? 1'b1
               : 1'b0;

// --- the bus request -------------------------------------------------------
// exec_impl.h::bus_read / bus_write: a staged write must run before the next
// cycle, so the row first pairs it (`emit_pending`) and then posts its own.
// F20 -- THE PRE-PAIR FLUSH IS A `deliver_read()`, NOT JUST A WAIT.  The
// model's guard is `if (pend_.active) { if (!opr_fresh_) deliver_read();
// emit_pending(); }`, and `deliver_read()` is `wait_read()` PLUS a POP of the
// completed-read store into OPR.  The EU had only the wait, and only its
// `wait_opr_free` half: nothing popped, nothing paired, `pend_active` never
// cleared.  `REP MOVS` is where it shows -- every iteration's store is paired
// by the NEXT iteration's read row -- and it is what tripped C3's assertion at
// CX==3 (two completed reads in the store because none were ever taken out).
wire pend_go = pend_active && opr_fresh;
wire row_pre_pair = row_bus && pend_active;
wire row_pre_wait = row_pre_pair && !opr_fresh && (nr_wait || !opr_free_now);

wire row_acts_ok = (st == S_ROW) && !row_blocked && (rowq >= row_qn) &&
                   !row_pre_wait;

wire pr_active = (st == S_PRERD) && !row_posted;
wire row_post_now = row_acts_ok && row_bus && !row_posted &&
                    !vector_reserved;
// First hardware acknowledge contested-slot law: ordinary microcode
// retirement reserves a direct CODE tail until the next idle clock.  A cold
// opcode-pop boundary, or recognition that already paid the IE floor, keeps
// the immediate replacement.  This separates the four slow collisions from
// the mc2/2062 immediate control without a bus-state table.
wire inta_first = row_is_inta && (upc_page == 3'd7) &&
                  (upc_opc == 8'h02) && (upc_loc == 4'd0);
// A HALT wake reaches the interrupt dispatcher while the wake prefetch may
// still be only an announcement.  Publish that one-clock provenance before
// 7.02.0 posts INTA so the BIU can withdraw the announced fetch instead of
// opening its T1.  The latch is set only by S_HALTED and spent by S_IRQ_D.
assign eu_post = (vector_early || pr_active || row_post_now) && !eu_slot_busy;
assign eu_ghost_preview = eu_post && ghost_rd_ready && ghost_rd_feed &&
                          !ghost_rd_discard;
assign eu_post_hold = (vector_early && vector_tail && !eu_direct_fetch &&
                      !q_ripe && !eu_slot_busy) ||
                      (row_post_now && inta_first && eu_direct_fetch &&
                       !irq_fast_inta && !eu_slot_busy) ||
                      (row_post_now && (rdq_n == 2'd2) && !eu_slot_busy) ||
                      // PF_LOST's retained 8F tail consumes the younger read
                      // at its data edge.  Reserve that completion eval; the
                      // successor request reaches the ordinary EU slot next.
                      ghost_preread_edge_lag;
assign eu_halt_irq = irq_halt_entry;
assign eu_vector_post = eu_post && vector_first;
assign eu_bs   = vector_early ? BS_MEMR
               : pr_active   ? BS_MEMR
               : row_is_inta ? BS_INTA
               : row_is_read ? (acc_io ? BS_IOR : BS_MEMR)
                             : (acc_io ? BS_IOW : BS_MEMW);
assign eu_addr = vector_early ? vector_phys_early
               : pr_active ? pr_phys : (row_is_inta ? 20'd0 : acc_phys);
assign eu_addr2= vector_early ? (vector_phys_early + 20'd1)
               : pr_active ? pr_phys2 : acc_phys2;
assign eu_split= vector_early ? 1'b0
               : pr_active ? pr_split : (!row_is_inta && acc_split);
assign eu_seg  = vector_early ? 2'd2
               : pr_active ? seg_code(pr_seg)
               : row_is_inta ? 2'd2 : (acc_io ? 2'd2 : seg_code(acc_seg));
assign eu_seg2 = vector_early ? 2'd2
               : pr_active ? seg_code(pr_seg2)
               : row_is_inta ? 2'd2 : (acc_io ? 2'd2 : seg_code(acc_seg));
assign eu_word = vector_early ? 1'b1
               // The stale 8F word-lane rail outlives its absent operand and
               // overlays the following byte pre-read on silicon.
               : pr_active ? (ghost_preread_tail ? 1'b1 : !pr_byte)
               // The ghost posts on the successor-opcode overlap.  Its lane
               // mux therefore sees the successor's ordinary PLA width; C0
               // and C6 successors are the two current-socket byte witnesses.
               // Retain the earlier full-phase C0 witness as the same mux's
               // absent-operand input.
               : (ghost_read_stale_alu &&
                  (ghost_next_byte ||
                   (eu_ghost_full && (modrm_reg == 3'd0) &&
                    (m_idx == 3'd0)))) ? 1'b0
               : (row_is_inta ? 1'b1 : !acc_byte);

// The data pairing (`emit_pending`).  The row's OWN `-> OPR` write counts:
// the model writes OPR and then emits, both on the row's clock, so the value
// the store takes is the one this row is about to put there.
//
// THE POST-`E` ROW IS A ROW WITH A CLOCK (family A, U2 pass 3).  F8 gave it no
// STATE -- it is a one-bit debt discharged at the top of the next edge -- but
// the model runs it as an ordinary row, and `exec_impl.h:1095`'s
// `if (pend_.active && opr_fresh_) emit_pending();` fires on it like any other.
// `50`'s store is paired by exactly that: the `E` row `0029` posts the cycle
// and the post-`E` row `002A` (`M -> OPR`) hands it the data.  The clock it
// happens on is the clock `poste` STANDS on, so the pairing act belongs there.
// Without this the deferred store ran with whatever OPR held (`50 idx 2`:
// data 0000 against the golden's AX) -- eu_pair never asserted at all.
wire opr_wr_gate = e_have1 &&
                   ((e_d1 == 5'd6) ||
                    ((e_d1 == 5'd18) && (r_kind == OK_MEM)) ||
                    ((e_d1 == 5'd19) && (m_kind == OK_MEM))) &&
                   !((e_s1 == 5'd20) && !sig_commits);
wire row_wr_opr = (st == S_ROW) && !row_blocked && (rowq >= row_qn) &&
                  opr_wr_gate;
wire poste_wr_opr = poste && opr_wr_gate;
// F20's delivery is a transfer like any other, so the `emit_pending()` that
// follows it takes the word it delivers -- the head of the completed-read
// store.  The row's OWN `-> OPR` write comes FIRST in the model's order
// (transfers, then the row-type block's bus call), so it wins.
wire row_pre_deliver = (st == S_ROW) && !row_blocked && (rowq >= row_qn) &&
                       !row_pre_wait && row_pre_pair && !opr_fresh &&
                       !row_wr_opr;
// ...and F11a's rule on the READ side applies to THIS delivery too.  When the
// completion IS the lookahead (`nr_have`'s `eu_rd_done_n`) the word is not in
// the store yet -- block (a) puts it there in the same edge the step then pops
// it -- so the act decode has to read `eu_rdata_n` directly.  `opr_live` had
// this for the `F` row and `opr_now` did not, so every `REP MOVS` middle
// iteration whose read landed on the pairing clock drove the STALE OPR:
// `F3A4 idx 1` row 14, exp 37032 got 0.
// The same untagged data rail is also the write-pairing rail.  If a successor
// store pairs while the ghost word is still in the BIU read latch, that word
// wins the collision.  Once the completion pulse has handed the word to the
// EU, the row's ordinary source wins instead.  No added latch is needed.
wire ghost_edge_pair = ghost_rd_discard && eu_pair && !eu_rd_done_n;
wire [15:0] opr_now = ghost_edge_pair                    ? eu_rd_edge_d
                    : (row_wr_opr || poste_wr_opr)       ? s1_now
                    : (row_pre_deliver && (rdq_n != 2'd0)) ? rdq0
                    : (row_pre_deliver && eu_rd_done_n)    ? eu_rdata_n
                    : opr;
assign eu_pair  = ((st == S_ROW) && !row_blocked && (rowq >= row_qn) &&
                   !row_pre_wait &&
                   (pend_active || row_is_wr || row_is_wb) &&
                   (opr_fresh || row_wr_opr || row_pre_deliver))
               || (poste && pend_active && (opr_fresh || poste_wr_opr));
assign eu_pair2 = pend_active ? pend_split : acc_split;
assign eu_wdata = opr_now;

// --- the CTL strobes -------------------------------------------------------
assign q_flush  = row_acts_ok && row_flush;
// The REP-withdrawal redirect can coincide with the tail of an element whose
// accepted bus result is still draining into the EU.  Publish that one-bit
// ownership fact with the flush; the BIU uses it only to choose which side of
// the redirect clock its next arbitration point occupies.
// `rep_kind` has already been retired on the no-tail arm by this row, while it
// remains live until S_TAIL_W on the pending arm.  The common control rail is
// therefore the withdrawal microaddress itself, not that travelling prefix
// latch: page 7 / opcode 40 is the one REP-withdrawal sequence in the ROM.
assign flush_pre = (st == S_ROW) && (upc_page == 3'd7) &&
                   (upc_opc == 8'h40) && (upc_loc == 4'd6) && !pend_active;
assign flush_rep = q_flush && (upc_page == 3'd7) && (upc_opc == 8'h40);
assign flush_stage = q_flush && pend_active;
// A staged tail delays the redirect only when an external withdrawal still
// has a completed element read to deliver.  A bare pairing latch has no read
// behind it and the single-step path consumes its tail in parallel with the
// redirect; neither owns an extra BIU arbitration point.
assign flush_pend = q_flush && pend_active && (rdq_n != 2'd0) && !brk_take;
// Publish the two interrupt rails that physically meet the withdrawal edge.
// NMI is the part's recognition latch; maskable INT is the live package pin,
// whose release distinguishes a short impulse from an asserted hold without
// adding history state to either unit.
assign flush_nmi = irq_nmi_lvl;
assign flush_int_live = q_flush && pin_int;
assign flush_cs = cs_now;
assign flush_cs_old = sreg[SR_CS];
assign flush_cs_we = wr_cs1;
assign flush_ip = pc_now;
// F25: ...and the prefetcher is held through the reset dispatch.
assign eu_susp  = (st == S_RESET) ||
                  (row_acts_ok && (e_type == TY_CTL) && !e_farjmp &&
                   (e_ictl == I_SUSP));
assign eu_resume = 1'b0;

// S9a -- HLT IS DECODED, NOT MICROCODED, AND THE DECODE IS WHERE IT ACTS.
// The model's `note_halt` lands in tick(c)'s PRE-ROW block, which in RTL is
// the edge ending c-1 (U1 finding, provenance sec.11.6) -- so `eu_halt` LEADS
// by one clock, and the clock it must ride is the OPCODE'S OWN POP CLOCK.
// One rule, both paths: the loader's own F pop and the E row's pre-pop.
//
// U2 pass 5 -- VERIFIED FOR THE FIRST TIME, and it is RIGHT: the model's HALT
// DISPLAY block sits at the TOP of `tick(c)` and writes `cmt_disp_ = c`, i.e.
// it claims the clock it runs on, where a GRANT (decided at the end of
// `tick(c)`) claims `c + 1`.  The RTL's register boundary supplies exactly
// that one clock of skew, so the pop-clock lead is what puts the HALT display
// on `pop + 1` -- which is where 300 of the 600 goldens have it.
//
// What was WRONG is the other half of the same edge, and it is a BIU fact, not
// this one: `halted` was applied to the prefetch grant IN THE SAME EDGE, so
// the eval at the end of the pop clock refused a fetch the part grants.  See
// `v30u_biu.sv`'s "M16 -- THE DECODE DOES NOT TAKE A COMMITTED FETCH BACK".
wire qb_is_halt = pla3_one_byte_logic(pla3_lookup(
                      mode8080 ? PLA3_MODE_8080 : PLA3_MODE_NATIVE, q_byte)) &&
                  (pla3_xop(pla3_lookup(
                      mode8080 ? PLA3_MODE_8080 : PLA3_MODE_NATIVE, q_byte))
                   == PLA3_BL1_HALT);
assign eu_halt = q_pop && q_ripe && q_first && qb_is_halt && !mode8080 &&
                 !eu_halted;

// THE WAKE.  A halted part has no instruction boundary to sample, so the
// decision sits at the EARLIEST CLOCK THE PIN PIPELINE ALLOWS -- which is the
// same pipeline, read in the same place.  `timed_runner.cpp` states the
// geometry and it falls straight out of `B = D + 1`, `entry = B + 2`:
//
//   HLT.RES  D = A+3 (the INT level matures), pop at A+4  -- "the prefetcher
//            resumes at the decision cycle, the next instruction pops one
//            cycle later"
//   HLT.INT  D = A+3, B = A+4, entry at A+6 -- "INTA request ready at
//            assert+6"; the prefetcher restarts at the DECISION
//   HLT.NMI  D = A+4 (the latch reads true at edge+4), B = A+5, entry at A+7
//            -- and the BUS IS HELD: the wake does not happen until the entry
//            clock, which is what `unhalt_pend` carries.
//
// The INT wake is IE-INDEPENDENT (measured, and it is where the V30 differs
// from the 8086): a masked INT resumes the stream without vectoring, which is
// the same `S_OPC_POP` with `irq_take` false.
wire hlt_wake_int = (st == S_HALTED) && !irq_nmi_lvl && irq_pin_int;
assign eu_unhalt = hlt_wake_int || unhalt_pend;

// F43 -- THE HALT-DISPLAY DECISION MUST TEST THE WAKE VISIBLE ON ITS OWN
// DECISION EDGE.  M20 threshold 1 says the display at clock `H` is suppressed
// when the wake decision `D` satisfies `D <= H`; `D = A + 3` is MEASURED at
// 100 % on all four `evt` cells.  The BIU decides the display at the edge
// ending `H-1`, where `eu_unhalt` reads `int_p[2]` (the pin at `c-3`) and so
// is true only for `D <= H-1`.  The remaining case `D == H` is visible AT THAT
// SAME EDGE one stage further down: `int_p[1]`, the pin at `c-2`.  Both taps
// are `edge - 4`; what changed is the REFERENCE EDGE (F40's shape).
wire hlt_wake_disp = (st == S_HALTED) && !irq_nmi_lvl && int_p[1];

// F54 (SM3 sitting 17) -- ...AND THE NMI HALF OF THE SAME SENTENCE.  MEASURED
// on the S16 display walk, 1,512 retained captures, `sw/sm3_haltsupp.py chip`:
// the HALT announcement at `H` is cancelled iff the assert clock `A` satisfies
// `A <= H - K`, with **K = 3 on the INT pin and K = 6 on the NMI pin** --
// one value per pin, invariant over the four wait levels, over six programs
// per form, and over IE (`HLT.INT` ie=1 and `HLT.RES` ie=0 share K=3).
//
// The INT half is already right, and it is right in TWO places: `eu_unhalt`
// clears the BIU's `halt_pending` outright (`v30u_biu.sv` "if (eu_unhalt)"),
// which covers every `A <= H-4`, and `eu_unhalt_disp` above covers the single
// remaining `D == H` clock.  The NMI wake reaches NEITHER until `unhalt_pend`,
// which `S_IRQ_D` sets at `c0+2` and which therefore reads true only from
// `c0+3 = A+7` -- so the ucore's effective K was **8**, and the 42 S16 cells
// at `A` in {H-7, H-6} are exactly that difference.  (S17 candidate V-A put
// `irq_nmi_lvl` in `hlt_wake_disp`; that test is ONE clock long, so it reached
// exactly `A == H-5` and nothing else.  It broke the 24 cells that predicts
// and closed none of the 42 -- registered, measured, reverted.)
//
// `c0` is the first `S_HALTED` clock with `nmi_latch` up.  The EU leaves
// `S_HALTED` on that edge and does not clear `eu_halted` until `S_IRQ_D`, so
// `eu_halted && (st != S_HALTED)` is true across `c0+1 .. c0+2` and
// `unhalt_pend` takes over at `c0+3`: the union is `c >= c0+1`, and the
// display decided at the edge ending `H-1` is therefore cancelled iff
// `c0 + 1 <= H - 1`, i.e. `A + 5 <= H - 1`, i.e. **`A <= H - 6`**.
// NO FLOP IS ADDED and no pin is tested: `eu_halted` and `st <= S_HALTED` are
// written in the SAME arm (`S_DECODE2`), so the term is false everywhere
// before the HALT, and the INT wake clears `eu_halted` in the same arm that
// leaves `S_HALTED`, so it is false there too.  It is NMI-specific by
// construction.
//   *Falsifier*: a silicon capture whose HALT announcement fires with the NMI
//   asserted at `A <= H-6`, or is cancelled with it asserted at `A >= H-5`.
wire hlt_wake_nmi_disp = eu_halted && (st != S_HALTED);
assign eu_unhalt_disp = hlt_wake_disp || unhalt_pend || hlt_wake_nmi_disp;

// CITF follows the interrupt prologue's FLAGS->OPR F row.  Its clear rail
// reaches the package-status mux through the ROM lookahead while that row
// retires, before the flag register itself is written.  The existing OPR-free
// rail is the row's enable; no history or interrupt-specific state is added.
wire citf_status_pre = (st == S_ROW) && (upc_page == 3'd7) &&
                       ((upc_opc == 8'h10) || (upc_opc == 8'h18)) &&
                       (upc_loc == 4'd8) && opr_free_now;
assign psw_ie  = psw[FIE] && !citf_status_pre;
assign md8080  = mode8080;

//----------------------------------------------------------------------------
// the backdoor / debug view
//----------------------------------------------------------------------------
assign dbg_regs = {psw, pc, sreg[3], sreg[2], sreg[1], sreg[0],
                   gpr[7], gpr[6], gpr[5], gpr[4],
                   gpr[3], gpr[2], gpr[1], gpr[0]};
assign dbg_first_pop = first_pop_seen;
assign dbg_pend = (rd_pending != 2'd0) || (rdq_n != 2'd0) || poste;

//--------------------------------------------------------------------------
// THE RESET NEXT-STATE (U4 pass 3, second structural pass -- sec.52.3)
//--------------------------------------------------------------------------
// `srst` used to be an ARM of the run next-state function, which put it in
// the same expression tree as the twelve-position chain and let Quartus
// distribute it through the whole cone -- measured: `c_reset_q` ->
// `v30u_eu|wb_seg[0]~0` -> `v30u_biu|q_ripe_lead_n` -> twelve chain
// positions -> `opc_base[4]`, 58.9 ns against a 31.25 ns requirement, and
// NOT coverable by the CE multicycle because it LAUNCHES OUTSIDE THE CORE
// (`system_large.sv:372`: `core_reset = c_reset_q | ~cfg_use_core`).
//
// Given its own function, the reset value is constants + the pin levels +
// the backdoor and nothing else, `_n` is provably independent of `srst`,
// and the register bank picks between them.  Same medicine as the CE fix
// and the same expectation: zero ladder deltas.
reg     [15:0] gpr_r [0:7];
reg     [15:0] sreg_r [0:3];
reg     [15:0] pc_r;
reg     [15:0] psw_r;
reg     [15:0] tmpa_r;
reg     [15:0] tmpb_r;
reg     [15:0] tmpc_r;
reg     [15:0] ea_residue_r;
reg     [15:0] ea_pair_rhs_r;
reg            ea_pair_valid_r;
reg     [15:0] opr_r;
reg     [15:0] ind_r;
reg     [15:0] count_r;
reg      [7:0] pfxcnt_r;
reg     [15:0] stat_r;
reg            sign_neg_r;
reg      [3:0] bit_n_r;
reg      [4:0] al_op_r;
reg      [1:0] al_tmp_r;
reg            al_byte_r;
reg            al_eaconst_r;
reg     [15:0] al_eaval_r;
reg      [1:0] al_adjust_r;
reg      [1:0] al_adjtmp_r;
reg            al_bitarm_r;
reg      [3:0] al_bitn_r;
reg            al_spent_r;
reg      [2:0] upc_page_r;
reg      [7:0] upc_opc_r;
reg      [3:0] upc_loc_r;
reg            seg_override_r;
reg      [1:0] seg_ovr_r;
reg      [2:0] rep_kind_r;
reg            lock_pfx_r;
reg      [7:0] opc_reg_r;
reg            op8_r;
reg            imm8_r;
reg      [4:0] opc_base_r;
reg            opc_from_modrm_r;
reg      [2:0] modrm_reg_r;
reg      [3:0] xop_r;
reg      [1:0] rep_test_r;
reg            rep_pol_r;
reg            bus_word_r;
reg            opc8080_r;
reg            mode8080_r;
reg            intr_pending_r;
reg            eu_halted_r;
reg      [3:0] int_p_r;
reg      [4:0] nmi_p_r;
reg            nmi_latch_r;
reg      [3:0] ie_p_r;
reg            rep_chain_r;
reg            irq_shadow_r;
reg            bnd_armed_r;
reg            irq_sel_nmi_r;
reg            irq_sel_brk_r;
reg [BRK_FLOOR-1:0] brk_p_r;
reg            brk_arm_r;
reg            brk_smp_r;
reg            unhalt_pend_r;
reg            irq_fast_inta_r;
reg            irq_halt_entry_r;
reg      [1:0] m_kind_r;
reg      [1:0] r_kind_r;
reg      [1:0] wb_kind_r;
reg      [2:0] m_idx_r;
reg      [2:0] r_idx_r;
reg      [2:0] wb_idx_r;
reg     [15:0] m_ea_r;
reg     [15:0] r_ea_r;
reg     [15:0] wb_ea_r;
reg      [2:0] m_seg_r;
reg      [2:0] r_seg_r;
reg      [2:0] wb_seg_r;
reg            m_byte_r;
reg            r_byte_r;
reg            wb_byte_r;
reg            pend_active_r;
reg     [15:0] pend_off_r;
reg      [2:0] pend_seg_r;
reg            pend_byte_r;
reg            pend_io_r;
reg            opr_fresh_r;
reg            opr_loaded_r;
reg     [15:0] rdq0_r;
reg     [15:0] rdq1_r;
reg      [1:0] rdq_n_r;
reg      [1:0] rd_pending_r;
reg            ghost_rd_discard_r;
reg            ghost_rd_feed_r;
reg            ghost_rd_ready_r;
reg      [1:0] rd_done_cnt_r;
reg            rd_age0_r;
reg            iend_owed_r;
reg      [2:0] rst_ctr_r;
reg     [15:0] tsel_r;
reg      [7:0] pe_opc_reg_r;
reg            pe_opc8080_r;
reg            pe_op8_r;
reg      [7:0] pe_pfxcnt_r;
reg      [1:0] wr_out_r;
reg            opc_valid_r;
reg      [7:0] opc_byte_r;
reg            opc_rm_valid_r;
reg      [7:0] opc_rm_byte_r;
reg            pop_is_first_r;
reg      [7:0] ld_b_r;
reg     [13:0] ld_pla_r;
reg            ld_ext_r;
reg      [2:0] ld_page_r;
reg            ld_hasrm_r;
reg      [7:0] ld_rm_r;
reg     [15:0] ld_disp_r;
reg      [7:0] ld_dlo_r;
reg            ld_grpd_r;
reg            ld_byte_r;
reg            ld_preread_r;
reg            ld_ripe_prev_r;
reg      [5:0] st_r;
reg      [1:0] chg_r;
reg            ending_r;
reg      [1:0] rowq_r;
reg            row_posted_r;
reg            row_paired_r;
reg     [15:0] rloop_n_r;
reg            suppress_commit_r;
reg            first_pop_seen_r;
reg      [7:0] rowb0_r;
reg      [7:0] rowb1_r;
reg            poste_r;
reg      [2:0] poll_pipe_r;

integer rsi;                // the reset function's own array index
always @* begin
    //-- preload, so an unassigned reset arm holds rather than latches
    for (rsi = 0; rsi < 8; rsi = rsi + 1) gpr_r[rsi] = gpr[rsi];
    for (rsi = 0; rsi < 4; rsi = rsi + 1) sreg_r[rsi] = sreg[rsi];
    pc_r = pc;
    psw_r = psw;
    tmpa_r = tmpa;
    tmpb_r = tmpb;
    tmpc_r = tmpc;
    ea_residue_r = ea_residue;
    ea_pair_rhs_r = ea_pair_rhs;
    ea_pair_valid_r = ea_pair_valid;
    opr_r = opr;
    ind_r = ind;
    count_r = count;
    pfxcnt_r = pfxcnt;
    stat_r = stat;
    sign_neg_r = sign_neg;
    bit_n_r = bit_n;
    al_op_r = al_op;
    al_tmp_r = al_tmp;
    al_byte_r = al_byte;
    al_eaconst_r = al_eaconst;
    al_eaval_r = al_eaval;
    al_adjust_r = al_adjust;
    al_adjtmp_r = al_adjtmp;
    al_bitarm_r = al_bitarm;
    al_bitn_r = al_bitn;
    al_spent_r = al_spent;
    upc_page_r = upc_page;
    upc_opc_r = upc_opc;
    upc_loc_r = upc_loc;
    seg_override_r = seg_override;
    seg_ovr_r = seg_ovr;
    rep_kind_r = rep_kind;
    lock_pfx_r = lock_pfx;
    opc_reg_r = opc_reg;
    op8_r = op8;
    imm8_r = imm8;
    opc_base_r = opc_base;
    opc_from_modrm_r = opc_from_modrm;
    modrm_reg_r = modrm_reg;
    xop_r = xop;
    rep_test_r = rep_test;
    rep_pol_r = rep_pol;
    bus_word_r = bus_word;
    opc8080_r = opc8080;
    mode8080_r = mode8080;
    intr_pending_r = intr_pending;
    eu_halted_r = eu_halted;
    int_p_r = int_p;
    nmi_p_r = nmi_p;
    nmi_latch_r = nmi_latch;
    ie_p_r = ie_p;
    rep_chain_r = rep_chain;
    irq_shadow_r = irq_shadow;
    bnd_armed_r = bnd_armed;
    irq_sel_nmi_r = irq_sel_nmi;
    irq_sel_brk_r = irq_sel_brk;
    brk_p_r = brk_p;
    brk_arm_r = brk_arm;
    brk_smp_r = brk_smp;
    unhalt_pend_r = unhalt_pend;
    irq_fast_inta_r = irq_fast_inta;
    irq_halt_entry_r = irq_halt_entry;
    m_kind_r = m_kind;
    r_kind_r = r_kind;
    wb_kind_r = wb_kind;
    m_idx_r = m_idx;
    r_idx_r = r_idx;
    wb_idx_r = wb_idx;
    m_ea_r = m_ea;
    r_ea_r = r_ea;
    wb_ea_r = wb_ea;
    m_seg_r = m_seg;
    r_seg_r = r_seg;
    wb_seg_r = wb_seg;
    m_byte_r = m_byte;
    r_byte_r = r_byte;
    wb_byte_r = wb_byte;
    pend_active_r = pend_active;
    pend_off_r = pend_off;
    pend_seg_r = pend_seg;
    pend_byte_r = pend_byte;
    pend_io_r = pend_io;
    opr_fresh_r = opr_fresh;
    opr_loaded_r = opr_loaded;
    rdq0_r = rdq0;
    rdq1_r = rdq1;
    rdq_n_r = rdq_n;
    rd_pending_r = rd_pending;
    ghost_rd_discard_r = ghost_rd_discard;
    ghost_rd_feed_r = ghost_rd_feed;
    ghost_rd_ready_r = ghost_rd_ready;
    rd_done_cnt_r = rd_done_cnt;
    rd_age0_r = rd_age0;
    iend_owed_r = iend_owed;
    rst_ctr_r = rst_ctr;
    tsel_r = tsel;
    pe_opc_reg_r = pe_opc_reg;
    pe_opc8080_r = pe_opc8080;
    pe_op8_r = pe_op8;
    pe_pfxcnt_r = pe_pfxcnt;
    wr_out_r = wr_out;
    opc_valid_r = opc_valid;
    opc_byte_r = opc_byte;
    opc_rm_valid_r = opc_rm_valid;
    opc_rm_byte_r = opc_rm_byte;
    pop_is_first_r = pop_is_first;
    ld_b_r = ld_b;
    ld_pla_r = ld_pla;
    ld_ext_r = ld_ext;
    ld_page_r = ld_page;
    ld_hasrm_r = ld_hasrm;
    ld_rm_r = ld_rm;
    ld_disp_r = ld_disp;
    ld_dlo_r = ld_dlo;
    ld_grpd_r = ld_grpd;
    ld_byte_r = ld_byte;
    ld_preread_r = ld_preread;
    ld_ripe_prev_r = ld_ripe_prev;
    st_r = st;
    chg_r = chg;
    ending_r = ending;
    rowq_r = rowq;
    row_posted_r = row_posted;
    row_paired_r = row_paired;
    rloop_n_r = rloop_n;
    suppress_commit_r = suppress_commit;
    first_pop_seen_r = first_pop_seen;
    rowb0_r = rowb0;
    rowb1_r = rowb1;
    poste_r = poste;
    poll_pipe_r = poll_pipe;

        //--------------------------------------------------------------------
        // RESET == begin_case() plus the backdoor injection
        //--------------------------------------------------------------------
        for (rsi = 0; rsi < 8; rsi = rsi + 1) gpr_r[rsi] = 16'd0;
        for (rsi = 0; rsi < 4; rsi = rsi + 1) sreg_r[rsi] = 16'd0;
        pc_r = 16'd0; psw_r = PSW_FORCED;
        tmpa_r = 16'd0; tmpb_r = 16'd0; tmpc_r = 16'd0;
        ea_residue_r = 16'd0;
        ea_pair_rhs_r = 16'd0; ea_pair_valid_r = 1'b0;
        opr_r = 16'd0; ind_r = 16'd0; count_r = 16'd0; pfxcnt_r = 8'd0;
        stat_r = 16'd0; sign_neg_r = 1'b0; bit_n_r = 4'd0;
        al_op_r = A_ADD; al_tmp_r = 2'd0; al_byte_r = 1'b0;
        al_eaconst_r = 1'b0; al_eaval_r = 16'd0;
        al_adjust_r = 2'd0; al_adjtmp_r = 2'd0; al_bitarm_r = 1'b0; al_bitn_r = 4'd0;
        al_spent_r = 1'b0;
        upc_page_r = 3'd0; upc_opc_r = 8'd0; upc_loc_r = 4'd0;
        seg_override_r = 1'b0; seg_ovr_r = 2'd3; rep_kind_r = REP_NONE;
        lock_pfx_r = 1'b0; opc_reg_r = 8'd0; op8_r = 1'b0; imm8_r = 1'b0;
        opc_base_r = 5'd0; opc_from_modrm_r = 1'b0; modrm_reg_r = 3'd0; xop_r = 4'd0;
        rep_test_r = TEST_NONE; rep_pol_r = 1'b0; bus_word_r = 1'b0;
        opc8080_r = 1'b0; mode8080_r = 1'b0; intr_pending_r = 1'b0; eu_halted_r = 1'b0;
        rep_chain_r = 1'b0;
        m_kind_r = OK_NONE; m_idx_r = 3'd0; m_ea_r = 16'd0; m_seg_r = 3'd3; m_byte_r = 1'b0;
        r_kind_r = OK_NONE; r_idx_r = 3'd0; r_ea_r = 16'd0; r_seg_r = 3'd3; r_byte_r = 1'b0;
        wb_kind_r = OK_NONE; wb_idx_r = 3'd0; wb_ea_r = 16'd0; wb_seg_r = 3'd3;
        wb_byte_r = 1'b0;
        pend_active_r = 1'b0; pend_off_r = 16'd0; pend_seg_r = 3'd3;
        pend_byte_r = 1'b0; pend_io_r = 1'b0; opr_fresh_r = 1'b0;
        opr_loaded_r = 1'b0;
        rdq0_r = 16'd0; rdq1_r = 16'd0; rdq_n_r = 2'd0;
        rd_pending_r = 2'd0; ghost_rd_discard_r = 1'b0;
        ghost_rd_feed_r = 1'b0;
        ghost_rd_ready_r = 1'b0;
        rd_done_cnt_r = 2'd0; rd_age0_r = 1'b0;
        iend_owed_r = 1'b0; pe_opc_reg_r = 8'd0; pe_opc8080_r = 1'b0; pe_op8_r = 1'b0;
        pe_pfxcnt_r = 8'd0;
        wr_out_r = 2'd0;
        opc_valid_r = 1'b0; opc_byte_r = 8'd0;
        opc_rm_valid_r = 1'b0; opc_rm_byte_r = 8'd0;
        pop_is_first_r = 1'b1;
        ld_b_r = 8'd0; ld_pla_r = 14'd0; ld_ext_r = 1'b0; ld_page_r = 3'd0;
        ld_hasrm_r = 1'b0; ld_rm_r = 8'd0; ld_disp_r = 16'd0; ld_dlo_r = 8'd0;
        ld_grpd_r = 1'b0; ld_byte_r = 1'b0; ld_preread_r = 1'b0; ld_ripe_prev_r = 1'b0;
        st_r = S_OPC_POP; chg_r = 2'd0; ending_r = 1'b0; poste_r = 1'b0;
        rowq_r = 2'd0; row_posted_r = 1'b0; row_paired_r = 1'b0; rloop_n_r = 16'd0;
        suppress_commit_r = 1'b0; first_pop_seen_r = 1'b0;
        rowb0_r = 8'd0; rowb1_r = 8'd0; poste_r = 1'b0;
        // the pin pipelines come out of reset holding the LEVEL they have been
        // seeing -- a shift register clocked since power-on cannot hold
        // anything else, and it is what `poll_busy()` / the INT sample assume.
        poll_pipe_r = {3{pin_poll_n}};
        int_p_r = {4{pin_int}}; nmi_p_r = {5{pin_nmi}}; ie_p_r = 4'd0;
        rep_chained = 1'b0;
        nmi_latch_r = 1'b0; irq_shadow_r = 1'b0; bnd_armed_r = 1'b0;
        irq_sel_nmi_r = 1'b0; unhalt_pend_r = 1'b0;
        irq_fast_inta_r = 1'b0;
        irq_halt_entry_r = 1'b0;
        irq_sel_brk_r = 1'b0; brk_p_r = '0; brk_arm_r = 1'b0;
        brk_smp_r = 1'b0;
        if (bkd_load) begin
            gpr_r[0] = bkd_regs[  0 +: 16];  gpr_r[1] = bkd_regs[ 16 +: 16];
            gpr_r[2] = bkd_regs[ 32 +: 16];  gpr_r[3] = bkd_regs[ 48 +: 16];
            gpr_r[4] = bkd_regs[ 64 +: 16];  gpr_r[5] = bkd_regs[ 80 +: 16];
            gpr_r[6] = bkd_regs[ 96 +: 16];  gpr_r[7] = bkd_regs[112 +: 16];
            sreg_r[0] = bkd_regs[128 +: 16]; sreg_r[1] = bkd_regs[144 +: 16];
            sreg_r[2] = bkd_regs[160 +: 16]; sreg_r[3] = bkd_regs[176 +: 16];
            pc_r = bkd_regs[192 +: 16];
            psw_r = (bkd_regs[208 +: 16] & PSW_WRITABLE) | PSW_FORCED;
        end else begin
            // F25 -- POWER-ON RESET IS A MICROCODE MARCH, not a state.
            // `CpuT::reset()` runs the ROM's own sequence at page 7 opcode
            // 00000011 (01D0..01D5: ZEROS -> DS/FLAGS/ES/SS, ONES -> CS,
            // ZEROS -> PC, FLUSH, MFS) and only THEN does the decoder see its
            // first byte.  The EU came out of reset in S_OPC_POP, so the whole
            // boot was seven clocks early and `check_boot --core ucore` broke
            // at release+7 (the real part shows the march's queue flush; the
            // EU had already popped).  A BACKDOOR-LOADED case starts mid-
            // stream and must NOT run it -- that is what `bkd_load` selects,
            // and it is why every v0.1 rung is unaffected.
            //
            // `run_timed_boot` states the geometry and the capture pins it:
            // the part comes out of reset with the PREFETCHER SUSPENDED (there
            // is no fetch pointer until 01D3 loads PS:PC and flushes), and the
            // internal dispatch is FOUR CLOCKS -- 01D0 runs at release+4, the
            // FLUSH row 01D3 at release+7 where the capture shows its `E`
            // blip, and the first CODE T1 at release+9.  Four clocks is the
            // one constant, not a per-row cost.
            upc_page_r = 3'd7;
            upc_opc_r  = 8'h03;
            upc_loc_r  = 4'd0;
            rst_ctr_r  = 3'd0;
            st_r = S_RESET;
        end
        ie_p_r = {4{psw_r[FIE]}};      // ...and so does the IE gate's own pipeline
end

//==========================================================================
// THE NEXT-STATE SHADOW  (U4 pass 3 -- the ENABLE-FORM refactor, sec.51.7)
//==========================================================================
// One `_n` per state register.  The clocked body below is now an `always @*`
// that preloads these from the flops and works on them with the SAME blocking
// assignments it always used -- so `_n` IS what a blocking write to the
// register meant there, and a module-level wire still reads the FLOP, i.e.
// the PRE-EDGE view this whole module is built on (F11b; see the recognition
// block's `EVERY read of these registers from inside the clocked step goes
// through these wires' note).  That is why the flops keep their names and the
// body was renamed, and not the other way round: nothing outside the body
// moves, so no combinational reader changes what it sees.
//
// THE POINT IS THE COMMIT.  `always @(posedge clk) if (ss_we||srst||ce)` puts
// CE on the register's ENABLE PORT.  Before this, `ce` was threaded through
// the EU's 61-level combinational cone to every FF's DATA input; Quartus
// extracted no clock enable (`report_timing`: `div_cnt[1]` -> `m_kind[0]`,
// 61 logic levels, terminating on `datac`/`dataf` and no `ena` node anywhere),
// so the registers clocked every SYS clock and the multicycle exception the
// divided CPU clock earns was INVALID BY ITS OWN FALSIFIER (sec.51.7).  `ce`
// does not appear in this cone at all now -- the body's third arm is the
// unconditional `else`.
//
// `upc_page_n` / `upc_opc_n` / `upc_loc_n` exist as wires as a free
// consequence -- the only thing a registered microcode ROM ever needed.
//
// *Falsifier*: a `v30u_eu` state register with no `ena` in the post-fit
// netlist, or a ladder cell that moves.  This is a SYNTHESIS-SHAPE change and
// the expectation is zero deltas.
reg     [15:0] gpr_n [0:7];
reg     [15:0] sreg_n [0:3];
reg     [15:0] pc_n;
reg     [15:0] psw_n;
reg     [15:0] tmpa_n;
reg     [15:0] tmpb_n;
reg     [15:0] tmpc_n;
reg     [15:0] ea_residue_n;
reg     [15:0] ea_pair_rhs_n;
reg            ea_pair_valid_n;
reg     [15:0] opr_n;
reg     [15:0] ind_n;
reg     [15:0] count_n;
reg      [7:0] pfxcnt_n;
reg     [15:0] stat_n;
reg            sign_neg_n;
reg      [3:0] bit_n_n;
reg      [4:0] al_op_n;
reg      [1:0] al_tmp_n;
reg            al_byte_n;
reg            al_eaconst_n;
reg     [15:0] al_eaval_n;
reg      [1:0] al_adjust_n;
reg      [1:0] al_adjtmp_n;
reg            al_bitarm_n;
reg      [3:0] al_bitn_n;
reg            al_spent_n;
reg      [2:0] upc_page_n;
reg      [7:0] upc_opc_n;
reg      [3:0] upc_loc_n;
reg            seg_override_n;
reg      [1:0] seg_ovr_n;
reg      [2:0] rep_kind_n;
reg            lock_pfx_n;
reg      [7:0] opc_reg_n;
reg            op8_n;
reg            imm8_n;
reg      [4:0] opc_base_n;
reg            opc_from_modrm_n;
reg      [2:0] modrm_reg_n;
reg      [3:0] xop_n;
reg      [1:0] rep_test_n;
reg            rep_pol_n;
reg            bus_word_n;
reg            opc8080_n;
reg            mode8080_n;
reg            intr_pending_n;
reg            eu_halted_n;
reg      [3:0] int_p_n;
reg      [4:0] nmi_p_n;
reg            nmi_latch_n;
reg      [3:0] ie_p_n;
reg            rep_chain_n;
reg            irq_shadow_n;
reg            bnd_armed_n;
reg            irq_sel_nmi_n;
reg            irq_sel_brk_n;
reg [BRK_FLOOR-1:0] brk_p_n;
reg            brk_arm_n;
reg            brk_smp_n;
reg            unhalt_pend_n;
reg            irq_fast_inta_n;
reg            irq_halt_entry_n;
reg      [1:0] m_kind_n;
reg      [1:0] r_kind_n;
reg      [1:0] wb_kind_n;
reg      [2:0] m_idx_n;
reg      [2:0] r_idx_n;
reg      [2:0] wb_idx_n;
reg     [15:0] m_ea_n;
reg     [15:0] r_ea_n;
reg     [15:0] wb_ea_n;
reg      [2:0] m_seg_n;
reg      [2:0] r_seg_n;
reg      [2:0] wb_seg_n;
reg            m_byte_n;
reg            r_byte_n;
reg            wb_byte_n;
reg            pend_active_n;
reg     [15:0] pend_off_n;
reg      [2:0] pend_seg_n;
reg            pend_byte_n;
reg            pend_io_n;
reg            opr_fresh_n;
reg            opr_loaded_n;
reg     [15:0] rdq0_n;
reg     [15:0] rdq1_n;
reg      [1:0] rdq_n_n;
reg      [1:0] rd_pending_n;
reg            ghost_rd_discard_n;
reg            ghost_rd_feed_n;
reg            ghost_rd_ready_n;
reg      [1:0] rd_done_cnt_n;
reg            rd_age0_n;
reg            iend_owed_n;
reg      [2:0] rst_ctr_n;
reg     [15:0] tsel_n;
reg      [7:0] pe_opc_reg_n;
reg            pe_opc8080_n;
reg            pe_op8_n;
reg      [7:0] pe_pfxcnt_n;
reg      [1:0] wr_out_n;
reg            opc_valid_n;
reg      [7:0] opc_byte_n;
reg            opc_rm_valid_n;
reg      [7:0] opc_rm_byte_n;
reg            pop_is_first_n;
reg      [7:0] ld_b_n;
reg     [13:0] ld_pla_n;
reg            ld_ext_n;
reg      [2:0] ld_page_n;
reg            ld_hasrm_n;
reg      [7:0] ld_rm_n;
reg     [15:0] ld_disp_n;
reg      [7:0] ld_dlo_n;
reg            ld_grpd_n;
reg            ld_byte_n;
reg            ld_preread_n;
reg            ld_ripe_prev_n;
reg      [5:0] st_n;
reg      [1:0] chg_n;
reg            ending_n;
reg      [1:0] rowq_n;
reg            row_posted_n;
reg            row_paired_n;
reg     [15:0] rloop_n_n;
reg            suppress_commit_n;
reg            first_pop_seen_n;
reg      [7:0] rowb0_n;
reg      [7:0] rowb1_n;
reg            poste_n;
reg      [2:0] poll_pipe_n;

//============================================================================
// THE CLOCK
//============================================================================
`ifndef SYNTHESIS
reg eutrace = 0;
initial if ($test$plusargs("eutrace")) eutrace = 1;
// §86 -- THE ARM'S OWN INSTRUMENT, and it is the one the model already has
// (`V30SIM_BRKTRACE`).  One line per SAMPLE INSTANT and one per TAKE, so the
// RTL's coordinate can be checked against the model's retire stream on
// captures with TF CLEAR -- no trap in the loop, which is what makes it a
// ruler rather than a score.  §85.2a's acceptance test, RTL side.
reg brktrace = 0;
initial if ($test$plusargs("brktrace")) brktrace = 1;
// the CE clock index, on the same contract the eutrace line number carries
// ("row index == CE clock index"): cleared by `srst`, advanced once per
// enabled clock.  Simulation only.
int unsigned ce_clk = 0;
// §86.G -- THE TWO FLAG-WRITE PROBES, and they are why `mc1/721` is decided.
// `1BL` reports every ONE_BYTE_LOGIC execute strobe's PSW before and after;
// `PE` reports every POST-`E` discharge's, with the row it is discharging.
// Together they say WHICH of two writes to the same register landed and IN
// WHAT ORDER -- which the save-state map's PSW word alone cannot, because both
// orders can end on the same value.  Simulation only, `+brktrace`, and they read registers.
reg [15:0] trc_1bl_pre, trc_1bl_post; reg trc_1bl_hit;
reg [15:0] trc_pe_pre, trc_pe_post; reg trc_pe_hit; reg [11:0] trc_pe_upc;
// wrfuzz W3.5 -- THE 1BL DECODE PROBE, and it exists to answer ONE question
// with a measurement instead of an assumption: `wrfuzz_provenance.md` §7.8
// booked "`irq_shadow` is a FLOP, so `brk_arm` and `brk_take` are not
// interchangeable in the RTL -- MEASURE it, do not assume it."  The thing that
// actually has to be measured is narrower and it is the ARM'S AVAILABILITY:
// `S_DECODE2`'s 1BL arm runs INSIDE the chain that the opcode pop rode, and
// `brk_smp` -- the sample instant §85.2a fixed at pop + 1 -- has not happened
// yet.  This line reports, at every 1BL decode, the four bits the gate could
// be written on (`q_ripe_lead_n`, `brk_seen`, `brk_arm`, `brk_smp_n`) so the
// difference between them is a number in a log and not a claim.
// Simulation only, `+brktrace`; reads registers and settled comb.
reg trc_1bld_hit, trc_1bld_ripe, trc_1bld_seen, trc_1bld_arm, trc_1bld_smp;
reg trc_1bld_shd;   // ...and `irq_shadow_n`, which §7.8 names by hand
reg chain_report = 0;
initial if ($test$plusargs("chaindepth")) chain_report = 1;
reg [3:0] chain_hi = 4'd0;
reg [3:0] chain_used;   // <- now a COMB output of the next-state block
// the eutrace SNAPSHOT: the flop-valued fields the trace line used to
// read MID-EDGE (after blocks (a)/(b), before the chain).  Everything
// else it prints is a module-level wire, which the observer reads at the
// edge and therefore reads pre-edge exactly as the old code did.
reg      [5:0] trc_st;
reg      [2:0] trc_upc_page;
reg      [7:0] trc_upc_opc;
reg      [3:0] trc_upc_loc;
reg      [1:0] trc_wr_out;
reg     [15:0] trc_pc;
reg     [15:0] trc_ind;
reg     [15:0] trc_opr;
reg            trc_opr_fresh;
reg            trc_pend_active;
reg            trc_poste;
reg      [1:0] trc_rdq_n;
reg      [1:0] trc_rd_done_cnt;
reg     [15:0] trc_tmpa;
reg     [15:0] trc_tmpb;
reg     [15:0] trc_tmpc;
reg [5:0] chain_first;
`ifdef CHAIN_PROBE
reg cp_seen [0:1023];
integer cpi;
initial for (cpi = 0; cpi < 1024; cpi = cpi + 1) cp_seen[cpi] = 1'b0;
`endif
`endif

// The bounded zero-cost chain.  See the loop at the bottom of the clocked
// block: `CHAIN_MAX` is how many of the model's zero-cost steps may ride one
// clock, and it costs a FULL UNROLLED COPY of the step case per unit -- which
// is why 24 of the 33 arms are folded out of positions >= 1 (v30u_eu_step.svh).
localparam bit [3:0] CHAIN_MAX = 4'd12;

integer i;
integer ci;                 // the commit block's own index
reg        stop;
reg  [3:0] chain;
reg [15:0] v1, v2;
reg        bsw;             // the row's byte-source flag (Q / CONST)
reg [13:0] pv;
reg  [3:0] nloc;
reg        carry, taken, bubble, retire_now;
reg        rep_chained;  // ...and the value `rep_chain` had at THIS boundary
reg        ie_now;      // block (g): the IE the gate's pipeline takes
reg        brk_now;     // §86: ...and the TF the arm's pipeline takes
reg [15:0] ea;
reg  [2:0] rseg;
reg  [1:0] rmmod;
reg  [2:0] rmreg, rmrm;
reg  [1:0] tk;
reg  [2:0] ti, ts;
reg [15:0] te;
reg        tb;

`define SETPSW(v) begin psw_n = ((v) & PSW_WRITABLE) | PSW_FORCED; end

// commit_flags
task automatic commit_flags(input [15:0] mask, input [15:0] fl);
    begin
        psw_n = ((psw_n & ~mask) | (fl & mask));
        psw_n = (psw_n & PSW_WRITABLE) | PSW_FORCED;
    end
endtask

//--------------------------------------------------------------------------
// THE NEXT-STATE FUNCTION.  This was `always @(posedge clk)` with blocking
// assignments; it is the SAME code on `_n`, and the only behavioural change
// is that `ce` no longer appears in it -- the third arm is the unconditional
// `else`, because a CE-low clock is now expressed by the COMMIT BLOCK not
// capturing rather than by this block not running.
//--------------------------------------------------------------------------
always @* begin
    //-- preload: `_n` starts at the flop, so an arm that assigns nothing holds
    for (i = 0; i < 8; i = i + 1) gpr_n[i] = gpr[i];
    for (i = 0; i < 4; i = i + 1) sreg_n[i] = sreg[i];
    pc_n = pc;
    psw_n = psw;
    tmpa_n = tmpa;
    tmpb_n = tmpb;
    tmpc_n = tmpc;
    ea_residue_n = ea_residue;
    ea_pair_rhs_n = ea_pair_rhs;
    ea_pair_valid_n = ea_pair_valid;
    opr_n = opr;
    ind_n = ind;
    count_n = count;
    pfxcnt_n = pfxcnt;
    stat_n = stat;
    sign_neg_n = sign_neg;
    bit_n_n = bit_n;
    al_op_n = al_op;
    al_tmp_n = al_tmp;
    al_byte_n = al_byte;
    al_eaconst_n = al_eaconst;
    al_eaval_n = al_eaval;
    al_adjust_n = al_adjust;
    al_adjtmp_n = al_adjtmp;
    al_bitarm_n = al_bitarm;
    al_bitn_n = al_bitn;
    al_spent_n = al_spent;
    upc_page_n = upc_page;
    upc_opc_n = upc_opc;
    upc_loc_n = upc_loc;
    seg_override_n = seg_override;
    seg_ovr_n = seg_ovr;
    rep_kind_n = rep_kind;
    lock_pfx_n = lock_pfx;
    opc_reg_n = opc_reg;
    op8_n = op8;
    imm8_n = imm8;
    opc_base_n = opc_base;
    opc_from_modrm_n = opc_from_modrm;
    modrm_reg_n = modrm_reg;
    xop_n = xop;
    rep_test_n = rep_test;
    rep_pol_n = rep_pol;
    bus_word_n = bus_word;
    opc8080_n = opc8080;
    mode8080_n = mode8080;
    intr_pending_n = intr_pending;
    eu_halted_n = eu_halted;
    int_p_n = int_p;
    nmi_p_n = nmi_p;
    nmi_latch_n = nmi_latch;
    ie_p_n = ie_p;
    rep_chain_n = rep_chain;
    irq_shadow_n = irq_shadow;
    bnd_armed_n = bnd_armed;
    irq_sel_nmi_n = irq_sel_nmi;
    irq_sel_brk_n = irq_sel_brk;
    brk_p_n = brk_p;
    brk_arm_n = brk_arm;
    brk_smp_n = brk_smp;
    unhalt_pend_n = unhalt_pend;
    irq_fast_inta_n = irq_fast_inta;
    irq_halt_entry_n = irq_halt_entry;
    m_kind_n = m_kind;
    r_kind_n = r_kind;
    wb_kind_n = wb_kind;
    m_idx_n = m_idx;
    r_idx_n = r_idx;
    wb_idx_n = wb_idx;
    m_ea_n = m_ea;
    r_ea_n = r_ea;
    wb_ea_n = wb_ea;
    m_seg_n = m_seg;
    r_seg_n = r_seg;
    wb_seg_n = wb_seg;
    m_byte_n = m_byte;
    r_byte_n = r_byte;
    wb_byte_n = wb_byte;
    pend_active_n = pend_active;
    pend_off_n = pend_off;
    pend_seg_n = pend_seg;
    pend_byte_n = pend_byte;
    pend_io_n = pend_io;
    opr_fresh_n = opr_fresh;
    opr_loaded_n = opr_loaded;
    rdq0_n = rdq0;
    rdq1_n = rdq1;
    rdq_n_n = rdq_n;
    rd_pending_n = rd_pending;
    ghost_rd_discard_n = ghost_rd_discard;
    ghost_rd_feed_n = ghost_rd_feed;
    ghost_rd_ready_n = ghost_rd_ready;
    rd_done_cnt_n = rd_done_cnt;
    rd_age0_n = rd_age0;
    iend_owed_n = iend_owed;
    rst_ctr_n = rst_ctr;
    tsel_n = tsel;
    pe_opc_reg_n = pe_opc_reg;
    pe_opc8080_n = pe_opc8080;
    pe_op8_n = pe_op8;
    pe_pfxcnt_n = pe_pfxcnt;
    wr_out_n = wr_out;
    opc_valid_n = opc_valid;
    opc_byte_n = opc_byte;
    opc_rm_valid_n = opc_rm_valid;
    opc_rm_byte_n = opc_rm_byte;
    pop_is_first_n = pop_is_first;
    ld_b_n = ld_b;
    ld_pla_n = ld_pla;
    ld_ext_n = ld_ext;
    ld_page_n = ld_page;
    ld_hasrm_n = ld_hasrm;
    ld_rm_n = ld_rm;
    ld_disp_n = ld_disp;
    ld_dlo_n = ld_dlo;
    ld_grpd_n = ld_grpd;
    ld_byte_n = ld_byte;
    ld_preread_n = ld_preread;
    ld_ripe_prev_n = ld_ripe_prev;
    st_n = st;
    chg_n = chg;
    ending_n = ending;
    rowq_n = rowq;
    row_posted_n = row_posted;
    row_paired_n = row_paired;
    rloop_n_n = rloop_n;
    suppress_commit_n = suppress_commit;
    first_pop_seen_n = first_pop_seen;
    rowb0_n = rowb0;
    rowb1_n = rowb1;
    poste_n = poste;
    poll_pipe_n = poll_pipe;

    if (ss_we) begin
        `include "v30u_eu_ss_write.svh"
    end else begin   // <- was `else if (srst)` then `else if (ce)`:
                     //    both selects are on the register bank now
        //====================================================================
        // (a) the BIU's completion pulses, sampled on the clock they ride
        //====================================================================
`ifndef SYNTHESIS
        trc_1bl_hit = 1'b0; trc_pe_hit = 1'b0; trc_1bld_hit = 1'b0;
`endif
        poll_pipe_n = {poll_pipe_n[1:0], pin_poll_n};
        // ...and the IE the gate's own pipeline is about to take, frozen HERE
        // because the chain below may write `psw` (see block (g)).
        ie_now = psw_n[FIE];
        brk_now = psw_n[FBRK];
        // A maskable request present when IE rises is remembered while the
        // existing three-clock IE floor matures.  `ie_now` and `int_p[0]` are
        // the two signals sampled on the same preceding clock; the older
        // boundary tap (`int_p[2]`) would retain a pin that had already fallen.
        if (ie_now && !ie_p_n[0] && int_p[0])
            intr_pending_n = 1'b1;
        // §86 -- THE ARM'S TWO EVENTS, BOTH READ OFF REGISTERS ONLY, AND BOTH
        // BEFORE THE CHAIN.  The SAMPLE lands one clock past the boundary's own
        // F pop (`brk_smp`, raised below on the pop's own clock); the TAKE is
        // in the chain, at the four arms that reach `S_IRQ_D`, and it CLEARS
        // the arm.  Ordering them this way round is what makes a take that
        // coincides with a sample resolve as the model resolves it -- the model
        // takes first and samples second, in one statement:
        //     if (brk_take_) { brk_arm_ = false; } else { brk_arm_ = seen; }
        if (brk_smp) brk_arm_n = brk_seen;
        // ...and the F pop that ENDS this boundary, which is exactly the pop
        // the `QS = 1` pins announce (`q_first`).  A PREFIX retires with one of
        // its own, so this single predicate is §85.3's "the retire boundaries
        // AND the prefix hand-over" with no second term.
        brk_smp_n = q_pop && q_ripe && q_first;
        unhalt_pend_n = 1'b0;
        if (bnd_fire)
            irq_fast_inta_n = bnd_opc || eu_bnd_post;
        if (eu_post && inta_first) begin
            irq_fast_inta_n = 1'b0;
            irq_halt_entry_n = 1'b0;
        end
        if (vector_first) irq_fast_inta_n = 1'b0;
        rd_age0_n = 1'b0;
        if (eu_rd_done_n) begin
            // (the two completed-read SVAs are in the clocked observer below --
            //  a combinational block fires them once per SETTLE, not once per CE
            //  clock.  They read only flop values, which is what they read here
            //  too: nothing above this point touches either counter.)
            // ...AND THE COUNTERS SATURATE RATHER THAN WRAP.  The assertions
            // above live inside `ifndef SYNTHESIS`, so in the BITSTREAM they do
            // not exist: the only thing standing between an over-deep store and
            // silently-wrong behaviour is what the counter itself does.  Before
            // this, `rd_done_cnt` went 3 -> 0 and DROPPED FOUR COMPLETIONS
            // (`nr_have` reads `rd_done_cnt != 0`, so the EU would then wait
            // forever on reads that had already landed), and `rdq_n` went 2 -> 3
            // and handed `rdq1` out twice.  Saturation is the house rule the BIU
            // already follows for every one of its bounded counters -- `sev`
            // (v30u_biu.sv:1377), `cdage` (:1270), `rq_n` (:957, :1287) -- and
            // it is what a store with a fixed number of slots physically does.
            // Inert on every graded path by sec.27.1's proof; load-bearing in
            // fabric, where the in-silicon fuzz runs with no assertions at all.
            if (rd_pending_n != 2'd0) rd_pending_n = rd_pending_n - 2'd1;
            // The mod3 POP read has no result tag.  Ordinarily its completion
            // is discarded.  If a successor read has already claimed the
            // untagged hand-off, however, the ghost word is its head and the
            // younger physical completion becomes the unmatched tail.
            if (ghost_row_tail) ghost_rd_feed_n = 1'b1;
            if (ghost_rd_discard_n &&
                (ghost_preread_tail || ghost_row_tail)) begin
                // The bus has no result tags.  If the younger pre-read has
                // already launched when the old ghost completes, the first
                // word feeds that waiting read and the younger physical word
                // becomes the discarded tail.  This is the same ownership
                // rule as an idle-phase overlap, reached one phase later.
                if (!ghost_rd_feed_n && !opc_valid && eu_slot_busy_n) begin
                    ghost_rd_feed_n = 1'b1;
                    ghost_rd_ready_n = 1'b1;
                end
                if (ghost_rd_feed_n) begin
                    // An idle-phase ghost is the untagged head seen by the
                    // successor.  Store it now; the successor's physical
                    // completion is discarded after its opcode pop below.
                    if (rd_done_cnt_n == 2'd0) rd_age0_n = 1'b1;
                    if (rd_done_cnt_n != 2'd3) rd_done_cnt_n = rd_done_cnt_n + 2'd1;
                    if (rdq_n_n == 2'd0) rdq0_n = eu_rdata_n; else rdq1_n = eu_rdata_n;
                    if (rdq_n_n != 2'd2) rdq_n_n = rdq_n_n + 2'd1;
                end
                // A full-phase ghost that overlaps S_PRERD is discarded here
                // but keeps its token until the opcode pop, preserving the
                // measured pre-read width and QS cadence without feeding OPR.
                // If the younger request was already counted when the ghost
                // landed, its T1 preceded that decision: leave it to the
                // ordinary completion pop and release the ghost token now.
                if (!ghost_rd_feed_n && opc_valid) begin
                    ghost_rd_discard_n = 1'b0;
                    ghost_rd_feed_n = 1'b0;
                    ghost_rd_ready_n = 1'b0;
                end else if (!ghost_rd_feed_n &&
                             (rd_pending_n != 2'd0)) begin
                    // Successor T1 has already passed.  Release completion
                    // ownership so its data lands normally, but remember
                    // that its T3 data edge owes the opcode pop.
                    ghost_rd_discard_n = 1'b0;
                    ghost_rd_feed_n = 1'b0;
                    ghost_rd_ready_n = 1'b1;
                end else if (rd_pending_n == 2'd0) begin
                    ghost_rd_ready_n = 1'b1;
                end
            end else if (ghost_rd_discard_n && (rd_pending_n == 2'd0) &&
                         (st == S_PRERD) && !opc_valid &&
                         (upc_page == 3'd1) && (upc_opc == 8'he4)) begin
                // The E4 overlap's physical tail can land after the younger
                // pre-read has posted.  It owns no data slot, but its presence
                // is the simple enable for that read's early decoder pops.
                ghost_rd_ready_n = 1'b1;
            end else if (ghost_rd_discard_n && (rd_pending_n == 2'd0)) begin
                // No overlap, or the overlapping opcode has already popped:
                // this is an unmatched physical completion.
                ghost_rd_discard_n = 1'b0;
                if ((st == S_NORM_CHG) && !opc_valid) begin
                    // The retained 8F-tail data edge already released the
                    // decoder.  Its later done pulse enables one successor
                    // request status preview.
                    ghost_rd_feed_n = 1'b1;
                    ghost_rd_ready_n = 1'b1;
                end else begin
                    ghost_rd_feed_n = 1'b0;
                    ghost_rd_ready_n = 1'b0;
                end
            end else begin
                if (rd_done_cnt_n == 2'd0) rd_age0_n = 1'b1;
                if (rd_done_cnt_n != 2'd3) rd_done_cnt_n = rd_done_cnt_n + 2'd1;
                if (rdq_n_n == 2'd0) rdq0_n = eu_rdata_n; else rdq1_n = eu_rdata_n;
                if (rdq_n_n != 2'd2) rdq_n_n = rdq_n_n + 2'd1;
            end
        end
        if (eu_wr_done_n && (wr_out_n != 2'd0)) wr_out_n = wr_out_n - 2'd1;
        if (q_pop && q_ripe && q_first && !first_pop_seen_n) first_pop_seen_n = 1'b1;
        // Once an opcode is held, GHOST_READY is no longer completion state:
        // it is a one-clock admission pulse for that opcode's adjacent
        // ModR/M byte.  If no ripe byte is present on that clock, it expires.
        if (ghost_rd_ready && opc_valid) ghost_rd_ready_n = 1'b0;
        if (ghost_rd_ready && ghost_rd_feed && !ghost_rd_discard) begin
            // This status-preview encoding is a one-clock pulse.
            ghost_rd_ready_n = 1'b0;
            ghost_rd_feed_n = 1'b0;
        end
        if (ghost_rm_pop) begin
            // This decoder-side take can ride either the younger pre-read or
            // the predecessor's final micro-row.  Capture it at the shared
            // queue-consumer point so both physical phases clear the arm.
            opc_rm_byte_n = q_byte;
            opc_rm_valid_n = 1'b1;
            ghost_rd_ready_n = 1'b0;
        end

        //--------------------------------------------------------------------
        // ...AND THE FLAG REGISTER IS FED BY THE DATA LATCH, NOT BY THE ROW.
        //--------------------------------------------------------------------
        // A row's destination write-enable is a LEVEL for as long as the row
        // STANDS.  For every other destination that is unobservable -- nothing
        // between the row's arrival and its release can read the register --
        // but FLAGS is wired to the outside world twice over (S5 on the status
        // pins, and the IE gate of the recognition pipeline), so on the two
        // rows whose destination is FLAGS and whose source is the read latch
        // the early load is VISIBLE.  `interrupt_model.md`, verbatim:
        //
        //     "POP PSW consumes the popped image at its read's data edge
        //      (the new IE shows in the PS bits during the read's own T4)"
        //
        // MEASURED, `INT.9D idx 1`: the golden's PS nibble is 5 on row 9 --
        // the read's T4, so the register already held the popped image when
        // that clock OPENED, i.e. it was loaded on the T3 -> T4 edge -- while
        // the `OPR -> FLAGS` row itself does not release until row 10.  Two
        // clocks, and they are exactly the two that decide the NEXT boundary:
        // `ie_p[2]` is IE at c-3, the boundary stands on row 13, and IE has to
        // be up by row 10 for it to be seen.  All 89 failing cases were
        // pre-IE=0 pops -- the 111 pre-IE=1 ones never needed it.
        //
        // The rule names NO OPCODE.  `OPR -> FLAGS ... F` is exactly two ROM
        // rows, 007A (POP PSW) and 01EA (RETI) -- which is the same pair E1
        // measured on silicon ("mu01EA's flag commit obeys the SAME race table
        // as POP PSW's mu007A", 108/108 H-IDENTICAL).  One rule, both rows.
        //
        // The row still performs its own `OPR -> FLAGS` when it releases; the
        // value is the same word, so the commit is idempotent.  (The frozen
        // FSM core renders this as `opc == 8'h9D && eu_rd_now` plus a second
        // copy for `iret_pw`; this is that behaviour with the opcode test and
        // the duplication taken out.)
        // SM3 sitting 12 (R7'): the write that used to stand HERE is now
        // `rd_edge_psw_take` / `rd_edge_psw` on the `psw` register's own `D`
        // pin -- see the declaration beside `row_blocked` for the measurement,
        // the two-case exactness argument and the two falsifiers.  Nothing
        // about the RULE changed; `ie_now` above still reads the un-overridden
        // `psw_n`, exactly as it did when the write was here.

        //====================================================================
        // (b) the post-E row's work, owed to THIS clock: it overlaps the
        //     successor's decode (exec_impl.h's cadence note) and the model
        //     runs it BEFORE the successor's step.
        //====================================================================
        if (poste_n) begin
            poste_n = 1'b0;
`ifndef SYNTHESIS
            trc_pe_hit = 1'b1; trc_pe_pre = psw_n;
            trc_pe_upc = {upc_opc_n, upc_loc_n};
`endif
            `include "v30u_eu_poste.svh"
            if (tmpa_n != tmpa) ea_residue_n = tmpa_n;
            if (tmpb_n != tmpb) ea_pair_valid_n = 1'b0;
`ifndef SYNTHESIS
            trc_pe_post = psw_n;
`endif
        end
        // F22: ...and the successor's latch reset it was standing in front of.
        if (iend_owed_n) begin
            iend_owed_n = 1'b0;
            `include "v30u_eu_iend_late.svh"
        end

`ifndef SYNTHESIS
        //-- the eutrace SNAPSHOT (the $display is in the clocked observer)
        trc_st = st_n;
        trc_upc_page = upc_page_n;
        trc_upc_opc = upc_opc_n;
        trc_upc_loc = upc_loc_n;
        trc_wr_out = wr_out_n;
        trc_pc = pc_n;
        trc_ind = ind_n;
        trc_opr = opr_n;
        trc_opr_fresh = opr_fresh_n;
        trc_pend_active = pend_active_n;
        trc_poste = poste_n;
        trc_rdq_n = rdq_n_n;
        trc_rd_done_cnt = rd_done_cnt_n;
        trc_tmpa = tmpa_n;
        trc_tmpb = tmpb_n;
        trc_tmpc = tmpc_n;
`endif
        stop = 1'b0;
`ifndef SYNTHESIS
        chain_used = 4'd0;
        chain_first = st_n;
`endif
        for (chain = 0; chain < CHAIN_MAX; chain = chain + 4'd1) begin
            if (!stop) begin
`ifndef SYNTHESIS
                chain_used = chain + 4'd1;
`ifdef CHAIN_PROBE
                if (!cp_seen[{chain, st_n}]) begin : cprobe
                    integer fd;
                    cp_seen[{chain, st_n}] = 1'b1;
                    fd = $fopen(`CHAIN_PROBE, "a");
                    $fwrite(fd, "POS %0d ST %0d\n", chain, st_n);
                    $fclose(fd);
                end
`endif
`endif
                // THE FAIL-SAFE.  The 24 position-0-only arms are folded out
                // of positions >= 1 (see v30u_eu_step.svh's header).  If one
                // ever DID stand there the folded copy would assign nothing,
                // `stop` would stay low, and the machine would sit on the same
                // state forever.  This turns that impossible case into a spent
                // clock -- the same failure the CHAIN OVERFLOW `$fatal` names,
                // but survivable in fabric where there is no assertion.
                if ((chain != 4'd0) && !st_zero_ok(st_n)) stop = 1'b1;
                else begin
                    `include "v30u_eu_step.svh"
                end
            end
        end
        // (CHAIN OVERFLOW and the depth tracker are in the clocked observer:
        //  `stop` / `chain_used` / `chain_first` are comb outputs of this block
        //  now, and the check must run ONCE per CE clock on the SETTLED values.)

        //====================================================================
        // (g) THE PIN PIPELINES, ADVANCED AT THE *END* OF THE EDGE
        //====================================================================
        // Not a style choice.  These registers are read from BOTH sides of the
        // module -- by the combinational act decode (`bnd_fire` gates `q_pop`)
        // and by the clocked step -- and the two MUST see the same clock, or
        // the demand and the take drift (F11, again).  Shifting them in block
        // (a) put the clock-c+1 view in front of the step while the act decode
        // still had clock c: MEASURED, `HLT.NMI` woke one clock early on every
        // case (entry at A+6 where the golden has A+7), because the step read
        // `nmi_latch` the moment block (a) set it.
        //
        // Advanced here, the registers carry the clock-c view for the WHOLE
        // edge and nothing depends on read order.  (The trap that made this
        // necessary: a wire that is a PURE ALIAS of a register -- `wire w = r;`
        // -- is substituted by the simulator, so it is NOT the pre-edge view
        // the rest of this module gets from a wire with real logic in it.
        // F11b's trap, third form.)
        int_p_n = {int_p_n[2:0], pin_int};
        nmi_p_n = {nmi_p_n[3:0], pin_nmi};
        ie_p_n  = {ie_p_n[2:0], ie_now};
        // §86 -- `psw[FBRK]` on the SAME pipeline, frozen at the same instant
        // and shifted in the same place, because it answers the same kind of
        // question: a flag written into the PSW reaches a recognition decision
        // through flops, and the arm may not act on one that rose too recently.
        brk_p_n = BRK_FLOOR'({brk_p_n, brk_now});
        // ...AND THE SAMPLE INSTANT IS ONE CLOCK PAST THE OPCODE POP.  Not a
        // choice: §85.2a MEASURED it in the model, engine-vs-chip with no trap
        // in the loop, by pairing every `brk_retire` clock with the chip's own
        // `QS = 1` pops on TF-CLEAR captures -- `90 9D 8B 8E B8 E7 CF` at
        // pop + 1, 459 of 459, and `F8` at pop + 0 until that path was
        // corrected, 70 of 70 at both wait levels, 0 violators over 2,900
        // boundaries afterwards.  `brk_smp` is that clock and nothing else.
        //
        // AND THIS IS WHY THERE IS NO PREFIX SPECIAL CASE.  §85.3 asks for the
        // retire boundaries AND the prefix hand-over; `q_first` is ONE
        // predicate that is both, because a prefix retires as its own 2-clock
        // instruction with its own F pop (`prefix_retire()` -> `pop_is_first`)
        // and the `0F` escape's first byte does too.  "A PREFIX BYTE ENDS AN
        // INSTRUCTION BOUNDARY" is already what the pop stream says.  Contrast
        // `bnd_armed`, which the INT recognition needs precisely to EXCLUDE the
        // prefix hand-over ("the measured *no sample between 26 and 8B*"):
        // sample and take are different events at the same boundary, and the
        // two lines that say so live in block (a) above, BEFORE the chain, so
        // that a boundary's own TAKE (which clears the arm inside the chain)
        // can never be overwritten by a sample landing on the same clock.

        // the NMI LATCH is an EDGE, set three clocks after it: `nmi_p[3]` is
        // the pin at c-3 and `nmi_p[4]` the pin at c-4, so the latch reads true
        // from c+1 = edge+4 -- "latest catching edge = B-4".
        if (nmi_p_n[3] && !nmi_p_n[4]) nmi_latch_n = 1'b1;
    end
end

//--------------------------------------------------------------------------
// THE COMMIT -- the ONLY place an EU state register is written, and the only
// place `ce` appears.  This is the clock-enable port.
//--------------------------------------------------------------------------
always @(posedge clk) begin
    //-- ss_we > srst > ce, exactly the priority the one clocked block had
    if (ss_we || srst || ce) begin
        for (ci = 0; ci < 8; ci = ci + 1)
            gpr[ci] <= (srst && !ss_we) ? gpr_r[ci] : gpr_n[ci];
        for (ci = 0; ci < 4; ci = ci + 1)
            sreg[ci] <= (srst && !ss_we) ? sreg_r[ci] : sreg_n[ci];
        pc <= (srst && !ss_we) ? pc_r : pc_n;
        // SM3 sitting 12 (R7'): the read's data edge is applied HERE, on the
        // `D` pin, instead of at the head of the chain.  Priority is unchanged
        // -- `ss_we` > `srst` > the data edge > the chain -- because the old
        // write sat inside the `else` of `if (ss_we)` and `srst` took `psw_r`
        // regardless of it.
        psw <= (srst && !ss_we)             ? psw_r
             : (rd_edge_psw_take && !ss_we) ? rd_edge_psw
             :                                psw_n;
        tmpa <= (srst && !ss_we) ? tmpa_r : tmpa_n;
        tmpb <= (srst && !ss_we) ? tmpb_r : tmpb_n;
        tmpc <= (srst && !ss_we) ? tmpc_r : tmpc_n;
        ea_residue <= (srst && !ss_we) ? ea_residue_r : ea_residue_n;
        ea_pair_rhs <= (srst && !ss_we) ? ea_pair_rhs_r : ea_pair_rhs_n;
        ea_pair_valid <= (srst && !ss_we) ? ea_pair_valid_r : ea_pair_valid_n;
        opr <= (srst && !ss_we) ? opr_r : opr_n;
        ind <= (srst && !ss_we) ? ind_r : ind_n;
        count <= (srst && !ss_we) ? count_r : count_n;
        pfxcnt <= (srst && !ss_we) ? pfxcnt_r : pfxcnt_n;
        stat <= (srst && !ss_we) ? stat_r : stat_n;
        sign_neg <= (srst && !ss_we) ? sign_neg_r : sign_neg_n;
        bit_n <= (srst && !ss_we) ? bit_n_r : bit_n_n;
        al_op <= (srst && !ss_we) ? al_op_r : al_op_n;
        al_tmp <= (srst && !ss_we) ? al_tmp_r : al_tmp_n;
        al_byte <= (srst && !ss_we) ? al_byte_r : al_byte_n;
        al_eaconst <= (srst && !ss_we) ? al_eaconst_r : al_eaconst_n;
        al_eaval <= (srst && !ss_we) ? al_eaval_r : al_eaval_n;
        al_adjust <= (srst && !ss_we) ? al_adjust_r : al_adjust_n;
        al_adjtmp <= (srst && !ss_we) ? al_adjtmp_r : al_adjtmp_n;
        al_bitarm <= (srst && !ss_we) ? al_bitarm_r : al_bitarm_n;
        al_bitn <= (srst && !ss_we) ? al_bitn_r : al_bitn_n;
        al_spent <= (srst && !ss_we) ? al_spent_r : al_spent_n;
        upc_page <= (srst && !ss_we) ? upc_page_r : upc_page_n;
        upc_opc <= (srst && !ss_we) ? upc_opc_r : upc_opc_n;
        upc_loc <= (srst && !ss_we) ? upc_loc_r : upc_loc_n;
        seg_override <= (srst && !ss_we) ? seg_override_r : seg_override_n;
        seg_ovr <= (srst && !ss_we) ? seg_ovr_r : seg_ovr_n;
        rep_kind <= (srst && !ss_we) ? rep_kind_r : rep_kind_n;
        lock_pfx <= (srst && !ss_we) ? lock_pfx_r : lock_pfx_n;
        opc_reg <= (srst && !ss_we) ? opc_reg_r : opc_reg_n;
        op8 <= (srst && !ss_we) ? op8_r : op8_n;
        imm8 <= (srst && !ss_we) ? imm8_r : imm8_n;
        opc_base <= (srst && !ss_we) ? opc_base_r : opc_base_n;
        opc_from_modrm <= (srst && !ss_we) ? opc_from_modrm_r : opc_from_modrm_n;
        modrm_reg <= (srst && !ss_we) ? modrm_reg_r : modrm_reg_n;
        xop <= (srst && !ss_we) ? xop_r : xop_n;
        rep_test <= (srst && !ss_we) ? rep_test_r : rep_test_n;
        rep_pol <= (srst && !ss_we) ? rep_pol_r : rep_pol_n;
        bus_word <= (srst && !ss_we) ? bus_word_r : bus_word_n;
        opc8080 <= (srst && !ss_we) ? opc8080_r : opc8080_n;
        mode8080 <= (srst && !ss_we) ? mode8080_r : mode8080_n;
        intr_pending <= (srst && !ss_we) ? intr_pending_r : intr_pending_n;
        eu_halted <= (srst && !ss_we) ? eu_halted_r : eu_halted_n;
        int_p <= (srst && !ss_we) ? int_p_r : int_p_n;
        nmi_p <= (srst && !ss_we) ? nmi_p_r : nmi_p_n;
        nmi_latch <= (srst && !ss_we) ? nmi_latch_r : nmi_latch_n;
        ie_p <= (srst && !ss_we) ? ie_p_r : ie_p_n;
        rep_chain <= (srst && !ss_we) ? rep_chain_r : rep_chain_n;
        irq_shadow <= (srst && !ss_we) ? irq_shadow_r : irq_shadow_n;
        bnd_armed <= (srst && !ss_we) ? bnd_armed_r : bnd_armed_n;
        irq_sel_nmi <= (srst && !ss_we) ? irq_sel_nmi_r : irq_sel_nmi_n;
        irq_sel_brk <= (srst && !ss_we) ? irq_sel_brk_r : irq_sel_brk_n;
        brk_p <= (srst && !ss_we) ? brk_p_r : brk_p_n;
        brk_arm <= (srst && !ss_we) ? brk_arm_r : brk_arm_n;
        brk_smp <= (srst && !ss_we) ? brk_smp_r : brk_smp_n;
        unhalt_pend <= (srst && !ss_we) ? unhalt_pend_r : unhalt_pend_n;
        irq_fast_inta <= (srst && !ss_we) ? irq_fast_inta_r
                                          : irq_fast_inta_n;
        irq_halt_entry <= (srst && !ss_we) ? irq_halt_entry_r
                                           : irq_halt_entry_n;
        m_kind <= (srst && !ss_we) ? m_kind_r : m_kind_n;
        r_kind <= (srst && !ss_we) ? r_kind_r : r_kind_n;
        wb_kind <= (srst && !ss_we) ? wb_kind_r : wb_kind_n;
        m_idx <= (srst && !ss_we) ? m_idx_r : m_idx_n;
        r_idx <= (srst && !ss_we) ? r_idx_r : r_idx_n;
        wb_idx <= (srst && !ss_we) ? wb_idx_r : wb_idx_n;
        m_ea <= (srst && !ss_we) ? m_ea_r : m_ea_n;
        r_ea <= (srst && !ss_we) ? r_ea_r : r_ea_n;
        wb_ea <= (srst && !ss_we) ? wb_ea_r : wb_ea_n;
        m_seg <= (srst && !ss_we) ? m_seg_r : m_seg_n;
        r_seg <= (srst && !ss_we) ? r_seg_r : r_seg_n;
        wb_seg <= (srst && !ss_we) ? wb_seg_r : wb_seg_n;
        m_byte <= (srst && !ss_we) ? m_byte_r : m_byte_n;
        r_byte <= (srst && !ss_we) ? r_byte_r : r_byte_n;
        wb_byte <= (srst && !ss_we) ? wb_byte_r : wb_byte_n;
        pend_active <= (srst && !ss_we) ? pend_active_r : pend_active_n;
        pend_off <= (srst && !ss_we) ? pend_off_r : pend_off_n;
        pend_seg <= (srst && !ss_we) ? pend_seg_r : pend_seg_n;
        pend_byte <= (srst && !ss_we) ? pend_byte_r : pend_byte_n;
        pend_io <= (srst && !ss_we) ? pend_io_r : pend_io_n;
        opr_fresh <= (srst && !ss_we) ? opr_fresh_r : opr_fresh_n;
        opr_loaded <= (srst && !ss_we) ? opr_loaded_r : opr_loaded_n;
        rdq0 <= (srst && !ss_we) ? rdq0_r : rdq0_n;
        rdq1 <= (srst && !ss_we) ? rdq1_r : rdq1_n;
        rdq_n <= (srst && !ss_we) ? rdq_n_r : rdq_n_n;
        rd_pending <= (srst && !ss_we) ? rd_pending_r : rd_pending_n;
        ghost_rd_discard <= (srst && !ss_we) ? ghost_rd_discard_r
                                             : ghost_rd_discard_n;
        ghost_rd_feed <= (srst && !ss_we) ? ghost_rd_feed_r
                                          : ghost_rd_feed_n;
        ghost_rd_ready <= (srst && !ss_we) ? ghost_rd_ready_r
                                           : ghost_rd_ready_n;
        rd_done_cnt <= (srst && !ss_we) ? rd_done_cnt_r : rd_done_cnt_n;
        rd_age0 <= (srst && !ss_we) ? rd_age0_r : rd_age0_n;
        iend_owed <= (srst && !ss_we) ? iend_owed_r : iend_owed_n;
        rst_ctr <= (srst && !ss_we) ? rst_ctr_r : rst_ctr_n;
        tsel <= (srst && !ss_we) ? tsel_r : tsel_n;
        pe_opc_reg <= (srst && !ss_we) ? pe_opc_reg_r : pe_opc_reg_n;
        pe_opc8080 <= (srst && !ss_we) ? pe_opc8080_r : pe_opc8080_n;
        pe_op8 <= (srst && !ss_we) ? pe_op8_r : pe_op8_n;
        pe_pfxcnt <= (srst && !ss_we) ? pe_pfxcnt_r : pe_pfxcnt_n;
        wr_out <= (srst && !ss_we) ? wr_out_r : wr_out_n;
        opc_valid <= (srst && !ss_we) ? opc_valid_r : opc_valid_n;
        opc_byte <= (srst && !ss_we) ? opc_byte_r : opc_byte_n;
        opc_rm_valid <= (srst && !ss_we) ? opc_rm_valid_r : opc_rm_valid_n;
        opc_rm_byte <= (srst && !ss_we) ? opc_rm_byte_r : opc_rm_byte_n;
        pop_is_first <= (srst && !ss_we) ? pop_is_first_r : pop_is_first_n;
        ld_b <= (srst && !ss_we) ? ld_b_r : ld_b_n;
        ld_pla <= (srst && !ss_we) ? ld_pla_r : ld_pla_n;
        ld_ext <= (srst && !ss_we) ? ld_ext_r : ld_ext_n;
        ld_page <= (srst && !ss_we) ? ld_page_r : ld_page_n;
        ld_hasrm <= (srst && !ss_we) ? ld_hasrm_r : ld_hasrm_n;
        ld_rm <= (srst && !ss_we) ? ld_rm_r : ld_rm_n;
        ld_disp <= (srst && !ss_we) ? ld_disp_r : ld_disp_n;
        ld_dlo <= (srst && !ss_we) ? ld_dlo_r : ld_dlo_n;
        ld_grpd <= (srst && !ss_we) ? ld_grpd_r : ld_grpd_n;
        ld_byte <= (srst && !ss_we) ? ld_byte_r : ld_byte_n;
        ld_preread <= (srst && !ss_we) ? ld_preread_r : ld_preread_n;
        ld_ripe_prev <= (srst && !ss_we) ? ld_ripe_prev_r : ld_ripe_prev_n;
        st <= (srst && !ss_we) ? st_r : st_n;
        chg <= (srst && !ss_we) ? chg_r : chg_n;
        ending <= (srst && !ss_we) ? ending_r : ending_n;
        rowq <= (srst && !ss_we) ? rowq_r : rowq_n;
        row_posted <= (srst && !ss_we) ? row_posted_r : row_posted_n;
        row_paired <= (srst && !ss_we) ? row_paired_r : row_paired_n;
        rloop_n <= (srst && !ss_we) ? rloop_n_r : rloop_n_n;
        suppress_commit <= (srst && !ss_we) ? suppress_commit_r : suppress_commit_n;
        first_pop_seen <= (srst && !ss_we) ? first_pop_seen_r : first_pop_seen_n;
        rowb0 <= (srst && !ss_we) ? rowb0_r : rowb0_n;
        rowb1 <= (srst && !ss_we) ? rowb1_r : rowb1_n;
        poste <= (srst && !ss_we) ? poste_r : poste_n;
        poll_pipe <= (srst && !ss_we) ? poll_pipe_r : poll_pipe_n;
    end
end

`ifndef SYNTHESIS
//--------------------------------------------------------------------------
// THE CLOCKED OBSERVER -- everything that must happen ONCE PER CE CLOCK.
//--------------------------------------------------------------------------
// A combinational block fires its side effects once per SETTLE, not once per
// clock, so none of this can live in the next-state function: `sw/uscope.py`'s
// contract is `row index == CE clock index == +eutrace line number`, and a
// $fatal on a transient value would take a run down for a state the machine
// never stood in.  Everything read here is either a FLOP or a comb value that
// has SETTLED by the edge, which is the same value the old in-line code read.
always @(posedge clk) begin
    if (srst) ce_clk <= 0;
    else if (ce && !ss_we) ce_clk <= ce_clk + 1;
    if (ce && !srst && !ss_we) begin
        // campaign risk #2 / F48: the completed-read store's bound.  Read off
        // the flops, which is what the old in-line assertions read too --
        // nothing before block (a) touches either counter.
        if (eu_rd_done_n) begin
            assert (rdq_n != 2'd2)
                else $warning("v30u_eu: completed-read store overflow (rdq_n=2)");
            assert (rd_done_cnt != 2'd3)
                else $warning("v30u_eu: rd_done_cnt saturated");
        end
        // SM3 sitting 12 (R7') -- THE TWO FALSIFIERS FOR THE DATA-EDGE PSW
        // MOVE (`sm3_s12b_prereg_2026-08-04.md` §3).  They are on the RAW take
        // -- without the `row_blocked` term -- so they see every clock the old
        // block-(a) write would have fired on.  They STAY: they are the only
        // thing between this pass and a silent behavioural change if the F
        // interlock's shape, or the `OPR -> FLAGS` row's own act set, ever
        // moves.
        if (rd_edge_take_raw && row_blocked)
            assert (!poste && !iend_owed)
                else $error("v30u_eu: R7' falsifier (A): data-edge PSW load with the chain stopped but poste=%0d iend_owed=%0d owed -- they read-modify-write psw_n between the old write site and the commit, so the D-pin form is NOT equivalent here (upc=%0d.%02X.%0d)",
                            poste, iend_owed, upc_page, upc_opc, upc_loc);
        if (rd_edge_take_raw && !row_blocked)
            assert (row_acts_ok && e_have1)
                else $error("v30u_eu: R7' falsifier (B): data-edge PSW load with the chain RUNNING but the row not performing its own OPR->FLAGS (row_acts_ok=%0d e_have1=%0d) -- the deleted block-(a) write was NOT dead here (f_wait=%0d nr_wait=%0d rd_done_cnt=%0d rd_pending=%0d upc=%0d.%02X.%0d)",
                            row_acts_ok, e_have1, f_wait, nr_wait,
                            rd_done_cnt, rd_pending,
                            upc_page, upc_opc, upc_loc);
        // v30u_eu_poste.svh's two shape assertions.  `poste` is not touched
        // before the post-E block, so the flop IS the value that guarded them.
        if (poste && row_bus)
            $error("v30u_eu: a post-E row carries a bus cycle (upc %0d.%02X.%0d)",
                   upc_page, upc_opc, upc_loc);
        if (poste && (row_q1 || row_q2))
            $error("v30u_eu: a post-E row pops a queue byte");
        if (brktrace && trc_pe_hit)
            $display("PE  clk=%0d pre=%04x post=%04x upc=%03X",
                     ce_clk, trc_pe_pre, trc_pe_post, trc_pe_upc);
        if (brktrace && trc_1bl_hit)
            $display("1BL clk=%0d pre=%04x post=%04x psw=%04x pswn=%04x",
                     ce_clk, trc_1bl_pre, trc_1bl_post, psw, psw_n);
        // wrfuzz W3.5: the 1BL DECODE, with the four bits a lead gate could
        // read.  `smp` is `brk_smp_n` -- "the opcode pop rode THIS clock, so
        // the arm is sampled at the END of the next one".
        if (brktrace && trc_1bld_hit)
            $display("1BLD clk=%0d ripe_lead=%0d seen=%0d arm=%0d smp=%0d shd=%0d",
                     ce_clk, trc_1bld_ripe, trc_1bld_seen, trc_1bld_arm,
                     trc_1bld_smp, trc_1bld_shd);
        if (brktrace) begin
            // the RISE stamp, in the same sense the model's `sample_ie()`
            // means it: at THIS clock the PSW carries TF and at the previous
            // one it did not.
            if (brk_now && !brk_p[0])
                $display("BRKR clk=%0d", ce_clk);
            if (brk_smp)
                $display("BRKS clk=%0d seen=%0d psw_brk=%0d brk_p=%0d arm=%0d",
                         ce_clk, brk_seen, psw[FBRK], brk_p, brk_arm);
            if (bnd_fire)
                $display("BRKT clk=%0d arm=%0d irq=%0d sel_brk=%0d",
                         ce_clk, brk_arm, irq_take, !irq_take);
        end
        if (eutrace)
            $display("EU st=%0d upc=%0d.%02X.%0d row=%07x q=%02x ripe=%0d slot=%0d post=%0d bs=%0d a=%05x pair=%0d wd=%04x rdd=%0d wrd=%0d oprf=%0d wr_out=%0d pc=%04x ind=%04x opr=%04x of=%0d pnd=%0d pe=%0d rdq=%0d rdc=%0d a=%04x b=%04x c=%04x sig=%04x pfx=%0d",
                     trc_st, trc_upc_page, trc_upc_opc, trc_upc_loc, row, q_byte, q_ripe,
                     eu_slot_busy_n, eu_post, eu_bs, eu_addr, eu_pair,
                     eu_wdata,
                     eu_rd_done_n, eu_wr_done_n, eu_opr_free, trc_wr_out, trc_pc,
                     trc_ind, trc_opr, trc_opr_fresh, trc_pend_active, trc_poste, trc_rdq_n,
                     trc_rd_done_cnt, trc_tmpa, trc_tmpb, trc_tmpc, sigma, pfxcnt_eff);
        // THE CHAIN BOUND IS A CLAIM, SO IT IS CHECKED.  `CHAIN_MAX` is the
        // number of ZERO-COST model steps that may ride one clock; running out
        // while `stop` is still low would silently push the remainder into the
        // NEXT clock -- a cadence error, not a hang, and therefore invisible
        // without this line.
        if (!stop)
            $fatal(1, "v30u_eu: CHAIN OVERFLOW at CHAIN_MAX=%0d (entered in st=%0d, now st=%0d)",
                   CHAIN_MAX, chain_first, st_n);
        if (chain_used > chain_hi) begin
            chain_hi = chain_used;
            if (chain_report)
                $display("CHAIN_DEPTH_MAX %0d entry_st %0d", chain_hi, chain_first);
            `ifdef CHAIN_PROBE
            begin : probe
                integer fd;
                fd = $fopen(`CHAIN_PROBE, "a");
                $fwrite(fd, "%0d %0d\n", chain_hi, chain_first);
                $fclose(fd);
            end
            `endif
        end
    end
end
`endif

//============================================================================
// save-state READ mux (arm #2 of the exactly-twice discipline)
//============================================================================
always @(posedge clk) begin
    `include "v30u_eu_ss_read.svh"
end

wire _unused_eu = &{1'b0, q_cnt, halted, lock_pfx, nmi_p[2:0], int_p[0],
                    int_p[3], ie_p[3], ie_p[1:0],
                    ar_full, ar_m, row_paired, imm8,
                    r_ictl, r_ectl, r_sr, r_f, r_w, r_e, r_type, r_nopmove,
                    r_hasconst, r_s1, r_d1, r_s2, r_d2, r_r, dec_valid,
                    SR_ES, SR_SS, SR_DS, R_CW, R_DW, R_SP, BS_PASV,
                    A_SHL6, A_NOT, A_PASS, C_OPC, I_SUSP, I_FLUSH, E_MEMR,
                    TEST_NONE, tk, ti, ts, te, tb, ea, rseg, nloc};

endmodule
