//============================================================================
// Imported from nu8051 (machine-cycle-accurate Intel 8051/8052 core, SystemVerilog)
// Source: rtl/nu8051_uart.sv, commit ad28e1b655ec7a38054174beaed157f013ea5643
// Do not hand-edit; re-import from upstream. The M72 integration wrapper is
// rtl/mcu.sv (distilled from nu8051 rtl/m72/mcu.sv), NOT a raw re-import.
//============================================================================
//============================================================================
//
//  nu8051_uart - serial port: SCON/SBUF + the four mode engines
//                (core_design.md §6.4, periph §5, savestate_design §2.7)
//
//  C5.4 SCOPE: all four modes, transmit and receive, at the instants periph
//  §5 states.  MAME is NOT the oracle for anything bit-timing here (PQ12):
//  its mode-0 output is "highly simplified and incorrect" by its own comment,
//  its mode-0 input is not emulated at all, and its receiver samples once per
//  bit with no 2-of-3 majority.  Every instant below cites the manual.
//
//  The two physical SBUF registers (periph §5.1): a write hits the transmit
//  side, a read returns the receive side.
//
//  Time bases, one per mode family:
//
//    mode 0     the machine cycle itself (fosc/12).  Shift at S6P2, mode-0
//               RXD sample at S5P2, TXD carries the SHIFT CLOCK: low during
//               S3-S5, high during S6-S2, i.e. transitions AT S3P1 and S6P1
//               (periph §5.5) - which in this core's edge convention means
//               txd_r is driven low by the edge ENDING S2P2 and high by the
//               edge ENDING S5P2, so the recorded pin value is low from S3P1
//               and high from S6P1.
//
//    modes 1/3  timer-1 overflow -> SMOD divide-by-2 -> divide-by-16
//               (periph §5.4).  `t1_ovf` is the timer's overflow pulse.
//               Figure 15's mode-1/3 maximum - 62.5K baud at 12 MHz with
//               SMOD = 1 and timer 1 in mode 2 reloading FFH - is 16
//               machine cycles per bit, and that is this suite's frame.
//
//    mode 2     phase-2 clock (fosc/2, one tick per STATE = every P2 phase)
//               -> SMOD-bypassed ÷2 -> ÷16 = fosc/64 or fosc/32 (periph
//               §5.4, Figure 19).  The chain keeps full oscillator
//               resolution here: a bit time is 5⅓ or 2⅔ machine cycles,
//               NOT MAME's 3-or-6-counts-per-machine-cycle approximation.
//               Figure 15's mode-2 maximum is 375K at 12 MHz = 32 osc.
//
//  In modes 1, 2 and 3 alike the ÷16 rollover lands inside a machine cycle
//  while the manual says transmission "commences at S1P1 of the machine
//  cycle FOLLOWING the next rollover" (3-19 and 3-20 say it in the same
//  words; Figures 18/19/20 annotate SEND's edge "S1P1").  `tx_bnd_pend`
//  carries the rollover to that boundary - the S6P2 edge, after which S1P1
//  is what the pin shows.  It is the one field this module holds beyond
//  savestate_design §2.7's 19-row inventory: QUESTION-P54-1.
//
//  RCLK/TCLK (8052 timer-2 baud, periph §5.8, work package C5.5): in modes 1
//  and 3 the receive ÷16 clock takes the timer-2 overflow when RCLK = 1 and
//  the transmit one when TCLK = 1, INDEPENDENTLY (Figure 16 - "the baud rates
//  for transmit and receive can be simultaneously different", 3-16), and the
//  timer-2 branch has NO divide-by-2: the SMOD stage sits only in the timer-1
//  branch, so a timer-2-clocked baud is the overflow rate ÷16 flat.  Modes 0
//  and 2 have their own fixed chains and ignore both bits.
//
//  Reset (§7.2): SCON 00H, SBUF rx read value 00H (PQ6, pinned), txd_r = 1.
//
//============================================================================

`timescale 1ns/1ps

module nu8051_uart (
    input  logic        CLK,
    input  logic        CE,
    input  logic        rst_hold,
    input  logic        prime_cycle,      // [D-10]: no engine advance there

    input  logic  [3:0] ph,               // §6.1 grants the UART `ph` itself
    input  logic        tk_s5p2,
    input  logic        tk_s6p2,

    input  logic        smod,             // PCON.7 (periph §7)
    input  logic        t1_ovf,           // timer-1 overflow pulse (S3P1)
    // C5.5: the 8052's timer-2 baud source (periph §5.8 / Figure 16).  RCLK
    // and TCLK are INDEPENDENT selects - "the baud rates for transmit and
    // receive can be simultaneously different" - and the timer-2 branch has
    // no SMOD divide-by-2 stage.  All three read 0 on a P_8052 = 0 build.
    input  logic        rclk, tclk,
    input  logic        t2_ovf,
    input  logic        RXD_IN,

    input  logic  [7:0] sfr_addr,
    output logic  [7:0] sfr_rdata,
    output logic        sfr_hit,
    input  logic        sfr_we,
    input  logic  [7:0] sfr_wdata,

    output logic        ri_q, ti_q,      // to the irq unit (periph §6.1)
    output logic        txd_r,           // D1.2 F-6 (mapped state), idle = 1
    output logic        rxd_out,
    output logic        rxd_oe,

    // save state (savestate_design §5.1/§5.2, map 0x0C0-0x0D3)
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

    // ---- state (savestate_design §2.7) ------------------------------------
    logic [7:0] scon_r, sbuf_tx_r, sbuf_rx_r;
    logic [8:0] tx_shift, rx_shift;
    logic [3:0] tx_bitcnt, rx_bitcnt, tx_div16, rx_div16;
    logic       tx_send, tx_dataf, tx_req;
    logic       rx_recv, rx_pend, rxd_prev;
    logic       smod_div, m2_div2;
    logic [1:0] rx_maj;
    logic       tx_bnd_pend;                       // SSA_U_TX_BND (E-2)

    // ---- SCON decode (periph §5.2) ----------------------------------------
    wire [1:0] mode = scon_r[7:6];                 // {SM0, SM1}
    wire       m0   = (mode == 2'd0);              // shift register, fosc/12
    wire       m2   = (mode == 2'd2);              // 9-bit UART, fosc/32|64
    wire       m9   = mode[1];                     // modes 2/3 carry a 9th bit
    wire       ren  = scon_r[SCON_REN];
    wire       sm2  = scon_r[SCON_SM2];

    assign ri_q    = scon_r[SCON_RI];
    assign ti_q    = scon_r[SCON_TI];

    // §6.4: RXD_OUT/RXD_OE are combinational from the tx shift/SEND state -
    // no registers.  SEND "enables SBUF's output onto the P3.0 alternate
    // output line" in mode 0 only (periph §5.5); in modes 1-3 RXD is an
    // input and the data goes out on TXD.
    assign rxd_out = tx_shift[0];
    assign rxd_oe  = m0 && tx_send;

    // ---- SFR bus ----------------------------------------------------------
    always_comb begin
        sfr_hit = 1'b1;
        case (sfr_addr)
            SFR_SCON: sfr_rdata = scon_r;
            SFR_SBUF: sfr_rdata = sbuf_rx_r;       // read -> receive register
            default:  begin sfr_rdata = 8'h00; sfr_hit = 1'b0; end
        endcase
    end

    wire hit_scon = (sfr_addr == SFR_SCON);
    wire hit_sbuf = (sfr_addr == SFR_SBUF);
    wire wr_scon  = sfr_we && hit_scon;            // sfr_we implies S6P2 (TQ6)
    wire wr_sbuf  = sfr_we && hit_sbuf;
    // The mode-0 receive arm is a CONDITION on SCON ("REN = 1 and RI = 0",
    // periph §5.5), and the write that clears RI establishes it AT its own
    // S6P2 - the load then happens "at S6P2 of the next machine cycle".  So
    // the arm test reads the post-write byte, not the register.
    wire [7:0] scon_nx = wr_scon ? sfr_wdata : scon_r;

    // ---- baud generation (periph §5.4/§5.8, Figures 16/19) -----------------
    // Figure 16 is the whole mechanism in one picture: the timer-1 overflow
    // goes through the SMOD divide-by-2, the timer-2 overflow does not, and
    // each of the two ÷16 clocks picks its own source with RCLK (receive) or
    // TCLK (transmit).  Mode 2 is off this diagram entirely - its rate is
    // fixed at fosc/32|64 by Figure 19's own chain - and mode 0 is fosc/12,
    // so RCLK/TCLK are read only on the modes-1/3 path.
    wire osc_p2  = ph[0];                    // fosc/2: one tick per STATE
    wire bd_t1   = t1_ovf && (smod || smod_div);
    wire bd_m2   = osc_p2 && (smod || m2_div2);
    wire bd_rx   = m2 ? bd_m2 : (rclk ? t2_ovf : bd_t1);
    wire bd_tx   = m2 ? bd_m2 : (tclk ? t2_ovf : bd_t1);

    // TX bit event.  "Transmission actually commences at S1P1 of the machine
    // cycle FOLLOWING the next rollover in the divide-by-16 counter" - the
    // same sentence appears in the mode-1 and the mode-2/3 text, and Figures
    // 18/19/20 all annotate SEND's activation edge "S1P1".  So the rollover
    // is carried to the next machine-cycle boundary in every mode: the
    // divider itself keeps full oscillator resolution (mode 2 really is the
    // ÷2·÷16 chain, not MAME's 3-or-6-counts-per-machine-cycle
    // approximation), and only the TXD transition is snapped to S1P1.
    // The fastest bit time in any mode is 2⅔ machine cycles (mode 2,
    // SMOD = 1), so a rollover can never be lost behind an unconsumed one.
    wire tx_roll = bd_tx && (tx_div16 == 4'd15);
    wire tx_bit  = tk_s6p2 && tx_bnd_pend;

    // RX ÷16 state numbering: the tick that detects the 1-to-0 transition is
    // state 0 (the counter "is immediately reset", periph §5.7), so state N
    // is the tick whose POST-increment value is N - samples at 7/8/9.
    wire [3:0] rx_state = rx_div16 + 4'd1;
    wire       rx_smp   = RXD_IN;
    wire       rx_maj_v = ((rx_maj[0] ? 2'd1 : 2'd0) + (rx_maj[1] ? 2'd1 : 2'd0)
                          + (rx_smp ? 2'd1 : 2'd0)) >= 2'd2;
    // "The receive portion is exactly the same as in Mode 1" (3-20): ten
    // accepted bits either way - start + 8 data + stop (mode 1) or start +
    // 9 data (modes 2/3) - and the final shift loads SBUF/RB8 and decides RI
    // in both.  `rx_bitcnt` counts ACCEPTED bits, so the final shift is the
    // one taken with rx_bitcnt = 9.  The only difference is what happens
    // afterwards: mode 1 re-arms "at this time", modes 2/3 "one bit time
    // later" (rx_bitcnt = 10 is that wait).  QUESTION-P54-4: this puts the
    // modes-2/3 RI one bit time EARLIER than periph §5.2/§5.7 say - the
    // figures are unambiguous (no bit-detector sample and no shift pulse
    // anywhere in the modes-2/3 stop bit) and MAME sets it a bit time late.
    localparam logic [3:0] RX_LAST = 4'd9;
    wire [8:0] rx_shf = {rx_maj_v, rx_shift[8:1]};

    // ---- save-state read mux (registered, savestate_design §5.1) ----------
    always_ff @(posedge CLK) begin
        case (ss_addr)
            SSA_U_SCON:      ss_rdata <= {8'b0, scon_r};
            SSA_U_SBUF_TX:   ss_rdata <= {8'b0, sbuf_tx_r};
            SSA_U_SBUF_RX:   ss_rdata <= {8'b0, sbuf_rx_r};
            SSA_U_TX_SHIFT:  ss_rdata <= {7'b0, tx_shift};
            SSA_U_TX_BITCNT: ss_rdata <= {12'b0, tx_bitcnt};
            SSA_U_TX_SEND:   ss_rdata <= {15'b0, tx_send};
            SSA_U_TX_DATAF:  ss_rdata <= {15'b0, tx_dataf};
            SSA_U_TX_REQ:    ss_rdata <= {15'b0, tx_req};
            SSA_U_TX_DIV16:  ss_rdata <= {12'b0, tx_div16};
            SSA_U_RX_DIV16:  ss_rdata <= {12'b0, rx_div16};
            SSA_U_SMOD_DIV:  ss_rdata <= {15'b0, smod_div};
            SSA_U_M2_DIV2:   ss_rdata <= {15'b0, m2_div2};
            SSA_U_RX_SHIFT:  ss_rdata <= {7'b0, rx_shift};
            SSA_U_RX_BITCNT: ss_rdata <= {12'b0, rx_bitcnt};
            SSA_U_RX_RECV:   ss_rdata <= {15'b0, rx_recv};
            SSA_U_RX_PEND:   ss_rdata <= {15'b0, rx_pend};
            SSA_U_RXD_PREV:  ss_rdata <= {15'b0, rxd_prev};
            SSA_U_RX_MAJ:    ss_rdata <= {14'b0, rx_maj};
            SSA_U_TXD:       ss_rdata <= {15'b0, txd_r};
            SSA_U_TX_BND:    ss_rdata <= {15'b0, tx_bnd_pend};
            default:         ss_rdata <= 16'h0000;
        endcase
    end

    // ---- the engines ------------------------------------------------------
    always_ff @(posedge CLK) begin
        // ---- save-state write decode (§5.2), restore-priority position ----
        if (ss_we) begin
            case (ss_addr)
                SSA_U_SCON:      scon_r      <= ss_wdata[7:0];
                SSA_U_SBUF_TX:   sbuf_tx_r   <= ss_wdata[7:0];
                SSA_U_SBUF_RX:   sbuf_rx_r   <= ss_wdata[7:0];
                SSA_U_TX_SHIFT:  tx_shift    <= ss_wdata[8:0];
                SSA_U_TX_BITCNT: tx_bitcnt   <= ss_wdata[3:0];
                SSA_U_TX_SEND:   tx_send     <= ss_wdata[0];
                SSA_U_TX_DATAF:  tx_dataf    <= ss_wdata[0];
                SSA_U_TX_REQ:    tx_req      <= ss_wdata[0];
                SSA_U_TX_DIV16:  tx_div16    <= ss_wdata[3:0];
                SSA_U_RX_DIV16:  rx_div16    <= ss_wdata[3:0];
                SSA_U_SMOD_DIV:  smod_div    <= ss_wdata[0];
                SSA_U_M2_DIV2:   m2_div2     <= ss_wdata[0];
                SSA_U_RX_SHIFT:  rx_shift    <= ss_wdata[8:0];
                SSA_U_RX_BITCNT: rx_bitcnt   <= ss_wdata[3:0];
                SSA_U_RX_RECV:   rx_recv     <= ss_wdata[0];
                SSA_U_RX_PEND:   rx_pend     <= ss_wdata[0];
                SSA_U_RXD_PREV:  rxd_prev    <= ss_wdata[0];
                SSA_U_RX_MAJ:    rx_maj      <= ss_wdata[1:0];
                SSA_U_TXD:       txd_r       <= ss_wdata[0];
                SSA_U_TX_BND:    tx_bnd_pend <= ss_wdata[0];
                default: ;
            endcase
        end else if (CE) begin
            if (rst_hold) begin
                scon_r      <= 8'h00;
                sbuf_rx_r   <= 8'h00;        // PQ6 pinned deterministic
                sbuf_tx_r   <= 8'h00;
                txd_r       <= 1'b1;         // idle line (§7.2)
                tx_shift    <= 9'h1FF;
                rx_shift    <= 9'h1FF;
                tx_bitcnt   <= 4'd0;
                rx_bitcnt   <= 4'd0;
                tx_div16    <= 4'd0;
                rx_div16    <= 4'd0;
                tx_send     <= 1'b0;
                tx_dataf    <= 1'b0;
                tx_req      <= 1'b0;
                tx_bnd_pend <= 1'b0;
                rx_recv     <= 1'b0;
                rx_pend     <= 1'b0;
                rxd_prev    <= 1'b1;
                smod_div    <= 1'b0;
                m2_div2     <= 1'b0;
                rx_maj      <= 2'b11;
            end else begin
                if (!prime_cycle) begin
                    //--------------------------------------------------------
                    // MODE 0 - the shift register, fosc/12 (periph §5.5)
                    //--------------------------------------------------------
                    if (m0) begin
                        // SHIFT CLOCK on TXD while SEND or RECEIVE is active:
                        // low S3-S5, high S6-S2.  Transitions at S3P1/S6P1.
                        if (tx_send || rx_recv) begin
                            if (ph == PH_S2P2) txd_r <= 1'b0;
                            if (ph == PH_S5P2) txd_r <= 1'b1;
                        end else begin
                            txd_r <= 1'b1;
                        end

                        // ---- transmit ----
                        if (tk_s6p2) begin
                            // "One full machine cycle elapses between write
                            // to SBUF and activation of SEND": tx_req was set
                            // at the PREVIOUS S6P2, so this edge is it.
                            if (tx_req) begin
                                tx_send   <= 1'b1;
                                tx_dataf  <= 1'b0;   // modes 1-3 field only
                                tx_req    <= 1'b0;
                                tx_bitcnt <= 4'd0;
                            end else if (tx_send) begin
                                // "At S6P2 of every machine cycle in which
                                // SEND is active the shift register shifts
                                // right one; zeroes come in from the left."
                                tx_shift  <= {1'b0, tx_shift[8:1]};
                                tx_bitcnt <= tx_bitcnt + 4'd1;
                                if (tx_bitcnt == 4'd7) begin
                                    // the 8th (last) shift: SEND off and TI
                                    // set, "both at S1P1 of the 10th machine
                                    // cycle after write to SBUF" - this edge
                                    // ends S6P2, so S1P1 next is what shows.
                                    tx_send  <= 1'b0;
                                    tx_dataf <= 1'b0;
                                    scon_r[SCON_TI] <= 1'b1;
                                end
                            end
                        end

                        // ---- receive ----
                        // "Reception is initiated by the condition REN = 1
                        // and RI = 0.  At S6P2 of the NEXT machine cycle RX
                        // Control writes 11111110 into the receive shift
                        // register and in the next clock phase activates
                        // RECEIVE."
                        if (tk_s6p2) begin
                            if (rx_pend) begin
                                rx_shift  <= 9'h1FE;
                                rx_recv   <= 1'b1;
                                rx_pend   <= 1'b0;
                                rx_bitcnt <= 4'd0;
                            end else if (!rx_recv && scon_nx[SCON_REN]
                                                  && !scon_nx[SCON_RI]) begin
                                rx_pend <= 1'b1;
                            end
                        end
                        // "the value shifted in from the right is the value
                        // sampled at the P3.0 pin at S5P2 of the same machine
                        // cycle"
                        if (tk_s5p2 && rx_recv) rxd_prev <= rx_smp;
                        if (tk_s6p2 && rx_recv) begin
                            rx_shift  <= {rxd_prev, rx_shift[8:1]};
                            rx_bitcnt <= rx_bitcnt + 4'd1;
                            if (rx_bitcnt == 4'd7) begin
                                // 8th shift: load SBUF, set RI, clear
                                // RECEIVE - "at S1P1 of the 10th machine
                                // cycle after the write to SCON that cleared
                                // RI".  Mode 0 has no SM2/stop-bit condition.
                                sbuf_rx_r <= {rxd_prev, rx_shift[8:2]};
                                rx_recv   <= 1'b0;
                                scon_r[SCON_RI] <= 1'b1;
                            end
                        end
                    end else begin
                        //--------------------------------------------------------
                        // MODES 1-3 - the ÷16 UART (periph §5.6, §5.7)
                        //--------------------------------------------------------
                        if (!tx_send) txd_r <= 1'b1;         // idle line

                        // ---- baud chain ----
                        if (m2) begin
                            if (osc_p2) m2_div2 <= !m2_div2;
                        end else begin
                            if (t1_ovf) smod_div <= !smod_div;
                        end
                        if (bd_tx) tx_div16 <= tx_div16 + 4'd1;
                        if (bd_rx) rx_div16 <= rx_div16 + 4'd1;
                        // The flag is cleared by the boundary that consumes
                        // it and set by the rollover; a mode-2 rollover that
                        // lands ON an S6P2 edge therefore sets it there and is
                        // consumed at the NEXT boundary - the ordinary meaning
                        // of "the machine cycle following the rollover" for a
                        // rollover that happens at a cycle's last edge.
                        if (tk_s6p2) tx_bnd_pend <= 1'b0;
                        if (tx_roll) tx_bnd_pend <= 1'b1;

                        // ---- transmit ----
                        if (tx_bit) begin
                            if (!tx_send) begin
                                if (tx_req) begin
                                    // SEND activates, start bit onto TXD
                                    tx_send   <= 1'b1;
                                    tx_dataf  <= 1'b0;
                                    tx_req    <= 1'b0;
                                    tx_bitcnt <= 4'd0;
                                    txd_r     <= 1'b0;
                                end
                            end else begin
                                // one bit time after SEND, DATA activates and
                                // the shift register drives TXD; the flag bit
                                // (mode 1: loaded 1; modes 2/3: the 1 clocked
                                // in by the first shift) becomes the stop bit
                                tx_dataf  <= 1'b1;
                                txd_r     <= tx_shift[0];
                                tx_shift  <= {(m9 && !tx_dataf),
                                              tx_shift[8:1]};
                                tx_bitcnt <= tx_bitcnt + 4'd1;
                                if (tx_bitcnt + 4'd1 == (m9 ? 4'd10 : 4'd9)) begin
                                    // the 10th (mode 1) / 11th (modes 2/3)
                                    // rollover after write to SBUF = the
                                    // beginning of the stop bit (periph §5.6)
                                    tx_send  <= 1'b0;
                                    tx_dataf <= 1'b0;
                                    scon_r[SCON_TI] <= 1'b1;
                                end
                            end
                        end

                        // ---- receive ----
                        if (bd_rx) begin
                            rxd_prev <= rx_smp;
                            if (!rx_recv) begin
                                // the 1-to-0 transition detector; on a hit
                                // the ÷16 counter is reset and 1FFH is
                                // written into the input shift register
                                if (ren && rxd_prev && !rx_smp) begin
                                    rx_recv   <= 1'b1;
                                    rx_div16  <= 4'd0;
                                    rx_shift  <= 9'h1FF;
                                    rx_bitcnt <= 4'd0;
                                end
                            end else begin
                                case (rx_state)
                                    4'd7: rx_maj[0] <= rx_smp;
                                    4'd8: rx_maj[1] <= rx_smp;
                                    4'd9: begin
                                        if (rx_bitcnt == 4'd0) begin
                                            // false-start rejection: the
                                            // value ACCEPTED in the start bit
                                            // time must be 0
                                            if (rx_maj_v) begin
                                                rx_recv <= 1'b0;
                                            end else begin
                                                rx_shift  <= rx_shf;
                                                rx_bitcnt <= 4'd1;
                                            end
                                        end else if (rx_bitcnt < RX_LAST) begin
                                            rx_shift  <= rx_shf;
                                            rx_bitcnt <= rx_bitcnt + 4'd1;
                                        end else if (rx_bitcnt == RX_LAST) begin
                                            // the final shift pulse: mode 1
                                            // brings the stop bit in, modes
                                            // 2/3 the 9th data bit.  Either
                                            // way it lands in bit 8 and the
                                            // conditions are tested HERE,
                                            // "at the time the final shift
                                            // pulse is generated".
                                            rx_shift  <= rx_shf;
                                            rx_bitcnt <= 4'd10;
                                            if (!scon_r[SCON_RI]
                                                && (!sm2 || rx_shf[8])) begin
                                                sbuf_rx_r <= rx_shf[7:0];
                                                scon_r[SCON_RB8] <= rx_shf[8];
                                                scon_r[SCON_RI]  <= 1'b1;
                                            end
                                            // mode 1 goes back to looking for
                                            // a 1-to-0 transition "at this
                                            // time"; modes 2/3 "one bit time
                                            // later" - the stop bit is never
                                            // sampled there at all.
                                            if (!m9) rx_recv <= 1'b0;
                                        end else begin
                                            rx_recv <= 1'b0;   // modes 2/3
                                        end
                                    end
                                    default: ;
                                endcase
                            end
                        end
                    end

                    //--------------------------------------------------------
                    // write to SBUF: loads the transmit shift register and
                    // flags TX Control, in every mode (periph §5.3/§5.5/§5.6).
                    // A second write while SEND is active reloads and
                    // re-flags - the frame restarts (QUESTION-P54-3).
                    //--------------------------------------------------------
                    if (wr_sbuf) begin
                        sbuf_tx_r <= sfr_wdata;
                        // mode 0/1 load a 1 into the 9th position; modes 2/3
                        // load TB8 there (periph §5.5/§5.6)
                        tx_shift  <= {m9 ? scon_r[SCON_TB8] : 1'b1, sfr_wdata};
                        tx_req    <= 1'b1;
                    end
                end

                //------------------------------------------------------------
                // Software SFR writes commit at S6P2 and BEAT the hardware
                // event of the same edge - the timer's ratified rule
                // (QUESTION-P52-5: `CLR TF0` in the overflow cycle wins).
                //------------------------------------------------------------
                if (wr_scon) scon_r <= sfr_wdata;
            end
        end
`ifdef NU8051_BACKDOOR
        else if (sfr_bkd_we) begin
            // §1.9: the backdoor SBUF write hits the RECEIVE buffer so a
            // subsequent core read returns it (QUESTION-T23-3 resolution);
            // there is no backdoor path into the transmit side, so a vector
            // can only create in-flight tx state by executing MOV SBUF,A.
            if (hit_scon) scon_r    <= sfr_wdata;
            if (hit_sbuf) sbuf_rx_r <= sfr_wdata;
        end
`endif
    end

    initial begin
        scon_r = 8'h00; sbuf_rx_r = 8'h00; sbuf_tx_r = 8'h00; txd_r = 1'b1;
        tx_shift = 9'h1FF; rx_shift = 9'h1FF;
        tx_bitcnt = 4'd0; rx_bitcnt = 4'd0; tx_div16 = 4'd0; rx_div16 = 4'd0;
        tx_send = 1'b0; tx_dataf = 1'b0; tx_req = 1'b0; tx_bnd_pend = 1'b0;
        rx_recv = 1'b0; rx_pend = 1'b0; rxd_prev = 1'b1;
        smod_div = 1'b0; m2_div2 = 1'b0; rx_maj = 2'b11;
        ss_rdata = 16'h0000;
    end

endmodule
