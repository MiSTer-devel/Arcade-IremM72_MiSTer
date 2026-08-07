//============================================================================
//  v30u_eu_cond.svh -- exec_impl.h::cond_true(op.cond).  Sets `taken`.
//  The ALU STATUS LATCH (`stat`) is what the JMP conditions read; the
//  architectural PSW is what [-0A-] / [-0F-] read.
//============================================================================
case (r_cond)
    C_C:      taken = stat_n[FCY];
    C_NC:     taken = !stat_n[FCY];
    C_Z:      taken = stat_n[FZ];
    C_NZ:     taken = !stat_n[FZ];
    C_OP8B:   taken = op8_n;
    // F19 -- BOTH COUNTING CONDITIONS READ THE TERMINATOR ONE CLOCK EARLY.
    // `count` is a LIVE BLOCKING variable here: after `count = count - 1` it
    // IS the post-decrement value, which is exactly what the model tests
    // (`m_.count = count - 1; return m_.count != 0;`).  Testing it against 1
    // stops one iteration early -- and at COUNT==1 the two conditions do
    // OPPOSITE wrong things: CNTZ walks one copy too few, REP takes the loop
    // back when the model falls through, so `REP MOVS` with CX==1 ran a SECOND
    // element, issued a second read nobody consumed, and tripped C3's
    // completed-read-store assertion.  This is F13's bug in its third and
    // fourth instance (sec.16's named class).
    C_CNTZ:   begin
                  count_n = count_n - 16'd1;
                  taken = (count_n != 16'd0);
              end
    C_L:      taken = (stat_n[FS] != stat_n[FV]);
    C_OP8:    taken = imm8_n;
    C_ALWAYS: taken = 1'b1;
    C_SIGN:   taken = !sign_neg_n;
    C_O:      taken = psw_n[FV];
    C_NS:     taken = !(op8_n ? tmpb_n[7] : tmpb_n[15]);
    C_REP:    begin
                  count_n = count_n - 16'd1;
                  // `++rep_elems_`: from here on this sequence's boundaries are
                  // CHAINED.  Read BEFORE the update, exactly as the model
                  // reads `rep_elems_ >= 2` after its own increment.
                  rep_chained = rep_chain_n;
                  rep_chain_n = 1'b1;
                  if (count_n == 16'd0) taken = 1'b0;      // F19
                  // "REP iterations are individually interruptible": the sample
                  // is `edge - 4` (interrupt_model.md, "REP abort") and the
                  // EDGE is one clock past this row on the FIRST boundary and
                  // two on a chained one (see `irq_rep_1st`/`irq_rep_chn`); the
                  // recognition is LATCHED -- the withdrawal path's own
                  // `0223 JMP INTR` is what reads it back.
                  else if ((rep_kind_n != REP_NONE) &&
                           (intr_pending_n ||
                            (rep_chained ? irq_rep_chn : irq_rep_1st))) begin
                      intr_pending_n = 1'b1;
                      taken = 1'b0;
                      // ...and the chained boundary's decision edge being one
                      // clock later moves the WHOLE withdrawal with it: the
                      // model's `if (rep_elems_ >= 2) biu_.charge(1)`, which is
                      // what puts the flush at accept+9 instead of edge+9.
                      if (rep_chained) bubble = 1'b1;
                  end
                  else begin
                      if (rep_test_n == TEST_Z)
                          taken = (psw_n[FZ] == rep_pol_n);
                      else if (rep_test_n == TEST_CY)
                          taken = (psw_n[FCY] == rep_pol_n);
                      else taken = 1'b1;
                      // A terminating REP condition retires normally.  TF
                      // only takes the withdrawal path when another element
                      // would otherwise run.
                      if (taken && brk_take && (rep_kind_n != REP_NONE)) begin
                          intr_pending_n = 1'b1;
                          taken = 1'b0;
                          bubble = 1'b1;
                      end
                  end
              end
    C_BUSY:   taken = poll_busy;
    C_INTR:   taken = intr_pending_n;
    default:  begin                              // C_OPC -- pla_2
                  case (opc_reg_n[3:0])
                      4'h0: taken = psw_n[FV];
                      4'h1: taken = !psw_n[FV];
                      4'h2: taken = psw_n[FCY];
                      4'h3: taken = !psw_n[FCY];
                      4'h4: taken = psw_n[FZ];
                      4'h5: taken = !psw_n[FZ];
                      4'h6: taken = psw_n[FCY] || psw_n[FZ];
                      4'h7: taken = !psw_n[FCY] && !psw_n[FZ];
                      4'h8: taken = psw_n[FS];
                      4'h9: taken = !psw_n[FS];
                      4'hA: taken = psw_n[FP];
                      4'hB: taken = !psw_n[FP];
                      4'hC: taken = (psw_n[FS] != psw_n[FV]);
                      4'hD: taken = (psw_n[FS] == psw_n[FV]);
                      4'hE: taken = psw_n[FZ] || (psw_n[FS] != psw_n[FV]);
                      default: taken = !psw_n[FZ] && (psw_n[FS] == psw_n[FV]);
                  endcase
              end
endcase
