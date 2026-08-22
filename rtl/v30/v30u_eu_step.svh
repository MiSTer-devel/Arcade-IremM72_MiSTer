//============================================================================
//  v30u_eu_step.svh -- ONE step of the model's program.
//
//  Included inside v30u_eu.sv's bounded chain loop.  Each arm either
//    * STALLS  (`stop = 1'b1`, `st` unchanged) -- a `while (...) tick()`, or
//    * COMPLETES and hands over to a state that occupies the NEXT clock
//      (`st = ...; stop = 1'b1;`), or
//    * is ZERO-COST and falls through to the next arm inside this same edge
//      (`st = ...;` with no `stop`) -- the model's steps that charge nothing.
//
//  Governance: sim/loader_impl.h and sim/exec_impl.h are the SPEC.
//
//  --- `if (chain == 4'd0)` -- WHY 24 OF THE 33 ARMS CARRY IT (U4 pass 2) ----
//
//  The chain loop in v30u_eu.sv is UNROLLED: every position costs a full copy
//  of this file, and at CHAIN_MAX = 12 that copy was measured at ~2,200 logic
//  cells -- 12 x 2,200 is what made the EU 30,621 cells and the design
//  unroutable (ucore_provenance.md sec.51).  But a state can only STAND at
//  chain position >= 1 if some arm hands over to it WITHOUT setting `stop`,
//  and only NINE do:
//
//      S_TAKE_OPC  S_DECODE  S_DECODE2  S_EA_CALC  S_BIND
//      S_ENTER     S_TAIL    S_TAIL_POP S_INSTR_END
//
//  Every other arm's predecessors all set `stop` -- read them: S_ROW is
//  entered only from S_ENTER / S_ROW_CHG / S_RLOOP / S_IRQ_D / S_RESET and all
//  five stop; S_DECODE2 stops on every path, which makes S_MODRM, S_NORM_CHG,
//  S_HALTED, S_1BL_LEAD and S_1BL_CHG position-0-only; S_BIND's
//  `if (st != S_ENTER) stop` makes S_PRERD and S_GRPD_CHG position-0-only; and
//  so on.  The 24 guarded arms are therefore UNREACHABLE at chain >= 1, the
//  guard folds them out of eleven of the twelve copies, and the property is
//  falsifiable two ways: the CHAIN OVERFLOW `$fatal` in v30u_eu.sv fires if a
//  guarded state ever stands at chain >= 1, and the `st_zero_ok` fail-safe
//  around this include spends a clock rather than hanging if one ever did in
//  fabric.  MEASURED as well as argued: a (position, state) census over the
//  golden suite + boot saw exactly those nine states at position >= 1.
//============================================================================
case (st_n)

//----------------------------------------------------------------------------
// loader_impl.h -- `pop_opcode`, the prefix loop
//----------------------------------------------------------------------------
S_OPC_POP: if (chain == 4'd0) begin
    // pop_opcode with an EMPTY latch: the pop rides this clock, then the
    // decoder spends one (`if (!pre) biu.charge(1)`).
    //
    // ...and when this pop is an INSTRUCTION BOUNDARY (`bnd_armed` -- a
    // pre-decode-executed predecessor, or the wake from HALT) the recognition
    // is tested FIRST and needs no byte.
    if (bnd_armed_n && bnd_take) begin
        bnd_armed_n   = 1'b0;
        irq_shadow_n  = 1'b0;
        irq_fast_inta_n = 1'b1;
        irq_sel_nmi_n = irq_nmi_lvl;
        irq_sel_brk_n = !irq_take;      // §86: an EXTERNAL recognition wins
        brk_arm_n     = brk_arm_n && irq_take;   // ...BUT IT DOES NOT SPEND
                                                 //    THE ARM (fz2 C1)
        st_n = S_IRQ_D;
        stop = 1'b1;
    end else if (!q_ripe) stop = 1'b1;
    else begin
        // the pop closes the window and spends the shadow
        bnd_armed_n = 1'b0; irq_shadow_n = 1'b0;
        ld_b_n = q_byte;
        pc_n   = pc_n + 16'd1;
        pop_is_first_n = 1'b0;
        st_n = S_DECODE;
    end
end

S_TAKE_OPC: begin
    // ZERO-COST: the successor's opcode was pre-popped on the E row's clock.
    ld_b_n = opc_byte_n;
    opc_valid_n = 1'b0;
    pc_n = pc_n + 16'd1;
    st_n = S_DECODE;
end

S_DECODE: begin
    // ZERO-COST: the prefix test.
    pv = pla3_native(ld_b_n);
    if (pla3_is_prefix(pv)) begin
        case (pla3_xop(pv))
            PLA3_BL1_SEG_PREFIX: begin
                seg_override_n = 1'b1; seg_ovr_n = ld_b_n[4:3];
            end
            PLA3_BL1_REP_PFX:   rep_kind_n = REP_E;
            PLA3_BL1_REPNE_PFX: rep_kind_n = REP_NE;
            PLA3_BL1_REPC_PFX:  rep_kind_n = REP_C;
            PLA3_BL1_REPNC_PFX: rep_kind_n = REP_NC;
            PLA3_BL1_LOCK, PLA3_BL1_LOCK_ALIAS: lock_pfx_n = 1'b1;
            PLA3_BL1_EXT_PREFIX: ld_ext_n = 1'b1;
            default: ;
        endcase
        pfxcnt_n = pfxcnt_n + 8'd1;
        // the 0F escape is a 2-clock re-decode; every other prefix retires as
        // its own 2-clock instruction with its own F pop.
        // KM (2026-08-11): that stays true of the POPS and the `QS` pins, and
        // it is deliberately NOT what the TF boundary reads any more -- silicon
        // samples the escape's OPCODE pop (`S_EXT_POP`) too, and the term that
        // says so is `q_bnd_pop` in v30u_eu.sv.  `S_EXT_CHG1` still sets
        // nothing: setting `pop_is_first_n` here would be INERT (`q_first`
        // consults it only at `S_OPC_POP`, and this branch's successor is
        // `S_EXT_POP`), so it would move neither the boundary nor the pins.
        st_n = (pla3_xop(pv) == PLA3_BL1_EXT_PREFIX) ? S_EXT_CHG1 : S_PFX_CHG;
        stop = 1'b1;
    end else begin
        st_n = S_DECODE2;
    end
end

S_PFX_CHG: if (chain == 4'd0) begin
    pop_is_first_n = 1'b1;                        // prefix_retire()
    st_n = S_OPC_POP;
    stop = 1'b1;
end

S_EXT_CHG1: if (chain == 4'd0) begin
    st_n = S_EXT_POP;
    stop = 1'b1;
end

S_EXT_POP: if (chain == 4'd0) begin
    // the second byte of the 0F page IS the opcode, and it pops as an S
    if (!q_ripe) stop = 1'b1;
    else begin
        ld_b_n = q_byte;
        pc_n   = pc_n + 16'd1;
        st_n = S_DECODE2;
    end
end

//----------------------------------------------------------------------------
// loader_impl.h -- the post-prefix-loop decode
//----------------------------------------------------------------------------
S_DECODE2: begin
    pv = ld_ext_n ? pla3_ext(ld_b_n) : pla3_native(ld_b_n);
    ld_pla_n = pv;
    if (!ld_ext_n && pla3_one_byte_logic(pv)) begin
        // fz2 P2 / `L-B` -- **`HLT` DOES NOT PARK WHEN IT RETIRES INTO THE
        // TRAP.**  `brk_seen` is the term, and it is the SAME signal the
        // sibling arm of this `if` reads on this same clock for the same
        // reason (W3.5's law below: at this decode the arm flop still carries
        // the previous boundary's value and `brk_seen` is the live one,
        // measured 23/23).  A parked part has no next boundary, so an arm
        // sampled at the `HLT`'s own retire has nowhere to be spent and the
        // trap is lost outright.
        //
        // MEASURED, banked FLASH #14 captures, chip against fabric core
        // (`docs/notes/fz2_p2_prereg_2026-08-10.md` §2.2).  Exemplar
        // `fz2e/535042`: an `IRET` restores `TF`, the redirect lands on a
        // byte `F4`, and the chip reads vector 1 with **no HALT cycle ever
        // driven** -- and the frame it pushes carries IP `0e84`, ONE PAST the
        // `F4`.  Silicon RETIRED the `HLT` and took the trap at its boundary;
        // it did not halt and then wake.  The core's own trace has the sample
        // right (`BRKS seen=1` at the correct boundary) and then parks.
        //
        // With the term false this falls into the `ONE_BYTE_LOGIC` arm `HALT`
        // is already a member of; `v30u_eu_1bl.svh` is `default: ;` for this
        // xop and ends with the identical `psw_n = (psw_n & PSW_WRITABLE) |
        // PSW_FORCED` this arm writes, so the fall-through makes the same
        // architectural write and then takes the ordinary boundary -- which is
        // where the trap is due.  No flop, no wire, no save-state address, and
        // no opcode named that this line did not already name.
        // *Falsifier*: a capture in which the chip drives a HALT cycle at a
        // `HLT` whose own retire boundary has the BRK/TF arm standing.
        if (pla3_xop(pv) == PLA3_BL1_HALT && !brk_seen) begin
            // S9a: `halt_decode()` is called HERE, after `pop_opcode`'s own
            // `charge(1)`, so `eu_halt` rides the DECODE clock -- the state
            // this arm hands over to.  One rule, both paths.
            eu_halted_n = 1'b1;
            // clear_consumed(): the wake's pop is a fresh instruction's `F`,
            // and the HALT path never reaches `S_INSTR_END` to say so.
            pop_is_first_n = 1'b1;
            psw_n = (psw_n & PSW_WRITABLE) | PSW_FORCED;
            st_n = S_HALTED;
        end else begin
            // §35.3 -- THE 1BL EXECUTE STROBE IS THIS EDGE, NOT THE NEXT ONE.
            // The model is `charge(1); wait_retire_lead(); <write>; charge(1);`
            // -- the write commits at clk_ = pop+1, i.e. it is VISIBLE DURING
            // the clock this arm hands over to, so it has to be made on the
            // edge that hands over.  The wait's condition is available here:
            // `q_ripe_lead_n` is the next-state view and says "the head is
            // poppable by pop+2", which is `wait_retire_lead`'s test AT
            // clk_ = pop+1, exactly.
            //
            // `S_1BL_LEAD` keeps the case where it is NOT yet satisfied and
            // becomes a PURE WAIT; `S_1BL_CHG` is the trailing `charge(1)`
            // both paths owe.  MEASURED, `FA idx 4`: the golden's status
            // nibble is already 2 (IE=0) on clock 1 and `S_1BL_LEAD` was
            // standing there, so the write landed on the edge ENDING clock 1.
`ifndef SYNTHESIS
            // wrfuzz W3.5's decode probe (see `trc_1bld_*` in v30u_eu.sv).
            trc_1bld_hit = 1'b1; trc_1bld_ripe = q_ripe_lead_n;
            trc_1bld_seen = brk_seen; trc_1bld_arm = brk_arm;
            trc_1bld_smp = brk_smp_n; trc_1bld_shd = irq_shadow_n;
`endif
            // wrfuzz W3.5 -- ...AND THE LEAD LEADS THE SUCCESSOR'S POP, SO A
            // FORM THAT RETIRES INTO THE TRAP HAS NO POP TO LEAD.
            //
            // `wrfuzz_provenance.md` §7's law, the `ucore`'s rendering of it:
            // `wait_retire_lead()` returns AT ONCE when the BRK/TF arm is set,
            // because when the arm is set this instruction retires INTO the
            // trap and its successor is never popped.  The WAIT ITSELF IS
            // KEPT -- §7.5 falsified deleting it (the odd-`ip` half of
            // `loader_impl.h`'s 250/250 golden says the FLAG WRITE does wait
            // for its byte to arrive; deleting the queue test moved `FA`,
            // `FB` and `INT.FB`, 181 row-diffs).  Two laws, one call.
            //
            // ⚠ THE GATE IS `brk_seen`, NOT `brk_arm`, AND THAT IS MEASURED,
            // NOT CHOSEN.  This arm rides the OPCODE POP'S OWN CLOCK (the
            // chain `S_OPC_POP -> S_DECODE -> S_DECODE2`, and equally
            // `S_EPOP -> S_TAIL -> S_INSTR_END -> S_TAKE_OPC -> ...`), so
            // `brk_smp` -- the sample instant §85.2a fixed at pop + 1 -- has
            // NOT HAPPENED YET and the arm flop still carries the value the
            // previous boundary spent.  MEASURED (`sw/w35_take.py arm`, the 23
            // P1 seeds, ZERO exceptions): `brk_arm` = 0 on 23/23 at this
            // decode, `brk_seen` = 1 on 23/23, `brk_smp_n` = 1 on 23/23, and
            // the sample lands exactly ONE clock later.  W3.4's mirror gate
            // read `brk_arm` here, could not fire, fell through to
            // `S_1BL_LEAD` and fired at decode + 2 -- which is `wr1/201055`'s
            // 2731 against the model's 2729, §7.8's reported miss, explained.
            //
            // AND THE BOUNDARY NEEDS NOTHING: `S_1BL_CHG` plus the zero-cost
            // `S_INSTR_END` put the successor's `S_OPC_POP` -- `bnd_opc` -- at
            // decode + 2, and decode + 2 IS the chip's take on 23/23 (§7.4's
            // "its own opcode pop + 2, WAIT-INDEPENDENT").  §7.8's proposed
            // second boundary arm for this path is NOT needed and is not taken.
            //
            // `irq_shadow` is NOT a term, for the model's own reason and now
            // by measurement: every pop state clears it and every opcode is
            // popped by one, so it is 0 at this decode on 23/23.  §7.8's named
            // trap is ANSWERED and INERT -- the difference that mattered was
            // never the shadow, it was the ARM'S AVAILABILITY.
            //
            // `S_1BL_LEAD` is deliberately NOT gated: the model tests
            // `brk_pending_` ONCE, at entry, and then loops.
            //
            // *Falsifier*: any capture in which a `ONE_BYTE_LOGIC` form
            // retiring into a BRK/TF take has its boundary later than its own
            // opcode pop + 2 with a dry queue; or any `FA` / `FB` golden that
            // moves when this gate is armed (they fire no trap, so `brk_seen`
            // is 0 in their goldens and they are untouched by construction).
            if (q_ripe_lead_n || brk_seen) begin
                `include "v30u_eu_1bl.svh"
                st_n = S_1BL_CHG;
            end else begin
                st_n = S_1BL_LEAD;
            end
        end
        stop = 1'b1;
    end else begin
        ld_byte_n = pla3_byte_only(pv) ? 1'b1
                : pla3_w_from_bit0(pv) ? (ld_b_n[0] == 1'b0)
                : 1'b0;
        op8_n  = ld_byte_n;
        imm8_n = ld_byte_n || (ld_b_n == 8'h83) || (ld_b_n == 8'h6B);
        xop_n  = pla3_xop(pv);
        // ...and the sreg-MOV class arms the recognition shadow (`8C` / `8E`,
        // load AND store -- both measured).  One boundary, spent by it.
        //
        // W3.1 -- AND THE `POP` sreg ENTRY IS IN THE SAME CLASS.  The shadow
        // is a property of the MICROCODE ENTRY, not of a PLA column: `8C`/`8E`
        // are `00?.100011?0.00` and `07`/`0F`/`17`/`1F` are `00?.000??111.00`,
        // and the ROM has no third entry between them.  There is no PLA bit
        // for the second one -- `pla3_sreg_mov` is exactly `8C`/`8E`, checked
        // against the generated table -- so it is three opcodes named in ONE
        // place.  `0F` is the extension escape on this part and is excluded by
        // `!ld_ext_n`, which is also what stops `0F 1F` reading as `POP DS`.
        //
        // MEASURED chip-side, `sw/w31_shadow.py`: `07` and `17` are grace 1 on
        // 6 of 6 pairs and grace 0 on none, while `PUSH` sreg (`06`/`16`, the
        // ADJACENT entry `00?.000??110.00`) is grace 0 on 5 of 5 and `LES` /
        // `LDS` grace 0 on 11 of 11.
        if (!ld_ext_n && (pla3_sreg_mov(pv) ||
                          ld_b_n == 8'h07 || ld_b_n == 8'h17 ||
                          ld_b_n == 8'h1F))
            irq_shadow_n = 1'b1;
        ld_page_n = ld_ext_n ? 3'd4 : ((rep_kind_n != REP_NONE) ? 3'd1 : 3'd0);
        opc_reg_n = ld_b_n;
        ld_hasrm_n = pla3_has_modrm(pv);
        st_n = pla3_has_modrm(pv) ? S_MODRM : S_NORM_CHG;
        stop = 1'b1;
    end
end

S_1BL_LEAD: if (chain == 4'd0) begin
    // `wait_retire_lead`, and NOTHING ELSE: the execute strobe is the clock
    // BEFORE the successor's opcode pop, so a late queue takes the flag write
    // with it -- but the write itself is made on the edge that hands over to
    // the clock it must be visible during (see S_DECODE2).
    if (!q_ripe_lead_n) stop = 1'b1;
    else begin
        `include "v30u_eu_1bl.svh"
        st_n = S_1BL_CHG;
        stop = 1'b1;
    end
end

S_1BL_CHG: if (chain == 4'd0) begin
    // "pre-decode-executed forms retire in 2 clocks" -- the trailing
    // `biu.charge(1)`, which both arms of the strobe owe.
    st_n = S_INSTR_END;
end

//----------------------------------------------------------------------------
// loader_impl.h -- ModR/M, displacement, effective address
//----------------------------------------------------------------------------
S_MODRM: if (chain == 4'd0) begin
    if (!q_ripe) stop = 1'b1;
    else begin
        ld_rm_n = q_byte;
        pc_n = pc_n + 16'd1;
        ld_disp_n = 16'd0;
        ld_ripe_prev_n = 1'b0;
        chg_n = 2'd0;
        if (q_byte[7:6] == 2'd1)                        st_n = S_D8_A;
        else if (q_byte[7:6] == 2'd2)                   st_n = S_D16_LO;
        else if ((q_byte[7:6] == 2'd0) && (q_byte[2:0] == 3'd6))
                                                        st_n = S_D16_LO;
        else if (q_byte[7:6] != 2'd3)                   st_n = S_EA_CHG;
        else                                            st_n = S_BIND;
        if (st_n != S_BIND) stop = 1'b1;
    end
end

S_D8_A: if (chain == 4'd0) begin
    // opcode+2: no byte is demanded.  M8's `pen` needs to know whether the
    // byte was ALREADY poppable when the demand arrives.
    ld_ripe_prev_n = q_ripe;
    st_n = S_D8_B;
    stop = 1'b1;
end

S_D8_B: if (chain == 4'd0) begin
    if (!q_ripe) stop = 1'b1;
    else if (!ld_ripe_prev_n && (chg_n == 2'd0)) begin
        chg_n = 2'd1;                                     // the `pen` clock
        stop = 1'b1;
    end else begin
        ld_disp_n = {{8{q_byte[7]}}, q_byte};
        pc_n = pc_n + 16'd1;
        st_n = S_EA_CALC;          // ...and the trailing charge(1) spent this clk
    end
end

S_D16_LO: if (chain == 4'd0) begin
    if (!q_ripe) stop = 1'b1;
    else begin
        ld_dlo_n = q_byte;
        pc_n = pc_n + 16'd1;
        st_n = S_D16_A;
        stop = 1'b1;
    end
end

S_D16_A: if (chain == 4'd0) begin
    ld_ripe_prev_n = q_ripe;
    chg_n = 2'd0;
    st_n = S_D16_HI;
    stop = 1'b1;
end

S_D16_HI: if (chain == 4'd0) begin
    if (!q_ripe) stop = 1'b1;
    else if (!ld_ripe_prev_n && (chg_n == 2'd0)) begin
        chg_n = 2'd1;
        stop = 1'b1;
    end else begin
        ld_disp_n = {q_byte, ld_dlo_n};
        pc_n = pc_n + 16'd1;
        st_n = S_EA_CALC;          // ...and the trailing charge(1) spent this clk
    end
end

S_EA_CHG: if (chain == 4'd0) begin
    st_n = S_EA_CALC;              // the EA-compute clock is THIS one
end

S_EA_CALC: begin
    // ZERO-COST: the address adder; the clocks it needs were spent above.
    rmmod = ld_rm_n[7:6];
    rmrm  = ld_rm_n[2:0];
    case (rmrm)
        3'd0: ea = gpr_n[R_BW] + gpr_n[R_IX];
        3'd1: ea = gpr_n[R_BW] + gpr_n[R_IY];
        3'd2: ea = gpr_n[R_BP] + gpr_n[R_IX];
        3'd3: ea = gpr_n[R_BP] + gpr_n[R_IY];
        3'd4: ea = gpr_n[R_IX];
        3'd5: ea = gpr_n[R_IY];
        3'd6: ea = (rmmod == 2'd0) ? 16'd0 : gpr_n[R_BP];
        default: ea = gpr_n[R_BW];
    endcase
    // Retain the EA adder's pre-displacement half without overwriting the
    // microcode tmpa register.  The distinction is normally invisible, but
    // register-form LEA exposes it through the still-armed ADD datapath.
    ea_residue_n = ea;
    // Only the four two-register address forms select a distinct RHS rail.
    // Remember both that select and the index value; a later unary EA clears
    // the select but leaves tmpb itself to evolve exactly as before.
    ea_pair_valid_n = (rmrm <= 3'd3);
    if ((rmrm == 3'd0) || (rmrm == 3'd2))
        ea_pair_rhs_n = gpr_n[R_IX];
    else if ((rmrm == 3'd1) || (rmrm == 3'd3))
        ea_pair_rhs_n = gpr_n[R_IY];
    ea = ea + ld_disp_n;
    rseg = seg_override_n ? {1'b0, seg_ovr_n}
         : ((rmrm == 3'd2) || (rmrm == 3'd3) ||
            ((rmrm == 3'd6) && (rmmod != 2'd0))) ? 3'd2 : 3'd3;
    ind_n = ea;
    al_eaconst_n = 1'b1;
    al_eaval_n   = ea;
    m_ea_n = ea;  m_seg_n = rseg;
    st_n = S_BIND;
end

//----------------------------------------------------------------------------
// loader_impl.h -- group dispatch, OPC select, operand binding
//----------------------------------------------------------------------------
S_NORM_CHG: if (chain == 4'd0) begin
    st_n = S_BIND;                 // the opcode+1 clock is THIS one
end

S_BIND: begin
    // ZERO-COST.
    pv = ld_pla_n;
    rmmod = ld_rm_n[7:6];
    rmreg = ld_rm_n[5:3];
    rmrm  = ld_rm_n[2:0];
    ld_grpd_n = 1'b0;
    // LEA exposes the address adder's full (post-displacement) result rather
    // than consuming it as a memory address.  That full result is therefore
    // what remains on the stale EA rail for a following register-form POP.
    if (!ld_ext_n && (ld_b_n == 8'h8D) && (rmmod != 2'd3))
        ea_residue_n = ind_n;
    if (ld_hasrm_n && (pla3_xop(pv) == 4'hB)) begin
        ld_page_n = ld_b_n[3] ? 3'd3 : 3'd2;
        opc_reg_n = ld_rm_n;
        ld_grpd_n = 1'b1;
    end
    modrm_reg_n = rmreg;
    opc_from_modrm_n = 1'b0;
    opc_base_n = 5'd0;
    if ((ld_page_n == 3'd2) || (ld_page_n == 3'd3)) begin
        opc_base_n = A_INC; opc_from_modrm_n = 1'b1;
    end else if (!ld_ext_n && (ld_b_n[7:2] == 6'b100000)) begin
        opc_base_n = 5'd0;  opc_from_modrm_n = 1'b1;
    end else if (!ld_ext_n && ((ld_b_n == 8'hC0) || (ld_b_n == 8'hC1) ||
                             (ld_b_n[7:2] == 6'b110100))) begin
        opc_base_n = A_ROL; opc_from_modrm_n = 1'b1;
    end else if (pla3_xop(pv) == 4'hC) begin
        opc_base_n = A_INC;
    end
    rep_test_n = TEST_NONE;
    rep_pol_n  = 1'b0;
    if (pla3_xop(pv) == 4'hE) begin
        if (!ld_ext_n && (ld_b_n[7:1] == 7'b1110000)) begin
            // A carry-repeat prefix on E0/E1 selects the CARRY rail, while
            // the LOOP opcode bit -- not the 64/65 prefix bit -- remains the
            // polarity input.  Socketed V30 controls over both prefixes,
            // E0/E1 and the three reachable CMP flag combinations measured:
            //   E0 + {64,65}: take iff CY=0
            //   E1 + {64,65}: take iff CY=1
            // This is one PLA-class mux plus the opcode polarity wire; it is
            // also the shared cause of fuzz mc1/1112 and mc1/2523.
            if ((rep_kind_n == REP_C) || (rep_kind_n == REP_NC))
                rep_test_n = TEST_CY;
            else
                rep_test_n = TEST_Z;
            rep_pol_n = ld_b_n[0];
        end else if ((rep_kind_n == REP_E) || (rep_kind_n == REP_NE)) begin
            rep_test_n = TEST_Z; rep_pol_n = (rep_kind_n == REP_E);
        end else if ((rep_kind_n == REP_C) || (rep_kind_n == REP_NC)) begin
            rep_test_n = TEST_CY; rep_pol_n = (rep_kind_n == REP_C);
        end
    end
    // the 0F BIT-FIELD group selects BYTE registers for both ModR/M operands
    bsw = ld_byte_n || (ld_ext_n && (pla3_xop(pv) == 4'h3));
    m_kind_n = OK_NONE; m_idx_n = 3'd0; m_byte_n = 1'b0;
    r_kind_n = OK_NONE; r_idx_n = 3'd0; r_byte_n = 1'b0; r_ea_n = 16'd0; r_seg_n = 3'd3;
    // 8D / mod=3 performs no EA calculation.  Silicon nevertheless executes
    // it and the row-1 SIGMA -> R transfer observes the retained EA-adder A
    // lane.  If the preceding EA had two register inputs, its index rail is
    // still selected on B; unary forms instead expose the ordinary tmpb rail.
    // mc2/491 is the two-input witness (2594+retained SI 0fa3=3537), while
    // mc1/1929 and t30-raw/624 are the unary counterexamples.
    if (!ld_ext_n && (ld_b_n == 8'h8D) && (rmmod == 2'd3)) begin
        tmpa_n = ea_residue_n;
        if (ea_pair_valid_n) tmpb_n = ea_pair_rhs_n;
    end
    if (ld_hasrm_n) begin
        if (pla3_sreg_mov(pv)) begin
            r_kind_n = OK_SREG; r_idx_n = {1'b0, rmreg[1:0]}; r_byte_n = 1'b0;
        end else begin
            r_kind_n = OK_REG;  r_idx_n = rmreg; r_byte_n = bsw;
        end
        // INS/EXT always use the ModR/M fields as the two byte-register
        // selectors.  The mod bits still determine how many displacement
        // bytes the loader consumes, but they do not turn either selector
        // into a memory operand: the bit-field stream itself is addressed by
        // DS:IX in microcode.  On 0F 33 A8 disp16 silicon therefore reads
        // DS:IX, with no speculative read of the decoded ModR/M EA.
        if ((rmmod == 2'd3) ||
            (ld_ext_n && (pla3_xop(pv) == 4'h3))) begin
            m_kind_n = OK_REG; m_idx_n = rmrm; m_byte_n = bsw;
        end else begin
            m_kind_n = OK_MEM; m_byte_n = ld_byte_n;
        end
        if (pla3_dir_from_bit1(pv) && ld_b_n[1]) begin
            tk = m_kind_n; m_kind_n = r_kind_n; r_kind_n = tk;
            ti = m_idx_n;  m_idx_n  = r_idx_n;  r_idx_n  = ti;
            te = m_ea_n;   m_ea_n   = r_ea_n;   r_ea_n   = te;
            ts = m_seg_n;  m_seg_n  = r_seg_n;  r_seg_n  = ts;
            tb = m_byte_n; m_byte_n = r_byte_n; r_byte_n = tb;
        end
    end else if (pla3_acc_w_operand(pv)) begin
        m_kind_n = OK_REG; m_idx_n = R_AW; m_byte_n = ld_byte_n;
    end else if ((ld_b_n < 8'h40) && (ld_b_n[2:0] >= 3'd6)) begin
        r_kind_n = OK_SREG; r_idx_n = {1'b0, ld_b_n[4:3]};
    end else begin
        m_kind_n = OK_REG; m_idx_n = ld_b_n[2:0]; m_byte_n = ld_byte_n;
    end
    wb_kind_n = m_kind_n; wb_idx_n = m_idx_n; wb_ea_n = m_ea_n;
    wb_seg_n = m_seg_n;   wb_byte_n = m_byte_n;

    ld_preread_n = 1'b0;
    if (ld_hasrm_n && (rmmod != 2'd3) && !(!ld_ext_n && pla3_modrm_store(pv)))
        if ((m_kind_n == OK_MEM) || (r_kind_n == OK_MEM)) begin
            ld_preread_n = 1'b1;
            // §87.A: the decoder's operand pre-read is the OTHER thing that
            // fills OPR, and it does not go through `rdq` -- `S_PRERD` writes
            // `opr` itself.  The model sets its flag straight off `ld.preread`
            // at the decode, not at the delivery, and so does this.  THIS IS
            // THE WHOLE `mod == 3` ASYMMETRY: the pre-read runs only for
            // `has_rm && mod != 3 && !MODRM_STORE`.
            opr_loaded_n = 1'b1;
        end

    row_posted_n = 1'b0;
    if (ld_preread_n)     st_n = S_PRERD;
    else if (ld_grpd_n)   st_n = S_GRPD_CHG;
    else                st_n = S_ENTER;
    if (st_n != S_ENTER) stop = 1'b1;
end

S_PRERD: if (chain == 4'd0) begin
    // the pre-decode operand read; `wait_opr` opens micro-row 0 at its T4 + 2
    if (!row_posted_n) begin
        if (eu_slot_busy_n) stop = 1'b1;
        else begin
            row_posted_n = 1'b1;
            // The pre-read fills OPR at the OPERAND's width, exactly as a
            // microcoded read does; the tag rides with the datum
            // (`loader_impl.h`: `m.opr_byte = mo.byte`).  Note it is the
            // OPERAND's width and not the bus cycle's: `eu_word` forces a word
            // cycle on the ghost pre-read tail, and the DATUM is still the
            // decoder's byte there.
            if (rd_pending_n == 2'd0)      rdp0_byte_n = pr_byte;
            else if (rd_pending_n == 2'd1) rdp1_byte_n = pr_byte;
            // F48/U4: saturate, do not wrap (see v30u_eu_row.svh).
            if (rd_pending_n != 2'd3) rd_pending_n = rd_pending_n + 2'd1;
            stop = 1'b1;
        end
    end else if (rd_done_cnt_n == 2'd0) begin
        stop = 1'b1;
    end else begin
        rd_done_cnt_n = rd_done_cnt_n - 2'd1;
        if (rdq_n_n != 2'd0) begin
            opr_n = rdq0_n; rdq0_n = rdq1_n; rdq_n_n = rdq_n_n - 2'd1;
                    // ...and the width tag pops out of the same slot
                    opr_byte_n = rdq0_byte_n; rdq0_byte_n = rdq1_byte_n;
            opr_loaded_n = 1'b1;                              // §87.A
        end
        // F27 -- THE PRE-DECODE READ DOES NOT MAKE OPR "FRESH".  The loader
        // assigns `m.opr = biu.mem_read(...)` DIRECTLY (loader_impl.h:495) and
        // never touches `opr_fresh_`, which `begin_sequence()` left false --
        // the pairing latch is armed by a `-> OPR` TRANSFER, not by a read.
        // With it set, `86`/`87`'s write-back row emitted the store the instant
        // it posted, handing the bus the operand the pre-read had just brought
        // IN (`86 idx 0`: 9054) instead of the register the post-`E` row
        // `tmpb -> M` is about to swap OUT (9f3e).
        row_posted_n = 1'b0;
        st_n = ld_grpd_n ? S_GRPD_CHG : S_ENTER;
        if (ld_grpd_n) stop = 1'b1;
    end
end

S_GRPD_CHG: if (chain == 4'd0) begin
    st_n = S_ENTER;
end

S_ENTER: begin
    // ZERO-COST: micro-row 0 opens on the clock the caller handed over
    upc_page_n = ld_page_n;
    upc_opc_n  = opc_reg_n;
    upc_loc_n  = 4'd0;
    ending_n = 1'b0; rowq_n = 2'd0; row_posted_n = 1'b0; row_paired_n = 1'b0;
    suppress_commit_n = 1'b0;
    st_n = S_ROW;
    stop = 1'b1;
end

//----------------------------------------------------------------------------
// exec_impl.h -- run_micro
//----------------------------------------------------------------------------
S_ROW: if (chain == 4'd0) begin
    if (row_blocked) begin
        stop = 1'b1;                                    // stall_opr
    end else if (row_need_q && !q_ripe) begin
        stop = 1'b1;                                    // stall_q
    end else if (row_need_q) begin
        if (row_q1 && (rowq_n == 2'd0)) rowb0_n = q_byte; else rowb1_n = q_byte;
        pc_n = pc_n + 16'd1;
        rowq_n = rowq_n + 2'd1;
        if ({1'b0, rowq_n} < row_qn) stop = 1'b1;         // a second byte
    end else if (row_pre_wait) begin
        stop = 1'b1;                                    // deliver_read
    end else if (row_slot_wait) begin
        // F11 AGAIN, AND THIS TIME IN THE PAIRING DIMENSION.  `bus_write` is
        //     if (pend_.active) { if (!opr_fresh_) deliver_read();
        //                         emit_pending(); }
        //     biu_.write_request(...);        <- the slot wait is IN HERE
        // so the staged write is PAIRED BEFORE the slot is waited on.  The act
        // decode had that right (`eu_pair` carries no slot term, and the BIU
        // took the word), and only the STEP stalled first -- so `pend_active`
        // stayed set, the row re-ran `row_pre_wait` against an OPR the pairing
        // had just re-taken, and the row cost two extra clocks.
        // MEASURED, `F3AA idx 2`: the third store row stands on golden rows
        // 13-17 where the model stands on 13-15, and rows 14-16 are exactly
        // this -- `eu_pair` already asserted on 13 with `pnd` still 1.
        if (row_bus && pend_active_n) begin
            if (!opr_fresh_n) begin
                if (rd_done_cnt_n != 2'd0) rd_done_cnt_n = rd_done_cnt_n - 2'd1;
                if (rdq_n_n != 2'd0) begin
                    opr_n = rdq0_n; rdq0_n = rdq1_n; rdq_n_n = rdq_n_n - 2'd1;
                    // ...and the width tag pops out of the same slot
                    opr_byte_n = rdq0_byte_n; rdq0_byte_n = rdq1_byte_n;
                    opr_loaded_n = 1'b1;                      // §87.A
                end
            end
            pend_active_n = 1'b0;
            opr_fresh_n   = 1'b0;
        end
        // stall_slot -- F11's rule, in the SLOT dimension.  `BiuTimed::post`
        // waits on the slot and THEN takes it, both inside the row, so a row
        // that posts this clock does NOT wait this clock.  `eu_slot_busy_n`
        // already carries this row's OWN post (it is what set it), so without
        // `!eu_post` the step stalls on an event it itself caused and the row
        // runs one clock too long -- invisible on an aligned store (the pairing
        // still lands before T1) and fatal on a SPLIT one, where the extra
        // clocks push the pairing past the FIRST half's T1 (`50 idx 1`, data
        // 0000 on rows 6-8 against the golden's AX).
        stop = 1'b1;
    end
    if (!stop) begin
        // the F interlock's own delivery, taken once
        if (e_f) begin
            if (row_reads_opr && (rd_done_cnt_n != 2'd0))
                rd_done_cnt_n = rd_done_cnt_n - 2'd1;
            if (rdq_n_n != 2'd0) begin
                opr_n = rdq0_n; rdq0_n = rdq1_n; rdq_n_n = rdq_n_n - 2'd1;
                    // ...and the width tag pops out of the same slot
                    opr_byte_n = rdq0_byte_n; rdq0_byte_n = rdq1_byte_n;
                opr_fresh_n = 1'b1;
                opr_loaded_n = 1'b1;                          // §87.A
            end
        end
        `include "v30u_eu_row.svh"
        if (tmpa_n != tmpa) begin
            // 8F row 0058 copies the standing SIGMA into tmpa.  If a distinct
            // ModR/M EA rail was already selected, the absent register-form
            // operand does not overwrite it; otherwise SIGMA becomes the
            // residue.  The next row observes the result directly, so no
            // extra address-history register is needed.
            if (!((upc_page == 3'd0) && (upc_opc == 8'h8f) &&
                  (upc_loc == 4'd0) && (m_kind_n == OK_REG) &&
                  (wb_kind_n == OK_REG) && (ea_residue_n != tmpa)))
                ea_residue_n = tmpa_n;
        end
        // Any real B-input micro-operation replaces the retained two-register
        // EA selection.  This is why an intervening INC or LDS exposes tmpb,
        // while mc2/491's byte load leaves the earlier SI rail standing.
        if (tmpb_n != tmpb) ea_pair_valid_n = 1'b0;
    end
end

S_ROW_CHG: if (chain == 4'd0) begin
    // the taken-JMP / FARJMP redirect bubble (M11 / 7.7)
    st_n = S_ROW;
    stop = 1'b1;
end

S_RLOOP: if (chain == 4'd0) begin
    // `R`: one iterative step per clock, COUNT times.  The row's own ALU
    // latch drives the ONE shared iterative unit; afterwards it is SPENT.
    count_n = count_n - 16'd1;
    rloop_n_n = rloop_n_n - 16'd1;
    if (it_fmask != 16'd0)
        stat_n = (stat_n & ~it_fmask) | (it_flags & it_fmask);
    if (!e_nopmv) begin
        v1 = it_val;
        bsw = 1'b0;
        // The loop's result carries the width the loop RAN at.  It needs no
        // latch: the tag the destination takes is `al_width_byte` itself, and
        // `al_width_byte` is an OR that already contains whichever tag this
        // write moves -- so the value is IDEMPOTENT across the iterations, and
        // that is what `exec_impl.h`'s hoisted `loop_byte` is saying.
        wb1 = al_width_byte;
        `include "v30u_eu_wd1.svh"
    end
    // `alu_step` writes `m.tmpa` DIRECTLY (the multiply/divide low half) and
    // does NOT touch its tag; neither does this.
    if (it_writes_tmpa) tmpa_n = it_tmpa;
    if (tmpa_n != tmpa) ea_residue_n = tmpa_n;
    if (tmpb_n != tmpb) ea_pair_valid_n = 1'b0;
    if (e_w && (it_fmask != 16'd0)) commit_flags(it_fmask, it_flags);
    // `while (count != 0)`: the model runs the operation COUNT times, so the
    // terminator is read AFTER this clock's decrement -- reading it before
    // (`== 1`) runs COUNT-1 iterations and, at COUNT==1, none at all: the
    // counter wraps and the state never leaves.  All sixteen D0.x/D1.x forms
    // (shift-by-1, so COUNT is always 1) hung on it.
    if (rloop_n_n == 16'd0) begin      // this was the last iteration
        al_spent_n = 1'b1;
        // F26: the sequencer leaves the `R` row HERE, not before the loop.
        if (upc_loc_n == 4'hF) upc_opc_n = upc_opc_n + 8'd1;
        upc_loc_n = upc_loc_n + 4'd1;
        st_n = S_ROW;
    end
    stop = 1'b1;
end

S_EPOP: if (chain == 4'd0) begin
    // the E row's successor pop, deferred past the retire deadline
    if (!retire_ok_n) stop = 1'b1;
    // ...and the BOUNDARY is that deadline alone (`boundary_no_pop`'s
    // `wait_bus()`), so it is taken here whether or not the byte is ripe.
    // The post-`E` row still runs -- the model reaches it through the same
    // `ending` pass -- so the debt is raised exactly as the pop path raises it.
    else if (bnd_take) begin
        irq_shadow_n = 1'b0;
        irq_sel_nmi_n = irq_nmi_lvl;
        irq_sel_brk_n = !irq_take;                                // §86
        brk_arm_n = brk_arm_n && irq_take;                        // fz2 C1
        poste_n = 1'b1; pe_opc_reg_n = opc_reg_n; pe_opc8080_n = opc8080_n;
        pe_op8_n = op8_n; pe_pfxcnt_n = pfxcnt_n;
        st_n = S_IRQ_D;
        stop = 1'b1;
    end
    else if (!q_ripe) stop = 1'b1;
    else begin
        irq_shadow_n = 1'b0;
        opc_byte_n = q_byte;
        opc_valid_n = 1'b1;
        pop_is_first_n = 1'b0;
        poste_n = 1'b1; pe_opc_reg_n = opc_reg_n; pe_opc8080_n = opc8080_n;  // F23
        pe_op8_n = op8_n; pe_pfxcnt_n = pfxcnt_n;                         // D1 / F22
        st_n = S_TAIL;          // ...and the E row's own charge(1) spent this clk
    end
end

S_TAIL: begin
    // ZERO-COST: run_micro's tail -- a staged write still owes the bus data.
    if (pend_active_n) st_n = S_TAIL_W;
    else if (opc_valid_n) st_n = S_INSTR_END;
    else st_n = S_TAIL_POP;
    if (st_n != S_INSTR_END) stop = 1'b1;
end

S_TAIL_W: if (chain == 4'd0) begin
    // `if (!opr_fresh_) deliver_read(); emit_pending();`
    if (!opr_fresh_n && (nr_wait || !opr_free_now)) stop = 1'b1;
    else begin
        if (!opr_fresh_n) begin
            if (rd_done_cnt_n != 2'd0) rd_done_cnt_n = rd_done_cnt_n - 2'd1;
            if (rdq_n_n != 2'd0) begin
                opr_n = rdq0_n; rdq0_n = rdq1_n; rdq_n_n = rdq_n_n - 2'd1;
                    // ...and the width tag pops out of the same slot
                    opr_byte_n = rdq0_byte_n; rdq0_byte_n = rdq1_byte_n;
                opr_loaded_n = 1'b1;                          // §87.A
            end
        end
        pend_active_n = 1'b0;
        opr_fresh_n   = 1'b0;
        // §35.4 -- `emit_pending()` IS ZERO CLOCKS, ALWAYS (it fills a slot the
        // bus has already reserved), and `deliver_read()` is a WAIT: zero when
        // the condition already holds.  So the satisfied arm must FALL THROUGH
        // to the tail's pop inside this same edge, not hand it the next clock.
        // The `stop` that stood here charged one clock against a step the
        // model charges nothing for.
        if (opc_valid_n) begin
            st_n = S_INSTR_END;
        end else if (retire_ok_n && bnd_take) begin
            // the tail's own boundary, taken right where the model takes it
            irq_shadow_n  = 1'b0;
            irq_sel_nmi_n = irq_nmi_lvl;
            irq_sel_brk_n = !irq_take;                            // §86
            brk_arm_n = brk_arm_n && irq_take;                    // fz2 C1
            st_n = S_IRQ_D;
            stop = 1'b1;
        end else begin
            st_n = S_TAIL_POP;
        end
    end
end

S_TAIL_POP: begin
    // the DEFERRED opcode pre-pop: `wait_bus()` then the pop, then M8b's
    // clock after it (`if (deferred) biu.charge(1)`).
    if (!retire_ok_n) stop = 1'b1;
    // the same boundary, on the deferred arm (`exec_impl.h`'s second
    // `at_fire_boundary()` call).  `poste` was raised by the `E` row itself
    // here, so only the decision is owed.
    else if (bnd_take) begin
        irq_shadow_n = 1'b0;
        irq_sel_nmi_n = irq_nmi_lvl;
        irq_sel_brk_n = !irq_take;                                // §86
        brk_arm_n = brk_arm_n && irq_take;                        // fz2 C1
        st_n = S_IRQ_D;
        stop = 1'b1;
    end
    else if (!q_ripe) stop = 1'b1;
    else begin
        irq_shadow_n = 1'b0;
        opc_byte_n = q_byte;
        opc_valid_n = 1'b1;
        pop_is_first_n = 1'b0;
        // M8b: the clock AFTER the pop belongs to the successor's decode --
        // `if (deferred) biu.charge(1)` -- which is this clock, spent here.
        st_n = S_INSTR_END;
    end
end

S_HALTED: if (chain == 4'd0) begin
    // stall_pin -- and THE WAKE.  A halted part has no boundary of its own, so
    // the decision clock D is simply the first clock the pin pipeline has
    // matured the event on; the would-pop clock is D+1 and the entry, as ever,
    // is two clocks past that.  `eu_unhalt` is combinational off this state
    // (INT) or owed to the entry clock (NMI, where the bus is HELD).
    if (irq_nmi_lvl) begin
        bnd_armed_n = 1'b1;
        st_n = S_OPC_POP;
    end else if (irq_pin_int) begin
        eu_halted_n = 1'b0;              // `eu_unhalt` rides THIS clock
        bnd_armed_n = 1'b1;
        irq_halt_entry_n = 1'b1;
        st_n = S_OPC_POP;
    end
    stop = 1'b1;
end

S_IRQ_D: if (chain == 4'd0) begin
    // The HALT provenance remains visible through the first INTA post.  That
    // single event both withdraws the wake fetch and defers INTA arbitration;
    // the post itself spends the bit.
    // `CpuT::interrupt()`.  ONE internal decision clock (the boundary was the
    // clock before), then the entry's first row.  The loader is BYPASSED, so
    // every latch it would have written is presented explicitly -- in
    // particular `xop`, without which the vector fetch's `SR = IO` would be
    // re-classified as a port access (ledger A24), and `op8`, without which
    // 01EC's `2*vector` truncates.
    // §86 -- THE THIRD DOOR IS THE ONE THAT WAS ALREADY THERE.  The ROM's
    // 01D8 entry is `CONST 1` at row 0 and `CONST 2` at row 2: the single-step
    // vector and the NMI vector are the SAME entry, two rows apart, and the
    // trap needs no new sequence -- only a `loc` of 0 where NMI takes 2.
    // `FBRK` is cleared on the way in by `I_CITF` (v30u_eu_poste.svh vector 1),
    // exactly as `FIE` is, so the arm's own floor drains behind the entry and
    // the handler does not single-step itself.
    upc_page_n = 3'd7;
    upc_opc_n  = (irq_sel_nmi_n || irq_sel_brk_n) ? 8'h00 : 8'h02;
    upc_loc_n  = irq_sel_nmi_n ? 4'd2  : 4'd0;
    if (irq_sel_nmi_n) nmi_latch_n = 1'b0;
    // NMI may still need the boundary-origin bit during row 7.10.0's fixed-
    // vector overlap decision.  BRK has no waited cold-pop overlap; the bit is
    // cleared for both paths no later than the first vector-read row.
    if (irq_sel_brk_n) irq_fast_inta_n = 1'b0;
    seg_override_n = 1'b0; seg_ovr_n = 2'd3; rep_kind_n = REP_NONE; lock_pfx_n = 1'b0;
    pfxcnt_n = 8'd0;
    m_kind_n = OK_NONE; m_idx_n = 3'd0; m_ea_n = 16'd0; m_seg_n = 3'd3; m_byte_n = 1'b0;
    r_kind_n = OK_NONE; r_idx_n = 3'd0; r_ea_n = 16'd0; r_seg_n = 3'd3; r_byte_n = 1'b0;
    wb_kind_n = OK_NONE; wb_idx_n = 3'd0; wb_ea_n = 16'd0; wb_seg_n = 3'd3;
    wb_byte_n = 1'b0;
    opc_base_n = 5'd0; opc_from_modrm_n = 1'b0; modrm_reg_n = 3'd0; opc_reg_n = 8'd0;
    rep_test_n = TEST_NONE; rep_pol_n = 1'b0; xop_n = 4'd0;
    op8_n = 1'b0; imm8_n = 1'b0; bus_word_n = 1'b0; opc8080_n = 1'b0;
    al_op_n = A_ADD; al_tmp_n = 2'd0;
    al_eaconst_n = 1'b0; al_eaval_n = 16'd0;
    al_adjust_n = 2'd0; al_adjtmp_n = 2'd0; al_bitarm_n = 1'b0; al_bitn_n = 4'd0;
    al_spent_n = 1'b0;
    // begin_sequence(): the pairing latch and the completed-read store
    pend_active_n = 1'b0; pend_off_n = 16'd0; pend_seg_n = 3'd3;
    pend_byte_n = 1'b0; pend_io_n = 1'b0; opr_fresh_n = 1'b0;
    opr_loaded_n = 1'b0;                                      // §87.A
    rdq0_n = 16'd0; rdq1_n = 16'd0; rdq_n_n = 2'd0;
    rdq0_byte_n = 1'b0; rdq1_byte_n = 1'b0;   // ...and their width tags
    ld_ext_n = 1'b0; ld_hasrm_n = 1'b0; ld_grpd_n = 1'b0; ld_preread_n = 1'b0;
    ld_rm_n = 8'd0; ld_disp_n = 16'd0;
    ending_n = 1'b0; rowq_n = 2'd0; row_posted_n = 1'b0; row_paired_n = 1'b0;
    suppress_commit_n = 1'b0;
    opc_valid_n = 1'b0; pop_is_first_n = 1'b1; bnd_armed_n = 1'b0;
    irq_shadow_n = 1'b0;
    intr_pending_n = 1'b0;                       // `m_.intr_pending = false`
    rep_chain_n = 1'b0;                          // `begin_sequence()`
    if (eu_halted_n) begin
        eu_halted_n = 1'b0;
        unhalt_pend_n = 1'b1;   // the NMI wake: `unhalt()` AT the entry clock
    end
    st_n = S_ROW;
    stop = 1'b1;
end

S_RESET: if (chain == 4'd0) begin
    // F25: `biu.susp(); biu.charge(kResetEntryClocks);` -- the internal reset
    // dispatch, before the ROM's own reset rows at 7.03.0 (01D0).
    rst_ctr_n = rst_ctr_n + 3'd1;
    if (rst_ctr_n == 3'd4) st_n = S_ROW;
    stop = 1'b1;
end

S_INSTR_END: begin
    // ZERO-COST: `step()` returns; the successor's `clear_consumed()` and
    // `loader_decode()`'s per-instruction latch reset run here.
    seg_override_n = 1'b0; seg_ovr_n = 2'd3; rep_kind_n = REP_NONE; lock_pfx_n = 1'b0;
    rep_test_n = TEST_NONE; rep_pol_n = 1'b0; bus_word_n = 1'b0; opc8080_n = 1'b0;
    // F22 SETTLED: `pfxcnt` is reset HERE, with the rest of the prologue, and
    // NOT in `iend_late` -- the prefix arm below writes it on this same edge,
    // so a deferred reset would land on top of the successor's own count.  The
    // post-`E` row reads its travelling copy (`pfxcnt_eff`).
    pfxcnt_n = 8'd0;
    ld_ext_n = 1'b0; ld_hasrm_n = 1'b0; ld_grpd_n = 1'b0; ld_preread_n = 1'b0;
    ld_rm_n = 8'd0; ld_disp_n = 16'd0;
    // F22: the four latches the post-`E` row still reads are reset AFTER it,
    // which is the model's own order.  When the discharge has not happened yet
    // the reset is OWED and the `poste` block pays it.
    if (poste_n) iend_owed_n = 1'b1;
    else begin
        `include "v30u_eu_iend_late.svh"
    end
    ending_n = 1'b0; rowq_n = 2'd0; row_posted_n = 1'b0; row_paired_n = 1'b0;
    rep_chain_n = 1'b0;                    // `begin_sequence()`: rep_elems_ = 0
    // S9b: a form the PRE-DECODE executed (`FA` `FB` `F5` `F8` ... -- no `E`
    // row, so no boundary was taken above) retires at a boundary too, and it
    // is the cold pop that follows.  Everything else has already had its.
    if (!opc_valid_n) begin
        pop_is_first_n = 1'b1;                            // clear_consumed()
        bnd_armed_n = 1'b1;
    end
    st_n = opc_valid_n ? S_TAKE_OPC : S_OPC_POP;
    if (st_n == S_OPC_POP) stop = 1'b1;
end

default: stop = 1'b1;
endcase
