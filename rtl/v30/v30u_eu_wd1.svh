//============================================================================
//  v30u_eu_wd1.svh -- exec_impl.h::wr_dst1(c, v1, bsrc1), verbatim.
//  Inputs: `e_d1` (the row's Dest1 field), `v1`, `b1` (the byte-source flag).
//============================================================================
case (e_d1)
    5'd0, 5'd1, 5'd2, 5'd3: sreg_n[e_d1[1:0]] = v1;
    5'd4:  pc_n = v1;
    5'd5:  ind_n = v1;
    5'd6:  begin opr_n = v1; opr_fresh_n = 1'b1; opr_loaded_n = 1'b1; end
    5'd7:  ;                                        // NULL
    5'd8:  gpr_n[R_AW][7:0] = v1[7:0];                // AL
    5'd12: tmpa_n = v1;
    5'd13: tmpb_n = v1;
    5'd14: tmpc_n = v1;
    5'd15: psw_n = (v1 & PSW_WRITABLE) | PSW_FORCED;
    5'd16: gpr_n[R_AW][15:8] = v1[7:0];               // AH
    5'd17: count_n = v1;
    5'd18: begin                                    // wr_operand(R, v)
        case (r_kind_n)
            OK_REG:  if (r_byte_n) begin
                         if (r_idx_n[2]) gpr_n[r_idx_n[1:0]][15:8] = v1[7:0];
                         else          gpr_n[r_idx_n[1:0]][7:0]  = v1[7:0];
                     end else gpr_n[r_idx_n] = v1;
            OK_SREG: sreg_n[r_idx_n[1:0]] = v1;
            OK_MEM:  begin opr_n = v1; opr_fresh_n = 1'b1; opr_loaded_n = 1'b1; end
            default: ;
        endcase
    end
    5'd19: begin                                    // wr_operand(M, v)
        case (m_kind_n)
            OK_REG:  if (m_byte_n) begin
                         if (m_idx_n[2]) gpr_n[m_idx_n[1:0]][15:8] = v1[7:0];
                         else          gpr_n[m_idx_n[1:0]][7:0]  = v1[7:0];
                     end else gpr_n[m_idx_n] = v1;
            OK_SREG: sreg_n[m_idx_n[1:0]] = v1;
            OK_MEM:  begin opr_n = v1; opr_fresh_n = 1'b1; opr_loaded_n = 1'b1; end
            default: ;
        endcase
    end
    // tmpaL zero-extends, tmpbL SIGN-extends (ledger, "L-half writes")
    5'd20: tmpa_n = {8'd0, v1[7:0]};
    5'd21: tmpb_n = {{8{v1[7]}}, v1[7:0]};
    // the H-half write takes bus bits 15:8; a byte source presents its byte
    // there
    5'd22: tmpa_n[15:8] = bsw ? v1[7:0] : v1[15:8];
    5'd23: tmpb_n[15:8] = bsw ? v1[7:0] : v1[15:8];
    default: if (e_d1 >= 5'd24) gpr_n[e_d1[2:0]] = v1;
endcase
