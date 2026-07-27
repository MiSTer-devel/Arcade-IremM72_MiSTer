//============================================================================
// Imported from nec_test (https://github.com/.../nec_test)
// Source: hdl/rtl/core, commit 650dc971cb170d7fd27c073049be99b1c1bbb278
// Cycle-accurate NEC V30 (uPD70116) max-mode core, verified vs golden traces.
// Do not hand-edit; re-import from upstream. No license header upstream (same author).
//============================================================================

// -----------------------------------------------------------------------------
// GENERATED - DO NOT HAND-EDIT
// Generator: gen_race_law.py 1.0
// int9d_race.hex SHA-256: 1f84dc2efb44c550777a725ad06bbe57148326e3cf67865b882a765e7217c8f2
// Pure-combinational staircase + diagonal + 68-cell exception race law.
// -----------------------------------------------------------------------------

function automatic [5:0] rl_g0(input [5:0] i);
    case (i)
        6'd0: rl_g0 = 6'd25;
        6'd1: rl_g0 = 6'd15;
        6'd2: rl_g0 = 6'd58;
        6'd3: rl_g0 = 6'd32;
        6'd4: rl_g0 = 6'd17;
        6'd5: rl_g0 = 6'd19;
        6'd6: rl_g0 = 6'd37;
        6'd7: rl_g0 = 6'd16;
        6'd8: rl_g0 = 6'd20;
        6'd9: rl_g0 = 6'd55;
        6'd10: rl_g0 = 6'd41;
        6'd11: rl_g0 = 6'd61;
        6'd12: rl_g0 = 6'd43;
        6'd13: rl_g0 = 6'd51;
        6'd14: rl_g0 = 6'd13;
        6'd15: rl_g0 = 6'd26;
        6'd16: rl_g0 = 6'd7;
        6'd17: rl_g0 = 6'd4;
        6'd18: rl_g0 = 6'd39;
        6'd19: rl_g0 = 6'd56;
        6'd20: rl_g0 = 6'd47;
        6'd21: rl_g0 = 6'd18;
        6'd22: rl_g0 = 6'd53;
        6'd23: rl_g0 = 6'd45;
        6'd24: rl_g0 = 6'd10;
        6'd25: rl_g0 = 6'd9;
        6'd26: rl_g0 = 6'd5;
        6'd27: rl_g0 = 6'd1;
        6'd28: rl_g0 = 6'd22;
        6'd29: rl_g0 = 6'd29;
        6'd30: rl_g0 = 6'd63;
        6'd31: rl_g0 = 6'd30;
        6'd32: rl_g0 = 6'd34;
        6'd33: rl_g0 = 6'd60;
        6'd34: rl_g0 = 6'd57;
        6'd35: rl_g0 = 6'd48;
        6'd36: rl_g0 = 6'd49;
        6'd37: rl_g0 = 6'd14;
        6'd38: rl_g0 = 6'd46;
        6'd39: rl_g0 = 6'd52;
        6'd40: rl_g0 = 6'd44;
        6'd41: rl_g0 = 6'd24;
        6'd42: rl_g0 = 6'd54;
        6'd43: rl_g0 = 6'd40;
        6'd44: rl_g0 = 6'd42;
        6'd45: rl_g0 = 6'd36;
        6'd46: rl_g0 = 6'd12;
        6'd47: rl_g0 = 6'd59;
        6'd48: rl_g0 = 6'd6;
        6'd49: rl_g0 = 6'd3;
        6'd50: rl_g0 = 6'd23;
        6'd51: rl_g0 = 6'd28;
        6'd52: rl_g0 = 6'd27;
        6'd53: rl_g0 = 6'd35;
        6'd54: rl_g0 = 6'd50;
        6'd55: rl_g0 = 6'd33;
        6'd56: rl_g0 = 6'd8;
        6'd57: rl_g0 = 6'd11;
        6'd58: rl_g0 = 6'd2;
        6'd59: rl_g0 = 6'd0;
        6'd60: rl_g0 = 6'd21;
        6'd61: rl_g0 = 6'd38;
        6'd62: rl_g0 = 6'd62;
        6'd63: rl_g0 = 6'd31;
        default: rl_g0 = 6'd0;
    endcase
endfunction

function automatic [5:0] rl_g1(input [5:0] i);
    case (i)
        6'd0: rl_g1 = 6'd61;
        6'd1: rl_g1 = 6'd56;
        6'd2: rl_g1 = 6'd12;
        6'd3: rl_g1 = 6'd36;
        6'd4: rl_g1 = 6'd57;
        6'd5: rl_g1 = 6'd50;
        6'd6: rl_g1 = 6'd4;
        6'd7: rl_g1 = 6'd24;
        6'd8: rl_g1 = 6'd38;
        6'd9: rl_g1 = 6'd9;
        6'd10: rl_g1 = 6'd16;
        6'd11: rl_g1 = 6'd13;
        6'd12: rl_g1 = 6'd31;
        6'd13: rl_g1 = 6'd26;
        6'd14: rl_g1 = 6'd6;
        6'd15: rl_g1 = 6'd19;
        6'd16: rl_g1 = 6'd43;
        6'd17: rl_g1 = 6'd15;
        6'd18: rl_g1 = 6'd62;
        6'd19: rl_g1 = 6'd48;
        6'd20: rl_g1 = 6'd27;
        6'd21: rl_g1 = 6'd23;
        6'd22: rl_g1 = 6'd32;
        6'd23: rl_g1 = 6'd10;
        6'd24: rl_g1 = 6'd47;
        6'd25: rl_g1 = 6'd44;
        6'd26: rl_g1 = 6'd42;
        6'd27: rl_g1 = 6'd0;
        6'd28: rl_g1 = 6'd55;
        6'd29: rl_g1 = 6'd52;
        6'd30: rl_g1 = 6'd5;
        6'd31: rl_g1 = 6'd2;
        6'd32: rl_g1 = 6'd63;
        6'd33: rl_g1 = 6'd58;
        6'd34: rl_g1 = 6'd7;
        6'd35: rl_g1 = 6'd25;
        6'd36: rl_g1 = 6'd60;
        6'd37: rl_g1 = 6'd51;
        6'd38: rl_g1 = 6'd21;
        6'd39: rl_g1 = 6'd18;
        6'd40: rl_g1 = 6'd37;
        6'd41: rl_g1 = 6'd3;
        6'd42: rl_g1 = 6'd34;
        6'd43: rl_g1 = 6'd17;
        6'd44: rl_g1 = 6'd35;
        6'd45: rl_g1 = 6'd14;
        6'd46: rl_g1 = 6'd30;
        6'd47: rl_g1 = 6'd22;
        6'd48: rl_g1 = 6'd40;
        6'd49: rl_g1 = 6'd41;
        6'd50: rl_g1 = 6'd54;
        6'd51: rl_g1 = 6'd53;
        6'd52: rl_g1 = 6'd11;
        6'd53: rl_g1 = 6'd29;
        6'd54: rl_g1 = 6'd28;
        6'd55: rl_g1 = 6'd8;
        6'd56: rl_g1 = 6'd45;
        6'd57: rl_g1 = 6'd46;
        6'd58: rl_g1 = 6'd39;
        6'd59: rl_g1 = 6'd1;
        6'd60: rl_g1 = 6'd49;
        6'd61: rl_g1 = 6'd59;
        6'd62: rl_g1 = 6'd33;
        6'd63: rl_g1 = 6'd20;
        default: rl_g1 = 6'd0;
    endcase
endfunction

function automatic [6:0] rl_h00(input [5:0] i);
    case (i)
        6'd0: rl_h00 = 7'd12;
        6'd1: rl_h00 = 7'd12;
        6'd2: rl_h00 = 7'd12;
        6'd3: rl_h00 = 7'd2;
        6'd4: rl_h00 = 7'd12;
        6'd5: rl_h00 = 7'd12;
        6'd6: rl_h00 = 7'd12;
        6'd7: rl_h00 = 7'd12;
        6'd8: rl_h00 = 7'd12;
        6'd9: rl_h00 = 7'd12;
        6'd10: rl_h00 = 7'd12;
        6'd11: rl_h00 = 7'd12;
        6'd12: rl_h00 = 7'd12;
        6'd13: rl_h00 = 7'd12;
        6'd14: rl_h00 = 7'd12;
        6'd15: rl_h00 = 7'd12;
        6'd16: rl_h00 = 7'd0;
        6'd17: rl_h00 = 7'd0;
        6'd18: rl_h00 = 7'd12;
        6'd19: rl_h00 = 7'd12;
        6'd20: rl_h00 = 7'd12;
        6'd21: rl_h00 = 7'd12;
        6'd22: rl_h00 = 7'd12;
        6'd23: rl_h00 = 7'd12;
        6'd24: rl_h00 = 7'd64;
        6'd25: rl_h00 = 7'd64;
        6'd26: rl_h00 = 7'd0;
        6'd27: rl_h00 = 7'd0;
        6'd28: rl_h00 = 7'd12;
        6'd29: rl_h00 = 7'd12;
        6'd30: rl_h00 = 7'd12;
        6'd31: rl_h00 = 7'd12;
        6'd32: rl_h00 = 7'd12;
        6'd33: rl_h00 = 7'd12;
        6'd34: rl_h00 = 7'd12;
        6'd35: rl_h00 = 7'd2;
        6'd36: rl_h00 = 7'd12;
        6'd37: rl_h00 = 7'd12;
        6'd38: rl_h00 = 7'd12;
        6'd39: rl_h00 = 7'd12;
        6'd40: rl_h00 = 7'd12;
        6'd41: rl_h00 = 7'd12;
        6'd42: rl_h00 = 7'd12;
        6'd43: rl_h00 = 7'd12;
        6'd44: rl_h00 = 7'd12;
        6'd45: rl_h00 = 7'd12;
        6'd46: rl_h00 = 7'd12;
        6'd47: rl_h00 = 7'd12;
        6'd48: rl_h00 = 7'd0;
        6'd49: rl_h00 = 7'd0;
        6'd50: rl_h00 = 7'd12;
        6'd51: rl_h00 = 7'd12;
        6'd52: rl_h00 = 7'd12;
        6'd53: rl_h00 = 7'd12;
        6'd54: rl_h00 = 7'd12;
        6'd55: rl_h00 = 7'd12;
        6'd56: rl_h00 = 7'd64;
        6'd57: rl_h00 = 7'd64;
        6'd58: rl_h00 = 7'd0;
        6'd59: rl_h00 = 7'd0;
        6'd60: rl_h00 = 7'd12;
        6'd61: rl_h00 = 7'd12;
        6'd62: rl_h00 = 7'd12;
        6'd63: rl_h00 = 7'd12;
        default: rl_h00 = 7'd0;
    endcase
endfunction

function automatic [6:0] rl_h01(input [5:0] i);
    case (i)
        6'd0: rl_h01 = 7'd12;
        6'd1: rl_h01 = 7'd14;
        6'd2: rl_h01 = 7'd12;
        6'd3: rl_h01 = 7'd2;
        6'd4: rl_h01 = 7'd12;
        6'd5: rl_h01 = 7'd12;
        6'd6: rl_h01 = 7'd12;
        6'd7: rl_h01 = 7'd12;
        6'd8: rl_h01 = 7'd12;
        6'd9: rl_h01 = 7'd12;
        6'd10: rl_h01 = 7'd12;
        6'd11: rl_h01 = 7'd12;
        6'd12: rl_h01 = 7'd12;
        6'd13: rl_h01 = 7'd12;
        6'd14: rl_h01 = 7'd12;
        6'd15: rl_h01 = 7'd12;
        6'd16: rl_h01 = 7'd0;
        6'd17: rl_h01 = 7'd0;
        6'd18: rl_h01 = 7'd12;
        6'd19: rl_h01 = 7'd14;
        6'd20: rl_h01 = 7'd12;
        6'd21: rl_h01 = 7'd12;
        6'd22: rl_h01 = 7'd12;
        6'd23: rl_h01 = 7'd12;
        6'd24: rl_h01 = 7'd64;
        6'd25: rl_h01 = 7'd64;
        6'd26: rl_h01 = 7'd0;
        6'd27: rl_h01 = 7'd0;
        6'd28: rl_h01 = 7'd12;
        6'd29: rl_h01 = 7'd12;
        6'd30: rl_h01 = 7'd12;
        6'd31: rl_h01 = 7'd12;
        6'd32: rl_h01 = 7'd8;
        6'd33: rl_h01 = 7'd8;
        6'd34: rl_h01 = 7'd12;
        6'd35: rl_h01 = 7'd2;
        6'd36: rl_h01 = 7'd8;
        6'd37: rl_h01 = 7'd8;
        6'd38: rl_h01 = 7'd12;
        6'd39: rl_h01 = 7'd12;
        6'd40: rl_h01 = 7'd8;
        6'd41: rl_h01 = 7'd8;
        6'd42: rl_h01 = 7'd8;
        6'd43: rl_h01 = 7'd8;
        6'd44: rl_h01 = 7'd12;
        6'd45: rl_h01 = 7'd12;
        6'd46: rl_h01 = 7'd12;
        6'd47: rl_h01 = 7'd12;
        6'd48: rl_h01 = 7'd0;
        6'd49: rl_h01 = 7'd0;
        6'd50: rl_h01 = 7'd8;
        6'd51: rl_h01 = 7'd12;
        6'd52: rl_h01 = 7'd8;
        6'd53: rl_h01 = 7'd8;
        6'd54: rl_h01 = 7'd8;
        6'd55: rl_h01 = 7'd8;
        6'd56: rl_h01 = 7'd62;
        6'd57: rl_h01 = 7'd62;
        6'd58: rl_h01 = 7'd0;
        6'd59: rl_h01 = 7'd0;
        6'd60: rl_h01 = 7'd8;
        6'd61: rl_h01 = 7'd8;
        6'd62: rl_h01 = 7'd8;
        6'd63: rl_h01 = 7'd8;
        default: rl_h01 = 7'd0;
    endcase
endfunction

function automatic [6:0] rl_h10(input [5:0] i);
    case (i)
        6'd0: rl_h10 = 7'd48;
        6'd1: rl_h10 = 7'd48;
        6'd2: rl_h10 = 7'd48;
        6'd3: rl_h10 = 7'd38;
        6'd4: rl_h10 = 7'd48;
        6'd5: rl_h10 = 7'd48;
        6'd6: rl_h10 = 7'd48;
        6'd7: rl_h10 = 7'd48;
        6'd8: rl_h10 = 7'd48;
        6'd9: rl_h10 = 7'd48;
        6'd10: rl_h10 = 7'd48;
        6'd11: rl_h10 = 7'd48;
        6'd12: rl_h10 = 7'd48;
        6'd13: rl_h10 = 7'd48;
        6'd14: rl_h10 = 7'd48;
        6'd15: rl_h10 = 7'd48;
        6'd16: rl_h10 = 7'd64;
        6'd17: rl_h10 = 7'd64;
        6'd18: rl_h10 = 7'd48;
        6'd19: rl_h10 = 7'd48;
        6'd20: rl_h10 = 7'd48;
        6'd21: rl_h10 = 7'd48;
        6'd22: rl_h10 = 7'd48;
        6'd23: rl_h10 = 7'd48;
        6'd24: rl_h10 = 7'd64;
        6'd25: rl_h10 = 7'd64;
        6'd26: rl_h10 = 7'd64;
        6'd27: rl_h10 = 7'd0;
        6'd28: rl_h10 = 7'd48;
        6'd29: rl_h10 = 7'd48;
        6'd30: rl_h10 = 7'd48;
        6'd31: rl_h10 = 7'd48;
        6'd32: rl_h10 = 7'd48;
        6'd33: rl_h10 = 7'd48;
        6'd34: rl_h10 = 7'd48;
        6'd35: rl_h10 = 7'd38;
        6'd36: rl_h10 = 7'd48;
        6'd37: rl_h10 = 7'd48;
        6'd38: rl_h10 = 7'd48;
        6'd39: rl_h10 = 7'd48;
        6'd40: rl_h10 = 7'd48;
        6'd41: rl_h10 = 7'd48;
        6'd42: rl_h10 = 7'd48;
        6'd43: rl_h10 = 7'd48;
        6'd44: rl_h10 = 7'd48;
        6'd45: rl_h10 = 7'd48;
        6'd46: rl_h10 = 7'd48;
        6'd47: rl_h10 = 7'd48;
        6'd48: rl_h10 = 7'd64;
        6'd49: rl_h10 = 7'd64;
        6'd50: rl_h10 = 7'd48;
        6'd51: rl_h10 = 7'd48;
        6'd52: rl_h10 = 7'd48;
        6'd53: rl_h10 = 7'd48;
        6'd54: rl_h10 = 7'd48;
        6'd55: rl_h10 = 7'd48;
        6'd56: rl_h10 = 7'd64;
        6'd57: rl_h10 = 7'd64;
        6'd58: rl_h10 = 7'd64;
        6'd59: rl_h10 = 7'd0;
        6'd60: rl_h10 = 7'd48;
        6'd61: rl_h10 = 7'd48;
        6'd62: rl_h10 = 7'd48;
        6'd63: rl_h10 = 7'd48;
        default: rl_h10 = 7'd0;
    endcase
endfunction

function automatic [6:0] rl_h11(input [5:0] i);
    case (i)
        6'd0: rl_h11 = 7'd48;
        6'd1: rl_h11 = 7'd48;
        6'd2: rl_h11 = 7'd48;
        6'd3: rl_h11 = 7'd38;
        6'd4: rl_h11 = 7'd48;
        6'd5: rl_h11 = 7'd48;
        6'd6: rl_h11 = 7'd48;
        6'd7: rl_h11 = 7'd48;
        6'd8: rl_h11 = 7'd48;
        6'd9: rl_h11 = 7'd48;
        6'd10: rl_h11 = 7'd48;
        6'd11: rl_h11 = 7'd48;
        6'd12: rl_h11 = 7'd48;
        6'd13: rl_h11 = 7'd48;
        6'd14: rl_h11 = 7'd48;
        6'd15: rl_h11 = 7'd48;
        6'd16: rl_h11 = 7'd64;
        6'd17: rl_h11 = 7'd64;
        6'd18: rl_h11 = 7'd48;
        6'd19: rl_h11 = 7'd48;
        6'd20: rl_h11 = 7'd48;
        6'd21: rl_h11 = 7'd48;
        6'd22: rl_h11 = 7'd48;
        6'd23: rl_h11 = 7'd48;
        6'd24: rl_h11 = 7'd64;
        6'd25: rl_h11 = 7'd64;
        6'd26: rl_h11 = 7'd64;
        6'd27: rl_h11 = 7'd0;
        6'd28: rl_h11 = 7'd48;
        6'd29: rl_h11 = 7'd48;
        6'd30: rl_h11 = 7'd48;
        6'd31: rl_h11 = 7'd48;
        6'd32: rl_h11 = 7'd44;
        6'd33: rl_h11 = 7'd44;
        6'd34: rl_h11 = 7'd48;
        6'd35: rl_h11 = 7'd38;
        6'd36: rl_h11 = 7'd44;
        6'd37: rl_h11 = 7'd44;
        6'd38: rl_h11 = 7'd48;
        6'd39: rl_h11 = 7'd48;
        6'd40: rl_h11 = 7'd44;
        6'd41: rl_h11 = 7'd44;
        6'd42: rl_h11 = 7'd44;
        6'd43: rl_h11 = 7'd44;
        6'd44: rl_h11 = 7'd48;
        6'd45: rl_h11 = 7'd48;
        6'd46: rl_h11 = 7'd48;
        6'd47: rl_h11 = 7'd48;
        6'd48: rl_h11 = 7'd64;
        6'd49: rl_h11 = 7'd64;
        6'd50: rl_h11 = 7'd44;
        6'd51: rl_h11 = 7'd48;
        6'd52: rl_h11 = 7'd44;
        6'd53: rl_h11 = 7'd44;
        6'd54: rl_h11 = 7'd44;
        6'd55: rl_h11 = 7'd44;
        6'd56: rl_h11 = 7'd2;
        6'd57: rl_h11 = 7'd2;
        6'd58: rl_h11 = 7'd64;
        6'd59: rl_h11 = 7'd0;
        6'd60: rl_h11 = 7'd44;
        6'd61: rl_h11 = 7'd44;
        6'd62: rl_h11 = 7'd44;
        6'd63: rl_h11 = 7'd44;
        default: rl_h11 = 7'd0;
    endcase
endfunction

function automatic rl_exc(input [13:0] a);
    case (a)
        14'h0c03,
        14'h0c23,
        14'h0c43,
        14'h0c63,
        14'h0c78,
        14'h0c79,
        14'h0c83,
        14'h0ca3,
        14'h0cc3,
        14'h0ce3,
        14'h0cf8,
        14'h0cf9,
        14'h1078,
        14'h1079,
        14'h10f8,
        14'h10f9,
        14'h1188,
        14'h11a8,
        14'h11c8,
        14'h11e8,
        14'h1278,
        14'h1279,
        14'h12f8,
        14'h12f9,
        14'h1978,
        14'h1979,
        14'h19f8,
        14'h19f9,
        14'h1c03,
        14'h1c23,
        14'h1c43,
        14'h1c63,
        14'h1c83,
        14'h1ca3,
        14'h1cc3,
        14'h1ce3,
        14'h1e78,
        14'h1e79,
        14'h1ef8,
        14'h1ef9,
        14'h2c03,
        14'h2c23,
        14'h2c43,
        14'h2c63,
        14'h2c78,
        14'h2c79,
        14'h2c83,
        14'h2ca3,
        14'h2cc3,
        14'h2ce3,
        14'h2cf8,
        14'h2cf9,
        14'h3078,
        14'h3079,
        14'h30f8,
        14'h30f9,
        14'h3278,
        14'h3279,
        14'h32f8,
        14'h32f9,
        14'h3978,
        14'h3979,
        14'h39f8,
        14'h39f9,
        14'h3e78,
        14'h3e79,
        14'h3ef8,
        14'h3ef9: rl_exc = 1'b1;
        default: rl_exc = 1'b0;
    endcase
endfunction

function automatic race_law(input [6:0] pre, input [6:0] pop);
    reg [5:0] rp;
    reg [5:0] rq;
    reg [6:0] rank;
    reg [6:0] threshold;
    reg base;
    begin
        rp = {pre[6], pre[4:0]};
        rq = {pop[6], pop[4:0]};
        case (pre[5])
            1'b0: rank = {1'b0, rl_g0(rp)};
            1'b1: rank = {1'b0, rl_g1(rp)};
        endcase
        case ({pre[5], pop[5]})
            2'b00: threshold = rl_h00(rq);
            2'b01: threshold = rl_h01(rq);
            2'b10: threshold = rl_h10(rq);
            2'b11: threshold = rl_h11(rq);
        endcase
        base = (rank >= threshold);
        race_law = (pre == pop) ? 1'b0 : (base ^ rl_exc({pre, pop}));
    end
endfunction
