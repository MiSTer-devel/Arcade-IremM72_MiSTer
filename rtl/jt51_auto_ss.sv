///////////////////////////////////////////
// MODULE jt51_timer
module jt51_timer #(
    parameter CW      = 8,  // counter bit width. This is the counter that can be loaded
              FREE_EN = 0   // enables a 4-bit free enable count
) (
    input                 rst,
    input                 clk,
    input                 cen,
    input                 zero,
    input        [CW-1:0] start_value,
    input                 load,
    input                 clr_flag,
    output reg            flag,
    output reg            overflow,
    input                 auto_ss_rd,
    input                 auto_ss_wr,
    input        [  31:0] auto_ss_data_in,
    input        [   7:0] auto_ss_device_idx,
    input        [  15:0] auto_ss_state_idx,
    input        [   7:0] auto_ss_base_device_idx,
    output logic [  31:0] auto_ss_data_out,
    output logic          auto_ss_ack

);
  wire device_match = (auto_ss_device_idx == auto_ss_base_device_idx);

  genvar auto_ss_idx;


  reg last_load;
  reg [CW-1:0] cnt, next;
  reg [3:0] free_cnt, free_next;
  reg free_ov;

  always @(posedge clk, posedge rst)
    if (rst) flag <= 1'b0;
    else if (auto_ss_wr && device_match) begin
      case (auto_ss_state_idx)
        0: begin
          flag <= auto_ss_data_in[4];
        end
        default: begin
        end
      endcase
    end else  /*if(cen)*/ begin
      if (clr_flag) flag <= 1'b0;
      else if (overflow) flag <= 1'b1;
    end

  always @(*) begin
    {free_ov, free_next} = {1'b0, free_cnt} + 1'b1;
    /* verilator lint_off WIDTH */
    {overflow, next}     = {1'b0, cnt} + (FREE_EN ? free_ov : 1'b1);
    /* verilator lint_on WIDTH */
  end

  always @(posedge clk) begin
    if (cen && zero) begin : counter
      last_load <= load;
      if ((load && !last_load) || overflow) begin
        cnt <= start_value;
      end else if (last_load) cnt <= next;
    end
    if (auto_ss_wr && device_match) begin
      case (auto_ss_state_idx)
        0: begin
          last_load <= auto_ss_data_in[5];
        end
        default: begin
        end
      endcase
    end
  end



  // Free running counter
  always @(posedge clk) begin
    begin
      if (rst) begin
        free_cnt <= 4'd0;
      end else if (cen && zero) begin
        free_cnt <= free_cnt + 4'd1;
      end
    end
    if (auto_ss_wr && device_match) begin
      case (auto_ss_state_idx)
        0: begin
          free_cnt <= auto_ss_data_in[3:0];
        end
        default: begin
        end
      endcase
    end
  end


  always_comb begin
    auto_ss_data_out = 32'h0;
    auto_ss_ack      = 1'b0;
    if (auto_ss_rd && device_match) begin
      case (auto_ss_state_idx)
        0: begin
          auto_ss_data_out[5:0] = {last_load, flag, free_cnt};
          auto_ss_ack           = 1'b1;
        end
        default: begin
        end
      endcase
    end
  end



endmodule


///////////////////////////////////////////
// MODULE jt51_timers
module jt51_timers (
    input               rst,
    input               clk,
    input               cen,
    input               zero,
    input        [ 9:0] value_A,
    input        [ 7:0] value_B,
    input               load_A,
    input               load_B,
    input               clr_flag_A,
    input               clr_flag_B,
    input               enable_irq_A,
    input               enable_irq_B,
    output              flag_A,
    output              flag_B,
    output              overflow_A,
    output              irq_n,
    input               auto_ss_rd,
    input               auto_ss_wr,
    input        [31:0] auto_ss_data_in,
    input        [ 7:0] auto_ss_device_idx,
    input        [15:0] auto_ss_state_idx,
    input        [ 7:0] auto_ss_base_device_idx,
    output logic [31:0] auto_ss_data_out,
    output logic        auto_ss_ack

);
  assign auto_ss_ack      = auto_ss_timer_A_ack | auto_ss_timer_B_ack;

  assign auto_ss_data_out = auto_ss_timer_A_data_out | auto_ss_timer_B_data_out;

  wire        auto_ss_timer_B_ack;

  wire [31:0] auto_ss_timer_B_data_out;

  wire        auto_ss_timer_A_ack;

  wire [31:0] auto_ss_timer_A_data_out;

  wire        device_match = (auto_ss_device_idx == auto_ss_base_device_idx);

  genvar auto_ss_idx;


  assign irq_n = ~((flag_A & enable_irq_A) | (flag_B & enable_irq_B));

  jt51_timer #(
      .CW(10)
  ) timer_A (
      .rst                    (rst),
      .clk                    (clk),
      .cen                    (cen),
      .zero                   (zero),
      .start_value            (value_A),
      .load                   (load_A),
      .clr_flag               (clr_flag_A),
      .flag                   (flag_A),
      .overflow               (overflow_A),
      .auto_ss_rd             (auto_ss_rd),
      .auto_ss_wr             (auto_ss_wr),
      .auto_ss_data_in        (auto_ss_data_in),
      .auto_ss_device_idx     (auto_ss_device_idx),
      .auto_ss_state_idx      (auto_ss_state_idx),
      .auto_ss_base_device_idx(auto_ss_base_device_idx + 1),
      .auto_ss_data_out       (auto_ss_timer_A_data_out),
      .auto_ss_ack            (auto_ss_timer_A_ack)

  );

  jt51_timer #(
      .CW     (8),
      .FREE_EN(1)
  ) timer_B (
      .rst                    (rst),
      .clk                    (clk),
      .cen                    (cen),
      .zero                   (zero),
      .start_value            (value_B),
      .load                   (load_B),
      .clr_flag               (clr_flag_B),
      .flag                   (flag_B),
      .overflow               (),
      .auto_ss_rd             (auto_ss_rd),
      .auto_ss_wr             (auto_ss_wr),
      .auto_ss_data_in        (auto_ss_data_in),
      .auto_ss_device_idx     (auto_ss_device_idx),
      .auto_ss_state_idx      (auto_ss_state_idx),
      .auto_ss_base_device_idx(auto_ss_base_device_idx + 2),
      .auto_ss_data_out       (auto_ss_timer_B_data_out),
      .auto_ss_ack            (auto_ss_timer_B_ack)

  );

endmodule


///////////////////////////////////////////
// MODULE jt51_lfo
module jt51_lfo (
    input       rst,
    input       clk,
    input       cen,
    input [4:0] cycles,

    // configuration
    input [7:0] lfo_freq,
    input [6:0] lfo_amd,
    input [6:0] lfo_pmd,
    input [1:0] lfo_w,
    input       lfo_up,
    input       noise,

    // test
    input      [7:0] test,
    output reg       lfo_clk,

    // data
    output reg   [ 7:0] am,
    output reg   [ 7:0] pm,
    input               auto_ss_rd,
    input               auto_ss_wr,
    input        [31:0] auto_ss_data_in,
    input        [ 7:0] auto_ss_device_idx,
    input        [15:0] auto_ss_state_idx,
    input        [ 7:0] auto_ss_base_device_idx,
    output logic [31:0] auto_ss_data_out,
    output logic        auto_ss_ack

);
  wire device_match = (auto_ss_device_idx == auto_ss_base_device_idx);

  genvar auto_ss_idx;


  localparam [1:0] SAWTOOTH = 2'd0, SQUARE = 2'd1, TRIANG = 2'd2, NOISE = 2'd3;

  reg [14:0] lfo_lut[0:15];

  // counters
  reg [3:0] cnt1, cnt3, bitcnt;
  reg [14:0] cnt2;
  reg [15:0] next_cnt2;
  reg [1:0] cnt1_ov, cnt2_ov;

  // LFO state (value)
  reg [15:0]
      val,  // counts next integrator step
      out2;  // integrator for PM/AM
  reg  [6:0] out1;
  wire       pm_sign;
  reg trig_sign, saw_sign;

  reg bitcnt_rst, cnt2_load, cnt3_step;
  wire lfo_clk_next;
  reg  lfo_clk_latch;

  wire cyc_5 = cycles[3:0] == 4'h5;
  wire cyc_6 = cycles[3:0] == 4'h6;
  wire cyc_c = cycles[3:0] == 4'hc;  // 12
  wire cyc_d = cycles[3:0] == 4'hd;  // 13
  wire cyc_e = cycles[3:0] == 4'he;  // 14
  wire cyc_f = cycles[3:0] == 4'hf;  // 15

  reg  cnt3_clk;
  wire ampm_sel = bitcnt[3];
  wire bit7 = &bitcnt[2:0];

  reg  lfo_up_latch;

  assign pm_sign      = lfo_w == TRIANG ? trig_sign : saw_sign;
  assign lfo_clk_next = test[2] | next_cnt2[15] | cnt3_step;

  always @(*) begin
    if (cnt2_load) begin
      next_cnt2 = {1'b0, lfo_lut[lfo_freq[7:4]]};
    end else begin
      next_cnt2 = {1'd0, cnt2} + {15'd0, cnt1_ov[1] | test[3]};
    end
  end

  always @(posedge clk) begin
    begin
      if (lfo_up) lfo_up_latch <= 1;
      else if (cen) lfo_up_latch <= 0;
    end
    if (auto_ss_wr && device_match) begin
      case (auto_ss_state_idx)
        2: begin
          lfo_up_latch <= auto_ss_data_in[23];
        end
        default: begin
        end
      endcase
    end
  end



  always @(posedge clk, posedge rst) begin
    if (rst) begin
      cnt1      <= 4'd0;
      cnt2      <= 15'd0;
      cnt3      <= 4'd0;
      cnt1_ov   <= 2'd0;
      cnt3_step <= 0;
      bitcnt    <= 4'h8;
    end else if (auto_ss_wr && device_match) begin
      case (auto_ss_state_idx)
        1: begin
          cnt2 <= auto_ss_data_in[14:0];
        end
        2: begin
          bitcnt        <= auto_ss_data_in[10:7];
          cnt1          <= auto_ss_data_in[14:11];
          cnt3          <= auto_ss_data_in[18:15];
          cnt1_ov       <= auto_ss_data_in[20:19];
          cnt2_ov       <= auto_ss_data_in[22:21];
          bitcnt_rst    <= auto_ss_data_in[24];
          cnt2_load     <= auto_ss_data_in[25];
          cnt3_clk      <= auto_ss_data_in[26];
          cnt3_step     <= auto_ss_data_in[27];
          lfo_clk       <= auto_ss_data_in[28];
          lfo_clk_latch <= auto_ss_data_in[29];
        end
        default: begin
        end
      endcase
    end else if (cen) begin
      // counter 1
      if (cyc_c) {cnt1_ov[0], cnt1} <= {1'b0, cnt1} + 1'd1;
      else cnt1_ov[0] <= 0;
      cnt1_ov[1] <= cnt1_ov[0];
      bitcnt_rst <= cnt1 == 4'd2;
      if (bitcnt_rst && !cyc_c) bitcnt <= 4'd0;
      else if (cyc_e) bitcnt <= bitcnt + 1'd1;
      // counter 2
      cnt2_load <= lfo_up_latch | next_cnt2[15];
      cnt2      <= next_cnt2[14:0];
      if (cyc_e) begin
        cnt2_ov[0]    <= next_cnt2[15];
        lfo_clk_latch <= lfo_clk_next;
      end
      if (cyc_5) cnt2_ov[1] <= cnt2_ov[0];
      // counter 3
      cnt3_step <= 0;
      if (cnt2_ov[1] & cyc_d) begin
        cnt3_clk <= 1;
        // frequency LSB control
        if (!cnt3[0]) cnt3_step <= lfo_freq[3];
        else if (!cnt3[1]) cnt3_step <= lfo_freq[2];
        else if (!cnt3[2]) cnt3_step <= lfo_freq[1];
        else if (!cnt3[3]) cnt3_step <= lfo_freq[0];
      end else begin
        cnt3_clk <= 0;
      end
      if (cnt3_clk) cnt3 <= cnt3 + 1'd1;
      // LFO clock
      lfo_clk <= lfo_clk_next;
    end
  end

  // LFO value
  reg [1:0] val_sum;
  reg val_c, wcarry, val0_next;
  reg w1, w2, w3, w4, w5, w6, w7, w8;

  reg [6:0] dmux;
  reg integ_c, out1bit;
  reg  [1:0] out2sum;
  wire [7:0] out2b;
  reg  [2:0] bitsel;

  assign out2b = out2[15:8];

  always @(*) begin
    w1        = !lfo_clk || lfo_w == NOISE || !cyc_f;
    w4        = lfo_clk_latch && lfo_w == NOISE;
    w3        = !w4 && val[15] && !test[1];
    w2        = !w1 && lfo_w == TRIANG;
    wcarry    = !w1 || (!cyc_f && lfo_w != NOISE && val_c);
    val_sum   = {1'b0, w2} + {1'b0, w3} + {1'b0, wcarry};
    val0_next = val_sum[0] || (lfo_w == NOISE && lfo_clk_latch && noise);
    // LFO compound output, AM/PM base value one after the other
    w5        = ampm_sel ? saw_sign : (!trig_sign || lfo_w != TRIANG);
    w6        = w5 ^ w3;
    w7        = cycles[3:0] < 4'd7 || cycles[3:0] == 4'd15;
    w8        = lfo_w == SQUARE ? (ampm_sel ? cyc_6 : !saw_sign) : w6;
    w8        = ~(w8 & w7);

    // Integrator
    dmux      = (ampm_sel ? lfo_pmd : lfo_amd) & ~out1;
    bitsel    = ~(bitcnt[2:0] + 3'd1);
    out1bit   = dmux[bitsel] & ~bit7;
    out2sum   = {1'b0, out1bit} + {1'b0, out2[0] && bitcnt[2:0] != 0} + {1'b0, integ_c & ~cyc_f};
  end

  always @(posedge clk, posedge rst) begin
    if (rst) begin
      val       <= 16'd0;
      val_c     <= 0;
      trig_sign <= 0;
      saw_sign  <= 0;
      out1      <= ~7'd0;
      out2      <= 16'd0;
      integ_c   <= 0;
    end else if (auto_ss_wr && device_match) begin
      case (auto_ss_state_idx)
        0: begin
          out2 <= auto_ss_data_in[15:0];
          val  <= auto_ss_data_in[31:16];
        end
        1: begin
          am <= auto_ss_data_in[22:15];
          pm <= auto_ss_data_in[30:23];
        end
        2: begin
          out1     <= auto_ss_data_in[6:0];
          integ_c  <= auto_ss_data_in[30];
          saw_sign <= auto_ss_data_in[31];
        end
        3: begin
          trig_sign <= auto_ss_data_in[0];
          val_c     <= auto_ss_data_in[1];
        end
        default: begin
        end
      endcase
    end else if (cen) begin
      val   <= {val[14:0], val0_next};
      val_c <= val_sum[1];
      if (cyc_f) begin
        trig_sign <= val[7];
        saw_sign  <= val[8];
      end
      // current step
      out1    <= {out1[5:0], w8};
      // integrator
      integ_c <= out2sum[1];
      out2    <= {out2sum[0], out2[15:1]};
      // final output
      if (bit7 & cyc_f) begin
        if (ampm_sel) pm <= lfo_pmd == 7'd0 ? 8'd0 : {out2b[7] ^ pm_sign, out2b[6:0]};
        else am <= out2b;
      end
    end
  end
  always_comb begin
    auto_ss_data_out = 32'h0;
    auto_ss_ack      = 1'b0;
    if (auto_ss_rd && device_match) begin
      case (auto_ss_state_idx)
        0: begin
          auto_ss_data_out[31:0] = {val, out2};
          auto_ss_ack            = 1'b1;
        end
        1: begin
          auto_ss_data_out[30:0] = {pm, am, cnt2};
          auto_ss_ack            = 1'b1;
        end
        2: begin
          auto_ss_data_out[31:0] = {
            saw_sign,
            integ_c,
            lfo_clk_latch,
            lfo_clk,
            cnt3_step,
            cnt3_clk,
            cnt2_load,
            bitcnt_rst,
            lfo_up_latch,
            cnt2_ov,
            cnt1_ov,
            cnt3,
            cnt1,
            bitcnt,
            out1
          };
          auto_ss_ack = 1'b1;
        end
        3: begin
          auto_ss_data_out[1:0] = {val_c, trig_sign};
          auto_ss_ack           = 1'b1;
        end
        default: begin
        end
      endcase
    end
  end



  initial begin
    lfo_lut[0]  = 15'h0000;
    lfo_lut[1]  = 15'h4000;
    lfo_lut[2]  = 15'h6000;
    lfo_lut[3]  = 15'h7000;

    lfo_lut[4]  = 15'h7800;
    lfo_lut[5]  = 15'h7c00;
    lfo_lut[6]  = 15'h7e00;
    lfo_lut[7]  = 15'h7f00;

    lfo_lut[8]  = 15'h7f80;
    lfo_lut[9]  = 15'h7fc0;
    lfo_lut[10] = 15'h7fe0;
    lfo_lut[11] = 15'h7ff0;

    lfo_lut[12] = 15'h7ff8;
    lfo_lut[13] = 15'h7ffc;
    lfo_lut[14] = 15'h7ffe;
    lfo_lut[15] = 15'h7fff;
  end

endmodule


///////////////////////////////////////////
// MODULE jt51_phinc_rom
module jt51_phinc_rom (
    // input				clk,
    input      [ 9:0] keycode,
    output reg [11:0] phinc
);

  always @(*) begin : read_lut
    case (keycode)
      10'd0:    phinc = {12'd1299};  // nota = 0, KF = 0
      10'd1:    phinc = {12'd1300};  // nota = 0, KF = 1
      10'd2:    phinc = {12'd1301};  // nota = 0, KF = 2
      10'd3:    phinc = {12'd1302};  // nota = 0, KF = 3
      10'd4:    phinc = {12'd1303};  // nota = 0, KF = 4
      10'd5:    phinc = {12'd1304};  // nota = 0, KF = 5
      10'd6:    phinc = {12'd1305};  // nota = 0, KF = 6
      10'd7:    phinc = {12'd1306};  // nota = 0, KF = 7
      10'd8:    phinc = {12'd1308};  // nota = 0, KF = 8
      10'd9:    phinc = {12'd1309};  // nota = 0, KF = 9
      10'd10:   phinc = {12'd1310};  // nota = 0, KF = 10
      10'd11:   phinc = {12'd1311};  // nota = 0, KF = 11
      10'd12:   phinc = {12'd1313};  // nota = 0, KF = 12
      10'd13:   phinc = {12'd1314};  // nota = 0, KF = 13
      10'd14:   phinc = {12'd1315};  // nota = 0, KF = 14
      10'd15:   phinc = {12'd1316};  // nota = 0, KF = 15
      10'd16:   phinc = {12'd1318};  // nota = 0, KF = 16
      10'd17:   phinc = {12'd1319};  // nota = 0, KF = 17
      10'd18:   phinc = {12'd1320};  // nota = 0, KF = 18
      10'd19:   phinc = {12'd1321};  // nota = 0, KF = 19
      10'd20:   phinc = {12'd1322};  // nota = 0, KF = 20
      10'd21:   phinc = {12'd1323};  // nota = 0, KF = 21
      10'd22:   phinc = {12'd1324};  // nota = 0, KF = 22
      10'd23:   phinc = {12'd1325};  // nota = 0, KF = 23
      10'd24:   phinc = {12'd1327};  // nota = 0, KF = 24
      10'd25:   phinc = {12'd1328};  // nota = 0, KF = 25
      10'd26:   phinc = {12'd1329};  // nota = 0, KF = 26
      10'd27:   phinc = {12'd1330};  // nota = 0, KF = 27
      10'd28:   phinc = {12'd1332};  // nota = 0, KF = 28
      10'd29:   phinc = {12'd1333};  // nota = 0, KF = 29
      10'd30:   phinc = {12'd1334};  // nota = 0, KF = 30
      10'd31:   phinc = {12'd1335};  // nota = 0, KF = 31
      10'd32:   phinc = {12'd1337};  // nota = 0, KF = 32
      10'd33:   phinc = {12'd1338};  // nota = 0, KF = 33
      10'd34:   phinc = {12'd1339};  // nota = 0, KF = 34
      10'd35:   phinc = {12'd1340};  // nota = 0, KF = 35
      10'd36:   phinc = {12'd1341};  // nota = 0, KF = 36
      10'd37:   phinc = {12'd1342};  // nota = 0, KF = 37
      10'd38:   phinc = {12'd1343};  // nota = 0, KF = 38
      10'd39:   phinc = {12'd1344};  // nota = 0, KF = 39
      10'd40:   phinc = {12'd1346};  // nota = 0, KF = 40
      10'd41:   phinc = {12'd1347};  // nota = 0, KF = 41
      10'd42:   phinc = {12'd1348};  // nota = 0, KF = 42
      10'd43:   phinc = {12'd1349};  // nota = 0, KF = 43
      10'd44:   phinc = {12'd1351};  // nota = 0, KF = 44
      10'd45:   phinc = {12'd1352};  // nota = 0, KF = 45
      10'd46:   phinc = {12'd1353};  // nota = 0, KF = 46
      10'd47:   phinc = {12'd1354};  // nota = 0, KF = 47
      10'd48:   phinc = {12'd1356};  // nota = 0, KF = 48
      10'd49:   phinc = {12'd1357};  // nota = 0, KF = 49
      10'd50:   phinc = {12'd1358};  // nota = 0, KF = 50
      10'd51:   phinc = {12'd1359};  // nota = 0, KF = 51
      10'd52:   phinc = {12'd1361};  // nota = 0, KF = 52
      10'd53:   phinc = {12'd1362};  // nota = 0, KF = 53
      10'd54:   phinc = {12'd1363};  // nota = 0, KF = 54
      10'd55:   phinc = {12'd1364};  // nota = 0, KF = 55
      10'd56:   phinc = {12'd1366};  // nota = 0, KF = 56
      10'd57:   phinc = {12'd1367};  // nota = 0, KF = 57
      10'd58:   phinc = {12'd1368};  // nota = 0, KF = 58
      10'd59:   phinc = {12'd1369};  // nota = 0, KF = 59
      10'd60:   phinc = {12'd1371};  // nota = 0, KF = 60
      10'd61:   phinc = {12'd1372};  // nota = 0, KF = 61
      10'd62:   phinc = {12'd1373};  // nota = 0, KF = 62
      10'd63:   phinc = {12'd1374};  // nota = 0, KF = 63
      10'd64:   phinc = {12'd1376};  // nota = 1, KF = 0
      10'd65:   phinc = {12'd1377};  // nota = 1, KF = 1
      10'd66:   phinc = {12'd1378};  // nota = 1, KF = 2
      10'd67:   phinc = {12'd1379};  // nota = 1, KF = 3
      10'd68:   phinc = {12'd1381};  // nota = 1, KF = 4
      10'd69:   phinc = {12'd1382};  // nota = 1, KF = 5
      10'd70:   phinc = {12'd1383};  // nota = 1, KF = 6
      10'd71:   phinc = {12'd1384};  // nota = 1, KF = 7
      10'd72:   phinc = {12'd1386};  // nota = 1, KF = 8
      10'd73:   phinc = {12'd1387};  // nota = 1, KF = 9
      10'd74:   phinc = {12'd1388};  // nota = 1, KF = 10
      10'd75:   phinc = {12'd1389};  // nota = 1, KF = 11
      10'd76:   phinc = {12'd1391};  // nota = 1, KF = 12
      10'd77:   phinc = {12'd1392};  // nota = 1, KF = 13
      10'd78:   phinc = {12'd1393};  // nota = 1, KF = 14
      10'd79:   phinc = {12'd1394};  // nota = 1, KF = 15
      10'd80:   phinc = {12'd1396};  // nota = 1, KF = 16
      10'd81:   phinc = {12'd1397};  // nota = 1, KF = 17
      10'd82:   phinc = {12'd1398};  // nota = 1, KF = 18
      10'd83:   phinc = {12'd1399};  // nota = 1, KF = 19
      10'd84:   phinc = {12'd1401};  // nota = 1, KF = 20
      10'd85:   phinc = {12'd1402};  // nota = 1, KF = 21
      10'd86:   phinc = {12'd1403};  // nota = 1, KF = 22
      10'd87:   phinc = {12'd1404};  // nota = 1, KF = 23
      10'd88:   phinc = {12'd1406};  // nota = 1, KF = 24
      10'd89:   phinc = {12'd1407};  // nota = 1, KF = 25
      10'd90:   phinc = {12'd1408};  // nota = 1, KF = 26
      10'd91:   phinc = {12'd1409};  // nota = 1, KF = 27
      10'd92:   phinc = {12'd1411};  // nota = 1, KF = 28
      10'd93:   phinc = {12'd1412};  // nota = 1, KF = 29
      10'd94:   phinc = {12'd1413};  // nota = 1, KF = 30
      10'd95:   phinc = {12'd1414};  // nota = 1, KF = 31
      10'd96:   phinc = {12'd1416};  // nota = 1, KF = 32
      10'd97:   phinc = {12'd1417};  // nota = 1, KF = 33
      10'd98:   phinc = {12'd1418};  // nota = 1, KF = 34
      10'd99:   phinc = {12'd1419};  // nota = 1, KF = 35
      10'd100:  phinc = {12'd1421};  // nota = 1, KF = 36
      10'd101:  phinc = {12'd1422};  // nota = 1, KF = 37
      10'd102:  phinc = {12'd1423};  // nota = 1, KF = 38
      10'd103:  phinc = {12'd1424};  // nota = 1, KF = 39
      10'd104:  phinc = {12'd1426};  // nota = 1, KF = 40
      10'd105:  phinc = {12'd1427};  // nota = 1, KF = 41
      10'd106:  phinc = {12'd1429};  // nota = 1, KF = 42
      10'd107:  phinc = {12'd1430};  // nota = 1, KF = 43
      10'd108:  phinc = {12'd1431};  // nota = 1, KF = 44
      10'd109:  phinc = {12'd1432};  // nota = 1, KF = 45
      10'd110:  phinc = {12'd1434};  // nota = 1, KF = 46
      10'd111:  phinc = {12'd1435};  // nota = 1, KF = 47
      10'd112:  phinc = {12'd1437};  // nota = 1, KF = 48
      10'd113:  phinc = {12'd1438};  // nota = 1, KF = 49
      10'd114:  phinc = {12'd1439};  // nota = 1, KF = 50
      10'd115:  phinc = {12'd1440};  // nota = 1, KF = 51
      10'd116:  phinc = {12'd1442};  // nota = 1, KF = 52
      10'd117:  phinc = {12'd1443};  // nota = 1, KF = 53
      10'd118:  phinc = {12'd1444};  // nota = 1, KF = 54
      10'd119:  phinc = {12'd1445};  // nota = 1, KF = 55
      10'd120:  phinc = {12'd1447};  // nota = 1, KF = 56
      10'd121:  phinc = {12'd1448};  // nota = 1, KF = 57
      10'd122:  phinc = {12'd1449};  // nota = 1, KF = 58
      10'd123:  phinc = {12'd1450};  // nota = 1, KF = 59
      10'd124:  phinc = {12'd1452};  // nota = 1, KF = 60
      10'd125:  phinc = {12'd1453};  // nota = 1, KF = 61
      10'd126:  phinc = {12'd1454};  // nota = 1, KF = 62
      10'd127:  phinc = {12'd1455};  // nota = 1, KF = 63
      10'd128:  phinc = {12'd1458};  // nota = 2, KF = 0
      10'd129:  phinc = {12'd1459};  // nota = 2, KF = 1
      10'd130:  phinc = {12'd1460};  // nota = 2, KF = 2
      10'd131:  phinc = {12'd1461};  // nota = 2, KF = 3
      10'd132:  phinc = {12'd1463};  // nota = 2, KF = 4
      10'd133:  phinc = {12'd1464};  // nota = 2, KF = 5
      10'd134:  phinc = {12'd1465};  // nota = 2, KF = 6
      10'd135:  phinc = {12'd1466};  // nota = 2, KF = 7
      10'd136:  phinc = {12'd1468};  // nota = 2, KF = 8
      10'd137:  phinc = {12'd1469};  // nota = 2, KF = 9
      10'd138:  phinc = {12'd1471};  // nota = 2, KF = 10
      10'd139:  phinc = {12'd1472};  // nota = 2, KF = 11
      10'd140:  phinc = {12'd1473};  // nota = 2, KF = 12
      10'd141:  phinc = {12'd1474};  // nota = 2, KF = 13
      10'd142:  phinc = {12'd1476};  // nota = 2, KF = 14
      10'd143:  phinc = {12'd1477};  // nota = 2, KF = 15
      10'd144:  phinc = {12'd1479};  // nota = 2, KF = 16
      10'd145:  phinc = {12'd1480};  // nota = 2, KF = 17
      10'd146:  phinc = {12'd1481};  // nota = 2, KF = 18
      10'd147:  phinc = {12'd1482};  // nota = 2, KF = 19
      10'd148:  phinc = {12'd1484};  // nota = 2, KF = 20
      10'd149:  phinc = {12'd1485};  // nota = 2, KF = 21
      10'd150:  phinc = {12'd1486};  // nota = 2, KF = 22
      10'd151:  phinc = {12'd1487};  // nota = 2, KF = 23
      10'd152:  phinc = {12'd1489};  // nota = 2, KF = 24
      10'd153:  phinc = {12'd1490};  // nota = 2, KF = 25
      10'd154:  phinc = {12'd1492};  // nota = 2, KF = 26
      10'd155:  phinc = {12'd1493};  // nota = 2, KF = 27
      10'd156:  phinc = {12'd1494};  // nota = 2, KF = 28
      10'd157:  phinc = {12'd1495};  // nota = 2, KF = 29
      10'd158:  phinc = {12'd1497};  // nota = 2, KF = 30
      10'd159:  phinc = {12'd1498};  // nota = 2, KF = 31
      10'd160:  phinc = {12'd1501};  // nota = 2, KF = 32
      10'd161:  phinc = {12'd1502};  // nota = 2, KF = 33
      10'd162:  phinc = {12'd1503};  // nota = 2, KF = 34
      10'd163:  phinc = {12'd1504};  // nota = 2, KF = 35
      10'd164:  phinc = {12'd1506};  // nota = 2, KF = 36
      10'd165:  phinc = {12'd1507};  // nota = 2, KF = 37
      10'd166:  phinc = {12'd1509};  // nota = 2, KF = 38
      10'd167:  phinc = {12'd1510};  // nota = 2, KF = 39
      10'd168:  phinc = {12'd1512};  // nota = 2, KF = 40
      10'd169:  phinc = {12'd1513};  // nota = 2, KF = 41
      10'd170:  phinc = {12'd1514};  // nota = 2, KF = 42
      10'd171:  phinc = {12'd1515};  // nota = 2, KF = 43
      10'd172:  phinc = {12'd1517};  // nota = 2, KF = 44
      10'd173:  phinc = {12'd1518};  // nota = 2, KF = 45
      10'd174:  phinc = {12'd1520};  // nota = 2, KF = 46
      10'd175:  phinc = {12'd1521};  // nota = 2, KF = 47
      10'd176:  phinc = {12'd1523};  // nota = 2, KF = 48
      10'd177:  phinc = {12'd1524};  // nota = 2, KF = 49
      10'd178:  phinc = {12'd1525};  // nota = 2, KF = 50
      10'd179:  phinc = {12'd1526};  // nota = 2, KF = 51
      10'd180:  phinc = {12'd1528};  // nota = 2, KF = 52
      10'd181:  phinc = {12'd1529};  // nota = 2, KF = 53
      10'd182:  phinc = {12'd1531};  // nota = 2, KF = 54
      10'd183:  phinc = {12'd1532};  // nota = 2, KF = 55
      10'd184:  phinc = {12'd1534};  // nota = 2, KF = 56
      10'd185:  phinc = {12'd1535};  // nota = 2, KF = 57
      10'd186:  phinc = {12'd1536};  // nota = 2, KF = 58
      10'd187:  phinc = {12'd1537};  // nota = 2, KF = 59
      10'd188:  phinc = {12'd1539};  // nota = 2, KF = 60
      10'd189:  phinc = {12'd1540};  // nota = 2, KF = 61
      10'd190:  phinc = {12'd1542};  // nota = 2, KF = 62
      10'd191:  phinc = {12'd1543};  // nota = 2, KF = 63
      10'd192:  phinc = {12'd1458};  // nota = 3, KF = 0
      10'd193:  phinc = {12'd1459};  // nota = 3, KF = 1
      10'd194:  phinc = {12'd1460};  // nota = 3, KF = 2
      10'd195:  phinc = {12'd1461};  // nota = 3, KF = 3
      10'd196:  phinc = {12'd1463};  // nota = 3, KF = 4
      10'd197:  phinc = {12'd1464};  // nota = 3, KF = 5
      10'd198:  phinc = {12'd1465};  // nota = 3, KF = 6
      10'd199:  phinc = {12'd1466};  // nota = 3, KF = 7
      10'd200:  phinc = {12'd1468};  // nota = 3, KF = 8
      10'd201:  phinc = {12'd1469};  // nota = 3, KF = 9
      10'd202:  phinc = {12'd1471};  // nota = 3, KF = 10
      10'd203:  phinc = {12'd1472};  // nota = 3, KF = 11
      10'd204:  phinc = {12'd1473};  // nota = 3, KF = 12
      10'd205:  phinc = {12'd1474};  // nota = 3, KF = 13
      10'd206:  phinc = {12'd1476};  // nota = 3, KF = 14
      10'd207:  phinc = {12'd1477};  // nota = 3, KF = 15
      10'd208:  phinc = {12'd1479};  // nota = 3, KF = 16
      10'd209:  phinc = {12'd1480};  // nota = 3, KF = 17
      10'd210:  phinc = {12'd1481};  // nota = 3, KF = 18
      10'd211:  phinc = {12'd1482};  // nota = 3, KF = 19
      10'd212:  phinc = {12'd1484};  // nota = 3, KF = 20
      10'd213:  phinc = {12'd1485};  // nota = 3, KF = 21
      10'd214:  phinc = {12'd1486};  // nota = 3, KF = 22
      10'd215:  phinc = {12'd1487};  // nota = 3, KF = 23
      10'd216:  phinc = {12'd1489};  // nota = 3, KF = 24
      10'd217:  phinc = {12'd1490};  // nota = 3, KF = 25
      10'd218:  phinc = {12'd1492};  // nota = 3, KF = 26
      10'd219:  phinc = {12'd1493};  // nota = 3, KF = 27
      10'd220:  phinc = {12'd1494};  // nota = 3, KF = 28
      10'd221:  phinc = {12'd1495};  // nota = 3, KF = 29
      10'd222:  phinc = {12'd1497};  // nota = 3, KF = 30
      10'd223:  phinc = {12'd1498};  // nota = 3, KF = 31
      10'd224:  phinc = {12'd1501};  // nota = 3, KF = 32
      10'd225:  phinc = {12'd1502};  // nota = 3, KF = 33
      10'd226:  phinc = {12'd1503};  // nota = 3, KF = 34
      10'd227:  phinc = {12'd1504};  // nota = 3, KF = 35
      10'd228:  phinc = {12'd1506};  // nota = 3, KF = 36
      10'd229:  phinc = {12'd1507};  // nota = 3, KF = 37
      10'd230:  phinc = {12'd1509};  // nota = 3, KF = 38
      10'd231:  phinc = {12'd1510};  // nota = 3, KF = 39
      10'd232:  phinc = {12'd1512};  // nota = 3, KF = 40
      10'd233:  phinc = {12'd1513};  // nota = 3, KF = 41
      10'd234:  phinc = {12'd1514};  // nota = 3, KF = 42
      10'd235:  phinc = {12'd1515};  // nota = 3, KF = 43
      10'd236:  phinc = {12'd1517};  // nota = 3, KF = 44
      10'd237:  phinc = {12'd1518};  // nota = 3, KF = 45
      10'd238:  phinc = {12'd1520};  // nota = 3, KF = 46
      10'd239:  phinc = {12'd1521};  // nota = 3, KF = 47
      10'd240:  phinc = {12'd1523};  // nota = 3, KF = 48
      10'd241:  phinc = {12'd1524};  // nota = 3, KF = 49
      10'd242:  phinc = {12'd1525};  // nota = 3, KF = 50
      10'd243:  phinc = {12'd1526};  // nota = 3, KF = 51
      10'd244:  phinc = {12'd1528};  // nota = 3, KF = 52
      10'd245:  phinc = {12'd1529};  // nota = 3, KF = 53
      10'd246:  phinc = {12'd1531};  // nota = 3, KF = 54
      10'd247:  phinc = {12'd1532};  // nota = 3, KF = 55
      10'd248:  phinc = {12'd1534};  // nota = 3, KF = 56
      10'd249:  phinc = {12'd1535};  // nota = 3, KF = 57
      10'd250:  phinc = {12'd1536};  // nota = 3, KF = 58
      10'd251:  phinc = {12'd1537};  // nota = 3, KF = 59
      10'd252:  phinc = {12'd1539};  // nota = 3, KF = 60
      10'd253:  phinc = {12'd1540};  // nota = 3, KF = 61
      10'd254:  phinc = {12'd1542};  // nota = 3, KF = 62
      10'd255:  phinc = {12'd1543};  // nota = 3, KF = 63
      10'd256:  phinc = {12'd1545};  // nota = 4, KF = 0
      10'd257:  phinc = {12'd1546};  // nota = 4, KF = 1
      10'd258:  phinc = {12'd1547};  // nota = 4, KF = 2
      10'd259:  phinc = {12'd1548};  // nota = 4, KF = 3
      10'd260:  phinc = {12'd1550};  // nota = 4, KF = 4
      10'd261:  phinc = {12'd1551};  // nota = 4, KF = 5
      10'd262:  phinc = {12'd1553};  // nota = 4, KF = 6
      10'd263:  phinc = {12'd1554};  // nota = 4, KF = 7
      10'd264:  phinc = {12'd1556};  // nota = 4, KF = 8
      10'd265:  phinc = {12'd1557};  // nota = 4, KF = 9
      10'd266:  phinc = {12'd1558};  // nota = 4, KF = 10
      10'd267:  phinc = {12'd1559};  // nota = 4, KF = 11
      10'd268:  phinc = {12'd1561};  // nota = 4, KF = 12
      10'd269:  phinc = {12'd1562};  // nota = 4, KF = 13
      10'd270:  phinc = {12'd1564};  // nota = 4, KF = 14
      10'd271:  phinc = {12'd1565};  // nota = 4, KF = 15
      10'd272:  phinc = {12'd1567};  // nota = 4, KF = 16
      10'd273:  phinc = {12'd1568};  // nota = 4, KF = 17
      10'd274:  phinc = {12'd1569};  // nota = 4, KF = 18
      10'd275:  phinc = {12'd1570};  // nota = 4, KF = 19
      10'd276:  phinc = {12'd1572};  // nota = 4, KF = 20
      10'd277:  phinc = {12'd1573};  // nota = 4, KF = 21
      10'd278:  phinc = {12'd1575};  // nota = 4, KF = 22
      10'd279:  phinc = {12'd1576};  // nota = 4, KF = 23
      10'd280:  phinc = {12'd1578};  // nota = 4, KF = 24
      10'd281:  phinc = {12'd1579};  // nota = 4, KF = 25
      10'd282:  phinc = {12'd1580};  // nota = 4, KF = 26
      10'd283:  phinc = {12'd1581};  // nota = 4, KF = 27
      10'd284:  phinc = {12'd1583};  // nota = 4, KF = 28
      10'd285:  phinc = {12'd1584};  // nota = 4, KF = 29
      10'd286:  phinc = {12'd1586};  // nota = 4, KF = 30
      10'd287:  phinc = {12'd1587};  // nota = 4, KF = 31
      10'd288:  phinc = {12'd1590};  // nota = 4, KF = 32
      10'd289:  phinc = {12'd1591};  // nota = 4, KF = 33
      10'd290:  phinc = {12'd1592};  // nota = 4, KF = 34
      10'd291:  phinc = {12'd1593};  // nota = 4, KF = 35
      10'd292:  phinc = {12'd1595};  // nota = 4, KF = 36
      10'd293:  phinc = {12'd1596};  // nota = 4, KF = 37
      10'd294:  phinc = {12'd1598};  // nota = 4, KF = 38
      10'd295:  phinc = {12'd1599};  // nota = 4, KF = 39
      10'd296:  phinc = {12'd1601};  // nota = 4, KF = 40
      10'd297:  phinc = {12'd1602};  // nota = 4, KF = 41
      10'd298:  phinc = {12'd1604};  // nota = 4, KF = 42
      10'd299:  phinc = {12'd1605};  // nota = 4, KF = 43
      10'd300:  phinc = {12'd1607};  // nota = 4, KF = 44
      10'd301:  phinc = {12'd1608};  // nota = 4, KF = 45
      10'd302:  phinc = {12'd1609};  // nota = 4, KF = 46
      10'd303:  phinc = {12'd1610};  // nota = 4, KF = 47
      10'd304:  phinc = {12'd1613};  // nota = 4, KF = 48
      10'd305:  phinc = {12'd1614};  // nota = 4, KF = 49
      10'd306:  phinc = {12'd1615};  // nota = 4, KF = 50
      10'd307:  phinc = {12'd1616};  // nota = 4, KF = 51
      10'd308:  phinc = {12'd1618};  // nota = 4, KF = 52
      10'd309:  phinc = {12'd1619};  // nota = 4, KF = 53
      10'd310:  phinc = {12'd1621};  // nota = 4, KF = 54
      10'd311:  phinc = {12'd1622};  // nota = 4, KF = 55
      10'd312:  phinc = {12'd1624};  // nota = 4, KF = 56
      10'd313:  phinc = {12'd1625};  // nota = 4, KF = 57
      10'd314:  phinc = {12'd1627};  // nota = 4, KF = 58
      10'd315:  phinc = {12'd1628};  // nota = 4, KF = 59
      10'd316:  phinc = {12'd1630};  // nota = 4, KF = 60
      10'd317:  phinc = {12'd1631};  // nota = 4, KF = 61
      10'd318:  phinc = {12'd1632};  // nota = 4, KF = 62
      10'd319:  phinc = {12'd1633};  // nota = 4, KF = 63
      10'd320:  phinc = {12'd1637};  // nota = 5, KF = 0
      10'd321:  phinc = {12'd1638};  // nota = 5, KF = 1
      10'd322:  phinc = {12'd1639};  // nota = 5, KF = 2
      10'd323:  phinc = {12'd1640};  // nota = 5, KF = 3
      10'd324:  phinc = {12'd1642};  // nota = 5, KF = 4
      10'd325:  phinc = {12'd1643};  // nota = 5, KF = 5
      10'd326:  phinc = {12'd1645};  // nota = 5, KF = 6
      10'd327:  phinc = {12'd1646};  // nota = 5, KF = 7
      10'd328:  phinc = {12'd1648};  // nota = 5, KF = 8
      10'd329:  phinc = {12'd1649};  // nota = 5, KF = 9
      10'd330:  phinc = {12'd1651};  // nota = 5, KF = 10
      10'd331:  phinc = {12'd1652};  // nota = 5, KF = 11
      10'd332:  phinc = {12'd1654};  // nota = 5, KF = 12
      10'd333:  phinc = {12'd1655};  // nota = 5, KF = 13
      10'd334:  phinc = {12'd1656};  // nota = 5, KF = 14
      10'd335:  phinc = {12'd1657};  // nota = 5, KF = 15
      10'd336:  phinc = {12'd1660};  // nota = 5, KF = 16
      10'd337:  phinc = {12'd1661};  // nota = 5, KF = 17
      10'd338:  phinc = {12'd1663};  // nota = 5, KF = 18
      10'd339:  phinc = {12'd1664};  // nota = 5, KF = 19
      10'd340:  phinc = {12'd1666};  // nota = 5, KF = 20
      10'd341:  phinc = {12'd1667};  // nota = 5, KF = 21
      10'd342:  phinc = {12'd1669};  // nota = 5, KF = 22
      10'd343:  phinc = {12'd1670};  // nota = 5, KF = 23
      10'd344:  phinc = {12'd1672};  // nota = 5, KF = 24
      10'd345:  phinc = {12'd1673};  // nota = 5, KF = 25
      10'd346:  phinc = {12'd1675};  // nota = 5, KF = 26
      10'd347:  phinc = {12'd1676};  // nota = 5, KF = 27
      10'd348:  phinc = {12'd1678};  // nota = 5, KF = 28
      10'd349:  phinc = {12'd1679};  // nota = 5, KF = 29
      10'd350:  phinc = {12'd1681};  // nota = 5, KF = 30
      10'd351:  phinc = {12'd1682};  // nota = 5, KF = 31
      10'd352:  phinc = {12'd1685};  // nota = 5, KF = 32
      10'd353:  phinc = {12'd1686};  // nota = 5, KF = 33
      10'd354:  phinc = {12'd1688};  // nota = 5, KF = 34
      10'd355:  phinc = {12'd1689};  // nota = 5, KF = 35
      10'd356:  phinc = {12'd1691};  // nota = 5, KF = 36
      10'd357:  phinc = {12'd1692};  // nota = 5, KF = 37
      10'd358:  phinc = {12'd1694};  // nota = 5, KF = 38
      10'd359:  phinc = {12'd1695};  // nota = 5, KF = 39
      10'd360:  phinc = {12'd1697};  // nota = 5, KF = 40
      10'd361:  phinc = {12'd1698};  // nota = 5, KF = 41
      10'd362:  phinc = {12'd1700};  // nota = 5, KF = 42
      10'd363:  phinc = {12'd1701};  // nota = 5, KF = 43
      10'd364:  phinc = {12'd1703};  // nota = 5, KF = 44
      10'd365:  phinc = {12'd1704};  // nota = 5, KF = 45
      10'd366:  phinc = {12'd1706};  // nota = 5, KF = 46
      10'd367:  phinc = {12'd1707};  // nota = 5, KF = 47
      10'd368:  phinc = {12'd1709};  // nota = 5, KF = 48
      10'd369:  phinc = {12'd1710};  // nota = 5, KF = 49
      10'd370:  phinc = {12'd1712};  // nota = 5, KF = 50
      10'd371:  phinc = {12'd1713};  // nota = 5, KF = 51
      10'd372:  phinc = {12'd1715};  // nota = 5, KF = 52
      10'd373:  phinc = {12'd1716};  // nota = 5, KF = 53
      10'd374:  phinc = {12'd1718};  // nota = 5, KF = 54
      10'd375:  phinc = {12'd1719};  // nota = 5, KF = 55
      10'd376:  phinc = {12'd1721};  // nota = 5, KF = 56
      10'd377:  phinc = {12'd1722};  // nota = 5, KF = 57
      10'd378:  phinc = {12'd1724};  // nota = 5, KF = 58
      10'd379:  phinc = {12'd1725};  // nota = 5, KF = 59
      10'd380:  phinc = {12'd1727};  // nota = 5, KF = 60
      10'd381:  phinc = {12'd1728};  // nota = 5, KF = 61
      10'd382:  phinc = {12'd1730};  // nota = 5, KF = 62
      10'd383:  phinc = {12'd1731};  // nota = 5, KF = 63
      10'd384:  phinc = {12'd1734};  // nota = 6, KF = 0
      10'd385:  phinc = {12'd1735};  // nota = 6, KF = 1
      10'd386:  phinc = {12'd1737};  // nota = 6, KF = 2
      10'd387:  phinc = {12'd1738};  // nota = 6, KF = 3
      10'd388:  phinc = {12'd1740};  // nota = 6, KF = 4
      10'd389:  phinc = {12'd1741};  // nota = 6, KF = 5
      10'd390:  phinc = {12'd1743};  // nota = 6, KF = 6
      10'd391:  phinc = {12'd1744};  // nota = 6, KF = 7
      10'd392:  phinc = {12'd1746};  // nota = 6, KF = 8
      10'd393:  phinc = {12'd1748};  // nota = 6, KF = 9
      10'd394:  phinc = {12'd1749};  // nota = 6, KF = 10
      10'd395:  phinc = {12'd1751};  // nota = 6, KF = 11
      10'd396:  phinc = {12'd1752};  // nota = 6, KF = 12
      10'd397:  phinc = {12'd1754};  // nota = 6, KF = 13
      10'd398:  phinc = {12'd1755};  // nota = 6, KF = 14
      10'd399:  phinc = {12'd1757};  // nota = 6, KF = 15
      10'd400:  phinc = {12'd1759};  // nota = 6, KF = 16
      10'd401:  phinc = {12'd1760};  // nota = 6, KF = 17
      10'd402:  phinc = {12'd1762};  // nota = 6, KF = 18
      10'd403:  phinc = {12'd1763};  // nota = 6, KF = 19
      10'd404:  phinc = {12'd1765};  // nota = 6, KF = 20
      10'd405:  phinc = {12'd1766};  // nota = 6, KF = 21
      10'd406:  phinc = {12'd1768};  // nota = 6, KF = 22
      10'd407:  phinc = {12'd1769};  // nota = 6, KF = 23
      10'd408:  phinc = {12'd1771};  // nota = 6, KF = 24
      10'd409:  phinc = {12'd1773};  // nota = 6, KF = 25
      10'd410:  phinc = {12'd1774};  // nota = 6, KF = 26
      10'd411:  phinc = {12'd1776};  // nota = 6, KF = 27
      10'd412:  phinc = {12'd1777};  // nota = 6, KF = 28
      10'd413:  phinc = {12'd1779};  // nota = 6, KF = 29
      10'd414:  phinc = {12'd1780};  // nota = 6, KF = 30
      10'd415:  phinc = {12'd1782};  // nota = 6, KF = 31
      10'd416:  phinc = {12'd1785};  // nota = 6, KF = 32
      10'd417:  phinc = {12'd1786};  // nota = 6, KF = 33
      10'd418:  phinc = {12'd1788};  // nota = 6, KF = 34
      10'd419:  phinc = {12'd1789};  // nota = 6, KF = 35
      10'd420:  phinc = {12'd1791};  // nota = 6, KF = 36
      10'd421:  phinc = {12'd1793};  // nota = 6, KF = 37
      10'd422:  phinc = {12'd1794};  // nota = 6, KF = 38
      10'd423:  phinc = {12'd1796};  // nota = 6, KF = 39
      10'd424:  phinc = {12'd1798};  // nota = 6, KF = 40
      10'd425:  phinc = {12'd1799};  // nota = 6, KF = 41
      10'd426:  phinc = {12'd1801};  // nota = 6, KF = 42
      10'd427:  phinc = {12'd1802};  // nota = 6, KF = 43
      10'd428:  phinc = {12'd1804};  // nota = 6, KF = 44
      10'd429:  phinc = {12'd1806};  // nota = 6, KF = 45
      10'd430:  phinc = {12'd1807};  // nota = 6, KF = 46
      10'd431:  phinc = {12'd1809};  // nota = 6, KF = 47
      10'd432:  phinc = {12'd1811};  // nota = 6, KF = 48
      10'd433:  phinc = {12'd1812};  // nota = 6, KF = 49
      10'd434:  phinc = {12'd1814};  // nota = 6, KF = 50
      10'd435:  phinc = {12'd1815};  // nota = 6, KF = 51
      10'd436:  phinc = {12'd1817};  // nota = 6, KF = 52
      10'd437:  phinc = {12'd1819};  // nota = 6, KF = 53
      10'd438:  phinc = {12'd1820};  // nota = 6, KF = 54
      10'd439:  phinc = {12'd1822};  // nota = 6, KF = 55
      10'd440:  phinc = {12'd1824};  // nota = 6, KF = 56
      10'd441:  phinc = {12'd1825};  // nota = 6, KF = 57
      10'd442:  phinc = {12'd1827};  // nota = 6, KF = 58
      10'd443:  phinc = {12'd1828};  // nota = 6, KF = 59
      10'd444:  phinc = {12'd1830};  // nota = 6, KF = 60
      10'd445:  phinc = {12'd1832};  // nota = 6, KF = 61
      10'd446:  phinc = {12'd1833};  // nota = 6, KF = 62
      10'd447:  phinc = {12'd1835};  // nota = 6, KF = 63
      10'd448:  phinc = {12'd1734};  // nota = 7, KF = 0
      10'd449:  phinc = {12'd1735};  // nota = 7, KF = 1
      10'd450:  phinc = {12'd1737};  // nota = 7, KF = 2
      10'd451:  phinc = {12'd1738};  // nota = 7, KF = 3
      10'd452:  phinc = {12'd1740};  // nota = 7, KF = 4
      10'd453:  phinc = {12'd1741};  // nota = 7, KF = 5
      10'd454:  phinc = {12'd1743};  // nota = 7, KF = 6
      10'd455:  phinc = {12'd1744};  // nota = 7, KF = 7
      10'd456:  phinc = {12'd1746};  // nota = 7, KF = 8
      10'd457:  phinc = {12'd1748};  // nota = 7, KF = 9
      10'd458:  phinc = {12'd1749};  // nota = 7, KF = 10
      10'd459:  phinc = {12'd1751};  // nota = 7, KF = 11
      10'd460:  phinc = {12'd1752};  // nota = 7, KF = 12
      10'd461:  phinc = {12'd1754};  // nota = 7, KF = 13
      10'd462:  phinc = {12'd1755};  // nota = 7, KF = 14
      10'd463:  phinc = {12'd1757};  // nota = 7, KF = 15
      10'd464:  phinc = {12'd1759};  // nota = 7, KF = 16
      10'd465:  phinc = {12'd1760};  // nota = 7, KF = 17
      10'd466:  phinc = {12'd1762};  // nota = 7, KF = 18
      10'd467:  phinc = {12'd1763};  // nota = 7, KF = 19
      10'd468:  phinc = {12'd1765};  // nota = 7, KF = 20
      10'd469:  phinc = {12'd1766};  // nota = 7, KF = 21
      10'd470:  phinc = {12'd1768};  // nota = 7, KF = 22
      10'd471:  phinc = {12'd1769};  // nota = 7, KF = 23
      10'd472:  phinc = {12'd1771};  // nota = 7, KF = 24
      10'd473:  phinc = {12'd1773};  // nota = 7, KF = 25
      10'd474:  phinc = {12'd1774};  // nota = 7, KF = 26
      10'd475:  phinc = {12'd1776};  // nota = 7, KF = 27
      10'd476:  phinc = {12'd1777};  // nota = 7, KF = 28
      10'd477:  phinc = {12'd1779};  // nota = 7, KF = 29
      10'd478:  phinc = {12'd1780};  // nota = 7, KF = 30
      10'd479:  phinc = {12'd1782};  // nota = 7, KF = 31
      10'd480:  phinc = {12'd1785};  // nota = 7, KF = 32
      10'd481:  phinc = {12'd1786};  // nota = 7, KF = 33
      10'd482:  phinc = {12'd1788};  // nota = 7, KF = 34
      10'd483:  phinc = {12'd1789};  // nota = 7, KF = 35
      10'd484:  phinc = {12'd1791};  // nota = 7, KF = 36
      10'd485:  phinc = {12'd1793};  // nota = 7, KF = 37
      10'd486:  phinc = {12'd1794};  // nota = 7, KF = 38
      10'd487:  phinc = {12'd1796};  // nota = 7, KF = 39
      10'd488:  phinc = {12'd1798};  // nota = 7, KF = 40
      10'd489:  phinc = {12'd1799};  // nota = 7, KF = 41
      10'd490:  phinc = {12'd1801};  // nota = 7, KF = 42
      10'd491:  phinc = {12'd1802};  // nota = 7, KF = 43
      10'd492:  phinc = {12'd1804};  // nota = 7, KF = 44
      10'd493:  phinc = {12'd1806};  // nota = 7, KF = 45
      10'd494:  phinc = {12'd1807};  // nota = 7, KF = 46
      10'd495:  phinc = {12'd1809};  // nota = 7, KF = 47
      10'd496:  phinc = {12'd1811};  // nota = 7, KF = 48
      10'd497:  phinc = {12'd1812};  // nota = 7, KF = 49
      10'd498:  phinc = {12'd1814};  // nota = 7, KF = 50
      10'd499:  phinc = {12'd1815};  // nota = 7, KF = 51
      10'd500:  phinc = {12'd1817};  // nota = 7, KF = 52
      10'd501:  phinc = {12'd1819};  // nota = 7, KF = 53
      10'd502:  phinc = {12'd1820};  // nota = 7, KF = 54
      10'd503:  phinc = {12'd1822};  // nota = 7, KF = 55
      10'd504:  phinc = {12'd1824};  // nota = 7, KF = 56
      10'd505:  phinc = {12'd1825};  // nota = 7, KF = 57
      10'd506:  phinc = {12'd1827};  // nota = 7, KF = 58
      10'd507:  phinc = {12'd1828};  // nota = 7, KF = 59
      10'd508:  phinc = {12'd1830};  // nota = 7, KF = 60
      10'd509:  phinc = {12'd1832};  // nota = 7, KF = 61
      10'd510:  phinc = {12'd1833};  // nota = 7, KF = 62
      10'd511:  phinc = {12'd1835};  // nota = 7, KF = 63
      10'd512:  phinc = {12'd1837};  // nota = 8, KF = 0
      10'd513:  phinc = {12'd1838};  // nota = 8, KF = 1
      10'd514:  phinc = {12'd1840};  // nota = 8, KF = 2
      10'd515:  phinc = {12'd1841};  // nota = 8, KF = 3
      10'd516:  phinc = {12'd1843};  // nota = 8, KF = 4
      10'd517:  phinc = {12'd1845};  // nota = 8, KF = 5
      10'd518:  phinc = {12'd1846};  // nota = 8, KF = 6
      10'd519:  phinc = {12'd1848};  // nota = 8, KF = 7
      10'd520:  phinc = {12'd1850};  // nota = 8, KF = 8
      10'd521:  phinc = {12'd1851};  // nota = 8, KF = 9
      10'd522:  phinc = {12'd1853};  // nota = 8, KF = 10
      10'd523:  phinc = {12'd1854};  // nota = 8, KF = 11
      10'd524:  phinc = {12'd1856};  // nota = 8, KF = 12
      10'd525:  phinc = {12'd1858};  // nota = 8, KF = 13
      10'd526:  phinc = {12'd1859};  // nota = 8, KF = 14
      10'd527:  phinc = {12'd1861};  // nota = 8, KF = 15
      10'd528:  phinc = {12'd1864};  // nota = 8, KF = 16
      10'd529:  phinc = {12'd1865};  // nota = 8, KF = 17
      10'd530:  phinc = {12'd1867};  // nota = 8, KF = 18
      10'd531:  phinc = {12'd1868};  // nota = 8, KF = 19
      10'd532:  phinc = {12'd1870};  // nota = 8, KF = 20
      10'd533:  phinc = {12'd1872};  // nota = 8, KF = 21
      10'd534:  phinc = {12'd1873};  // nota = 8, KF = 22
      10'd535:  phinc = {12'd1875};  // nota = 8, KF = 23
      10'd536:  phinc = {12'd1877};  // nota = 8, KF = 24
      10'd537:  phinc = {12'd1879};  // nota = 8, KF = 25
      10'd538:  phinc = {12'd1880};  // nota = 8, KF = 26
      10'd539:  phinc = {12'd1882};  // nota = 8, KF = 27
      10'd540:  phinc = {12'd1884};  // nota = 8, KF = 28
      10'd541:  phinc = {12'd1885};  // nota = 8, KF = 29
      10'd542:  phinc = {12'd1887};  // nota = 8, KF = 30
      10'd543:  phinc = {12'd1888};  // nota = 8, KF = 31
      10'd544:  phinc = {12'd1891};  // nota = 8, KF = 32
      10'd545:  phinc = {12'd1892};  // nota = 8, KF = 33
      10'd546:  phinc = {12'd1894};  // nota = 8, KF = 34
      10'd547:  phinc = {12'd1895};  // nota = 8, KF = 35
      10'd548:  phinc = {12'd1897};  // nota = 8, KF = 36
      10'd549:  phinc = {12'd1899};  // nota = 8, KF = 37
      10'd550:  phinc = {12'd1900};  // nota = 8, KF = 38
      10'd551:  phinc = {12'd1902};  // nota = 8, KF = 39
      10'd552:  phinc = {12'd1904};  // nota = 8, KF = 40
      10'd553:  phinc = {12'd1906};  // nota = 8, KF = 41
      10'd554:  phinc = {12'd1907};  // nota = 8, KF = 42
      10'd555:  phinc = {12'd1909};  // nota = 8, KF = 43
      10'd556:  phinc = {12'd1911};  // nota = 8, KF = 44
      10'd557:  phinc = {12'd1912};  // nota = 8, KF = 45
      10'd558:  phinc = {12'd1914};  // nota = 8, KF = 46
      10'd559:  phinc = {12'd1915};  // nota = 8, KF = 47
      10'd560:  phinc = {12'd1918};  // nota = 8, KF = 48
      10'd561:  phinc = {12'd1919};  // nota = 8, KF = 49
      10'd562:  phinc = {12'd1921};  // nota = 8, KF = 50
      10'd563:  phinc = {12'd1923};  // nota = 8, KF = 51
      10'd564:  phinc = {12'd1925};  // nota = 8, KF = 52
      10'd565:  phinc = {12'd1926};  // nota = 8, KF = 53
      10'd566:  phinc = {12'd1928};  // nota = 8, KF = 54
      10'd567:  phinc = {12'd1930};  // nota = 8, KF = 55
      10'd568:  phinc = {12'd1932};  // nota = 8, KF = 56
      10'd569:  phinc = {12'd1933};  // nota = 8, KF = 57
      10'd570:  phinc = {12'd1935};  // nota = 8, KF = 58
      10'd571:  phinc = {12'd1937};  // nota = 8, KF = 59
      10'd572:  phinc = {12'd1939};  // nota = 8, KF = 60
      10'd573:  phinc = {12'd1940};  // nota = 8, KF = 61
      10'd574:  phinc = {12'd1942};  // nota = 8, KF = 62
      10'd575:  phinc = {12'd1944};  // nota = 8, KF = 63
      10'd576:  phinc = {12'd1946};  // nota = 9, KF = 0
      10'd577:  phinc = {12'd1947};  // nota = 9, KF = 1
      10'd578:  phinc = {12'd1949};  // nota = 9, KF = 2
      10'd579:  phinc = {12'd1951};  // nota = 9, KF = 3
      10'd580:  phinc = {12'd1953};  // nota = 9, KF = 4
      10'd581:  phinc = {12'd1954};  // nota = 9, KF = 5
      10'd582:  phinc = {12'd1956};  // nota = 9, KF = 6
      10'd583:  phinc = {12'd1958};  // nota = 9, KF = 7
      10'd584:  phinc = {12'd1960};  // nota = 9, KF = 8
      10'd585:  phinc = {12'd1961};  // nota = 9, KF = 9
      10'd586:  phinc = {12'd1963};  // nota = 9, KF = 10
      10'd587:  phinc = {12'd1965};  // nota = 9, KF = 11
      10'd588:  phinc = {12'd1967};  // nota = 9, KF = 12
      10'd589:  phinc = {12'd1968};  // nota = 9, KF = 13
      10'd590:  phinc = {12'd1970};  // nota = 9, KF = 14
      10'd591:  phinc = {12'd1972};  // nota = 9, KF = 15
      10'd592:  phinc = {12'd1975};  // nota = 9, KF = 16
      10'd593:  phinc = {12'd1976};  // nota = 9, KF = 17
      10'd594:  phinc = {12'd1978};  // nota = 9, KF = 18
      10'd595:  phinc = {12'd1980};  // nota = 9, KF = 19
      10'd596:  phinc = {12'd1982};  // nota = 9, KF = 20
      10'd597:  phinc = {12'd1983};  // nota = 9, KF = 21
      10'd598:  phinc = {12'd1985};  // nota = 9, KF = 22
      10'd599:  phinc = {12'd1987};  // nota = 9, KF = 23
      10'd600:  phinc = {12'd1989};  // nota = 9, KF = 24
      10'd601:  phinc = {12'd1990};  // nota = 9, KF = 25
      10'd602:  phinc = {12'd1992};  // nota = 9, KF = 26
      10'd603:  phinc = {12'd1994};  // nota = 9, KF = 27
      10'd604:  phinc = {12'd1996};  // nota = 9, KF = 28
      10'd605:  phinc = {12'd1997};  // nota = 9, KF = 29
      10'd606:  phinc = {12'd1999};  // nota = 9, KF = 30
      10'd607:  phinc = {12'd2001};  // nota = 9, KF = 31
      10'd608:  phinc = {12'd2003};  // nota = 9, KF = 32
      10'd609:  phinc = {12'd2004};  // nota = 9, KF = 33
      10'd610:  phinc = {12'd2006};  // nota = 9, KF = 34
      10'd611:  phinc = {12'd2008};  // nota = 9, KF = 35
      10'd612:  phinc = {12'd2010};  // nota = 9, KF = 36
      10'd613:  phinc = {12'd2011};  // nota = 9, KF = 37
      10'd614:  phinc = {12'd2013};  // nota = 9, KF = 38
      10'd615:  phinc = {12'd2015};  // nota = 9, KF = 39
      10'd616:  phinc = {12'd2017};  // nota = 9, KF = 40
      10'd617:  phinc = {12'd2019};  // nota = 9, KF = 41
      10'd618:  phinc = {12'd2021};  // nota = 9, KF = 42
      10'd619:  phinc = {12'd2022};  // nota = 9, KF = 43
      10'd620:  phinc = {12'd2024};  // nota = 9, KF = 44
      10'd621:  phinc = {12'd2026};  // nota = 9, KF = 45
      10'd622:  phinc = {12'd2028};  // nota = 9, KF = 46
      10'd623:  phinc = {12'd2029};  // nota = 9, KF = 47
      10'd624:  phinc = {12'd2032};  // nota = 9, KF = 48
      10'd625:  phinc = {12'd2033};  // nota = 9, KF = 49
      10'd626:  phinc = {12'd2035};  // nota = 9, KF = 50
      10'd627:  phinc = {12'd2037};  // nota = 9, KF = 51
      10'd628:  phinc = {12'd2039};  // nota = 9, KF = 52
      10'd629:  phinc = {12'd2041};  // nota = 9, KF = 53
      10'd630:  phinc = {12'd2043};  // nota = 9, KF = 54
      10'd631:  phinc = {12'd2044};  // nota = 9, KF = 55
      10'd632:  phinc = {12'd2047};  // nota = 9, KF = 56
      10'd633:  phinc = {12'd2048};  // nota = 9, KF = 57
      10'd634:  phinc = {12'd2050};  // nota = 9, KF = 58
      10'd635:  phinc = {12'd2052};  // nota = 9, KF = 59
      10'd636:  phinc = {12'd2054};  // nota = 9, KF = 60
      10'd637:  phinc = {12'd2056};  // nota = 9, KF = 61
      10'd638:  phinc = {12'd2058};  // nota = 9, KF = 62
      10'd639:  phinc = {12'd2059};  // nota = 9, KF = 63
      10'd640:  phinc = {12'd2062};  // nota = 10, KF = 0
      10'd641:  phinc = {12'd2063};  // nota = 10, KF = 1
      10'd642:  phinc = {12'd2065};  // nota = 10, KF = 2
      10'd643:  phinc = {12'd2067};  // nota = 10, KF = 3
      10'd644:  phinc = {12'd2069};  // nota = 10, KF = 4
      10'd645:  phinc = {12'd2071};  // nota = 10, KF = 5
      10'd646:  phinc = {12'd2073};  // nota = 10, KF = 6
      10'd647:  phinc = {12'd2074};  // nota = 10, KF = 7
      10'd648:  phinc = {12'd2077};  // nota = 10, KF = 8
      10'd649:  phinc = {12'd2078};  // nota = 10, KF = 9
      10'd650:  phinc = {12'd2080};  // nota = 10, KF = 10
      10'd651:  phinc = {12'd2082};  // nota = 10, KF = 11
      10'd652:  phinc = {12'd2084};  // nota = 10, KF = 12
      10'd653:  phinc = {12'd2086};  // nota = 10, KF = 13
      10'd654:  phinc = {12'd2088};  // nota = 10, KF = 14
      10'd655:  phinc = {12'd2089};  // nota = 10, KF = 15
      10'd656:  phinc = {12'd2092};  // nota = 10, KF = 16
      10'd657:  phinc = {12'd2093};  // nota = 10, KF = 17
      10'd658:  phinc = {12'd2095};  // nota = 10, KF = 18
      10'd659:  phinc = {12'd2097};  // nota = 10, KF = 19
      10'd660:  phinc = {12'd2099};  // nota = 10, KF = 20
      10'd661:  phinc = {12'd2101};  // nota = 10, KF = 21
      10'd662:  phinc = {12'd2103};  // nota = 10, KF = 22
      10'd663:  phinc = {12'd2104};  // nota = 10, KF = 23
      10'd664:  phinc = {12'd2107};  // nota = 10, KF = 24
      10'd665:  phinc = {12'd2108};  // nota = 10, KF = 25
      10'd666:  phinc = {12'd2110};  // nota = 10, KF = 26
      10'd667:  phinc = {12'd2112};  // nota = 10, KF = 27
      10'd668:  phinc = {12'd2114};  // nota = 10, KF = 28
      10'd669:  phinc = {12'd2116};  // nota = 10, KF = 29
      10'd670:  phinc = {12'd2118};  // nota = 10, KF = 30
      10'd671:  phinc = {12'd2119};  // nota = 10, KF = 31
      10'd672:  phinc = {12'd2122};  // nota = 10, KF = 32
      10'd673:  phinc = {12'd2123};  // nota = 10, KF = 33
      10'd674:  phinc = {12'd2125};  // nota = 10, KF = 34
      10'd675:  phinc = {12'd2127};  // nota = 10, KF = 35
      10'd676:  phinc = {12'd2129};  // nota = 10, KF = 36
      10'd677:  phinc = {12'd2131};  // nota = 10, KF = 37
      10'd678:  phinc = {12'd2133};  // nota = 10, KF = 38
      10'd679:  phinc = {12'd2134};  // nota = 10, KF = 39
      10'd680:  phinc = {12'd2137};  // nota = 10, KF = 40
      10'd681:  phinc = {12'd2139};  // nota = 10, KF = 41
      10'd682:  phinc = {12'd2141};  // nota = 10, KF = 42
      10'd683:  phinc = {12'd2142};  // nota = 10, KF = 43
      10'd684:  phinc = {12'd2145};  // nota = 10, KF = 44
      10'd685:  phinc = {12'd2146};  // nota = 10, KF = 45
      10'd686:  phinc = {12'd2148};  // nota = 10, KF = 46
      10'd687:  phinc = {12'd2150};  // nota = 10, KF = 47
      10'd688:  phinc = {12'd2153};  // nota = 10, KF = 48
      10'd689:  phinc = {12'd2154};  // nota = 10, KF = 49
      10'd690:  phinc = {12'd2156};  // nota = 10, KF = 50
      10'd691:  phinc = {12'd2158};  // nota = 10, KF = 51
      10'd692:  phinc = {12'd2160};  // nota = 10, KF = 52
      10'd693:  phinc = {12'd2162};  // nota = 10, KF = 53
      10'd694:  phinc = {12'd2164};  // nota = 10, KF = 54
      10'd695:  phinc = {12'd2165};  // nota = 10, KF = 55
      10'd696:  phinc = {12'd2168};  // nota = 10, KF = 56
      10'd697:  phinc = {12'd2170};  // nota = 10, KF = 57
      10'd698:  phinc = {12'd2172};  // nota = 10, KF = 58
      10'd699:  phinc = {12'd2173};  // nota = 10, KF = 59
      10'd700:  phinc = {12'd2176};  // nota = 10, KF = 60
      10'd701:  phinc = {12'd2177};  // nota = 10, KF = 61
      10'd702:  phinc = {12'd2179};  // nota = 10, KF = 62
      10'd703:  phinc = {12'd2181};  // nota = 10, KF = 63
      10'd704:  phinc = {12'd2062};  // nota = 11, KF = 0
      10'd705:  phinc = {12'd2063};  // nota = 11, KF = 1
      10'd706:  phinc = {12'd2065};  // nota = 11, KF = 2
      10'd707:  phinc = {12'd2067};  // nota = 11, KF = 3
      10'd708:  phinc = {12'd2069};  // nota = 11, KF = 4
      10'd709:  phinc = {12'd2071};  // nota = 11, KF = 5
      10'd710:  phinc = {12'd2073};  // nota = 11, KF = 6
      10'd711:  phinc = {12'd2074};  // nota = 11, KF = 7
      10'd712:  phinc = {12'd2077};  // nota = 11, KF = 8
      10'd713:  phinc = {12'd2078};  // nota = 11, KF = 9
      10'd714:  phinc = {12'd2080};  // nota = 11, KF = 10
      10'd715:  phinc = {12'd2082};  // nota = 11, KF = 11
      10'd716:  phinc = {12'd2084};  // nota = 11, KF = 12
      10'd717:  phinc = {12'd2086};  // nota = 11, KF = 13
      10'd718:  phinc = {12'd2088};  // nota = 11, KF = 14
      10'd719:  phinc = {12'd2089};  // nota = 11, KF = 15
      10'd720:  phinc = {12'd2092};  // nota = 11, KF = 16
      10'd721:  phinc = {12'd2093};  // nota = 11, KF = 17
      10'd722:  phinc = {12'd2095};  // nota = 11, KF = 18
      10'd723:  phinc = {12'd2097};  // nota = 11, KF = 19
      10'd724:  phinc = {12'd2099};  // nota = 11, KF = 20
      10'd725:  phinc = {12'd2101};  // nota = 11, KF = 21
      10'd726:  phinc = {12'd2103};  // nota = 11, KF = 22
      10'd727:  phinc = {12'd2104};  // nota = 11, KF = 23
      10'd728:  phinc = {12'd2107};  // nota = 11, KF = 24
      10'd729:  phinc = {12'd2108};  // nota = 11, KF = 25
      10'd730:  phinc = {12'd2110};  // nota = 11, KF = 26
      10'd731:  phinc = {12'd2112};  // nota = 11, KF = 27
      10'd732:  phinc = {12'd2114};  // nota = 11, KF = 28
      10'd733:  phinc = {12'd2116};  // nota = 11, KF = 29
      10'd734:  phinc = {12'd2118};  // nota = 11, KF = 30
      10'd735:  phinc = {12'd2119};  // nota = 11, KF = 31
      10'd736:  phinc = {12'd2122};  // nota = 11, KF = 32
      10'd737:  phinc = {12'd2123};  // nota = 11, KF = 33
      10'd738:  phinc = {12'd2125};  // nota = 11, KF = 34
      10'd739:  phinc = {12'd2127};  // nota = 11, KF = 35
      10'd740:  phinc = {12'd2129};  // nota = 11, KF = 36
      10'd741:  phinc = {12'd2131};  // nota = 11, KF = 37
      10'd742:  phinc = {12'd2133};  // nota = 11, KF = 38
      10'd743:  phinc = {12'd2134};  // nota = 11, KF = 39
      10'd744:  phinc = {12'd2137};  // nota = 11, KF = 40
      10'd745:  phinc = {12'd2139};  // nota = 11, KF = 41
      10'd746:  phinc = {12'd2141};  // nota = 11, KF = 42
      10'd747:  phinc = {12'd2142};  // nota = 11, KF = 43
      10'd748:  phinc = {12'd2145};  // nota = 11, KF = 44
      10'd749:  phinc = {12'd2146};  // nota = 11, KF = 45
      10'd750:  phinc = {12'd2148};  // nota = 11, KF = 46
      10'd751:  phinc = {12'd2150};  // nota = 11, KF = 47
      10'd752:  phinc = {12'd2153};  // nota = 11, KF = 48
      10'd753:  phinc = {12'd2154};  // nota = 11, KF = 49
      10'd754:  phinc = {12'd2156};  // nota = 11, KF = 50
      10'd755:  phinc = {12'd2158};  // nota = 11, KF = 51
      10'd756:  phinc = {12'd2160};  // nota = 11, KF = 52
      10'd757:  phinc = {12'd2162};  // nota = 11, KF = 53
      10'd758:  phinc = {12'd2164};  // nota = 11, KF = 54
      10'd759:  phinc = {12'd2165};  // nota = 11, KF = 55
      10'd760:  phinc = {12'd2168};  // nota = 11, KF = 56
      10'd761:  phinc = {12'd2170};  // nota = 11, KF = 57
      10'd762:  phinc = {12'd2172};  // nota = 11, KF = 58
      10'd763:  phinc = {12'd2173};  // nota = 11, KF = 59
      10'd764:  phinc = {12'd2176};  // nota = 11, KF = 60
      10'd765:  phinc = {12'd2177};  // nota = 11, KF = 61
      10'd766:  phinc = {12'd2179};  // nota = 11, KF = 62
      10'd767:  phinc = {12'd2181};  // nota = 11, KF = 63
      10'd768:  phinc = {12'd2185};  // nota = 12, KF = 0
      10'd769:  phinc = {12'd2186};  // nota = 12, KF = 1
      10'd770:  phinc = {12'd2188};  // nota = 12, KF = 2
      10'd771:  phinc = {12'd2190};  // nota = 12, KF = 3
      10'd772:  phinc = {12'd2192};  // nota = 12, KF = 4
      10'd773:  phinc = {12'd2194};  // nota = 12, KF = 5
      10'd774:  phinc = {12'd2196};  // nota = 12, KF = 6
      10'd775:  phinc = {12'd2197};  // nota = 12, KF = 7
      10'd776:  phinc = {12'd2200};  // nota = 12, KF = 8
      10'd777:  phinc = {12'd2202};  // nota = 12, KF = 9
      10'd778:  phinc = {12'd2204};  // nota = 12, KF = 10
      10'd779:  phinc = {12'd2205};  // nota = 12, KF = 11
      10'd780:  phinc = {12'd2208};  // nota = 12, KF = 12
      10'd781:  phinc = {12'd2209};  // nota = 12, KF = 13
      10'd782:  phinc = {12'd2211};  // nota = 12, KF = 14
      10'd783:  phinc = {12'd2213};  // nota = 12, KF = 15
      10'd784:  phinc = {12'd2216};  // nota = 12, KF = 16
      10'd785:  phinc = {12'd2218};  // nota = 12, KF = 17
      10'd786:  phinc = {12'd2220};  // nota = 12, KF = 18
      10'd787:  phinc = {12'd2222};  // nota = 12, KF = 19
      10'd788:  phinc = {12'd2223};  // nota = 12, KF = 20
      10'd789:  phinc = {12'd2226};  // nota = 12, KF = 21
      10'd790:  phinc = {12'd2227};  // nota = 12, KF = 22
      10'd791:  phinc = {12'd2230};  // nota = 12, KF = 23
      10'd792:  phinc = {12'd2232};  // nota = 12, KF = 24
      10'd793:  phinc = {12'd2234};  // nota = 12, KF = 25
      10'd794:  phinc = {12'd2236};  // nota = 12, KF = 26
      10'd795:  phinc = {12'd2238};  // nota = 12, KF = 27
      10'd796:  phinc = {12'd2239};  // nota = 12, KF = 28
      10'd797:  phinc = {12'd2242};  // nota = 12, KF = 29
      10'd798:  phinc = {12'd2243};  // nota = 12, KF = 30
      10'd799:  phinc = {12'd2246};  // nota = 12, KF = 31
      10'd800:  phinc = {12'd2249};  // nota = 12, KF = 32
      10'd801:  phinc = {12'd2251};  // nota = 12, KF = 33
      10'd802:  phinc = {12'd2253};  // nota = 12, KF = 34
      10'd803:  phinc = {12'd2255};  // nota = 12, KF = 35
      10'd804:  phinc = {12'd2256};  // nota = 12, KF = 36
      10'd805:  phinc = {12'd2259};  // nota = 12, KF = 37
      10'd806:  phinc = {12'd2260};  // nota = 12, KF = 38
      10'd807:  phinc = {12'd2263};  // nota = 12, KF = 39
      10'd808:  phinc = {12'd2265};  // nota = 12, KF = 40
      10'd809:  phinc = {12'd2267};  // nota = 12, KF = 41
      10'd810:  phinc = {12'd2269};  // nota = 12, KF = 42
      10'd811:  phinc = {12'd2271};  // nota = 12, KF = 43
      10'd812:  phinc = {12'd2272};  // nota = 12, KF = 44
      10'd813:  phinc = {12'd2275};  // nota = 12, KF = 45
      10'd814:  phinc = {12'd2276};  // nota = 12, KF = 46
      10'd815:  phinc = {12'd2279};  // nota = 12, KF = 47
      10'd816:  phinc = {12'd2281};  // nota = 12, KF = 48
      10'd817:  phinc = {12'd2283};  // nota = 12, KF = 49
      10'd818:  phinc = {12'd2285};  // nota = 12, KF = 50
      10'd819:  phinc = {12'd2287};  // nota = 12, KF = 51
      10'd820:  phinc = {12'd2288};  // nota = 12, KF = 52
      10'd821:  phinc = {12'd2291};  // nota = 12, KF = 53
      10'd822:  phinc = {12'd2292};  // nota = 12, KF = 54
      10'd823:  phinc = {12'd2295};  // nota = 12, KF = 55
      10'd824:  phinc = {12'd2297};  // nota = 12, KF = 56
      10'd825:  phinc = {12'd2299};  // nota = 12, KF = 57
      10'd826:  phinc = {12'd2301};  // nota = 12, KF = 58
      10'd827:  phinc = {12'd2303};  // nota = 12, KF = 59
      10'd828:  phinc = {12'd2304};  // nota = 12, KF = 60
      10'd829:  phinc = {12'd2307};  // nota = 12, KF = 61
      10'd830:  phinc = {12'd2308};  // nota = 12, KF = 62
      10'd831:  phinc = {12'd2311};  // nota = 12, KF = 63
      10'd832:  phinc = {12'd2315};  // nota = 13, KF = 0
      10'd833:  phinc = {12'd2317};  // nota = 13, KF = 1
      10'd834:  phinc = {12'd2319};  // nota = 13, KF = 2
      10'd835:  phinc = {12'd2321};  // nota = 13, KF = 3
      10'd836:  phinc = {12'd2322};  // nota = 13, KF = 4
      10'd837:  phinc = {12'd2325};  // nota = 13, KF = 5
      10'd838:  phinc = {12'd2326};  // nota = 13, KF = 6
      10'd839:  phinc = {12'd2329};  // nota = 13, KF = 7
      10'd840:  phinc = {12'd2331};  // nota = 13, KF = 8
      10'd841:  phinc = {12'd2333};  // nota = 13, KF = 9
      10'd842:  phinc = {12'd2335};  // nota = 13, KF = 10
      10'd843:  phinc = {12'd2337};  // nota = 13, KF = 11
      10'd844:  phinc = {12'd2338};  // nota = 13, KF = 12
      10'd845:  phinc = {12'd2341};  // nota = 13, KF = 13
      10'd846:  phinc = {12'd2342};  // nota = 13, KF = 14
      10'd847:  phinc = {12'd2345};  // nota = 13, KF = 15
      10'd848:  phinc = {12'd2348};  // nota = 13, KF = 16
      10'd849:  phinc = {12'd2350};  // nota = 13, KF = 17
      10'd850:  phinc = {12'd2352};  // nota = 13, KF = 18
      10'd851:  phinc = {12'd2354};  // nota = 13, KF = 19
      10'd852:  phinc = {12'd2355};  // nota = 13, KF = 20
      10'd853:  phinc = {12'd2358};  // nota = 13, KF = 21
      10'd854:  phinc = {12'd2359};  // nota = 13, KF = 22
      10'd855:  phinc = {12'd2362};  // nota = 13, KF = 23
      10'd856:  phinc = {12'd2364};  // nota = 13, KF = 24
      10'd857:  phinc = {12'd2366};  // nota = 13, KF = 25
      10'd858:  phinc = {12'd2368};  // nota = 13, KF = 26
      10'd859:  phinc = {12'd2370};  // nota = 13, KF = 27
      10'd860:  phinc = {12'd2371};  // nota = 13, KF = 28
      10'd861:  phinc = {12'd2374};  // nota = 13, KF = 29
      10'd862:  phinc = {12'd2375};  // nota = 13, KF = 30
      10'd863:  phinc = {12'd2378};  // nota = 13, KF = 31
      10'd864:  phinc = {12'd2382};  // nota = 13, KF = 32
      10'd865:  phinc = {12'd2384};  // nota = 13, KF = 33
      10'd866:  phinc = {12'd2386};  // nota = 13, KF = 34
      10'd867:  phinc = {12'd2388};  // nota = 13, KF = 35
      10'd868:  phinc = {12'd2389};  // nota = 13, KF = 36
      10'd869:  phinc = {12'd2392};  // nota = 13, KF = 37
      10'd870:  phinc = {12'd2393};  // nota = 13, KF = 38
      10'd871:  phinc = {12'd2396};  // nota = 13, KF = 39
      10'd872:  phinc = {12'd2398};  // nota = 13, KF = 40
      10'd873:  phinc = {12'd2400};  // nota = 13, KF = 41
      10'd874:  phinc = {12'd2402};  // nota = 13, KF = 42
      10'd875:  phinc = {12'd2404};  // nota = 13, KF = 43
      10'd876:  phinc = {12'd2407};  // nota = 13, KF = 44
      10'd877:  phinc = {12'd2410};  // nota = 13, KF = 45
      10'd878:  phinc = {12'd2411};  // nota = 13, KF = 46
      10'd879:  phinc = {12'd2414};  // nota = 13, KF = 47
      10'd880:  phinc = {12'd2417};  // nota = 13, KF = 48
      10'd881:  phinc = {12'd2419};  // nota = 13, KF = 49
      10'd882:  phinc = {12'd2421};  // nota = 13, KF = 50
      10'd883:  phinc = {12'd2423};  // nota = 13, KF = 51
      10'd884:  phinc = {12'd2424};  // nota = 13, KF = 52
      10'd885:  phinc = {12'd2427};  // nota = 13, KF = 53
      10'd886:  phinc = {12'd2428};  // nota = 13, KF = 54
      10'd887:  phinc = {12'd2431};  // nota = 13, KF = 55
      10'd888:  phinc = {12'd2433};  // nota = 13, KF = 56
      10'd889:  phinc = {12'd2435};  // nota = 13, KF = 57
      10'd890:  phinc = {12'd2437};  // nota = 13, KF = 58
      10'd891:  phinc = {12'd2439};  // nota = 13, KF = 59
      10'd892:  phinc = {12'd2442};  // nota = 13, KF = 60
      10'd893:  phinc = {12'd2445};  // nota = 13, KF = 61
      10'd894:  phinc = {12'd2446};  // nota = 13, KF = 62
      10'd895:  phinc = {12'd2449};  // nota = 13, KF = 63
      10'd896:  phinc = {12'd2452};  // nota = 14, KF = 0
      10'd897:  phinc = {12'd2454};  // nota = 14, KF = 1
      10'd898:  phinc = {12'd2456};  // nota = 14, KF = 2
      10'd899:  phinc = {12'd2458};  // nota = 14, KF = 3
      10'd900:  phinc = {12'd2459};  // nota = 14, KF = 4
      10'd901:  phinc = {12'd2462};  // nota = 14, KF = 5
      10'd902:  phinc = {12'd2463};  // nota = 14, KF = 6
      10'd903:  phinc = {12'd2466};  // nota = 14, KF = 7
      10'd904:  phinc = {12'd2468};  // nota = 14, KF = 8
      10'd905:  phinc = {12'd2470};  // nota = 14, KF = 9
      10'd906:  phinc = {12'd2472};  // nota = 14, KF = 10
      10'd907:  phinc = {12'd2474};  // nota = 14, KF = 11
      10'd908:  phinc = {12'd2477};  // nota = 14, KF = 12
      10'd909:  phinc = {12'd2480};  // nota = 14, KF = 13
      10'd910:  phinc = {12'd2481};  // nota = 14, KF = 14
      10'd911:  phinc = {12'd2484};  // nota = 14, KF = 15
      10'd912:  phinc = {12'd2488};  // nota = 14, KF = 16
      10'd913:  phinc = {12'd2490};  // nota = 14, KF = 17
      10'd914:  phinc = {12'd2492};  // nota = 14, KF = 18
      10'd915:  phinc = {12'd2494};  // nota = 14, KF = 19
      10'd916:  phinc = {12'd2495};  // nota = 14, KF = 20
      10'd917:  phinc = {12'd2498};  // nota = 14, KF = 21
      10'd918:  phinc = {12'd2499};  // nota = 14, KF = 22
      10'd919:  phinc = {12'd2502};  // nota = 14, KF = 23
      10'd920:  phinc = {12'd2504};  // nota = 14, KF = 24
      10'd921:  phinc = {12'd2506};  // nota = 14, KF = 25
      10'd922:  phinc = {12'd2508};  // nota = 14, KF = 26
      10'd923:  phinc = {12'd2510};  // nota = 14, KF = 27
      10'd924:  phinc = {12'd2513};  // nota = 14, KF = 28
      10'd925:  phinc = {12'd2516};  // nota = 14, KF = 29
      10'd926:  phinc = {12'd2517};  // nota = 14, KF = 30
      10'd927:  phinc = {12'd2520};  // nota = 14, KF = 31
      10'd928:  phinc = {12'd2524};  // nota = 14, KF = 32
      10'd929:  phinc = {12'd2526};  // nota = 14, KF = 33
      10'd930:  phinc = {12'd2528};  // nota = 14, KF = 34
      10'd931:  phinc = {12'd2530};  // nota = 14, KF = 35
      10'd932:  phinc = {12'd2531};  // nota = 14, KF = 36
      10'd933:  phinc = {12'd2534};  // nota = 14, KF = 37
      10'd934:  phinc = {12'd2535};  // nota = 14, KF = 38
      10'd935:  phinc = {12'd2538};  // nota = 14, KF = 39
      10'd936:  phinc = {12'd2540};  // nota = 14, KF = 40
      10'd937:  phinc = {12'd2542};  // nota = 14, KF = 41
      10'd938:  phinc = {12'd2544};  // nota = 14, KF = 42
      10'd939:  phinc = {12'd2546};  // nota = 14, KF = 43
      10'd940:  phinc = {12'd2549};  // nota = 14, KF = 44
      10'd941:  phinc = {12'd2552};  // nota = 14, KF = 45
      10'd942:  phinc = {12'd2553};  // nota = 14, KF = 46
      10'd943:  phinc = {12'd2556};  // nota = 14, KF = 47
      10'd944:  phinc = {12'd2561};  // nota = 14, KF = 48
      10'd945:  phinc = {12'd2563};  // nota = 14, KF = 49
      10'd946:  phinc = {12'd2565};  // nota = 14, KF = 50
      10'd947:  phinc = {12'd2567};  // nota = 14, KF = 51
      10'd948:  phinc = {12'd2568};  // nota = 14, KF = 52
      10'd949:  phinc = {12'd2571};  // nota = 14, KF = 53
      10'd950:  phinc = {12'd2572};  // nota = 14, KF = 54
      10'd951:  phinc = {12'd2575};  // nota = 14, KF = 55
      10'd952:  phinc = {12'd2577};  // nota = 14, KF = 56
      10'd953:  phinc = {12'd2579};  // nota = 14, KF = 57
      10'd954:  phinc = {12'd2581};  // nota = 14, KF = 58
      10'd955:  phinc = {12'd2583};  // nota = 14, KF = 59
      10'd956:  phinc = {12'd2586};  // nota = 14, KF = 60
      10'd957:  phinc = {12'd2589};  // nota = 14, KF = 61
      10'd958:  phinc = {12'd2590};  // nota = 14, KF = 62
      10'd959:  phinc = {12'd2593};  // nota = 14, KF = 63
      10'd960:  phinc = {12'd2452};  // nota = 15, KF = 0
      10'd961:  phinc = {12'd2454};  // nota = 15, KF = 1
      10'd962:  phinc = {12'd2456};  // nota = 15, KF = 2
      10'd963:  phinc = {12'd2458};  // nota = 15, KF = 3
      10'd964:  phinc = {12'd2459};  // nota = 15, KF = 4
      10'd965:  phinc = {12'd2462};  // nota = 15, KF = 5
      10'd966:  phinc = {12'd2463};  // nota = 15, KF = 6
      10'd967:  phinc = {12'd2466};  // nota = 15, KF = 7
      10'd968:  phinc = {12'd2468};  // nota = 15, KF = 8
      10'd969:  phinc = {12'd2470};  // nota = 15, KF = 9
      10'd970:  phinc = {12'd2472};  // nota = 15, KF = 10
      10'd971:  phinc = {12'd2474};  // nota = 15, KF = 11
      10'd972:  phinc = {12'd2477};  // nota = 15, KF = 12
      10'd973:  phinc = {12'd2480};  // nota = 15, KF = 13
      10'd974:  phinc = {12'd2481};  // nota = 15, KF = 14
      10'd975:  phinc = {12'd2484};  // nota = 15, KF = 15
      10'd976:  phinc = {12'd2488};  // nota = 15, KF = 16
      10'd977:  phinc = {12'd2490};  // nota = 15, KF = 17
      10'd978:  phinc = {12'd2492};  // nota = 15, KF = 18
      10'd979:  phinc = {12'd2494};  // nota = 15, KF = 19
      10'd980:  phinc = {12'd2495};  // nota = 15, KF = 20
      10'd981:  phinc = {12'd2498};  // nota = 15, KF = 21
      10'd982:  phinc = {12'd2499};  // nota = 15, KF = 22
      10'd983:  phinc = {12'd2502};  // nota = 15, KF = 23
      10'd984:  phinc = {12'd2504};  // nota = 15, KF = 24
      10'd985:  phinc = {12'd2506};  // nota = 15, KF = 25
      10'd986:  phinc = {12'd2508};  // nota = 15, KF = 26
      10'd987:  phinc = {12'd2510};  // nota = 15, KF = 27
      10'd988:  phinc = {12'd2513};  // nota = 15, KF = 28
      10'd989:  phinc = {12'd2516};  // nota = 15, KF = 29
      10'd990:  phinc = {12'd2517};  // nota = 15, KF = 30
      10'd991:  phinc = {12'd2520};  // nota = 15, KF = 31
      10'd992:  phinc = {12'd2524};  // nota = 15, KF = 32
      10'd993:  phinc = {12'd2526};  // nota = 15, KF = 33
      10'd994:  phinc = {12'd2528};  // nota = 15, KF = 34
      10'd995:  phinc = {12'd2530};  // nota = 15, KF = 35
      10'd996:  phinc = {12'd2531};  // nota = 15, KF = 36
      10'd997:  phinc = {12'd2534};  // nota = 15, KF = 37
      10'd998:  phinc = {12'd2535};  // nota = 15, KF = 38
      10'd999:  phinc = {12'd2538};  // nota = 15, KF = 39
      10'd1000: phinc = {12'd2540};  // nota = 15, KF = 40
      10'd1001: phinc = {12'd2542};  // nota = 15, KF = 41
      10'd1002: phinc = {12'd2544};  // nota = 15, KF = 42
      10'd1003: phinc = {12'd2546};  // nota = 15, KF = 43
      10'd1004: phinc = {12'd2549};  // nota = 15, KF = 44
      10'd1005: phinc = {12'd2552};  // nota = 15, KF = 45
      10'd1006: phinc = {12'd2553};  // nota = 15, KF = 46
      10'd1007: phinc = {12'd2556};  // nota = 15, KF = 47
      10'd1008: phinc = {12'd2561};  // nota = 15, KF = 48
      10'd1009: phinc = {12'd2563};  // nota = 15, KF = 49
      10'd1010: phinc = {12'd2565};  // nota = 15, KF = 50
      10'd1011: phinc = {12'd2567};  // nota = 15, KF = 51
      10'd1012: phinc = {12'd2568};  // nota = 15, KF = 52
      10'd1013: phinc = {12'd2571};  // nota = 15, KF = 53
      10'd1014: phinc = {12'd2572};  // nota = 15, KF = 54
      10'd1015: phinc = {12'd2575};  // nota = 15, KF = 55
      10'd1016: phinc = {12'd2577};  // nota = 15, KF = 56
      10'd1017: phinc = {12'd2579};  // nota = 15, KF = 57
      10'd1018: phinc = {12'd2581};  // nota = 15, KF = 58
      10'd1019: phinc = {12'd2583};  // nota = 15, KF = 59
      10'd1020: phinc = {12'd2586};  // nota = 15, KF = 60
      10'd1021: phinc = {12'd2589};  // nota = 15, KF = 61
      10'd1022: phinc = {12'd2590};  // nota = 15, KF = 62
      10'd1023: phinc = {12'd2593};  // nota = 15, KF = 63
    endcase
  end

endmodule


///////////////////////////////////////////
// MODULE jt51_pm
module jt51_pm (
    input      [ 6:0] kc_I,
    input      [ 5:0] kf_I,
    input      [ 8:0] mod_I,
    input             add,
    output reg [12:0] kcex
);

  reg [9:0] lim;
  reg [13:0] kcex0, kcex1;
  reg [1:0] extra;

  reg [6:0] kcin;
  reg       carry;

  always @(*) begin : kc_input_cleaner
    {carry, kcin} = kc_I[1:0] == 2'd3 ? {1'b0, kc_I} + 8'd1 : {1'b0, kc_I};
  end

  always @(*) begin : addition
    lim = {1'd0, mod_I} + {4'd0, kf_I};
    case (kcin[3:0])
      default:
      if (lim >= 10'd448) extra = 2'd2;
      else if (lim >= 10'd256) extra = 2'd1;
      else extra = 2'd0;
      4'd1, 4'd5, 4'd9, 4'd13:
      if (lim >= 10'd384) extra = 2'd2;
      else if (lim >= 10'd192) extra = 2'd1;
      else extra = 2'd0;
      4'd2, 4'd6, 4'd10, 4'd14:
      if (lim >= 10'd512) extra = 2'd3;
      else if (lim >= 10'd320) extra = 2'd2;
      else if (lim >= 10'd128) extra = 2'd1;
      else extra = 2'd0;
    endcase
    kcex0 = {1'b0, kcin, kf_I} + {6'd0, extra, 6'd0} + {5'd0, mod_I};
    kcex1 = kcex0[7:6] == 2'd3 ? kcex0 + 14'd64 : kcex0;
  end

  reg signed [9:0] slim;
  reg        [1:0] sextra;
  reg [13:0] skcex0, skcex1;

  always @(*) begin : subtraction
    slim = {1'd0, mod_I} - {4'd0, kf_I};
    case (kcin[3:0])
      default:
      if (slim >= 10'sd449) sextra = 2'd3;
      else if (slim >= 10'sd257) sextra = 2'd2;
      else if (slim >= 10'sd65) sextra = 2'd1;
      else sextra = 2'd0;
      4'd1, 4'd5, 4'd9, 4'd13:
      if (slim >= 10'sd321) sextra = 2'd2;
      else if (slim >= 10'sd129) sextra = 2'd1;
      else sextra = 2'd0;
      4'd2, 4'd6, 4'd10, 4'd14:
      if (slim >= 10'sd385) sextra = 2'd2;
      else if (slim >= 10'sd193) sextra = 2'd1;
      else sextra = 2'd0;
    endcase
    skcex0 = {1'b0, kcin, kf_I} - {6'd0, sextra, 6'd0} - {5'd0, mod_I};
    skcex1 = skcex0[7:6] == 2'd3 ? skcex0 - 14'd64 : skcex0;
  end

  always @(*) begin : mux
    if (add) kcex = kcex1[13] | carry ? {3'd7, 4'd14, 6'd63} : kcex1[12:0];
    else kcex = carry ? {3'd7, 4'd14, 6'd63} : (skcex1[13] ? 13'd0 : skcex1[12:0]);
  end

endmodule


///////////////////////////////////////////
// MODULE jt51_sh
module jt51_sh #(
    parameter width  = 5,
              stages = 32,
              rstval = 1'b0
) (
    input                    rst,
    input                    clk,
    input                    cen,
    input        [width-1:0] din,
    output       [width-1:0] drop,
    input                    auto_ss_rd,
    input                    auto_ss_wr,
    input        [     31:0] auto_ss_data_in,
    input        [      7:0] auto_ss_device_idx,
    input        [     15:0] auto_ss_state_idx,
    input        [      7:0] auto_ss_base_device_idx,
    output logic [     31:0] auto_ss_data_out,
    output logic             auto_ss_ack

);
  wire device_match = (auto_ss_device_idx == auto_ss_base_device_idx);

  genvar auto_ss_idx;


  reg     [stages-1:0] bits[width-1:0];

  // Module-scope clocked shift (functionally identical to the original
  // per-genvar always).  Kept at module scope - not inside the generate loop -
  // so the state_module.py savestate generator injects its restore-write and
  // read-mux once at module scope instead of replicating them per genvar bit
  // (which produced a multidriven auto_ss_data_out and crippled sim speed).
  // Mirrors the jt12_sh structure the generator is known to handle.
  integer              k;
  always @(posedge clk, posedge rst) begin
    if (rst) for (k = 0; k < width; k = k + 1) bits[k] <= {stages{rstval}};
    else if (auto_ss_wr && device_match) begin
      if (auto_ss_state_idx < (width)) begin
        bits[auto_ss_state_idx] <= auto_ss_data_in[stages-1:0];
      end
    end else if (cen) for (k = 0; k < width; k = k + 1) bits[k] <= {bits[k][stages-2:0], din[k]};
  end
  always_comb begin
    auto_ss_data_out = 32'h0;
    auto_ss_ack      = 1'b0;
    if (auto_ss_rd && device_match) begin
      if (auto_ss_state_idx < (width)) begin
        auto_ss_data_out[stages-1:0] = bits[auto_ss_state_idx];
        auto_ss_ack                  = 1'b1;
      end
    end
  end



  genvar i;
  generate
    for (i = 0; i < width; i = i + 1) begin : bit_shifter
      assign drop[i] = bits[i][stages-1];
    end
  endgenerate

endmodule


///////////////////////////////////////////
// MODULE jt51_pg
module jt51_pg (
    input               rst,
    input               clk,
    input               cen  /* direct_enable */,
    input               zero,
    // Channel frequency
    input        [ 6:0] kc_I,
    input        [ 5:0] kf_I,
    // Operator multiplying
    input        [ 3:0] mul_VI,
    // Operator detuning
    input        [ 2:0] dt1_II,
    input        [ 1:0] dt2_I,
    // phase modulation from LFO
    input        [ 7:0] pm,
    input        [ 2:0] pms_I,
    // phase operation
    input               pg_rst_III,
    output reg   [ 4:0] keycode_III,
    output       [ 9:0] pg_phase_X,
    input               auto_ss_rd,
    input               auto_ss_wr,
    input        [31:0] auto_ss_data_in,
    input        [ 7:0] auto_ss_device_idx,
    input        [15:0] auto_ss_state_idx,
    input        [ 7:0] auto_ss_base_device_idx,
    output logic [31:0] auto_ss_data_out,
    output logic        auto_ss_ack


);
  assign auto_ss_ack = auto_ss_local_ack | auto_ss_u_phsh_ack | auto_ss_u_pgrstsh_ack;

  assign auto_ss_data_out = auto_ss_local_data_out | auto_ss_u_phsh_data_out | auto_ss_u_pgrstsh_data_out;

  logic        auto_ss_local_ack;

  logic [31:0] auto_ss_local_data_out;

  wire         auto_ss_u_pgrstsh_ack;

  wire  [31:0] auto_ss_u_pgrstsh_data_out;

  wire         auto_ss_u_phsh_ack;

  wire  [31:0] auto_ss_u_phsh_data_out;

  wire         device_match = (auto_ss_device_idx == auto_ss_base_device_idx);

  genvar auto_ss_idx;


  wire [19:0] ph_VII;

  reg [19:0] phase_base_VI, phase_step_VII, ph_VIII;
  reg [17:0] phase_base_IV, phase_base_V;
  wire        pg_rst_VII;

  wire [11:0] phinc_III;

  reg  [ 9:0] phinc_addr_III;

  reg  [13:0] keycode_II;
  reg  [ 5:0] dt1_kf_III;
  reg  [ 2:0] dt1_kf_IV;

  reg  [ 4:0] pow2;
  reg  [ 4:0] dt1_offset_V;
  reg  [ 2:0] pow2ind_IV;

  reg [2:0] dt1_III, dt1_IV, dt1_V;



  jt51_phinc_rom u_phinctable (
      // .clk     ( clk        ),
      .keycode(phinc_addr_III[9:0]),
      .phinc  (phinc_III)
  );

  always @(*) begin : calcpow2
    case (pow2ind_IV)
      3'd0: pow2 = 5'd16;
      3'd1: pow2 = 5'd17;
      3'd2: pow2 = 5'd19;
      3'd3: pow2 = 5'd20;
      3'd4: pow2 = 5'd22;
      3'd5: pow2 = 5'd24;
      3'd6: pow2 = 5'd26;
      3'd7: pow2 = 5'd29;
    endcase
  end

  reg [5:0] dt1_limit, dt1_unlimited;
  reg [4:0] dt1_limited_IV;

  always @(*) begin : dt1_limit_mux
    case (dt1_IV[1:0])
      default: dt1_limit = 6'd8;
      2'd1: dt1_limit    = 6'd8;
      2'd2: dt1_limit    = 6'd16;
      2'd3: dt1_limit    = 6'd22;
    endcase
    case (dt1_kf_IV)
      3'd0:   dt1_unlimited = { 5'd0, pow2[4]   }; // <2
      3'd1:   dt1_unlimited = { 4'd0, pow2[4:3] }; // <4
      3'd2:   dt1_unlimited = { 3'd0, pow2[4:2] }; // <8
      3'd3:   dt1_unlimited = { 2'd0, pow2[4:1] };
      3'd4:   dt1_unlimited = { 1'd0, pow2[4:0] };
      3'd5:   dt1_unlimited = { pow2[4:0], 1'd0 };
      default:dt1_unlimited = 6'd0;
    endcase
    dt1_limited_IV = dt1_unlimited > dt1_limit ? dt1_limit[4:0] : dt1_unlimited[4:0];
  end

  reg signed [8:0] mod_I;

  always @(*) begin
    case (pms_I)  // comprobar en silicio
      3'd0: mod_I = 9'd0;
      3'd1: mod_I = {7'd0, pm[6:5]};
      3'd2: mod_I = {6'd0, pm[6:4]};
      3'd3: mod_I = {5'd0, pm[6:3]};
      3'd4: mod_I = {4'd0, pm[6:2]};
      3'd5: mod_I = {3'd0, pm[6:1]};
      3'd6: mod_I = {1'd0, pm[6:0], 1'b0};
      3'd7: mod_I = {pm[6:0], 2'b0};
    endcase
  end


  reg  [ 3:0] octave_III;

  wire [12:0] keycode_I;

  jt51_pm u_pm (
      // Channel frequency
      .kc_I (kc_I),
      .kf_I (kf_I),
      .add  (~pm[7]),
      .mod_I(mod_I),
      .kcex (keycode_I)
  );

  // limit value at which we add +64 to the keycode
  // I assume this is to avoid the note==3 violation somehow
  parameter dt2_lim2 = 8'd11 + 8'd64;
  parameter dt2_lim3 = 8'd31 + 8'd64;

  // I
  always @(posedge clk) begin
    if (cen) begin : phase_calculation
      case (dt2_I)
        2'd0: keycode_II <= {1'b0, keycode_I} + (keycode_I[7:6] == 2'd3 ? 14'd64 : 14'd0);
        2'd1: keycode_II <= {1'b0, keycode_I} + 14'd512 + (keycode_I[7:6] == 2'd3 ? 14'd64 : 14'd0);
        2'd2:
        keycode_II <= {1'b0, keycode_I} + 14'd628 + (keycode_I[7:0] > dt2_lim2 ? 14'd64 : 14'd0);
        2'd3:
        keycode_II <= {1'b0, keycode_I} + 14'd800 + (keycode_I[7:0] > dt2_lim3 ? 14'd64 : 14'd0);
      endcase
    end
    if (auto_ss_wr && device_match) begin
      case (auto_ss_state_idx)
        6: begin
          keycode_II <= auto_ss_data_in[31:18];
        end
        default: begin
        end
      endcase
    end
  end



  // II
  always @(posedge clk) begin
    if (cen) begin
      phinc_addr_III <= keycode_II[9:0];
      octave_III     <= keycode_II[13:10];
      keycode_III    <= keycode_II[12:8];
      // Using bits 13:9 fixes Double Dragon issue #14
      // but notes get too long in Jackal
      case (dt1_II[1:0])
        2'd1:   dt1_kf_III  <=  keycode_II[13:8]    - (6'b1<<2);
        2'd2:   dt1_kf_III  <=  keycode_II[13:8]    + (6'b1<<2);
        2'd3:   dt1_kf_III  <=  keycode_II[13:8]    + (6'b1<<3);
        default:dt1_kf_III  <=  keycode_II[13:8];
      endcase
      dt1_III <= dt1_II;
    end
    if (auto_ss_wr && device_match) begin
      case (auto_ss_state_idx)
        7: begin
          phinc_addr_III <= auto_ss_data_in[9:0];
          dt1_kf_III     <= auto_ss_data_in[15:10];
          keycode_III    <= auto_ss_data_in[20:16];
          octave_III     <= auto_ss_data_in[29:26];
        end
        8: begin
          dt1_III <= auto_ss_data_in[2:0];
        end
        default: begin
        end
      endcase
    end
  end



  // III
  always @(posedge clk) begin
    if (cen) begin
      case (octave_III)
        4'd0:    phase_base_IV <= {8'd0, phinc_III[11:2]};
        4'd1:    phase_base_IV <= {7'd0, phinc_III[11:1]};
        4'd2:    phase_base_IV <= {6'd0, phinc_III[11:0]};
        4'd3:    phase_base_IV <= {5'd0, phinc_III[11:0], 1'b0};
        4'd4:    phase_base_IV <= {4'd0, phinc_III[11:0], 2'b0};
        4'd5:    phase_base_IV <= {3'd0, phinc_III[11:0], 3'b0};
        4'd6:    phase_base_IV <= {2'd0, phinc_III[11:0], 4'b0};
        4'd7:    phase_base_IV <= {1'd0, phinc_III[11:0], 5'b0};
        4'd8:    phase_base_IV <= {phinc_III[11:0], 6'b0};
        default: phase_base_IV <= 18'd0;
      endcase
      pow2ind_IV <= dt1_kf_III[2:0];
      dt1_IV     <= dt1_III;
      dt1_kf_IV  <= dt1_kf_III[5:3];
    end
    if (auto_ss_wr && device_match) begin
      case (auto_ss_state_idx)
        5: begin
          phase_base_IV <= auto_ss_data_in[17:0];
        end
        8: begin
          dt1_IV     <= auto_ss_data_in[5:3];
          dt1_kf_IV  <= auto_ss_data_in[8:6];
          pow2ind_IV <= auto_ss_data_in[11:9];
        end
        default: begin
        end
      endcase
    end
  end



  // IV LIMIT_BASE
  always @(posedge clk) begin
    if (cen) begin
      if (phase_base_IV > 18'd82976) phase_base_V <= 18'd82976;
      else phase_base_V <= phase_base_IV;
      dt1_offset_V <= dt1_limited_IV;
      dt1_V        <= dt1_IV;
    end
    if (auto_ss_wr && device_match) begin
      case (auto_ss_state_idx)
        6: begin
          phase_base_V <= auto_ss_data_in[17:0];
        end
        7: begin
          dt1_offset_V <= auto_ss_data_in[25:21];
        end
        8: begin
          dt1_V <= auto_ss_data_in[14:12];
        end
        default: begin
        end
      endcase
    end
  end



  // V APPLY_DT1
  always @(posedge clk) begin
    if (cen) begin
      if (dt1_V[1:0] == 2'd0) phase_base_VI <= {2'b0, phase_base_V};
      else begin
        if (!dt1_V[2]) phase_base_VI <= {2'b0, phase_base_V} + {15'd0, dt1_offset_V};
        else phase_base_VI <= {2'b0, phase_base_V} - {15'd0, dt1_offset_V};
      end
    end
    if (auto_ss_wr && device_match) begin
      case (auto_ss_state_idx)
        0: begin
          phase_base_VI <= auto_ss_data_in[19:0];
        end
        default: begin
        end
      endcase
    end
  end



  // VI APPLY_MUL
  always @(posedge clk) begin
    if (cen) begin
      if (mul_VI == 4'd0) phase_step_VII <= {1'b0, phase_base_VI[19:1]};
      else phase_step_VII <= phase_base_VI * mul_VI;
    end
    if (auto_ss_wr && device_match) begin
      case (auto_ss_state_idx)
        1: begin
          phase_step_VII <= auto_ss_data_in[19:0];
        end
        default: begin
        end
      endcase
    end
  end



  // VII have same number of stages as jt51_envelope
  always @(posedge clk, posedge rst) begin
    if (rst) ph_VIII <= 20'd0;
    else if (auto_ss_wr && device_match) begin
      case (auto_ss_state_idx)
        2: begin
          ph_VIII <= auto_ss_data_in[19:0];
        end
        default: begin
        end
      endcase
    end else if (cen) begin
      ph_VIII <= pg_rst_VII ? 20'd0 : ph_VII + phase_step_VII;

    end
  end

  // VIII
  reg [19:0] ph_IX;
  always @(posedge clk, posedge rst) begin
    if (rst) ph_IX <= 20'd0;
    else if (auto_ss_wr && device_match) begin
      case (auto_ss_state_idx)
        3: begin
          ph_IX <= auto_ss_data_in[19:0];
        end
        default: begin
        end
      endcase
    end else if (cen) begin
      ph_IX <= ph_VIII[19:0];
    end
  end

  // IX
  reg [19:0] ph_X;
  assign pg_phase_X = ph_X[19:10];
  always @(posedge clk, posedge rst) begin
    if (rst) begin
      ph_X <= 20'd0;
    end else if (auto_ss_wr && device_match) begin
      case (auto_ss_state_idx)
        4: begin
          ph_X <= auto_ss_data_in[19:0];
        end
        default: begin
        end
      endcase
    end else if (cen) begin
      ph_X <= ph_IX;
    end
  end
  always_comb begin
    auto_ss_local_data_out = 32'h0;
    auto_ss_local_ack      = 1'b0;
    if (auto_ss_rd && device_match) begin
      case (auto_ss_state_idx)
        0: begin
          auto_ss_local_data_out[20-1:0] = phase_base_VI;
          auto_ss_local_ack              = 1'b1;
        end
        1: begin
          auto_ss_local_data_out[20-1:0] = phase_step_VII;
          auto_ss_local_ack              = 1'b1;
        end
        2: begin
          auto_ss_local_data_out[20-1:0] = ph_VIII;
          auto_ss_local_ack              = 1'b1;
        end
        3: begin
          auto_ss_local_data_out[20-1:0] = ph_IX;
          auto_ss_local_ack              = 1'b1;
        end
        4: begin
          auto_ss_local_data_out[20-1:0] = ph_X;
          auto_ss_local_ack              = 1'b1;
        end
        5: begin
          auto_ss_local_data_out[18-1:0] = phase_base_IV;
          auto_ss_local_ack              = 1'b1;
        end
        6: begin
          auto_ss_local_data_out[31:0] = {keycode_II, phase_base_V};
          auto_ss_local_ack            = 1'b1;
        end
        7: begin
          auto_ss_local_data_out[29:0] = {
            octave_III, dt1_offset_V, keycode_III, dt1_kf_III, phinc_addr_III
          };
          auto_ss_local_ack = 1'b1;
        end
        8: begin
          auto_ss_local_data_out[14:0] = {dt1_V, pow2ind_IV, dt1_kf_IV, dt1_IV, dt1_III};
          auto_ss_local_ack            = 1'b1;
        end
        default: begin
        end
      endcase
    end
  end



  jt51_sh #(
      .width (20),
      .stages(32 - 3)
  ) u_phsh (
      .rst                    (rst),
      .clk                    (clk),
      .cen                    (cen),
      .din                    (ph_X),
      .drop                   (ph_VII),
      .auto_ss_rd             (auto_ss_rd),
      .auto_ss_wr             (auto_ss_wr),
      .auto_ss_data_in        (auto_ss_data_in),
      .auto_ss_device_idx     (auto_ss_device_idx),
      .auto_ss_state_idx      (auto_ss_state_idx),
      .auto_ss_base_device_idx(auto_ss_base_device_idx + 1),
      .auto_ss_data_out       (auto_ss_u_phsh_data_out),
      .auto_ss_ack            (auto_ss_u_phsh_ack)

  );

  jt51_sh #(
      .width (1),
      .stages(4)
  ) u_pgrstsh (
      .rst                    (rst),
      .clk                    (clk),
      .cen                    (cen),
      .din                    (pg_rst_III),
      .drop                   (pg_rst_VII),
      .auto_ss_rd             (auto_ss_rd),
      .auto_ss_wr             (auto_ss_wr),
      .auto_ss_data_in        (auto_ss_data_in),
      .auto_ss_device_idx     (auto_ss_device_idx),
      .auto_ss_state_idx      (auto_ss_state_idx),
      .auto_ss_base_device_idx(auto_ss_base_device_idx + 2),
      .auto_ss_data_out       (auto_ss_u_pgrstsh_data_out),
      .auto_ss_ack            (auto_ss_u_pgrstsh_ack)

  );



endmodule


///////////////////////////////////////////
// MODULE jt51_eg
module jt51_eg (

    input               rst,
    input               clk,
    input               cen,
    input               zero,
    // envelope configuration
    input        [ 4:0] keycode_III,
    input        [ 4:0] arate_II,
    input        [ 4:0] rate1_II,
    input        [ 4:0] rate2_II,
    input        [ 3:0] rrate_II,
    input        [ 3:0] d1l_I,
    input        [ 1:0] ks_III,
    // envelope operation
    input               keyon_II,
    output reg          pg_rst_III,
    // envelope number
    input        [ 6:0] tl_VII,
    input        [ 7:0] am,
    input        [ 1:0] ams_VII,
    input               amsen_VII,
    output       [ 9:0] eg_XI,
    input               auto_ss_rd,
    input               auto_ss_wr,
    input        [31:0] auto_ss_data_in,
    input        [ 7:0] auto_ss_device_idx,
    input        [15:0] auto_ss_state_idx,
    input        [ 7:0] auto_ss_base_device_idx,
    output logic [31:0] auto_ss_data_out,
    output logic        auto_ss_ack

);
  assign auto_ss_ack = auto_ss_local_ack | auto_ss_u_egpadding_ack | auto_ss_u_eg1sh_ack | auto_ss_u_eg2sh_ack | auto_ss_u_aroffsh_ack | auto_ss_u_konsh_ack | auto_ss_u_cntsh_ack | auto_ss_u_statesh_ack;

  assign auto_ss_data_out = auto_ss_local_data_out | auto_ss_u_egpadding_data_out | auto_ss_u_eg1sh_data_out | auto_ss_u_eg2sh_data_out | auto_ss_u_aroffsh_data_out | auto_ss_u_konsh_data_out | auto_ss_u_cntsh_data_out | auto_ss_u_statesh_data_out;

  logic        auto_ss_local_ack;

  logic [31:0] auto_ss_local_data_out;

  wire         auto_ss_u_statesh_ack;

  wire  [31:0] auto_ss_u_statesh_data_out;

  wire         auto_ss_u_cntsh_ack;

  wire  [31:0] auto_ss_u_cntsh_data_out;

  wire         auto_ss_u_konsh_ack;

  wire  [31:0] auto_ss_u_konsh_data_out;

  wire         auto_ss_u_aroffsh_ack;

  wire  [31:0] auto_ss_u_aroffsh_data_out;

  wire         auto_ss_u_eg2sh_ack;

  wire  [31:0] auto_ss_u_eg2sh_data_out;

  wire         auto_ss_u_eg1sh_ack;

  wire  [31:0] auto_ss_u_eg1sh_data_out;

  wire         auto_ss_u_egpadding_ack;

  wire  [31:0] auto_ss_u_egpadding_data_out;

  wire         device_match = (auto_ss_device_idx == auto_ss_base_device_idx);

  genvar auto_ss_idx;


  // eg[9:6] -> direct attenuation (divide by 2)
  // eg[5:0] -> mantisa attenuation (uses LUT)
  // 1 LSB of eg is -0.09257 dB

  localparam ATTACK = 2'd0, DECAY1 = 2'd1, DECAY2 = 2'd2, RELEASE = 2'd3;

  reg  [4:0] d1level_II;
  reg  [2:0] cnt_V;
  reg  [5:0] rate_IV;
  wire [9:0] eg_VI;
  reg [9:0] eg_VII, eg_VIII;
  wire [ 9:0] eg_II;
  reg  [11:0] sum_eg_tl_VII;

  reg step_V, step_VI;
  reg        sum_up;
  reg [ 5:0] rate_V;
  reg [ 5:1] rate_VI;

  // remember: { log_msb, pow_addr } <= log_val[11:0] + { tl, 5'd0 } + { eg, 2'd0 };

  reg [ 1:0] eg_cnt_base;
  reg [14:0] eg_cnt  /*verilator public*/;

  reg [ 9:0] am_final_VII;

  always @(posedge clk) begin
    begin : envelope_counter
      if (rst) begin
        eg_cnt_base <= 2'd0;
        eg_cnt      <= 15'd0;
      end else if (cen) begin
        if (zero) begin
          // envelope counter increases every 3 output samples,
          // there is one sample every 32 clock ticks
          if (eg_cnt_base == 2'd2) begin
            eg_cnt      <= eg_cnt + 1'b1;
            eg_cnt_base <= 2'd0;
          end else eg_cnt_base <= eg_cnt_base + 1'b1;
        end
      end
    end
    if (auto_ss_wr && device_match) begin
      case (auto_ss_state_idx)
        0: begin
          eg_cnt <= auto_ss_data_in[14:0];
        end
        2: begin
          eg_cnt_base <= auto_ss_data_in[9:8];
        end
        default: begin
        end
      endcase
    end
  end



  wire       cnt_out;  // = all_cnt_last[3*31-1:3*30];

  reg  [6:0] pre_rate_III;
  reg  [4:0] kshift_III;
  reg  [4:0] cfg_III;

  always @(*) begin : pre_rate_calc
    kshift_III   = keycode_III >> ~ks_III;
    pre_rate_III = {1'b0, cfg_III, 1'b0} + {2'b0, kshift_III};
  end



  reg [7:0] step_idx;
  reg [1:0] state_in_III, state_in_IV, state_in_V, state_in_VI;

  always @(*) begin : rate_step
    if (rate_V[5:4] == 2'b11) begin  // 0 means 1x, 1 means 2x
      if (rate_V[5:2] == 4'hf && state_in_V == ATTACK)
        step_idx = 8'b11111111;  // Maximum attack speed, rates 60&61
      else
        case (rate_V[1:0])
          2'd0: step_idx = 8'b00000000;
          2'd1: step_idx = 8'b10001000;  // 2
          2'd2: step_idx = 8'b10101010;  // 4
          2'd3: step_idx = 8'b11101110;  // 6
        endcase
    end else begin
      if (rate_V[5:2] == 4'd0 && state_in_V != ATTACK)
        step_idx = 8'b11111110;  // limit slowest decay rate_IV
      else
        case (rate_V[1:0])
          2'd0: step_idx = 8'b10101010;  // 4
          2'd1: step_idx = 8'b11101010;  // 5
          2'd2: step_idx = 8'b11101110;  // 6
          2'd3: step_idx = 8'b11111110;  // 7
        endcase
    end
    // a rate_IV of zero keeps the level still
    step_V = rate_V[5:1] == 5'd0 ? 1'b0 : step_idx[cnt_V];
  end



  wire ar_off_VI;

  always @(posedge clk) begin
    if (cen) begin
      // I
      if (d1l_I == 4'd15) d1level_II <= 5'h10;  // 48dB
      else d1level_II <= {1'b0, d1l_I};
    end
    if (auto_ss_wr && device_match) begin
      case (auto_ss_state_idx)
        1: begin
          d1level_II <= auto_ss_data_in[26:22];
        end
        default: begin
        end
      endcase
    end
  end



  //  II
  wire       keyon_last_II;
  wire       keyon_now_II = !keyon_last_II && keyon_II;
  wire       keyoff_now_II = keyon_last_II && !keyon_II;
  wire       ar_off_II = keyon_now_II && (arate_II == 5'h1f);
  wire [1:0] state_II;

  always @(posedge clk) begin
    if (cen) begin
      pg_rst_III <= keyon_now_II;
      // trigger release
      if (keyoff_now_II) begin
        cfg_III      <= {rrate_II, 1'b1};
        state_in_III <= RELEASE;
      end else begin
        // trigger 1st decay
        if (keyon_now_II) begin
          cfg_III      <= arate_II;
          state_in_III <= ATTACK;
        end else begin : sel_rate
          case (state_II)
            ATTACK: begin
              if (eg_II == 10'd0) begin
                state_in_III <= DECAY1;
                cfg_III      <= rate1_II;
              end else begin
                state_in_III <= state_II;  // attack
                cfg_III      <= arate_II;
              end
            end
            DECAY1: begin
              if (eg_II[9:5] >= d1level_II) begin
                cfg_III      <= rate2_II;
                state_in_III <= DECAY2;
              end else begin
                cfg_III      <= rate1_II;
                state_in_III <= state_II;  // decay1
              end
            end
            DECAY2: begin
              cfg_III      <= rate2_II;
              state_in_III <= state_II;  // decay2
            end
            RELEASE: begin
              cfg_III      <= {rrate_II, 1'b1};
              state_in_III <= state_II;  // release
            end
          endcase
        end
      end
    end
    if (auto_ss_wr && device_match) begin
      case (auto_ss_state_idx)
        1: begin
          cfg_III <= auto_ss_data_in[31:27];
        end
        2: begin
          state_in_III <= auto_ss_data_in[11:10];
          pg_rst_III   <= auto_ss_data_in[18];
        end
        default: begin
        end
      endcase
    end
  end



  // III
  always @(posedge clk) begin
    if (cen) begin
      state_in_IV <= state_in_III;
      rate_IV     <= pre_rate_III[6] ? 6'd63 : pre_rate_III[5:0];
    end
    if (auto_ss_wr && device_match) begin
      case (auto_ss_state_idx)
        1: begin
          rate_IV <= auto_ss_data_in[15:10];
        end
        2: begin
          state_in_IV <= auto_ss_data_in[13:12];
        end
        default: begin
        end
      endcase
    end
  end



  // IV
  always @(posedge clk) begin
    if (cen) begin
      state_in_V <= state_in_IV;
      rate_V     <= rate_IV;
      if (state_in_IV == ATTACK)
        case (rate_IV[5:2])
          4'h0: cnt_V <= eg_cnt[13:11];
          4'h1: cnt_V <= eg_cnt[12:10];
          4'h2: cnt_V <= eg_cnt[11:9];
          4'h3: cnt_V <= eg_cnt[10:8];
          4'h4: cnt_V <= eg_cnt[9:7];
          4'h5: cnt_V <= eg_cnt[8:6];
          4'h6: cnt_V <= eg_cnt[7:5];
          4'h7: cnt_V <= eg_cnt[6:4];
          4'h8: cnt_V <= eg_cnt[5:3];
          4'h9: cnt_V <= eg_cnt[4:2];
          4'ha: cnt_V <= eg_cnt[3:1];
          default: cnt_V <= eg_cnt[2:0];
        endcase
      else
        case (rate_IV[5:2])
          4'h0: cnt_V <= eg_cnt[14:12];
          4'h1: cnt_V <= eg_cnt[13:11];
          4'h2: cnt_V <= eg_cnt[12:10];
          4'h3: cnt_V <= eg_cnt[11:9];
          4'h4: cnt_V <= eg_cnt[10:8];
          4'h5: cnt_V <= eg_cnt[9:7];
          4'h6: cnt_V <= eg_cnt[8:6];
          4'h7: cnt_V <= eg_cnt[7:5];
          4'h8: cnt_V <= eg_cnt[6:4];
          4'h9: cnt_V <= eg_cnt[5:3];
          4'ha: cnt_V <= eg_cnt[4:2];
          4'hb: cnt_V <= eg_cnt[3:1];
          default: cnt_V <= eg_cnt[2:0];
        endcase
    end
    if (auto_ss_wr && device_match) begin
      case (auto_ss_state_idx)
        1: begin
          rate_V <= auto_ss_data_in[21:16];
        end
        2: begin
          cnt_V      <= auto_ss_data_in[7:5];
          state_in_V <= auto_ss_data_in[15:14];
        end
        default: begin
        end
      endcase
    end
  end



  // V
  always @(posedge clk) begin
    if (cen) begin
      state_in_VI <= state_in_V;
      rate_VI     <= rate_V[5:1];
      sum_up      <= cnt_V[0] != cnt_out;
      step_VI     <= step_V;
    end
    if (auto_ss_wr && device_match) begin
      case (auto_ss_state_idx)
        2: begin
          rate_VI     <= auto_ss_data_in[4:0];
          state_in_VI <= auto_ss_data_in[17:16];
          step_VI     <= auto_ss_data_in[19];
          sum_up      <= auto_ss_data_in[20];
        end
        default: begin
        end
      endcase
    end
  end



  ///////////////////////////////////////
  // VI
  reg [8:0] ar_sum0_VI;
  reg [9:0] ar_result_VI, ar_sum_VI;

  always @(*) begin : ar_calculation
    casez (rate_VI[5:2])
      default: ar_sum0_VI = {3'd0, eg_VI[9:4]} + 9'd1;
      4'b1100: ar_sum0_VI = {3'd0, eg_VI[9:4]} + 9'd1;
      4'b1101: ar_sum0_VI = {2'd0, eg_VI[9:3]} + 9'd1;
      4'b111?: ar_sum0_VI = {1'd0, eg_VI[9:2]} + 9'd1;
    endcase
    if (rate_VI[5:4] == 2'b11) ar_sum_VI = step_VI ? {ar_sum0_VI, 1'b0} : {1'b0, ar_sum0_VI};
    else ar_sum_VI = step_VI ? {1'b0, ar_sum0_VI} : 10'd0;
    ar_result_VI = ar_sum_VI < eg_VI ? eg_VI - ar_sum_VI : 10'd0;
  end


  always @(posedge clk) begin
    if (cen) begin
      if (ar_off_VI) eg_VII <= 10'd0;
      else if (state_in_VI == ATTACK) begin
        if (sum_up && eg_VI != 10'd0)
          if (rate_VI[5:1] == 5'hf) eg_VII <= 10'd0;
          else eg_VII <= ar_result_VI;
        else eg_VII <= eg_VI;
      end else begin : DECAY_SUM
        if (sum_up) begin
          if (eg_VI <= (10'd1023 - 10'd8))
            case (rate_VI[5:2])
              4'b1100: eg_VII <= eg_VI + {8'd0, step_VI, ~step_VI};  // 12
              4'b1101: eg_VII <= eg_VI + {7'd0, step_VI, ~step_VI, 1'b0};  // 13
              4'b1110: eg_VII <= eg_VI + {6'd0, step_VI, ~step_VI, 2'b0};  // 14
              4'b1111: eg_VII <= eg_VI + 10'd8;  // 15
              default: eg_VII <= eg_VI + {8'd0, step_VI, 1'b0};
            endcase
          else eg_VII <= 10'h3FF;
        end else eg_VII <= eg_VI;
      end
    end
    if (auto_ss_wr && device_match) begin
      case (auto_ss_state_idx)
        0: begin
          eg_VII <= auto_ss_data_in[24:15];
        end
        default: begin
        end
      endcase
    end
  end



  // VII
  always @(*) begin : sum_eg_and_tl
    casez ({
      amsen_VII, ams_VII
    })
      3'b0_??, 3'b1_00: am_final_VII = 10'd0;
      3'b1_01:          am_final_VII = {2'b0, am};  // 23.9 dB max
      3'b1_10:          am_final_VII = {1'b0, am, 1'b0};  // 47   dB
      3'b1_11:          am_final_VII = {am, 2'b0};  // 95.6 dB
    endcase

    sum_eg_tl_VII = {2'b0, tl_VII, 3'd0}  // 0.75 dB steps
    + {2'b0, eg_VII}  // 0.094 dB steps
    + {2'b0, am_final_VII};
  end

  always @(posedge clk) begin
    if (cen) begin
      eg_VIII <= |sum_eg_tl_VII[11:10] ? {10{1'b1}} : sum_eg_tl_VII[9:0];
    end
    if (auto_ss_wr && device_match) begin
      case (auto_ss_state_idx)
        1: begin
          eg_VIII <= auto_ss_data_in[9:0];
        end
        default: begin
        end
      endcase
    end
  end


  always_comb begin
    auto_ss_local_data_out = 32'h0;
    auto_ss_local_ack      = 1'b0;
    if (auto_ss_rd && device_match) begin
      case (auto_ss_state_idx)
        0: begin
          auto_ss_local_data_out[24:0] = {eg_VII, eg_cnt};
          auto_ss_local_ack            = 1'b1;
        end
        1: begin
          auto_ss_local_data_out[31:0] = {cfg_III, d1level_II, rate_V, rate_IV, eg_VIII};
          auto_ss_local_ack            = 1'b1;
        end
        2: begin
          auto_ss_local_data_out[20:0] = {
            sum_up,
            step_VI,
            pg_rst_III,
            state_in_VI,
            state_in_V,
            state_in_IV,
            state_in_III,
            eg_cnt_base,
            cnt_V,
            rate_VI
          };
          auto_ss_local_ack = 1'b1;
        end
        default: begin
        end
      endcase
    end
  end



  jt51_sh #(
      .width (10),
      .stages(3)
  ) u_egpadding (
      .rst                    (rst),
      .clk                    (clk),
      .cen                    (cen),
      .din                    (eg_VIII),
      .drop                   (eg_XI),
      .auto_ss_rd             (auto_ss_rd),
      .auto_ss_wr             (auto_ss_wr),
      .auto_ss_data_in        (auto_ss_data_in),
      .auto_ss_device_idx     (auto_ss_device_idx),
      .auto_ss_state_idx      (auto_ss_state_idx),
      .auto_ss_base_device_idx(auto_ss_base_device_idx + 1),
      .auto_ss_data_out       (auto_ss_u_egpadding_data_out),
      .auto_ss_ack            (auto_ss_u_egpadding_ack)

  );


  // Shift registers

  jt51_sh #(
      .width (10),
      .stages(32 - 7 + 2),
      .rstval(1'b1)
  ) u_eg1sh (
      .rst                    (rst),
      .clk                    (clk),
      .cen                    (cen),
      .din                    (eg_VII),
      .drop                   (eg_II),
      .auto_ss_rd             (auto_ss_rd),
      .auto_ss_wr             (auto_ss_wr),
      .auto_ss_data_in        (auto_ss_data_in),
      .auto_ss_device_idx     (auto_ss_device_idx),
      .auto_ss_state_idx      (auto_ss_state_idx),
      .auto_ss_base_device_idx(auto_ss_base_device_idx + 2),
      .auto_ss_data_out       (auto_ss_u_eg1sh_data_out),
      .auto_ss_ack            (auto_ss_u_eg1sh_ack)

  );

  jt51_sh #(
      .width (10),
      .stages(4),
      .rstval(1'b1)
  ) u_eg2sh (
      .rst                    (rst),
      .clk                    (clk),
      .cen                    (cen),
      .din                    (eg_II),
      .drop                   (eg_VI),
      .auto_ss_rd             (auto_ss_rd),
      .auto_ss_wr             (auto_ss_wr),
      .auto_ss_data_in        (auto_ss_data_in),
      .auto_ss_device_idx     (auto_ss_device_idx),
      .auto_ss_state_idx      (auto_ss_state_idx),
      .auto_ss_base_device_idx(auto_ss_base_device_idx + 3),
      .auto_ss_data_out       (auto_ss_u_eg2sh_data_out),
      .auto_ss_ack            (auto_ss_u_eg2sh_ack)

  );

  jt51_sh #(
      .width (1),
      .stages(4)
  ) u_aroffsh (
      .rst                    (rst),
      .clk                    (clk),
      .cen                    (cen),
      .din                    (ar_off_II),
      .drop                   (ar_off_VI),
      .auto_ss_rd             (auto_ss_rd),
      .auto_ss_wr             (auto_ss_wr),
      .auto_ss_data_in        (auto_ss_data_in),
      .auto_ss_device_idx     (auto_ss_device_idx),
      .auto_ss_state_idx      (auto_ss_state_idx),
      .auto_ss_base_device_idx(auto_ss_base_device_idx + 4),
      .auto_ss_data_out       (auto_ss_u_aroffsh_data_out),
      .auto_ss_ack            (auto_ss_u_aroffsh_ack)

  );

  jt51_sh #(
      .width (1),
      .stages(32)
  ) u_konsh (
      .rst                    (rst),
      .clk                    (clk),
      .din                    (keyon_II),
      .cen                    (cen),
      .drop                   (keyon_last_II),
      .auto_ss_rd             (auto_ss_rd),
      .auto_ss_wr             (auto_ss_wr),
      .auto_ss_data_in        (auto_ss_data_in),
      .auto_ss_device_idx     (auto_ss_device_idx),
      .auto_ss_state_idx      (auto_ss_state_idx),
      .auto_ss_base_device_idx(auto_ss_base_device_idx + 5),
      .auto_ss_data_out       (auto_ss_u_konsh_data_out),
      .auto_ss_ack            (auto_ss_u_konsh_ack)

  );

  jt51_sh #(
      .width (1),
      .stages(32)
  ) u_cntsh (
      .rst                    (rst),
      .clk                    (clk),
      .cen                    (cen),
      .din                    (cnt_V[0]),
      .drop                   (cnt_out),
      .auto_ss_rd             (auto_ss_rd),
      .auto_ss_wr             (auto_ss_wr),
      .auto_ss_data_in        (auto_ss_data_in),
      .auto_ss_device_idx     (auto_ss_device_idx),
      .auto_ss_state_idx      (auto_ss_state_idx),
      .auto_ss_base_device_idx(auto_ss_base_device_idx + 6),
      .auto_ss_data_out       (auto_ss_u_cntsh_data_out),
      .auto_ss_ack            (auto_ss_u_cntsh_ack)

  );

  jt51_sh #(
      .width (2),
      .stages(32 - 3 + 2),
      .rstval(1'b1)
  ) u_statesh (
      .rst                    (rst),
      .clk                    (clk),
      .cen                    (cen),
      .din                    (state_in_III),
      .drop                   (state_II),
      .auto_ss_rd             (auto_ss_rd),
      .auto_ss_wr             (auto_ss_wr),
      .auto_ss_data_in        (auto_ss_data_in),
      .auto_ss_device_idx     (auto_ss_device_idx),
      .auto_ss_state_idx      (auto_ss_state_idx),
      .auto_ss_base_device_idx(auto_ss_base_device_idx + 7),
      .auto_ss_data_out       (auto_ss_u_statesh_data_out),
      .auto_ss_ack            (auto_ss_u_statesh_ack)

  );



endmodule


///////////////////////////////////////////
// MODULE jt51_phrom
module jt51_phrom (
    input        [ 4:0] addr,
    input               clk,
    input               cen,
    output reg   [45:0] ph,
    input               auto_ss_rd,
    input               auto_ss_wr,
    input        [31:0] auto_ss_data_in,
    input        [ 7:0] auto_ss_device_idx,
    input        [15:0] auto_ss_state_idx,
    input        [ 7:0] auto_ss_base_device_idx,
    output logic [31:0] auto_ss_data_out,
    output logic        auto_ss_ack

);
  wire device_match = (auto_ss_device_idx == auto_ss_base_device_idx);

  genvar auto_ss_idx;


  reg [45:0] sinetable[31:0];
  initial begin
    sinetable[5'd0]  = 46'b0001100000100100010001000010101010101001010010;
    sinetable[5'd1]  = 46'b0001100000110100000100000010010001001101000001;
    sinetable[5'd2]  = 46'b0001100000110100000100110010001011001101100000;
    sinetable[5'd3]  = 46'b0001110000010000000000110010110001001101110010;
    sinetable[5'd4]  = 46'b0001110000010000001100000010111010001101101001;
    sinetable[5'd5]  = 46'b0001110000010100001001100010000000101101111010;
    sinetable[5'd6]  = 46'b0001110000010100001101100010010011001101011010;
    sinetable[5'd7]  = 46'b0001110000011100000101010010111000101111111100;
    sinetable[5'd8]  = 46'b0001110000111000000001110010101110001101110111;
    sinetable[5'd9]  = 46'b0001110000111000010100111000011101011010100110;
    sinetable[5'd10] = 46'b0001110000111100011000011000111100001001111010;
    sinetable[5'd11] = 46'b0001110000111100011100111001101011001001110111;
    sinetable[5'd12] = 46'b0100100001010000010001011001001000111010110111;
    sinetable[5'd13] = 46'b0100100001010100010001001001110001111100101010;
    sinetable[5'd14] = 46'b0100100001010100010101101101111110100101000110;
    sinetable[5'd15] = 46'b0100100011100000001000011001010110101101111001;
    sinetable[5'd16] = 46'b0100100011100100001000101011100101001011101111;
    sinetable[5'd17] = 46'b0100100011101100000111011010000001011010110001;
    sinetable[5'd18] = 46'b0100110011001000000111101010000010111010111111;
    sinetable[5'd19] = 46'b0100110011001100001011011110101110110110000001;
    sinetable[5'd20] = 46'b0100110011101000011010111011001010001101110001;
    sinetable[5'd21] = 46'b0100110011101101011010110101111001010100001111;
    sinetable[5'd22] = 46'b0111000010000001010111000101010101010110010111;
    sinetable[5'd23] = 46'b0111000010000101010111110111110101010010111011;
    sinetable[5'd24] = 46'b0111000010110101101000101100001000010000011001;
    sinetable[5'd25] = 46'b0111010010011001100100011110100100010010010010;
    sinetable[5'd26] = 46'b0111010010111010100101100101000000110100100011;
    sinetable[5'd27] = 46'b1010000010011010101101011101100001110010011010;
    sinetable[5'd28] = 46'b1010000010111111111100100111010100010000111001;
    sinetable[5'd29] = 46'b1010010111110100110010001100111001010110100000;
    sinetable[5'd30] = 46'b1011010111010011111011011110000100110010100001;
    sinetable[5'd31] = 46'b1110011011110001111011100111100001110110100111;

  end

  always @(posedge clk) begin
    if (cen) ph <= sinetable[addr];
    if (auto_ss_wr && device_match) begin
    end
  end


  always_comb begin
    auto_ss_data_out = 32'h0;
    auto_ss_ack      = 1'b0;
    if (auto_ss_rd && device_match) begin
    end
  end



endmodule


///////////////////////////////////////////
// MODULE jt51_exprom
module jt51_exprom (
    input        [ 4:0] addr,
    input               clk,
    input               cen,
    output reg   [44:0] exp,
    input               auto_ss_rd,
    input               auto_ss_wr,
    input        [31:0] auto_ss_data_in,
    input        [ 7:0] auto_ss_device_idx,
    input        [15:0] auto_ss_state_idx,
    input        [ 7:0] auto_ss_base_device_idx,
    output logic [31:0] auto_ss_data_out,
    output logic        auto_ss_ack

);
  wire device_match = (auto_ss_device_idx == auto_ss_base_device_idx);

  genvar auto_ss_idx;


  reg [44:0] explut[31:0];
  initial begin
    explut[0]  = 45'b111110101011010110001011010000010010111011011;
    explut[1]  = 45'b111101010011010101000011001100101110110101011;
    explut[2]  = 45'b111011111011010011110111001000110010101110011;
    explut[3]  = 45'b111010100101010010101111000100110010101000011;
    explut[4]  = 45'b111001001101010001100111000000110010100001011;
    explut[5]  = 45'b110111111011010000011110111101010010011011011;
    explut[6]  = 45'b110110100011001111010110111001010010010100100;
    explut[7]  = 45'b110101001011001110001110110101110010001110011;
    explut[8]  = 45'b110011111011001101000110110001110010001000011;
    explut[9]  = 45'b110010100011001011111110101110010010000010011;
    explut[10] = 45'b110001010011001010111010101010010001111011011;
    explut[11] = 45'b101111111011001001110010100110110001110101011;
    explut[12] = 45'b101110101011001000101010100011001101101111011;
    explut[13] = 45'b101101010101000111100110011111010001101001011;
    explut[14] = 45'b101100000011000110100010011011110001100011011;
    explut[15] = 45'b101010110011000101011110011000010001011101011;
    explut[16] = 45'b101001100011000100011010010100101101010111011;
    explut[17] = 45'b101000010011000011010010010001001101010001011;
    explut[18] = 45'b100111000011000010010010001101101101001011011;
    explut[19] = 45'b100101110011000001001110001010001101000101011;
    explut[20] = 45'b100100100011000000001010000110010000111111011;
    explut[21] = 45'b100011010010111111001010000011001100111001011;
    explut[22] = 45'b100010000010111110000101111111101100110011011;
    explut[23] = 45'b100000110010111101000001111100001100101101011;
    explut[24] = 45'b011111101010111100000001111000101100101000010;
    explut[25] = 45'b011110011010111011000001110101001100100010011;
    explut[26] = 45'b011101001010111010000001110001110000011100011;
    explut[27] = 45'b011100000010111001000001101110010000010110011;
    explut[28] = 45'b011010110010111000000001101011001100010001011;
    explut[29] = 45'b011001101010110111000001100111101100001011011;
    explut[30] = 45'b011000100000110110000001100100010000000110010;
    explut[31] = 45'b010111010010110101000001100001001100000000011;
  end

  always @(posedge clk) begin
    if (cen) begin
      exp <= explut[addr];
    end
    if (auto_ss_wr && device_match) begin
    end
  end


  always_comb begin
    auto_ss_data_out = 32'h0;
    auto_ss_ack      = 1'b0;
    if (auto_ss_rd && device_match) begin
    end
  end



endmodule


///////////////////////////////////////////
// MODULE jt51_op
module jt51_op (

    input       rst,
    input       clk,
    input       cen,             // P1
    input [9:0] pg_phase_X,
    input [2:0] con_I,
    input [2:0] fb_II,
    // volume
    input [9:0] eg_atten_XI,
    // modulation
    input       use_prevprev1,
    input       use_internal_x,
    input       use_internal_y,
    input       use_prev2,
    input       use_prev1,
    input       test_214,

    input m1_enters,
    input c1_enters,

    // output data
    output signed [13:0] op_XVII,
    input                auto_ss_rd,
    input                auto_ss_wr,
    input         [31:0] auto_ss_data_in,
    input         [ 7:0] auto_ss_device_idx,
    input         [15:0] auto_ss_state_idx,
    input         [ 7:0] auto_ss_base_device_idx,
    output logic  [31:0] auto_ss_data_out,
    output logic         auto_ss_ack

);
  assign auto_ss_ack = auto_ss_local_ack | auto_ss_prev1_buffer_ack | auto_ss_prevprev1_buffer_ack | auto_ss_prev2_buffer_ack | auto_ss_phasemod_sh_ack | auto_ss_u_phrom_ack | auto_ss_u_exprom_ack | auto_ss_out_padding_ack | auto_ss_shsignbit_ack;

  assign auto_ss_data_out = auto_ss_local_data_out | auto_ss_prev1_buffer_data_out | auto_ss_prevprev1_buffer_data_out | auto_ss_prev2_buffer_data_out | auto_ss_phasemod_sh_data_out | auto_ss_u_phrom_data_out | auto_ss_u_exprom_data_out | auto_ss_out_padding_data_out | auto_ss_shsignbit_data_out;

  logic        auto_ss_local_ack;

  logic [31:0] auto_ss_local_data_out;

  wire         auto_ss_shsignbit_ack;

  wire  [31:0] auto_ss_shsignbit_data_out;

  wire         auto_ss_out_padding_ack;

  wire  [31:0] auto_ss_out_padding_data_out;

  wire         auto_ss_u_exprom_ack;

  wire  [31:0] auto_ss_u_exprom_data_out;

  wire         auto_ss_u_phrom_ack;

  wire  [31:0] auto_ss_u_phrom_data_out;

  wire         auto_ss_phasemod_sh_ack;

  wire  [31:0] auto_ss_phasemod_sh_data_out;

  wire         auto_ss_prev2_buffer_ack;

  wire  [31:0] auto_ss_prev2_buffer_data_out;

  wire         auto_ss_prevprev1_buffer_ack;

  wire  [31:0] auto_ss_prevprev1_buffer_data_out;

  wire         auto_ss_prev1_buffer_ack;

  wire  [31:0] auto_ss_prev1_buffer_data_out;

  wire         device_match = (auto_ss_device_idx == auto_ss_base_device_idx);

  genvar auto_ss_idx;


  /*  enters  exits
    m1      c1
    m2      S4
    c1      m1
    S4      m2
*/

  wire signed [13:0] prev1, prevprev1, prev2;

  jt51_sh #(
      .width (14),
      .stages(8)
  ) prev1_buffer (
      .rst                    (rst),
      .clk                    (clk),
      .cen                    (cen),
      .din                    (c1_enters ? op_XVII : prev1),
      .drop                   (prev1),
      .auto_ss_rd             (auto_ss_rd),
      .auto_ss_wr             (auto_ss_wr),
      .auto_ss_data_in        (auto_ss_data_in),
      .auto_ss_device_idx     (auto_ss_device_idx),
      .auto_ss_state_idx      (auto_ss_state_idx),
      .auto_ss_base_device_idx(auto_ss_base_device_idx + 1),
      .auto_ss_data_out       (auto_ss_prev1_buffer_data_out),
      .auto_ss_ack            (auto_ss_prev1_buffer_ack)

  );

  jt51_sh #(
      .width (14),
      .stages(8)
  ) prevprev1_buffer (
      .rst                    (rst),
      .clk                    (clk),
      .cen                    (cen),
      .din                    (c1_enters ? prev1 : prevprev1),
      .drop                   (prevprev1),
      .auto_ss_rd             (auto_ss_rd),
      .auto_ss_wr             (auto_ss_wr),
      .auto_ss_data_in        (auto_ss_data_in),
      .auto_ss_device_idx     (auto_ss_device_idx),
      .auto_ss_state_idx      (auto_ss_state_idx),
      .auto_ss_base_device_idx(auto_ss_base_device_idx + 2),
      .auto_ss_data_out       (auto_ss_prevprev1_buffer_data_out),
      .auto_ss_ack            (auto_ss_prevprev1_buffer_ack)

  );

  jt51_sh #(
      .width (14),
      .stages(8)
  ) prev2_buffer (
      .rst                    (rst),
      .clk                    (clk),
      .cen                    (cen),
      .din                    (m1_enters ? op_XVII : prev2),
      .drop                   (prev2),
      .auto_ss_rd             (auto_ss_rd),
      .auto_ss_wr             (auto_ss_wr),
      .auto_ss_data_in        (auto_ss_data_in),
      .auto_ss_device_idx     (auto_ss_device_idx),
      .auto_ss_state_idx      (auto_ss_state_idx),
      .auto_ss_base_device_idx(auto_ss_base_device_idx + 3),
      .auto_ss_data_out       (auto_ss_prev2_buffer_data_out),
      .auto_ss_ack            (auto_ss_prev2_buffer_ack)

  );

  // REGISTER/CYCLE 1
  // Creation of phase modulation (FM) feedback signal, before shifting
  reg [13:0] x, y;
  reg [14:0] xs, ys, pm_preshift_II;
  reg m1_II;

  always @(*) begin
    x  = ( {14{use_prevprev1}}  & prevprev1 ) |
          ( {14{use_internal_x}} & op_XVII ) |
          ( {14{use_prev2}}      & prev2 );
    y = ({14{use_prev1}} & prev1) | ({14{use_internal_y}} & op_XVII);
    xs = {x[13], x};  // sign-extend
    ys = {y[13], y};  // sign-extend
  end

  always @(posedge clk) begin
    if (cen) begin
      pm_preshift_II <= xs + ys;  // carry is discarded
      m1_II          <= m1_enters;
    end
    if (auto_ss_wr && device_match) begin
      case (auto_ss_state_idx)
        0: begin
          pm_preshift_II <= auto_ss_data_in[14:0];
        end
        1: begin
          m1_II <= auto_ss_data_in[22];
        end
        default: begin
        end
      endcase
    end
  end



  /* REGISTER/CYCLE 2-7 (also YM2612 extra cycles 1-6)
   Shifting of FM feedback signal, adding phase from PG to FM phase
   In YM2203, phasemod_II is not registered at all, it is latched on the first edge 
   in add_pg_phase and the second edge is the output of add_pg_phase. In the YM2612, there
   are 6 cycles worth of registers between the generated (non-registered) phasemod_II signal
   and the input to add_pg_phase.     */

  reg  [9:0] phasemod_II;
  wire [9:0] phasemod_X;

  always @(*) begin
    // Shift FM feedback signal
    if (!m1_II)  // Not m1
      phasemod_II = pm_preshift_II[10:1];  // Bit 0 of pm_preshift_II is never used
    else  // m1
      case (fb_II)
        3'd0:    phasemod_II = 10'd0;
        3'd1:    phasemod_II = {{4{pm_preshift_II[14]}}, pm_preshift_II[14:9]};
        3'd2:    phasemod_II = {{3{pm_preshift_II[14]}}, pm_preshift_II[14:8]};
        3'd3:    phasemod_II = {{2{pm_preshift_II[14]}}, pm_preshift_II[14:7]};
        3'd4:    phasemod_II = {pm_preshift_II[14], pm_preshift_II[14:6]};
        3'd5:    phasemod_II = pm_preshift_II[14:5];
        3'd6:    phasemod_II = pm_preshift_II[13:4];
        3'd7:    phasemod_II = pm_preshift_II[12:3];
        default: phasemod_II = 10'dx;
      endcase
  end

  // REGISTER/CYCLE 2-9
  jt51_sh #(
      .width (10),
      .stages(8)
  ) phasemod_sh (
      .rst                    (rst),
      .clk                    (clk),
      .cen                    (cen),
      .din                    (phasemod_II),
      .drop                   (phasemod_X),
      .auto_ss_rd             (auto_ss_rd),
      .auto_ss_wr             (auto_ss_wr),
      .auto_ss_data_in        (auto_ss_data_in),
      .auto_ss_device_idx     (auto_ss_device_idx),
      .auto_ss_state_idx      (auto_ss_state_idx),
      .auto_ss_base_device_idx(auto_ss_base_device_idx + 4),
      .auto_ss_data_out       (auto_ss_phasemod_sh_data_out),
      .auto_ss_ack            (auto_ss_phasemod_sh_ack)

  );


  // REGISTER/CYCLE 10
  reg [9:0] phase;
  // Sets the maximum number of fanouts for a register or combinational
  // cell.  The Quartus II software will replicate the cell and split
  // the fanouts among the duplicates until the fanout of each cell
  // is below the maximum.

  reg [7:0] phaselo_XI, aux_X;
  reg signbit_X;

  always @(*) begin
    phase     = phasemod_X + pg_phase_X;
    aux_X     = phase[7:0] ^ {8{~phase[8]}};
    signbit_X = phase[9];
  end

  always @(posedge clk) begin
    if (cen) begin
      phaselo_XI <= aux_X;
    end
    if (auto_ss_wr && device_match) begin
      case (auto_ss_state_idx)
        1: begin
          phaselo_XI <= auto_ss_data_in[17:10];
        end
        default: begin
        end
      endcase
    end
  end



  wire [45:0] sta_XI;

  jt51_phrom u_phrom (
      .clk                    (clk),
      .cen                    (cen),
      .addr                   (aux_X[5:1]),
      .ph                     (sta_XI),
      .auto_ss_rd             (auto_ss_rd),
      .auto_ss_wr             (auto_ss_wr),
      .auto_ss_data_in        (auto_ss_data_in),
      .auto_ss_device_idx     (auto_ss_device_idx),
      .auto_ss_state_idx      (auto_ss_state_idx),
      .auto_ss_base_device_idx(auto_ss_base_device_idx + 5),
      .auto_ss_data_out       (auto_ss_u_phrom_data_out),
      .auto_ss_ack            (auto_ss_u_phrom_ack)

  );

  // REGISTER/CYCLE 11
  // Sine table    
  // Main sine table body
  reg [18:0] stb;
  reg [10:0] stf, stg;
  reg [11:0] logsin;
  reg [10:0] subtresult;
  reg [11:0] atten_internal_XI;

  always @(*) begin
    //sta_XI = sinetable[ phaselo_XI[5:1] ];
    // 2-bit row chooser
    case (phaselo_XI[7:6])
      2'b00:
      stb = {
        10'b0, sta_XI[29], sta_XI[25], 2'b0, sta_XI[18], sta_XI[14], 1'b0, sta_XI[7], sta_XI[3]
      };
      2'b01:
      stb = {
        6'b0,
        sta_XI[37],
        sta_XI[34],
        2'b0,
        sta_XI[28],
        sta_XI[24],
        2'b0,
        sta_XI[17],
        sta_XI[13],
        sta_XI[10],
        sta_XI[6],
        sta_XI[2]
      };
      2'b10:
      stb = {
        2'b0,
        sta_XI[43],
        sta_XI[41],
        2'b0,
        sta_XI[36],
        sta_XI[33],
        2'b0,
        sta_XI[27],
        sta_XI[23],
        1'b0,
        sta_XI[20],
        sta_XI[16],
        sta_XI[12],
        sta_XI[9],
        sta_XI[5],
        sta_XI[1]
      };
      2'b11:
      stb = {
        sta_XI[45],
        sta_XI[44],
        sta_XI[42],
        sta_XI[40]
        , sta_XI[39],
        sta_XI[38],
        sta_XI[35],
        sta_XI[32]
        , sta_XI[31],
        sta_XI[30],
        sta_XI[26],
        sta_XI[22]
        , sta_XI[21],
        sta_XI[19],
        sta_XI[15],
        sta_XI[11]
        , sta_XI[8],
        sta_XI[4],
        sta_XI[0]
      };
      default: stb = 19'dx;
    endcase
    // Fixed value to sum
    stf = {stb[18:15], stb[12:11], stb[8:7], stb[4:3], stb[0]};
    // Gated value to sum; bit 14 is indeed used twice
    if (phaselo_XI[0]) stg = {2'b0, stb[14], stb[14:13], stb[10:9], stb[6:5], stb[2:1]};
    else stg = 11'd0;
    // Sum to produce final logsin value
    logsin            = stf + stg;  // Carry-out of 11-bit addition becomes 12th bit
    // Invert-subtract logsin value from EG attenuation value, with inverted carry
    // In the actual chip, the output of the above logsin sum is already inverted.
    // The two LSBs go through inverters (so they're non-inverted); the eg_atten_XI signal goes through inverters.
    // The adder is normal except the carry-in is 1. It's a 10-bit adder.
    // The outputs are inverted outputs, including the carry bit.
    //subtresult = not (('0' & not eg_atten_XI) - ('1' & logsin([11:2])));
    // After a little pencil-and-paper, turns out this is equivalent to a regular adder!
    subtresult        = eg_atten_XI + logsin[11:2];
    // Place all but carry bit into result; also two LSBs of logsin
    // If addition overflowed, make it the largest value (saturate)
    atten_internal_XI = {subtresult[9:0], logsin[1:0]} | {12{subtresult[10]}};
  end


  // Register cycle 12
  // Exponential table
  wire [44:0] exp_XII;
  reg  [11:0] totalatten_XII;
  reg  [12:0] etb;
  reg  [ 9:0] etf;
  reg  [ 2:0] etg;

  jt51_exprom u_exprom (
      .clk                    (clk),
      .cen                    (cen),
      .addr                   (atten_internal_XI[5:1]),
      .exp                    (exp_XII),
      .auto_ss_rd             (auto_ss_rd),
      .auto_ss_wr             (auto_ss_wr),
      .auto_ss_data_in        (auto_ss_data_in),
      .auto_ss_device_idx     (auto_ss_device_idx),
      .auto_ss_state_idx      (auto_ss_state_idx),
      .auto_ss_base_device_idx(auto_ss_base_device_idx + 6),
      .auto_ss_data_out       (auto_ss_u_exprom_data_out),
      .auto_ss_ack            (auto_ss_u_exprom_ack)

  );

  always @(posedge clk) begin
    if (cen) begin
      totalatten_XII <= atten_internal_XI;
    end
    if (auto_ss_wr && device_match) begin
      case (auto_ss_state_idx)
        0: begin
          totalatten_XII <= auto_ss_data_in[26:15];
        end
        default: begin
        end
      endcase
    end
  end



  //wire [1:0] et_sel  = totalatten_XII[7:6];
  //wire [4:0] et_fine = totalatten_XII[5:1];

  // Main sine table body
  always @(*) begin
    // 2-bit row chooser    
    case (totalatten_XII[7:6])
      2'b00: begin
        etf = {1'b1, exp_XII[44:36]};
        etg = {1'b1, exp_XII[35:34]};
      end
      2'b01: begin
        etf = exp_XII[33:24];
        etg = {2'b10, exp_XII[23]};
      end
      2'b10: begin
        etf = {1'b0, exp_XII[22:14]};
        etg = exp_XII[13:11];
      end
      2'b11: begin
        etf = {2'b00, exp_XII[10:3]};
        etg = exp_XII[2:0];
      end
    endcase
  end

  reg [9:0] mantissa_XIII;
  reg [3:0] exponent_XIII;

  always @(posedge clk) begin
    if (cen) begin
      //RESULT
      mantissa_XIII <= etf + {7'd0, totalatten_XII[0] ? 3'd0 : etg};  //carry-out discarded
      exponent_XIII <= totalatten_XII[11:8];
    end
    if (auto_ss_wr && device_match) begin
      case (auto_ss_state_idx)
        1: begin
          mantissa_XIII <= auto_ss_data_in[9:0];
          exponent_XIII <= auto_ss_data_in[21:18];
        end
        default: begin
        end
      endcase
    end
  end


  always_comb begin
    auto_ss_local_data_out = 32'h0;
    auto_ss_local_ack      = 1'b0;
    if (auto_ss_rd && device_match) begin
      case (auto_ss_state_idx)
        0: begin
          auto_ss_local_data_out[26:0] = {totalatten_XII, pm_preshift_II};
          auto_ss_local_ack            = 1'b1;
        end
        1: begin
          auto_ss_local_data_out[22:0] = {m1_II, exponent_XIII, phaselo_XI, mantissa_XIII};
          auto_ss_local_ack            = 1'b1;
        end
        default: begin
        end
      endcase
    end
  end



  // REGISTER/CYCLE 13
  // Introduce test bit as MSB, 2's complement & Carry-out discarded
  reg [12:0] shifter, shifter_2, shifter_3;

  always @(*) begin
    // Floating-point to integer, and incorporating sign bit
    // Two-stage shifting of mantissa_XIII by exponent_XIII
    shifter = {3'b001, mantissa_XIII};
    case (~exponent_XIII[1:0])
      2'b00: shifter_2 = {1'b0, shifter[12:1]};  // LSB discarded
      2'b01: shifter_2 = shifter;
      2'b10: shifter_2 = {shifter[11:0], 1'b0};
      2'b11: shifter_2 = {shifter[10:0], 2'b0};
    endcase
    case (~exponent_XIII[3:2])
      2'b00: shifter_3 = {12'b0, shifter_2[12]};
      2'b01: shifter_3 = {8'b0, shifter_2[12:8]};
      2'b10: shifter_3 = {4'b0, shifter_2[12:4]};
      2'b11: shifter_3 = shifter_2;
    endcase
  end

  reg signed [13:0] op_XIII;
  wire              signbit_XIII;

  always @(*) begin
    op_XIII = ({test_214, shifter_3} ^ {14{signbit_XIII}}) + {13'd0, signbit_XIII};
  end

  jt51_sh #(
      .width (14),
      .stages(4)
  ) out_padding (
      .rst(rst),
      .clk(clk),
      .cen(cen),
      .din(op_XIII),  // note op_XIII was not latched, is a comb output
      .drop(op_XVII),
      .auto_ss_rd(auto_ss_rd),
      .auto_ss_wr(auto_ss_wr),
      .auto_ss_data_in(auto_ss_data_in),
      .auto_ss_device_idx(auto_ss_device_idx),
      .auto_ss_state_idx(auto_ss_state_idx),
      .auto_ss_base_device_idx(auto_ss_base_device_idx + 7),
      .auto_ss_data_out(auto_ss_out_padding_data_out),
      .auto_ss_ack(auto_ss_out_padding_ack)

  );

  jt51_sh #(
      .width (1),
      .stages(3)
  ) shsignbit (
      .rst                    (rst),
      .clk                    (clk),
      .cen                    (cen),
      .din                    (signbit_X),
      .drop                   (signbit_XIII),
      .auto_ss_rd             (auto_ss_rd),
      .auto_ss_wr             (auto_ss_wr),
      .auto_ss_data_in        (auto_ss_data_in),
      .auto_ss_device_idx     (auto_ss_device_idx),
      .auto_ss_state_idx      (auto_ss_state_idx),
      .auto_ss_base_device_idx(auto_ss_base_device_idx + 8),
      .auto_ss_data_out       (auto_ss_shsignbit_data_out),
      .auto_ss_ack            (auto_ss_shsignbit_ack)

  );

  /////////////////// Debug


endmodule


///////////////////////////////////////////
// MODULE jt51_noise
module jt51_noise (
    input       rst,
    input       clk,
    input       cen,    // phi 1
    input [4:0] cycles,

    // Noise Frequency
    input [4:0] nfrq,

    // Noise envelope
    input [9:0] eg,      // serial signal in the original design
    input       op31_no,

    output              out,
    output reg   [11:0] mix,
    input               auto_ss_rd,
    input               auto_ss_wr,
    input        [31:0] auto_ss_data_in,
    input        [ 7:0] auto_ss_device_idx,
    input        [15:0] auto_ss_state_idx,
    input        [ 7:0] auto_ss_base_device_idx,
    output logic [31:0] auto_ss_data_out,
    output logic        auto_ss_ack

);
  wire device_match = (auto_ss_device_idx == auto_ss_base_device_idx);

  genvar auto_ss_idx;


  reg update, nfrq_met;
  reg [ 4:0] cnt;
  reg [15:0] lfsr;
  reg        last_lfsr0;
  wire all1, fb;
  wire mix_sgn;

  assign out = lfsr[0];

  // period counter

  always @(posedge clk, posedge rst) begin
    if (rst) begin
      cnt <= 5'b0;
    end else if (auto_ss_wr && device_match) begin
      case (auto_ss_state_idx)
        1: begin
          cnt      <= auto_ss_data_in[4:0];
          nfrq_met <= auto_ss_data_in[5];
          update   <= auto_ss_data_in[6];
        end
        default: begin
        end
      endcase
    end else if (cen) begin
      if (&cycles[3:0]) begin
        cnt <= update ? 5'd0 : (cnt + 5'd1);
      end
      update   <= nfrq_met;
      nfrq_met <= ~nfrq == cnt;
    end
  end

  // LFSR

  assign fb   = update ? ~((all1 & ~last_lfsr0) | (lfsr[2] ^ last_lfsr0)) : ~lfsr[0];
  assign all1 = &lfsr;

  always @(posedge clk, posedge rst) begin
    if (rst) begin
      lfsr       <= 16'hffff;
      last_lfsr0 <= 1'b0;
    end else if (auto_ss_wr && device_match) begin
      case (auto_ss_state_idx)
        0: begin
          lfsr <= auto_ss_data_in[15:0];
        end
        1: begin
          last_lfsr0 <= auto_ss_data_in[7];
        end
        default: begin
        end
      endcase
    end else if (cen) begin
      lfsr <= {fb, lfsr[15:1]};
      if (update) last_lfsr0 <= ~lfsr[0];
    end
  end


  // Noise mix

  assign mix_sgn =  /*eg!=10'd0 ^*/ ~out;

  always @(posedge clk, posedge rst) begin
    if (rst) begin
      mix <= 12'd0;
    end else if (auto_ss_wr && device_match) begin
      case (auto_ss_state_idx)
        0: begin
          mix <= auto_ss_data_in[27:16];
        end
        default: begin
        end
      endcase
    end else if (op31_no && cen) begin
      mix <= {mix_sgn, eg[9:2] ^ {8{out}}, {3{mix_sgn}}};
    end
  end
  always_comb begin
    auto_ss_data_out = 32'h0;
    auto_ss_ack      = 1'b0;
    if (auto_ss_rd && device_match) begin
      case (auto_ss_state_idx)
        0: begin
          auto_ss_data_out[27:0] = {mix, lfsr};
          auto_ss_ack            = 1'b1;
        end
        1: begin
          auto_ss_data_out[7:0] = {last_lfsr0, update, nfrq_met, cnt};
          auto_ss_ack           = 1'b1;
        end
        default: begin
        end
      endcase
    end
  end



endmodule


///////////////////////////////////////////
// MODULE jt51_exp2lin
module jt51_exp2lin (
    output reg signed [15:0] lin,
    input  signed     [ 9:0] man,
    input             [ 2:0] exp
);

  always @(*) begin
    case (exp)
      3'd7: lin = {man, 6'b0};
      3'd6: lin = {{1{man[9]}}, man, 5'b0};
      3'd5: lin = {{2{man[9]}}, man, 4'b0};
      3'd4: lin = {{3{man[9]}}, man, 3'b0};
      3'd3: lin = {{4{man[9]}}, man, 2'b0};
      3'd2: lin = {{5{man[9]}}, man, 1'b0};
      3'd1: lin = {{6{man[9]}}, man};
      3'd0: lin = 16'd0;
    endcase
  end

endmodule


///////////////////////////////////////////
// MODULE jt51_lin2exp
module jt51_lin2exp (
    input      [15:0] lin,
    output reg [ 9:0] man,
    output reg [ 2:0] exp
);

  always @(*) begin
    casez (lin[15:9])
      // negative numbers
      7'b10?????: begin
        man = lin[15:6];
        exp = 3'd7;
      end
      7'b110????: begin
        man = lin[14:5];
        exp = 3'd6;
      end
      7'b1110???: begin
        man = lin[13:4];
        exp = 3'd5;
      end
      7'b11110??: begin
        man = lin[12:3];
        exp = 3'd4;
      end
      7'b111110?: begin
        man = lin[11:2];
        exp = 3'd3;
      end
      7'b1111110: begin
        man = lin[10:1];
        exp = 3'd2;
      end
      7'b1111111: begin
        man = lin[9:0];
        exp = 3'd1;
      end
      // positive numbers
      7'b01?????: begin
        man = lin[15:6];
        exp = 3'd7;
      end
      7'b001????: begin
        man = lin[14:5];
        exp = 3'd6;
      end
      7'b0001???: begin
        man = lin[13:4];
        exp = 3'd5;
      end
      7'b00001??: begin
        man = lin[12:3];
        exp = 3'd4;
      end
      7'b000001?: begin
        man = lin[11:2];
        exp = 3'd3;
      end
      7'b0000001: begin
        man = lin[10:1];
        exp = 3'd2;
      end
      7'b0000000: begin
        man = lin[9:0];
        exp = 3'd1;
      end

      default: begin
        man = lin[9:0];
        exp = 3'd1;
      end
    endcase
  end

endmodule


///////////////////////////////////////////
// MODULE jt51_acc
module jt51_acc (
    input                    rst,
    input                    clk,
    input                    cen,
    input                    m1_enters,
    input                    m2_enters,
    input                    c1_enters,
    input                    c2_enters,
    input                    op31_acc,
    input             [ 1:0] rl_I,
    input             [ 2:0] con_I,
    input  signed     [13:0] op_out,
    input                    ne,                       // noise enable
    input  signed     [11:0] noise_mix,
    output signed     [15:0] left,
    output signed     [15:0] right,
    output reg signed [15:0] xleft,                    // exact outputs
    output reg signed [15:0] xright,
    input                    auto_ss_rd,
    input                    auto_ss_wr,
    input             [31:0] auto_ss_data_in,
    input             [ 7:0] auto_ss_device_idx,
    input             [15:0] auto_ss_state_idx,
    input             [ 7:0] auto_ss_base_device_idx,
    output logic      [31:0] auto_ss_data_out,
    output logic             auto_ss_ack

);
  assign auto_ss_ack      = auto_ss_local_ack | auto_ss_u_acc_ack;

  assign auto_ss_data_out = auto_ss_local_data_out | auto_ss_u_acc_data_out;

  logic        auto_ss_local_ack;

  logic [31:0] auto_ss_local_data_out;

  wire         auto_ss_u_acc_ack;

  wire  [31:0] auto_ss_u_acc_data_out;

  wire         device_match = (auto_ss_device_idx == auto_ss_base_device_idx);

  genvar auto_ss_idx;


  reg signed [13:0] op_val;

  always @(*) begin
    if (ne && op31_acc)  // cambiar a OP 31
      op_val = {{2{noise_mix[11]}}, noise_mix};
    else op_val = op_out;
  end

  reg sum_en;

  always @(*) begin
    case (con_I)
      3'd0, 3'd1, 3'd2, 3'd3: sum_en = m2_enters;
      3'd4:                   sum_en = m1_enters | m2_enters;
      3'd5, 3'd6:             sum_en = ~c1_enters;
      3'd7:                   sum_en = 1'b1;
      default:                sum_en = 1'bx;
    endcase
  end

  wire ren = rl_I[1];
  wire len = rl_I[0];
  reg signed [16:0] pre_left, pre_right;
  wire signed [15:0] total;
  wire signed [16:0] total_ex = {total[15], total};

  reg                sum_all;

  wire               rst_sum = c2_enters;
  //wire rst_sum = c1_enters;
  //wire rst_sum = m1_enters;
  //wire rst_sum = m2_enters;

  function signed [15:0] lim16;
    input signed [16:0] din;
    lim16 = !din[16] && din[15] ? 16'h7fff : (din[16] && !din[15] ? 16'h8000 : din[15:0]);
  endfunction


  always @(posedge clk) begin
    begin
      if (rst) begin
        sum_all <= 1'b0;
      end else if (cen) begin
        if (rst_sum) begin
          sum_all <= 1'b1;
          if (!sum_all) begin
            pre_right <= ren ? total_ex : 17'd0;
            pre_left  <= len ? total_ex : 17'd0;
          end else begin
            pre_right <= pre_right + (ren ? total_ex : 17'd0);
            pre_left  <= pre_left + (len ? total_ex : 17'd0);
          end
        end
        if (c1_enters) begin
          sum_all <= 1'b0;
          xleft   <= lim16(pre_left);
          xright  <= lim16(pre_right);
        end
      end
    end
    if (auto_ss_wr && device_match) begin
      case (auto_ss_state_idx)
        0: begin
          pre_left <= auto_ss_data_in[16:0];
        end
        1: begin
          pre_right <= auto_ss_data_in[16:0];
        end
        2: begin
          xleft  <= auto_ss_data_in[15:0];
          xright <= auto_ss_data_in[31:16];
        end
        3: begin
          sum_all <= auto_ss_data_in[0];
        end
        default: begin
        end
      endcase
    end
  end


  always_comb begin
    auto_ss_local_data_out = 32'h0;
    auto_ss_local_ack      = 1'b0;
    if (auto_ss_rd && device_match) begin
      case (auto_ss_state_idx)
        0: begin
          auto_ss_local_data_out[17-1:0] = pre_left;
          auto_ss_local_ack              = 1'b1;
        end
        1: begin
          auto_ss_local_data_out[17-1:0] = pre_right;
          auto_ss_local_ack              = 1'b1;
        end
        2: begin
          auto_ss_local_data_out[31:0] = {xright, xleft};
          auto_ss_local_ack            = 1'b1;
        end
        3: begin
          auto_ss_local_data_out[0] = sum_all;
          auto_ss_local_ack         = 1'b1;
        end
        default: begin
        end
      endcase
    end
  end



  reg signed  [15:0] opsum;
  wire signed [16:0] opsum10 = {{3{op_val[13]}}, op_val} + {total[15], total};

  always @(*) begin
    if (rst_sum) opsum = sum_en ? {{2{op_val[13]}}, op_val} : 16'd0;
    else begin
      if (sum_en)
        if (opsum10[16] == opsum10[15]) opsum = opsum10[15:0];
        else begin
          opsum = opsum10[16] ? 16'h8000 : 16'h7fff;
        end
      else opsum = total;
    end
  end

  jt51_sh #(
      .width (16),
      .stages(8)
  ) u_acc (
      .rst                    (rst),
      .clk                    (clk),
      .cen                    (cen),
      .din                    (opsum),
      .drop                   (total),
      .auto_ss_rd             (auto_ss_rd),
      .auto_ss_wr             (auto_ss_wr),
      .auto_ss_data_in        (auto_ss_data_in),
      .auto_ss_device_idx     (auto_ss_device_idx),
      .auto_ss_state_idx      (auto_ss_state_idx),
      .auto_ss_base_device_idx(auto_ss_base_device_idx + 1),
      .auto_ss_data_out       (auto_ss_u_acc_data_out),
      .auto_ss_ack            (auto_ss_u_acc_ack)

  );


  wire signed [9:0] left_man, right_man;
  wire [2:0] left_exp, right_exp;

  jt51_exp2lin left_reconstruct (
      .lin(left),
      .man(left_man),
      .exp(left_exp)
  );

  jt51_exp2lin right_reconstruct (
      .lin(right),
      .man(right_man),
      .exp(right_exp)
  );

  jt51_lin2exp left2exp (
      .lin(xleft),
      .man(left_man),
      .exp(left_exp)
  );

  jt51_lin2exp right2exp (
      .lin(xright),
      .man(right_man),
      .exp(right_exp)
  );



endmodule


///////////////////////////////////////////
// MODULE jt51_kon
module jt51_kon (
    input       rst,
    input       clk,
    input       cen,
    input [3:0] keyon_op,
    input [2:0] keyon_ch,
    input [1:0] cur_op,
    input [2:0] cur_ch,
    input       up_keyon,
    input       csm,
    input       overflow_A,

    output reg          keyon_II,
    input               auto_ss_rd,
    input               auto_ss_wr,
    input        [31:0] auto_ss_data_in,
    input        [ 7:0] auto_ss_device_idx,
    input        [15:0] auto_ss_state_idx,
    input        [ 7:0] auto_ss_base_device_idx,
    output logic [31:0] auto_ss_data_out,
    output logic        auto_ss_ack

);
  assign auto_ss_ack      = auto_ss_local_ack | auto_ss_u_konch_ack;

  assign auto_ss_data_out = auto_ss_local_data_out | auto_ss_u_konch_data_out;

  logic        auto_ss_local_ack;

  logic [31:0] auto_ss_local_data_out;

  wire         auto_ss_u_konch_ack;

  wire  [31:0] auto_ss_u_konch_data_out;

  wire         device_match = (auto_ss_device_idx == auto_ss_base_device_idx);

  genvar auto_ss_idx;


  //reg csm_copy;

  reg        din;
  wire       drop;

  reg  [3:0] cur_op_hot;

  always @(posedge clk) begin
    if (cen) keyon_II <= (csm && overflow_A) || drop;
    if (auto_ss_wr && device_match) begin
      case (auto_ss_state_idx)
        0: begin
          keyon_II <= auto_ss_data_in[0];
        end
        default: begin
        end
      endcase
    end
  end


  always_comb begin
    auto_ss_local_data_out = 32'h0;
    auto_ss_local_ack      = 1'b0;
    if (auto_ss_rd && device_match) begin
      case (auto_ss_state_idx)
        0: begin
          auto_ss_local_data_out[0] = keyon_II;
          auto_ss_local_ack         = 1'b1;
        end
        default: begin
        end
      endcase
    end
  end



  always @(*) begin
    case (cur_op)
      2'd0: cur_op_hot = 4'b0001;  // S1 / M1
      2'd1: cur_op_hot = 4'b0100;  // S3 / M2
      2'd2: cur_op_hot = 4'b0010;  // S2 / C1
      2'd3: cur_op_hot = 4'b1000;  // S4 / C2
    endcase
    din = keyon_ch == cur_ch && up_keyon ? |(keyon_op & cur_op_hot) : drop;
  end

  jt51_sh #(
      .width (1),
      .stages(32)
  ) u_konch (
      .rst                    (rst),
      .clk                    (clk),
      .cen                    (cen),
      .din                    (din),
      .drop                   (drop),
      .auto_ss_rd             (auto_ss_rd),
      .auto_ss_wr             (auto_ss_wr),
      .auto_ss_data_in        (auto_ss_data_in),
      .auto_ss_device_idx     (auto_ss_device_idx),
      .auto_ss_state_idx      (auto_ss_state_idx),
      .auto_ss_base_device_idx(auto_ss_base_device_idx + 1),
      .auto_ss_data_out       (auto_ss_u_konch_data_out),
      .auto_ss_ack            (auto_ss_u_konch_ack)

  );

endmodule


///////////////////////////////////////////
// MODULE jt51_mod
module jt51_mod (
    input m1_enters,
    input m2_enters,
    input c1_enters,
    input c2_enters,

    input [2:0] alg_I,

    output reg use_prevprev1,
    output reg use_internal_x,
    output reg use_internal_y,
    output reg use_prev2,
    output reg use_prev1
);

  reg [7:0] alg_hot;

  always @(*) begin
    case (alg_I)
      3'd0: alg_hot = 8'h1;  // D0
      3'd1: alg_hot = 8'h2;  // D1
      3'd2: alg_hot = 8'h4;  // D2
      3'd3: alg_hot = 8'h8;  // D3
      3'd4: alg_hot = 8'h10;  // D4
      3'd5: alg_hot = 8'h20;  // D5
      3'd6: alg_hot = 8'h40;  // D6
      3'd7: alg_hot = 8'h80;  // D7
      default: alg_hot = 8'hx;
    endcase
  end

  always @(*) begin
    use_prevprev1 = m1_enters | (m2_enters & alg_hot[5]);
    use_prev2 = (m2_enters & (|alg_hot[2:0])) | (c2_enters & alg_hot[3]);
    use_internal_x = c2_enters & alg_hot[2];
    use_internal_y = c2_enters & (|{alg_hot[4:3], alg_hot[1:0]});
    use_prev1       = m1_enters | (m2_enters&alg_hot[1]) |
        (c1_enters&(|{alg_hot[6:3],alg_hot[0]}) )|
        (c2_enters&(|{alg_hot[5],alg_hot[2]}));
  end

endmodule


///////////////////////////////////////////
// MODULE jt51_csr_op
module jt51_csr_op (
    input       rst,
    input       clk,
    input       cen,
    input [7:0] din,
    input       up_dt1_op,
    input       up_mul_op,
    input       up_tl_op,
    input       up_ks_op,
    input       up_amsen_op,
    input       up_dt2_op,
    input       up_d1l_op,
    input       up_ar_op,
    input       up_d1r_op,
    input       up_d2r_op,
    input       up_rr_op,

    output       [ 2:0] dt1,
    output       [ 3:0] mul,
    output       [ 6:0] tl,
    output       [ 1:0] ks,
    output              amsen,
    output       [ 1:0] dt2,
    output       [ 3:0] d1l,
    output       [ 4:0] arate,
    output       [ 4:0] rate1,
    output       [ 4:0] rate2,
    output       [ 3:0] rrate,
    input               auto_ss_rd,
    input               auto_ss_wr,
    input        [31:0] auto_ss_data_in,
    input        [ 7:0] auto_ss_device_idx,
    input        [15:0] auto_ss_state_idx,
    input        [ 7:0] auto_ss_base_device_idx,
    output logic [31:0] auto_ss_data_out,
    output logic        auto_ss_ack

);
  assign auto_ss_ack      = auto_ss_u_reg0op_ack | auto_ss_u_reg1op_ack;

  assign auto_ss_data_out = auto_ss_u_reg0op_data_out | auto_ss_u_reg1op_data_out;

  wire        auto_ss_u_reg1op_ack;

  wire [31:0] auto_ss_u_reg1op_data_out;

  wire        auto_ss_u_reg0op_ack;

  wire [31:0] auto_ss_u_reg0op_data_out;

  wire        device_match = (auto_ss_device_idx == auto_ss_base_device_idx);

  genvar auto_ss_idx;


  wire [2:0] dt1_in = din[6:4];
  wire [3:0] mul_in = din[3:0];
  wire [6:0] tl_in = din[6:0];
  wire [1:0] ks_in = din[7:6];
  wire amsen_in = din[7];
  wire [1:0] dt2_in = din[7:6];
  wire [3:0] d1l_in = din[7:4];
  wire [4:0] ar_in = din[4:0];
  wire [4:0] d1r_in = din[4:0];
  wire [4:0] d2r_in = din[4:0];
  wire [3:0] rr_in = din[3:0];

  wire [30:0] reg0_in = {
    up_dt1_op ? dt1_in : dt1,  // 3
    up_mul_op ? mul_in : mul,  // 4
    up_ks_op ? ks_in : ks,  // 2
    up_amsen_op ? amsen_in : amsen,  // 1
    up_dt2_op ? dt2_in : dt2,  // 2
    up_d1l_op ? d1l_in : d1l,  // 4

    up_ar_op ? ar_in : arate,  // 5
    up_d1r_op ? d1r_in : rate1,  // 5
    up_d2r_op ? d2r_in : rate2
  };  // 5

  wire [10:0] reg1_in = {
    up_tl_op ? tl_in : tl,  // 7
    up_rr_op ? rr_in : rrate
  };  // 4

  wire [30:0] reg0_out;
  wire [10:0] reg1_out;

  assign {dt1, mul, ks, amsen, dt2, d1l, arate, rate1, rate2, tl, rrate} = {reg0_out, reg1_out};

  // reset to zero
  jt51_sh #(
      .width (31),
      .stages(32)
  ) u_reg0op (
      .rst                    (rst),
      .clk                    (clk),
      .cen                    (cen),
      .din                    (reg0_in),
      .drop                   (reg0_out),
      .auto_ss_rd             (auto_ss_rd),
      .auto_ss_wr             (auto_ss_wr),
      .auto_ss_data_in        (auto_ss_data_in),
      .auto_ss_device_idx     (auto_ss_device_idx),
      .auto_ss_state_idx      (auto_ss_state_idx),
      .auto_ss_base_device_idx(auto_ss_base_device_idx + 1),
      .auto_ss_data_out       (auto_ss_u_reg0op_data_out),
      .auto_ss_ack            (auto_ss_u_reg0op_ack)

  );

  // reset to one
  jt51_sh #(
      .width (11),
      .stages(32),
      .rstval(1'b1)
  ) u_reg1op (
      .rst                    (rst),
      .clk                    (clk),
      .cen                    (cen),
      .din                    (reg1_in),
      .drop                   (reg1_out),
      .auto_ss_rd             (auto_ss_rd),
      .auto_ss_wr             (auto_ss_wr),
      .auto_ss_data_in        (auto_ss_data_in),
      .auto_ss_device_idx     (auto_ss_device_idx),
      .auto_ss_state_idx      (auto_ss_state_idx),
      .auto_ss_base_device_idx(auto_ss_base_device_idx + 2),
      .auto_ss_data_out       (auto_ss_u_reg1op_data_out),
      .auto_ss_ack            (auto_ss_u_reg1op_ack)

  );


endmodule


///////////////////////////////////////////
// MODULE jt51_csr_ch
module jt51_csr_ch (
    input       rst,
    input       clk,
    input       cen,
    input [7:0] din,

    input up_rl_ch,
    input up_fb_ch,
    input up_con_ch,
    input up_kc_ch,
    input up_kf_ch,
    input up_ams_ch,
    input up_pms_ch,

    output       [ 1:0] rl,
    output       [ 2:0] fb,
    output       [ 2:0] con,
    output       [ 6:0] kc,
    output       [ 5:0] kf,
    output       [ 1:0] ams,
    output       [ 2:0] pms,
    input               auto_ss_rd,
    input               auto_ss_wr,
    input        [31:0] auto_ss_data_in,
    input        [ 7:0] auto_ss_device_idx,
    input        [15:0] auto_ss_state_idx,
    input        [ 7:0] auto_ss_base_device_idx,
    output logic [31:0] auto_ss_data_out,
    output logic        auto_ss_ack

);
  assign auto_ss_ack      = auto_ss_u_regop_ack;

  assign auto_ss_data_out = auto_ss_u_regop_data_out;

  wire        auto_ss_u_regop_ack;

  wire [31:0] auto_ss_u_regop_data_out;

  wire        device_match = (auto_ss_device_idx == auto_ss_base_device_idx);

  genvar auto_ss_idx;


  wire [1:0] rl_in = din[7:6];
  wire [2:0] fb_in = din[5:3];
  wire [2:0] con_in = din[2:0];
  wire [6:0] kc_in = din[6:0];
  wire [5:0] kf_in = din[7:2];
  wire [1:0] ams_in = din[1:0];
  wire [2:0] pms_in = din[6:4];

  wire [25:0] reg_in = {
    up_rl_ch ? rl_in : rl,
    up_fb_ch ? fb_in : fb,
    up_con_ch ? con_in : con,
    up_kc_ch ? kc_in : kc,
    up_kf_ch ? kf_in : kf,
    up_ams_ch ? ams_in : ams,
    up_pms_ch ? pms_in : pms
  };

  wire [25:0] reg_out;

  assign {rl, fb, con, kc, kf, ams, pms} = reg_out;

  jt51_sh #(
      .width (26),
      .stages(8)
  ) u_regop (
      .rst                    (rst),
      .clk                    (clk),
      .cen                    (cen),
      .din                    (reg_in),
      .drop                   (reg_out),
      .auto_ss_rd             (auto_ss_rd),
      .auto_ss_wr             (auto_ss_wr),
      .auto_ss_data_in        (auto_ss_data_in),
      .auto_ss_device_idx     (auto_ss_device_idx),
      .auto_ss_state_idx      (auto_ss_state_idx),
      .auto_ss_base_device_idx(auto_ss_base_device_idx + 1),
      .auto_ss_data_out       (auto_ss_u_regop_data_out),
      .auto_ss_ack            (auto_ss_u_regop_ack)

  );


endmodule


///////////////////////////////////////////
// MODULE jt51_reg
module jt51_reg (
    input       rst,
    input       clk,
    input       cen,  // P1
    input [7:0] din,

    input       up_rl,
    input       up_kc,
    input       up_kf,
    input       up_pms,
    input       up_dt1,
    input       up_tl,
    input       up_ks,
    input       up_amsen,
    input       up_dt2,
    input       up_d1l,
    input       up_keyon,
    input [1:0] op,        // operator to update
    input [2:0] ch,        // channel to update

    input csm,
    input overflow_A,

    output [1:0] rl_I,
    output [2:0] fb_II,
    output [2:0] con_I,
    output [6:0] kc_I,
    output [5:0] kf_I,
    output [2:0] pms_I,
    output [1:0] ams_VII,
    output [2:0] dt1_II,
    output [3:0] mul_VI,
    output [6:0] tl_VII,
    output [1:0] ks_III,
    output       amsen_VII,

    output [4:0] arate_II,
    output [4:0] rate1_II,
    output [4:0] rate2_II,
    output [3:0] rrate_II,

    output [1:0] dt2_I,
    output [3:0] d1l_I,
    output       keyon_II,

    // Pipeline order
    output reg       zero,
    output reg       half,
    output     [4:0] cycles,
    output reg       m1_enters,
    output reg       m2_enters,
    output reg       c1_enters,
    output reg       c2_enters,
    // Operator
    output           use_prevprev1,
    output           use_internal_x,
    output           use_internal_y,
    output           use_prev2,
    output           use_prev1,

    output       [ 1:0] cur_op,
    output reg          op31_no,
    output reg          op31_acc,
    input               auto_ss_rd,
    input               auto_ss_wr,
    input        [31:0] auto_ss_data_in,
    input        [ 7:0] auto_ss_device_idx,
    input        [15:0] auto_ss_state_idx,
    input        [ 7:0] auto_ss_base_device_idx,
    output logic [31:0] auto_ss_data_out,
    output logic        auto_ss_ack

);
  assign auto_ss_ack = auto_ss_local_ack | auto_ss_u_kon_ack | auto_ss_u_csr_op_ack | auto_ss_u_csr_ch_ack;

  assign auto_ss_data_out = auto_ss_local_data_out | auto_ss_u_kon_data_out | auto_ss_u_csr_op_data_out | auto_ss_u_csr_ch_data_out;

  logic        auto_ss_local_ack;

  logic [31:0] auto_ss_local_data_out;

  wire         auto_ss_u_csr_ch_ack;

  wire  [31:0] auto_ss_u_csr_ch_data_out;

  wire         auto_ss_u_csr_op_ack;

  wire  [31:0] auto_ss_u_csr_op_data_out;

  wire         auto_ss_u_kon_ack;

  wire  [31:0] auto_ss_u_kon_data_out;

  wire         device_match = (auto_ss_device_idx == auto_ss_base_device_idx);

  genvar auto_ss_idx;


  reg kon, koff;
  reg [1:0] csm_state;
  reg [4:0] csm_cnt;

  // wire csm_kon  = csm_state[0];
  // wire csm_koff = csm_state[1];

  always @(*) begin
    m1_enters = cur_op == 2'b00;
    m2_enters = cur_op == 2'b01;
    c1_enters = cur_op == 2'b10;
    c2_enters = cur_op == 2'b11;
  end



  reg [4:0] cur;

  always @(posedge clk) begin
    if (cen) begin
      op31_no  <= cur == 5'o10;
      op31_acc <= cur == 5'o16;
    end
    if (auto_ss_wr && device_match) begin
      case (auto_ss_state_idx)
        0: begin
          op31_acc <= auto_ss_data_in[5];
          op31_no  <= auto_ss_data_in[6];
        end
        default: begin
        end
      endcase
    end
  end



  assign cur_op = cur[4:3];
  assign cycles = cur;

  wire [4:0] req_I = {op, ch};
  wire [4:0] req_II = req_I + 5'd1;
  wire [4:0] req_III = req_II + 5'd1;
  wire [4:0] req_IV = req_III + 5'd1;
  wire [4:0] req_V = req_IV + 5'd1;
  wire [4:0] req_VI = req_V + 5'd1;
  wire [4:0] req_VII = req_VI + 5'd1;


  wire       update_op_I = cur == req_I;
  wire       update_op_II = cur == req_II;
  wire       update_op_III = cur == req_III;
  // wire    update_op_IV    = cur == req_IV;
  // wire    update_op_V     = cur == req_V;
  wire       update_op_VI = cur == req_VI;
  wire       update_op_VII = cur == req_VII;

  wire       up_rl_ch = up_rl & update_op_I;
  wire       up_fb_ch = up_rl & update_op_II;
  wire       up_con_ch = up_rl & update_op_I;

  wire       up_kc_ch = up_kc & update_op_I;
  wire       up_kf_ch = up_kf & update_op_I;
  wire       up_pms_ch = up_pms & update_op_I;
  wire       up_ams_ch = up_pms & update_op_VII;

  wire       up_dt1_op = up_dt1 & update_op_II;  // DT1, MUL
  wire       up_mul_op = up_dt1 & update_op_VI;  // DT1, MUL
  wire       up_tl_op = up_tl & update_op_VII;
  wire       up_ks_op = up_ks & update_op_III;  // KS, AR
  wire       up_amsen_op = up_amsen & update_op_VII;  // AMS-EN, D1R
  wire       up_dt2_op = up_dt2 & update_op_I;  // DT2, D2R
  wire       up_d1l_op = up_d1l & update_op_I;  // D1L, RR

  wire       up_ar_op = up_ks & update_op_II;  // KS, AR
  wire       up_d1r_op = up_amsen & update_op_II;  // AMS-EN, D1R
  wire       up_d2r_op = up_dt2 & update_op_II;  // DT2, D2R
  wire       up_rr_op = up_d1l & update_op_II;  // D1L, RR

  wire [4:0] next = cur + 5'd1;

  always @(posedge clk, posedge rst) begin : up_counter
    if (rst) begin
      cur  <= 5'h0;
      zero <= 1'b0;
      half <= 1'b0;
    end else if (auto_ss_wr && device_match) begin
      case (auto_ss_state_idx)
        0: begin
          cur  <= auto_ss_data_in[4:0];
          half <= auto_ss_data_in[7];
          zero <= auto_ss_data_in[8];
        end
        default: begin
        end
      endcase
    end else if (cen) begin
      cur  <= next;
      zero <= next == 5'd0;
      half <= next[3:0] == 4'd0;
    end
  end
  always_comb begin
    auto_ss_local_data_out = 32'h0;
    auto_ss_local_ack      = 1'b0;
    if (auto_ss_rd && device_match) begin
      case (auto_ss_state_idx)
        0: begin
          auto_ss_local_data_out[8:0] = {zero, half, op31_no, op31_acc, cur};
          auto_ss_local_ack           = 1'b1;
        end
        default: begin
        end
      endcase
    end
  end



  wire [2:0] cur_ch = cur[2:0];
  wire [3:0] keyon_op = din[6:3];
  wire [2:0] keyon_ch = din[2:0];

  jt51_kon u_kon (
      .rst                    (rst),
      .clk                    (clk),
      .cen                    (cen),
      .keyon_op               (keyon_op),
      .keyon_ch               (keyon_ch),
      .cur_op                 (cur_op),
      .cur_ch                 (cur_ch),
      .up_keyon               (up_keyon),
      .csm                    (csm),
      .overflow_A             (overflow_A),
      .keyon_II               (keyon_II),
      .auto_ss_rd             (auto_ss_rd),
      .auto_ss_wr             (auto_ss_wr),
      .auto_ss_data_in        (auto_ss_data_in),
      .auto_ss_device_idx     (auto_ss_device_idx),
      .auto_ss_state_idx      (auto_ss_state_idx),
      .auto_ss_base_device_idx(auto_ss_base_device_idx + 1),
      .auto_ss_data_out       (auto_ss_u_kon_data_out),
      .auto_ss_ack            (auto_ss_u_kon_ack)

  );


  jt51_mod u_mod (
      .alg_I    (con_I),
      .m1_enters(m1_enters),
      .m2_enters(m2_enters),
      .c1_enters(c1_enters),
      .c2_enters(c2_enters),

      .use_prevprev1 (use_prevprev1),
      .use_internal_x(use_internal_x),
      .use_internal_y(use_internal_y),
      .use_prev2     (use_prev2),
      .use_prev1     (use_prev1)
  );

  jt51_csr_op u_csr_op (
      .rst(rst),
      .clk(clk),
      .cen(cen),  // P1
      .din(din),

      .up_dt1_op  (up_dt1_op),
      .up_mul_op  (up_mul_op),
      .up_tl_op   (up_tl_op),
      .up_ks_op   (up_ks_op),
      .up_amsen_op(up_amsen_op),
      .up_dt2_op  (up_dt2_op),
      .up_d1l_op  (up_d1l_op),
      .up_ar_op   (up_ar_op),
      .up_d1r_op  (up_d1r_op),
      .up_d2r_op  (up_d2r_op),
      .up_rr_op   (up_rr_op),

      .dt1                    (dt1_II),
      .mul                    (mul_VI),
      .tl                     (tl_VII),
      .ks                     (ks_III),
      .amsen                  (amsen_VII),
      .dt2                    (dt2_I),
      .d1l                    (d1l_I),
      .arate                  (arate_II),
      .rate1                  (rate1_II),
      .rate2                  (rate2_II),
      .rrate                  (rrate_II),
      .auto_ss_rd             (auto_ss_rd),
      .auto_ss_wr             (auto_ss_wr),
      .auto_ss_data_in        (auto_ss_data_in),
      .auto_ss_device_idx     (auto_ss_device_idx),
      .auto_ss_state_idx      (auto_ss_state_idx),
      .auto_ss_base_device_idx(auto_ss_base_device_idx + 3),
      .auto_ss_data_out       (auto_ss_u_csr_op_data_out),
      .auto_ss_ack            (auto_ss_u_csr_op_ack)

  );

  jt51_csr_ch u_csr_ch (
      .rst(rst),
      .clk(clk),
      .cen(cen),
      .din(din),

      .up_rl_ch (up_rl_ch),
      .up_fb_ch (up_fb_ch),
      .up_con_ch(up_con_ch),
      .up_kc_ch (up_kc_ch),
      .up_kf_ch (up_kf_ch),
      .up_ams_ch(up_ams_ch),
      .up_pms_ch(up_pms_ch),

      .rl                     (rl_I),
      .fb                     (fb_II),
      .con                    (con_I),
      .kc                     (kc_I),
      .kf                     (kf_I),
      .ams                    (ams_VII),
      .pms                    (pms_I),
      .auto_ss_rd             (auto_ss_rd),
      .auto_ss_wr             (auto_ss_wr),
      .auto_ss_data_in        (auto_ss_data_in),
      .auto_ss_device_idx     (auto_ss_device_idx),
      .auto_ss_state_idx      (auto_ss_state_idx),
      .auto_ss_base_device_idx(auto_ss_base_device_idx + 6),
      .auto_ss_data_out       (auto_ss_u_csr_ch_data_out),
      .auto_ss_ack            (auto_ss_u_csr_ch_ack)

  );

  //////////////////// Debug


endmodule


///////////////////////////////////////////
// MODULE jt51_mmr
module jt51_mmr (
    input            rst,
    input            clk,
    input            cen,    // P1
    input      [7:0] din,
    input            write,
    input            a0,
    output reg       busy,

    // Original test bits
    output reg [7:0] test_mode,

    // CT
    output reg ct1,
    output reg ct2,

    // Noise
    output reg       ne,
    output reg [4:0] nfrq,

    // LFO
    output reg [7:0] lfo_freq,
    output reg [1:0] lfo_w,
    output reg [6:0] lfo_amd,
    output reg [6:0] lfo_pmd,
    output reg       lfo_up,
    // Timers
    output reg [9:0] value_A,
    output reg [7:0] value_B,
    output reg       load_A,
    output reg       load_B,
    output reg       enable_irq_A,
    output reg       enable_irq_B,
    output reg       clr_flag_A,
    output reg       clr_flag_B,
    input            overflow_A,


    // REG
    output [1:0] rl_I,
    output [2:0] fb_II,
    output [2:0] con_I,
    output [6:0] kc_I,
    output [5:0] kf_I,
    output [2:0] pms_I,
    output [1:0] ams_VII,
    output [2:0] dt1_II,
    output [3:0] mul_VI,
    output [6:0] tl_VII,
    output [1:0] ks_III,
    output [4:0] arate_II,
    output       amsen_VII,
    output [4:0] rate1_II,
    output [1:0] dt2_I,
    output [4:0] rate2_II,
    output [3:0] d1l_I,
    output [3:0] rrate_II,
    output       keyon_II,

    output [1:0] cur_op,
    output       op31_no,
    output       op31_acc,

    output              zero,                     // high once per round
    output              half,                     // high twice per round
    output       [ 4:0] cycles,
    output              m1_enters,
    output              m2_enters,
    output              c1_enters,
    output              c2_enters,
    // Operator
    output              use_prevprev1,
    output              use_internal_x,
    output              use_internal_y,
    output              use_prev2,
    output              use_prev1,
    input               auto_ss_rd,
    input               auto_ss_wr,
    input        [31:0] auto_ss_data_in,
    input        [ 7:0] auto_ss_device_idx,
    input        [15:0] auto_ss_state_idx,
    input        [ 7:0] auto_ss_base_device_idx,
    output logic [31:0] auto_ss_data_out,
    output logic        auto_ss_ack

);
  assign auto_ss_ack      = auto_ss_local_ack | auto_ss_u_reg_ack;

  assign auto_ss_data_out = auto_ss_local_data_out | auto_ss_u_reg_data_out;

  logic        auto_ss_local_ack;

  logic [31:0] auto_ss_local_data_out;

  wire         auto_ss_u_reg_ack;

  wire  [31:0] auto_ss_u_reg_data_out;

  wire         device_match = (auto_ss_device_idx == auto_ss_base_device_idx);

  genvar auto_ss_idx;


  reg [7:0] selected_register, din_copy;

  reg up_rl, up_kc, up_kf, up_pms, up_dt1, up_tl, up_ks, up_dt2, up_d1l, up_keyon, up_amsen;
  reg [1:0] up_op;
  reg [2:0] up_ch;



  parameter   REG_TEST    =   8'h01,
            REG_TEST2   =   8'h02,
            REG_KON     =   8'h08,
            REG_NOISE   =   8'h0f,
            REG_CLKA1   =   8'h10,
            REG_CLKA2   =   8'h11,
            REG_CLKB    =   8'h12,
            REG_TIMER   =   8'h14,
            REG_LFRQ    =   8'h18,
            REG_PMDAMD  =   8'h19,
            REG_CTW     =   8'h1b,
            REG_DUMP    =   8'h1f;

  reg csm;

  always @(posedge clk, posedge rst) begin : memory_mapped_registers
    if (rst) begin
      selected_register <= 8'h0;
      { up_rl, up_kc, up_kf, up_pms, up_dt1, up_tl,
                up_ks, up_amsen, up_dt2, up_d1l, up_keyon } <= 11'd0;

      // noise
      nfrq <= 5'b0;
      ne <= 1'b0;
      // timers
      {value_A, value_B} <= 18'd0;
      {clr_flag_B, clr_flag_A, enable_irq_B, enable_irq_A, load_B, load_A} <= 6'd0;
      // LFO
      {lfo_amd, lfo_pmd} <= 14'h0;
      lfo_up <= 1'b0;
      lfo_freq <= 8'd0;
      lfo_w <= 2'd0;
      {ct2, ct1} <= 2'd0;
      csm <= 1'b0;
      din_copy <= 8'd0;
      test_mode <= 8'd0;

    end else if (auto_ss_wr && device_match) begin
      case (auto_ss_state_idx)
        0: begin
          value_A  <= auto_ss_data_in[9:0];
          din_copy <= auto_ss_data_in[17:10];
          lfo_freq <= auto_ss_data_in[25:18];
        end
        1: begin
          selected_register <= auto_ss_data_in[7:0];
          test_mode         <= auto_ss_data_in[15:8];
          value_B           <= auto_ss_data_in[23:16];
          lfo_amd           <= auto_ss_data_in[30:24];
        end
        2: begin
          lfo_pmd    <= auto_ss_data_in[6:0];
          nfrq       <= auto_ss_data_in[11:7];
          up_ch      <= auto_ss_data_in[19:17];
          lfo_w      <= auto_ss_data_in[21:20];
          up_op      <= auto_ss_data_in[23:22];
          clr_flag_B <= auto_ss_data_in[24];
          csm        <= auto_ss_data_in[25];
          ct2        <= auto_ss_data_in[26];
          lfo_up     <= auto_ss_data_in[27];
          ne         <= auto_ss_data_in[28];
          up_amsen   <= auto_ss_data_in[29];
          up_d1l     <= auto_ss_data_in[30];
          up_dt1     <= auto_ss_data_in[31];
        end
        3: begin
          up_dt2   <= auto_ss_data_in[0];
          up_kc    <= auto_ss_data_in[1];
          up_keyon <= auto_ss_data_in[2];
          up_kf    <= auto_ss_data_in[3];
          up_ks    <= auto_ss_data_in[4];
          up_pms   <= auto_ss_data_in[5];
          up_rl    <= auto_ss_data_in[6];
          up_tl    <= auto_ss_data_in[7];
        end
        default: begin
        end
      endcase
    end else begin
      // WRITE IN REGISTERS
      if (write) begin
        if (!a0) selected_register <= din;
        else begin
          din_copy <= din;
          up_op    <= selected_register[4:3];  // operator to update
          up_ch    <= selected_register[2:0];  // channel to update
          up_rl    <= 1'b0;
          up_kc    <= 1'b0;
          up_kf    <= 1'b0;
          up_pms   <= 1'b0;
          up_dt1   <= 1'b0;
          up_tl    <= 1'b0;
          up_ks    <= 1'b0;
          up_amsen <= 1'b0;
          up_dt2   <= 1'b0;
          up_d1l   <= 1'b0;
          up_keyon <= 1'b0;
          // Global registers
          if (selected_register < 8'h20) begin
            case (selected_register)
              // registros especiales
              REG_TEST: test_mode <= din;  // regardless of din

              REG_KON:   up_keyon <= 1'b1;
              REG_NOISE: {ne, nfrq} <= {din[7], din[4:0]};
              REG_CLKA1: value_A[9:2] <= din;
              REG_CLKA2: value_A[1:0] <= din[1:0];
              REG_CLKB:  value_B <= din;
              REG_TIMER: begin
                csm                                                                  <= din[7];
                {clr_flag_B, clr_flag_A, enable_irq_B, enable_irq_A, load_B, load_A} <= din[5:0];
              end
              REG_LFRQ: begin
                lfo_freq <= din;
                lfo_up   <= 1;
              end
              REG_PMDAMD: begin
                if (!din[7]) lfo_amd <= din[6:0];
                else lfo_pmd <= din[6:0];
              end
              REG_CTW: begin
                {ct2, ct1} <= din[7:6];
                lfo_w      <= din[1:0];
              end

              default: ;
            endcase
          end else
          // channel registers
          if (selected_register < 8'h40) begin
            case (selected_register[4:3])
              2'h0: up_rl <= 1'b1;
              2'h1: up_kc <= 1'b1;
              2'h2: up_kf <= 1'b1;
              2'h3: up_pms <= 1'b1;
            endcase
          end else
          // operator registers
          begin
            case (selected_register[7:5])
              3'h2:    up_dt1 <= 1'b1;
              3'h3:    up_tl <= 1'b1;
              3'h4:    up_ks <= 1'b1;
              3'h5:    up_amsen <= 1'b1;
              3'h6:    up_dt2 <= 1'b1;
              3'h7:    up_d1l <= 1'b1;
              default: ;
            endcase
          end
        end
      end else begin  /* clear once-only bits */

        csm                      <= 0;
        lfo_up                   <= 0;
        {clr_flag_B, clr_flag_A} <= 2'd0;
      end
    end
  end

  reg [4:0] busy_cnt;  // busy lasts for 32 synth clock cycles
  reg       old_write;

  always @(posedge clk) begin

    if (rst) begin
      busy     <= 1'b0;
      busy_cnt <= 5'd0;
    end else begin
      old_write <= write;
      if (!old_write && write && a0) begin  // only set for data writes
        busy     <= 1'b1;
        busy_cnt <= 5'd0;
      end else if (cen) begin
        if (busy_cnt == 5'd31) busy <= 1'b0;
        busy_cnt <= busy_cnt + 5'd1;
      end
    end
    if (auto_ss_wr && device_match) begin
      case (auto_ss_state_idx)
        2: begin
          busy_cnt <= auto_ss_data_in[16:12];
        end
        3: begin
          busy      <= auto_ss_data_in[8];
          old_write <= auto_ss_data_in[9];
        end
        default: begin
        end
      endcase
    end
  end


  always_comb begin
    auto_ss_local_data_out = 32'h0;
    auto_ss_local_ack      = 1'b0;
    if (auto_ss_rd && device_match) begin
      case (auto_ss_state_idx)
        0: begin
          auto_ss_local_data_out[25:0] = {lfo_freq, din_copy, value_A};
          auto_ss_local_ack            = 1'b1;
        end
        1: begin
          auto_ss_local_data_out[30:0] = {lfo_amd, value_B, test_mode, selected_register};
          auto_ss_local_ack            = 1'b1;
        end
        2: begin
          auto_ss_local_data_out[31:0] = {
            up_dt1,
            up_d1l,
            up_amsen,
            ne,
            lfo_up,
            ct2,
            csm,
            clr_flag_B,
            up_op,
            lfo_w,
            up_ch,
            busy_cnt,
            nfrq,
            lfo_pmd
          };
          auto_ss_local_ack = 1'b1;
        end
        3: begin
          auto_ss_local_data_out[9:0] = {
            old_write, busy, up_tl, up_rl, up_pms, up_ks, up_kf, up_keyon, up_kc, up_dt2
          };
          auto_ss_local_ack = 1'b1;
        end
        default: begin
        end
      endcase
    end
  end



  jt51_reg u_reg (
      .rst(rst),
      .clk(clk),      // P1
      .cen(cen),      // P1
      .din(din_copy),

      .up_rl   (up_rl),
      .up_kc   (up_kc),
      .up_kf   (up_kf),
      .up_pms  (up_pms),
      .up_dt1  (up_dt1),
      .up_tl   (up_tl),
      .up_ks   (up_ks),
      .up_amsen(up_amsen),
      .up_dt2  (up_dt2),
      .up_d1l  (up_d1l),
      .up_keyon(up_keyon),
      .op      (up_op),     // operator to update
      .ch      (up_ch),     // channel to update

      .csm       (csm),
      .overflow_A(overflow_A),

      .rl_I (rl_I),
      .fb_II(fb_II),
      .con_I(con_I),

      .kc_I   (kc_I),
      .kf_I   (kf_I),
      .pms_I  (pms_I),
      .ams_VII(ams_VII),

      .dt1_II(dt1_II),
      .dt2_I (dt2_I),
      .mul_VI(mul_VI),
      .tl_VII(tl_VII),
      .ks_III(ks_III),

      .arate_II (arate_II),
      .amsen_VII(amsen_VII),
      .rate1_II (rate1_II),
      .rate2_II (rate2_II),
      .rrate_II (rrate_II),
      .d1l_I    (d1l_I),
      .keyon_II (keyon_II),

      .cur_op                 (cur_op),
      .op31_no                (op31_no),
      .op31_acc               (op31_acc),
      .zero                   (zero),
      .half                   (half),
      .cycles                 (cycles),
      .m1_enters              (m1_enters),
      .m2_enters              (m2_enters),
      .c1_enters              (c1_enters),
      .c2_enters              (c2_enters),
      // Operator
      .use_prevprev1          (use_prevprev1),
      .use_internal_x         (use_internal_x),
      .use_internal_y         (use_internal_y),
      .use_prev2              (use_prev2),
      .use_prev1              (use_prev1),
      .auto_ss_rd             (auto_ss_rd),
      .auto_ss_wr             (auto_ss_wr),
      .auto_ss_data_in        (auto_ss_data_in),
      .auto_ss_device_idx     (auto_ss_device_idx),
      .auto_ss_state_idx      (auto_ss_state_idx),
      .auto_ss_base_device_idx(auto_ss_base_device_idx + 1),
      .auto_ss_data_out       (auto_ss_u_reg_data_out),
      .auto_ss_ack            (auto_ss_u_reg_ack)

  );






endmodule


///////////////////////////////////////////
// MODULE jt51
module jt51 (
    input rst,  // reset
    input clk,  // main clock
    (* direct_enable *) input cen,  // clock enable
    (* direct_enable *) input cen_p1,  // clock enable at half the speed
    input cs_n,  // chip select
    input wr_n,  // write
    input a0,
    input [7:0] din,  // data in
    output [7:0] dout,  // data out
    // peripheral control
    output ct1,
    output ct2,
    output irq_n,  // I do not synchronize this signal
    // Low resolution output (same as real chip)
    output sample,  // marks new output sample
    output signed [15:0] left,
    output signed [15:0] right,
    // Full resolution output
    output signed [15:0] xleft,
    output signed [15:0] xright,
    input auto_ss_rd,
    input auto_ss_wr,
    input [31:0] auto_ss_data_in,
    input [7:0] auto_ss_device_idx,
    input [15:0] auto_ss_state_idx,
    input [7:0] auto_ss_base_device_idx,
    output logic [31:0] auto_ss_data_out,
    output logic auto_ss_ack

);
  assign auto_ss_ack = auto_ss_u_timers_ack | auto_ss_u_lfo_ack | auto_ss_u_pg_ack | auto_ss_u_eg_ack | auto_ss_u_op_ack | auto_ss_u_noise_ack | auto_ss_u_acc_ack | auto_ss_u_mmr_ack;

  assign auto_ss_data_out = auto_ss_u_timers_data_out | auto_ss_u_lfo_data_out | auto_ss_u_pg_data_out | auto_ss_u_eg_data_out | auto_ss_u_op_data_out | auto_ss_u_noise_data_out | auto_ss_u_acc_data_out | auto_ss_u_mmr_data_out;

  wire        auto_ss_u_mmr_ack;

  wire [31:0] auto_ss_u_mmr_data_out;

  wire        auto_ss_u_acc_ack;

  wire [31:0] auto_ss_u_acc_data_out;

  wire        auto_ss_u_noise_ack;

  wire [31:0] auto_ss_u_noise_data_out;

  wire        auto_ss_u_op_ack;

  wire [31:0] auto_ss_u_op_data_out;

  wire        auto_ss_u_eg_ack;

  wire [31:0] auto_ss_u_eg_data_out;

  wire        auto_ss_u_pg_ack;

  wire [31:0] auto_ss_u_pg_data_out;

  wire        auto_ss_u_lfo_ack;

  wire [31:0] auto_ss_u_lfo_data_out;

  wire        auto_ss_u_timers_ack;

  wire [31:0] auto_ss_u_timers_data_out;

  wire        device_match = (auto_ss_device_idx == auto_ss_base_device_idx);

  genvar auto_ss_idx;


  // Timers
  wire [9:0] value_A;
  wire [7:0] value_B;
  wire load_A, load_B;
  wire enable_irq_A, enable_irq_B;
  wire clr_flag_A, clr_flag_B;
  wire flag_A, flag_B, overflow_A;
  wire zero, half;
  wire [4:0] cycles;

  jt51_timers u_timers (
      .clk                    (clk),
      .cen                    (cen_p1),
      .rst                    (rst),
      .zero                   (zero),
      .value_A                (value_A),
      .value_B                (value_B),
      .load_A                 (load_A),
      .load_B                 (load_B),
      .enable_irq_A           (enable_irq_A),
      .enable_irq_B           (enable_irq_B),
      .clr_flag_A             (clr_flag_A),
      .clr_flag_B             (clr_flag_B),
      .flag_A                 (flag_A),
      .flag_B                 (flag_B),
      .overflow_A             (overflow_A),
      .irq_n                  (irq_n),
      .auto_ss_rd             (auto_ss_rd),
      .auto_ss_wr             (auto_ss_wr),
      .auto_ss_data_in        (auto_ss_data_in),
      .auto_ss_device_idx     (auto_ss_device_idx),
      .auto_ss_state_idx      (auto_ss_state_idx),
      .auto_ss_base_device_idx(auto_ss_base_device_idx + 1),
      .auto_ss_data_out       (auto_ss_u_timers_data_out),
      .auto_ss_ack            (auto_ss_u_timers_ack)

  );

  /*verilator tracing_on*/



  wire [1:0] rl_I;
  wire [2:0] fb_II;
  wire [2:0] con_I;
  wire [6:0] kc_I;
  wire [5:0] kf_I;
  wire [2:0] pms_I;
  wire [1:0] ams_VII;
  wire [2:0] dt1_II;
  wire [3:0] mul_VI;
  wire [6:0] tl_VII;
  wire [1:0] ks_III;
  wire [4:0] arate_II;
  wire       amsen_VII;
  wire [4:0] rate1_II;
  wire [1:0] dt2_I;
  wire [4:0] rate2_II;
  wire [3:0] d1l_I;
  wire [3:0] rrate_II;

  wire [1:0] cur_op;
  wire       keyon_II;

  wire [7:0] lfo_freq;
  wire [1:0] lfo_w;
  wire       lfo_up;
  wire [7:0] am;
  wire [7:0] pm;
  wire [6:0] amd, pmd;
  wire [7:0] test_mode;
  wire       noise;

  wire m1_enters, m2_enters, c1_enters, c2_enters;
  wire use_prevprev1, use_internal_x, use_internal_y, use_prev2, use_prev1;

  assign sample = zero & cen_p1;  // single strobe

  jt51_lfo u_lfo (
      .rst   (rst),
      .clk   (clk),
      .cen   (cen_p1),
      .cycles(cycles),

      // Configuration
      .lfo_freq(lfo_freq),
      .lfo_w   (lfo_w),
      .lfo_amd (amd),
      .lfo_pmd (pmd),
      .lfo_up  (lfo_up),
      .noise   (noise),

      // Test
      .test   (test_mode),
      .lfo_clk(),

      .am                     (am),
      .pm                     (pm),
      .auto_ss_rd             (auto_ss_rd),
      .auto_ss_wr             (auto_ss_wr),
      .auto_ss_data_in        (auto_ss_data_in),
      .auto_ss_device_idx     (auto_ss_device_idx),
      .auto_ss_state_idx      (auto_ss_state_idx),
      .auto_ss_base_device_idx(auto_ss_base_device_idx + 4),
      .auto_ss_data_out       (auto_ss_u_lfo_data_out),
      .auto_ss_ack            (auto_ss_u_lfo_ack)

  );

  wire [4:0] keycode_III;
  wire [9:0] ph_X;
  wire       pg_rst_III;

  /*verilator tracing_on*/


  jt51_pg u_pg (
      .rst                    (rst),
      .clk                    (clk),                          // P1
      .cen                    (cen_p1),
      .zero                   (zero),
      // Channel frequency
      .kc_I                   (kc_I),
      .kf_I                   (kf_I),
      // Operator multiplying
      .mul_VI                 (mul_VI),
      // Operator detuning
      .dt1_II                 (dt1_II),
      .dt2_I                  (dt2_I),
      // phase modulation from LFO
      .pms_I                  (pms_I),
      .pm                     (pm),
      // phase operation
      .pg_rst_III             (pg_rst_III),
      .keycode_III            (keycode_III),
      .pg_phase_X             (ph_X),
      .auto_ss_rd             (auto_ss_rd),
      .auto_ss_wr             (auto_ss_wr),
      .auto_ss_data_in        (auto_ss_data_in),
      .auto_ss_device_idx     (auto_ss_device_idx),
      .auto_ss_state_idx      (auto_ss_state_idx),
      .auto_ss_base_device_idx(auto_ss_base_device_idx + 5),
      .auto_ss_data_out       (auto_ss_u_pg_data_out),
      .auto_ss_ack            (auto_ss_u_pg_ack)

  );


  wire [9:0] eg_XI;

  jt51_eg u_eg (

      .rst                    (rst),
      .clk                    (clk),
      .cen                    (cen_p1),
      .zero                   (zero),
      // envelope configuration
      .keycode_III            (keycode_III),                  // used in stage III
      .arate_II               (arate_II),
      .rate1_II               (rate1_II),
      .rate2_II               (rate2_II),
      .rrate_II               (rrate_II),
      .d1l_I                  (d1l_I),
      .ks_III                 (ks_III),
      // envelope operation
      .keyon_II               (keyon_II),
      .pg_rst_III             (pg_rst_III),
      // envelope number
      .tl_VII                 (tl_VII),
      .am                     (am),
      .ams_VII                (ams_VII),
      .amsen_VII              (amsen_VII),
      .eg_XI                  (eg_XI),
      .auto_ss_rd             (auto_ss_rd),
      .auto_ss_wr             (auto_ss_wr),
      .auto_ss_data_in        (auto_ss_data_in),
      .auto_ss_device_idx     (auto_ss_device_idx),
      .auto_ss_state_idx      (auto_ss_state_idx),
      .auto_ss_base_device_idx(auto_ss_base_device_idx + 8),
      .auto_ss_data_out       (auto_ss_u_eg_data_out),
      .auto_ss_ack            (auto_ss_u_eg_ack)

  );

  /*verilator tracing_off*/
  wire signed [13:0] op_out;

  jt51_op u_op (

      .rst           (rst),
      .clk           (clk),
      .cen           (cen_p1),
      .pg_phase_X    (ph_X),
      .con_I         (con_I),
      .fb_II         (fb_II),
      // volume
      .eg_atten_XI   (eg_XI),
      // modulation
      .m1_enters     (m1_enters),
      .c1_enters     (c1_enters),
      // Operator
      .use_prevprev1 (use_prevprev1),
      .use_internal_x(use_internal_x),
      .use_internal_y(use_internal_y),
      .use_prev2     (use_prev2),
      .use_prev1     (use_prev1),
      .test_214      (1'b0),

      // output data
      .op_XVII                (op_out),
      .auto_ss_rd             (auto_ss_rd),
      .auto_ss_wr             (auto_ss_wr),
      .auto_ss_data_in        (auto_ss_data_in),
      .auto_ss_device_idx     (auto_ss_device_idx),
      .auto_ss_state_idx      (auto_ss_state_idx),
      .auto_ss_base_device_idx(auto_ss_base_device_idx + 16),
      .auto_ss_data_out       (auto_ss_u_op_data_out),
      .auto_ss_ack            (auto_ss_u_op_ack)

  );

  wire [ 4:0] nfrq;
  wire [11:0] noise_mix;
  wire ne, op31_acc, op31_no;

  jt51_noise u_noise (
      .rst                    (rst),
      .clk                    (clk),
      .cen                    (cen_p1),
      .cycles                 (cycles),
      .nfrq                   (nfrq),
      .eg                     (eg_XI),
      .op31_no                (op31_no),
      .out                    (noise),
      .mix                    (noise_mix),
      .auto_ss_rd             (auto_ss_rd),
      .auto_ss_wr             (auto_ss_wr),
      .auto_ss_data_in        (auto_ss_data_in),
      .auto_ss_device_idx     (auto_ss_device_idx),
      .auto_ss_state_idx      (auto_ss_state_idx),
      .auto_ss_base_device_idx(auto_ss_base_device_idx + 25),
      .auto_ss_data_out       (auto_ss_u_noise_data_out),
      .auto_ss_ack            (auto_ss_u_noise_ack)

  );

  jt51_acc u_acc (
      .rst                    (rst),
      .clk                    (clk),
      .cen                    (cen_p1),
      .m1_enters              (m1_enters),
      .m2_enters              (m2_enters),
      .c1_enters              (c1_enters),
      .c2_enters              (c2_enters),
      .op31_acc               (op31_acc),
      .rl_I                   (rl_I),
      .con_I                  (con_I),
      .op_out                 (op_out),
      .ne                     (ne),
      .noise_mix              (noise_mix),
      .left                   (left),
      .right                  (right),
      .xleft                  (xleft),
      .xright                 (xright),
      .auto_ss_rd             (auto_ss_rd),
      .auto_ss_wr             (auto_ss_wr),
      .auto_ss_data_in        (auto_ss_data_in),
      .auto_ss_device_idx     (auto_ss_device_idx),
      .auto_ss_state_idx      (auto_ss_state_idx),
      .auto_ss_base_device_idx(auto_ss_base_device_idx + 26),
      .auto_ss_data_out       (auto_ss_u_acc_data_out),
      .auto_ss_ack            (auto_ss_u_acc_ack)

  );


  wire busy;
  wire write = !cs_n && !wr_n;

  assign dout = {busy, 5'h0, flag_B, flag_A};

  /*verilator tracing_on*/

  jt51_mmr u_mmr (
      .rst  (rst),
      .clk  (clk),
      .cen  (cen_p1),
      .a0   (a0),
      .write(write),
      .din  (din),
      .busy (busy),

      .test_mode(test_mode),
      // CT
      .ct1      (ct1),        // the LFO clock can be outputted via CT1 -not implemented-
      .ct2      (ct2),
      // LFO
      .lfo_freq (lfo_freq),
      .lfo_w    (lfo_w),
      .lfo_amd  (amd),
      .lfo_pmd  (pmd),
      .lfo_up   (lfo_up),

      // Noise
      .ne  (ne),
      .nfrq(nfrq),

      // Timers
      .value_A     (value_A),
      .value_B     (value_B),
      .load_A      (load_A),
      .load_B      (load_B),
      .enable_irq_A(enable_irq_A),
      .enable_irq_B(enable_irq_B),
      .clr_flag_A  (clr_flag_A),
      .clr_flag_B  (clr_flag_B),
      .overflow_A  (overflow_A),

      // REG
      .rl_I     (rl_I),
      .fb_II    (fb_II),
      .con_I    (con_I),
      .kc_I     (kc_I),
      .kf_I     (kf_I),
      .pms_I    (pms_I),
      .ams_VII  (ams_VII),
      .dt1_II   (dt1_II),
      .mul_VI   (mul_VI),
      .tl_VII   (tl_VII),
      .ks_III   (ks_III),
      .arate_II (arate_II),
      .amsen_VII(amsen_VII),
      .rate1_II (rate1_II),
      .dt2_I    (dt2_I),
      .rate2_II (rate2_II),
      .d1l_I    (d1l_I),
      .rrate_II (rrate_II),
      .keyon_II (keyon_II),

      .cur_op                 (cur_op),
      .op31_no                (op31_no),
      .op31_acc               (op31_acc),
      .zero                   (zero),
      .half                   (half),
      .cycles                 (cycles),
      .m1_enters              (m1_enters),
      .m2_enters              (m2_enters),
      .c1_enters              (c1_enters),
      .c2_enters              (c2_enters),
      // Operator
      .use_prevprev1          (use_prevprev1),
      .use_internal_x         (use_internal_x),
      .use_internal_y         (use_internal_y),
      .use_prev2              (use_prev2),
      .use_prev1              (use_prev1),
      .auto_ss_rd             (auto_ss_rd),
      .auto_ss_wr             (auto_ss_wr),
      .auto_ss_data_in        (auto_ss_data_in),
      .auto_ss_device_idx     (auto_ss_device_idx),
      .auto_ss_state_idx      (auto_ss_state_idx),
      .auto_ss_base_device_idx(auto_ss_base_device_idx + 28),
      .auto_ss_data_out       (auto_ss_u_mmr_data_out),
      .auto_ss_ack            (auto_ss_u_mmr_ack)

  );



endmodule


