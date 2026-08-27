module sample_rom #(
    parameter SS_IDX = -1
) (
    input clk,

    input [15:0] sample_addr_in,
    input [1:0] sample_addr_wr,

    output [7:0] sample_data,
    input sample_inc,

    // ioctl
    input clk_bram,
    input bram_wr,
    input [7:0] bram_data,
    input [19:0] bram_addr,
    input bram_cs,

    // savestate: SSIDX_SAMPLE - the live 18-bit playback pointer (2x16-bit
    // words).  Shared by both the MCU and the M84 Z80 sample paths.
    ssbus_if.slave ssbus
);

/// SAMPLE ROM
dpramv #(.widthad_a(17)) sample_rom
(
    .clock_a(clk),
    .address_a(sample_addr[16:0]),
    .q_a(sample_data),
    .wren_a(1'b0),
    .data_a(),

    .clock_b(clk_bram),
    .address_b(bram_addr[16:0]),
    .data_b(bram_data),
    .wren_b(bram_cs & bram_wr),
    .q_b()
);

reg [17:0] sample_addr = 0;

wire ss_wr = ssbus.access(SS_IDX) & ssbus.write;

always_ff @(posedge clk) begin
    if (sample_inc) sample_addr <= sample_addr + 18'd1;

    if (sample_addr_wr[0]) sample_addr[12:0] <= {sample_addr_in[7:0], 5'd0};
    if (sample_addr_wr[1]) sample_addr[17:13] <= sample_addr_in[12:8];

    // Savestate restore (frozen under pause, so no live update competes).
    if (ss_wr) begin
        if (~ssbus.addr[0]) sample_addr[15:0]  <= ssbus.data[15:0];
        else                sample_addr[17:16] <= ssbus.data[1:0];
    end
end

// Savestate slave protocol: 2 x 16-bit words over the 18-bit pointer.
always_ff @(posedge clk) begin
    ssbus.setup(SS_IDX, 2, 1);

    if (ssbus.access(SS_IDX)) begin
        if (ssbus.write) begin
            ssbus.write_ack(SS_IDX);
        end else if (ssbus.read) begin
            if (~ssbus.addr[0]) ssbus.read_response(SS_IDX, { 48'd0, sample_addr[15:0] });
            else                ssbus.read_response(SS_IDX, { 62'd0, sample_addr[17:16] });
        end
    end
end

endmodule