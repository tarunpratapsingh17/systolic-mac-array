// -----------------------------------------------------------------------------
// systolic_top.v - systolic MAC array accelerator as an AXI4-Lite SoC subsystem
//
//   aclk domain (host)                         core_clk domain (compute)
//   +----------------+   cmd async FIFO   +----------------+   +-----------------+
//   | AXI4-Lite      |------------------->| mac_controller |-->| systolic_array  |
//   | register slave |<-------------------|                |<--| N x N MAC PEs   |
//   +----------------+  result async FIFO +----------------+   +-----------------+
//
// Register map (32-bit, word aligned)
//   0x00  WEIGHT_ROW  W   push the next row of W (int8 element i in bits [8i+7:8i])
//   0x04  ACT_ROW     W   push one row of A      (same packing)
//   0x08  RESULT      R   next int32 of the result: N reads return one row C[m][0..N-1],
//                         the row is popped after the N-th read (SLVERR if no result)
//   0x0C  STATUS      R   [0] cmd_full  [1] result_valid  [2] cmd_almost_full
//                         [15:8] cmd FIFO level  [23:16] result rows waiting  [31:24] RESULT lane
//   0x10  ID          R   0x534D4143 ("SMAC")
//   0x14  CONFIG      R   [7:0] N  [15:8] DW  [23:16] ACC
//
// A write to WEIGHT_ROW/ACT_ROW is back-pressured (AWREADY/WREADY held low)
// while the command FIFO is full. Writes to other addresses return SLVERR.
// Full-word writes only (WSTRB ignored).
//
// Usage: write N weight rows, then any number of activation rows, then read
// N words of RESULT per activation row. Weights can be reloaded at any time;
// the controller waits for in-flight rows to finish first.
// -----------------------------------------------------------------------------
`timescale 1ns/1ps
`default_nettype none

module systolic_top #(
    parameter N       = 4,     // array is N x N; N*DW must fit in 32 bits
    parameter DW      = 8,     // int8 activations / weights
    parameter ACC     = 32,    // int32 accumulators (one AXI word per result)
    parameter FIFO_AW = 4      // log2 depth of both async FIFOs
) (
    // ---------------- host / AXI4-Lite (aclk domain) ----------------
    input  wire        aclk,
    input  wire        aresetn,

    /* verilator lint_off UNUSEDSIGNAL */      // byte-offset bits / unused data bits
    input  wire [7:0]  s_axil_awaddr,
    input  wire        s_axil_awvalid,
    output wire        s_axil_awready,
    input  wire [31:0] s_axil_wdata,
    input  wire [3:0]  s_axil_wstrb,
    /* verilator lint_on UNUSEDSIGNAL */
    input  wire        s_axil_wvalid,
    output wire        s_axil_wready,
    output reg  [1:0]  s_axil_bresp,
    output reg         s_axil_bvalid,
    input  wire        s_axil_bready,

    /* verilator lint_off UNUSEDSIGNAL */
    input  wire [7:0]  s_axil_araddr,
    /* verilator lint_on UNUSEDSIGNAL */
    input  wire        s_axil_arvalid,
    output wire        s_axil_arready,
    output reg  [31:0] s_axil_rdata,
    output reg  [1:0]  s_axil_rresp,
    output reg         s_axil_rvalid,
    input  wire        s_axil_rready,

    output wire        irq,          // level: a result row is waiting

    // ---------------- compute clock ----------------
    input  wire        core_clk
);

    localparam [7:0] ADDR_WEIGHT = 8'h00,
                     ADDR_ACT    = 8'h04,
                     ADDR_RESULT = 8'h08,
                     ADDR_STATUS = 8'h0C,
                     ADDR_ID     = 8'h10,
                     ADDR_CONFIG = 8'h14;
    localparam [1:0] RESP_OKAY   = 2'b00,
                     RESP_SLVERR = 2'b10;
    localparam       LW          = (N > 1) ? $clog2(N) : 1;   // RESULT lane index width

`ifndef SYNTHESIS
    initial begin
        if (N * DW > 32 || ACC != 32 || N < 2) begin
            $display("ERROR: systolic_top needs N*DW <= 32, ACC == 32, N >= 2");
            $finish;
        end
    end
`endif

    // =========================================================================
    // Resets: aresetn is the master reset, synchronized into the core domain
    // =========================================================================
    wire core_rst_n;
    reset_sync u_core_rst (.clk(core_clk), .arst_n(aresetn), .rst_n(core_rst_n));

    // =========================================================================
    // Clock-domain crossing FIFOs
    // =========================================================================
    wire              cmd_wr, cmd_full, cmd_almost_full;
    wire [N*DW:0]     cmd_wdata;
    wire [FIFO_AW:0]  cmd_level;
    wire              cmd_rd, cmd_empty;
    wire [N*DW:0]     cmd_rdata;

    wire              res_wr, res_full;
    wire [N*ACC-1:0]  res_wdata;
    wire              res_rd, res_empty;
    wire [N*ACC-1:0]  res_rdata;
    wire [FIFO_AW:0]  res_level;

    /* verilator lint_off PINCONNECTEMPTY */
    async_fifo #(.DATA_WIDTH(N*DW + 1), .ADDR_WIDTH(FIFO_AW)) u_cmd_fifo (
        .wr_clk (aclk),     .wr_rst_n (aresetn),    .wr_en (cmd_wr), .wr_data (cmd_wdata),
        .full   (cmd_full), .almost_full (cmd_almost_full), .wr_level (cmd_level),
        .rd_clk (core_clk), .rd_rst_n (core_rst_n), .rd_en (cmd_rd), .rd_data (cmd_rdata),
        .empty  (cmd_empty), .almost_empty (), .rd_level ()
    );

    async_fifo #(.DATA_WIDTH(N*ACC), .ADDR_WIDTH(FIFO_AW)) u_res_fifo (
        .wr_clk (core_clk), .wr_rst_n (core_rst_n), .wr_en (res_wr), .wr_data (res_wdata),
        .full   (res_full), .almost_full (), .wr_level (),
        .rd_clk (aclk),     .rd_rst_n (aresetn),    .rd_en (res_rd), .rd_data (res_rdata),
        .empty  (res_empty), .almost_empty (), .rd_level (res_level)
    );
    /* verilator lint_on PINCONNECTEMPTY */

    // =========================================================================
    // Compute domain: controller + array
    // =========================================================================
    wire              en, a_valid, c_valid;
    wire [N-1:0]      w_load_row;
    wire [N*DW-1:0]   w_row, a_row;
    wire [N*ACC-1:0]  c_row;

    /* verilator lint_off PINCONNECTEMPTY */
    mac_controller #(.N(N), .DW(DW), .ACC(ACC)) u_ctrl (
        .clk        (core_clk),
        .rst_n      (core_rst_n),
        .cmd_empty  (cmd_empty),
        .cmd_data   (cmd_rdata),
        .cmd_rd     (cmd_rd),
        .res_full   (res_full),
        .res_wr     (res_wr),
        .res_data   (res_wdata),
        .en         (en),
        .w_load_row (w_load_row),
        .w_row      (w_row),
        .a_valid    (a_valid),
        .a_row      (a_row),
        .c_valid    (c_valid),
        .c_row      (c_row),
        .busy       ()
    );
    /* verilator lint_on PINCONNECTEMPTY */

    systolic_array #(.N(N), .DW(DW), .ACC(ACC)) u_array (
        .clk        (core_clk),
        .rst_n      (core_rst_n),
        .en         (en),
        .w_load_row (w_load_row),
        .w_row      (w_row),
        .a_valid    (a_valid),
        .a_row      (a_row),
        .c_valid    (c_valid),
        .c_row      (c_row)
    );

    // =========================================================================
    // AXI4-Lite write channel
    // =========================================================================
    wire [7:0] waddr      = {s_axil_awaddr[7:2], 2'b00};
    wire       wr_is_data = (waddr == ADDR_WEIGHT) || (waddr == ADDR_ACT);

    // accept a write when address and data are both present, no response is
    // pending, and (for data registers) the command FIFO has room
    wire wr_go = s_axil_awvalid && s_axil_wvalid && !s_axil_bvalid &&
                 (!wr_is_data || !cmd_full);

    assign s_axil_awready = wr_go;
    assign s_axil_wready  = wr_go;
    assign cmd_wr         = wr_go && wr_is_data;
    assign cmd_wdata      = {(waddr == ADDR_WEIGHT), s_axil_wdata[N*DW-1:0]};

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            s_axil_bvalid <= 1'b0;
            s_axil_bresp  <= RESP_OKAY;
        end else if (wr_go) begin
            s_axil_bvalid <= 1'b1;
            s_axil_bresp  <= wr_is_data ? RESP_OKAY : RESP_SLVERR;
        end else if (s_axil_bready) begin
            s_axil_bvalid <= 1'b0;
        end
    end

    // =========================================================================
    // AXI4-Lite read channel
    // =========================================================================
    wire [7:0] raddr  = {s_axil_araddr[7:2], 2'b00};
    wire       rd_go  = s_axil_arvalid && !s_axil_rvalid;
    reg [LW-1:0] lane;                         // which int32 of the head row is next

    assign s_axil_arready = rd_go;

    wire rd_result = rd_go && (raddr == ADDR_RESULT) && !res_empty;
    wire last_lane = ({{(32-LW){1'b0}}, lane} == N - 1);
    assign res_rd  = rd_result && last_lane;   // pop the row after its last element

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            s_axil_rvalid <= 1'b0;
            s_axil_rdata  <= 32'd0;
            s_axil_rresp  <= RESP_OKAY;
            lane          <= {LW{1'b0}};
        end else if (rd_go) begin
            s_axil_rvalid <= 1'b1;
            s_axil_rresp  <= RESP_OKAY;
            case (raddr)
                ADDR_RESULT: begin
                    if (res_empty) begin
                        s_axil_rdata <= 32'd0;
                        s_axil_rresp <= RESP_SLVERR;
                    end else begin
                        s_axil_rdata <= res_rdata[lane*ACC +: ACC];
                        lane         <= last_lane ? {LW{1'b0}} : lane + 1'b1;
                    end
                end
                ADDR_STATUS: s_axil_rdata <= {{(8-LW){1'b0}}, lane,
                                              {(7-FIFO_AW){1'b0}}, res_level,
                                              {(7-FIFO_AW){1'b0}}, cmd_level,
                                              5'd0, cmd_almost_full, ~res_empty, cmd_full};
                ADDR_ID:     s_axil_rdata <= 32'h534D_4143;
                ADDR_CONFIG: s_axil_rdata <= {8'd0, ACC[7:0], DW[7:0], N[7:0]};
                default: begin
                    s_axil_rdata <= 32'd0;
                    s_axil_rresp <= RESP_SLVERR;
                end
            endcase
        end else if (s_axil_rready) begin
            s_axil_rvalid <= 1'b0;
        end
    end

    assign irq = ~res_empty;

endmodule

`default_nettype wire
