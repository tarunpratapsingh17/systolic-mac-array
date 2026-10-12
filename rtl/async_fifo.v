// -----------------------------------------------------------------------------
// async_fifo.v  -  Asynchronous (dual-clock) FIFO, top level
//
//   wr_clk domain                             rd_clk domain
//   +--------------+   wr_ptr_gray   +-----------+   +--------------+
//   | fifo_wr_ctrl |---------------->| sync 2FF  |-->| fifo_rd_ctrl |
//   |  (full)      |<----------------| sync 2FF  |<--|  (empty)     |
//   +--------------+   rd_ptr_gray   +-----------+   +--------------+
//          | wr_addr                                        | rd_addr
//          +-------------------> fifo_dpram <---------------+
//
// Parameters
//   DATA_WIDTH        width of each word
//   ADDR_WIDTH        log2(depth); depth = 2**ADDR_WIDTH (min 4 -> ADDR_WIDTH >= 2)
//   ALMOST_FULL_LVL   almost_full  asserts when wr_level >= this
//   ALMOST_EMPTY_LVL  almost_empty asserts when rd_level <= this
//
// Behaviour
//   * write is accepted on wr_clk when  wr_en && !full
//   * read  is accepted on rd_clk when  rd_en && !empty
//   * rd_data shows the head word whenever !empty (first-word fall-through)
//   * full/empty are conservative: they may stay asserted a few cycles longer
//     than strictly necessary, but never allow overflow or underflow.
//
// Resets
//   wr_rst_n / rd_rst_n are active-low, asserted asynchronously. Each must be
//   de-asserted synchronously to its own clock (use a reset synchronizer at
//   the system level) and both domains should be reset together.
// -----------------------------------------------------------------------------
`timescale 1ns/1ps
`default_nettype none

module async_fifo #(
    parameter DATA_WIDTH       = 8,
    parameter ADDR_WIDTH       = 4,
    parameter ALMOST_FULL_LVL  = (1 << ADDR_WIDTH) - ((1 << ADDR_WIDTH) / 4),
    parameter ALMOST_EMPTY_LVL = (1 << ADDR_WIDTH) / 4
) (
    // ---------------- write side (wr_clk domain) ----------------
    input  wire                  wr_clk,
    input  wire                  wr_rst_n,
    input  wire                  wr_en,
    input  wire [DATA_WIDTH-1:0] wr_data,
    output wire                  full,
    output wire                  almost_full,
    output wire [ADDR_WIDTH:0]   wr_level,
    // ---------------- read side (rd_clk domain) -----------------
    input  wire                  rd_clk,
    input  wire                  rd_rst_n,
    input  wire                  rd_en,
    output wire [DATA_WIDTH-1:0] rd_data,
    output wire                  empty,
    output wire                  almost_empty,
    output wire [ADDR_WIDTH:0]   rd_level
);

    // Elaboration-time sanity check (simulation only; SYNTHESIS is predefined by
    // Vivado and Yosys during synthesis).
`ifndef SYNTHESIS
    initial begin
        if (ADDR_WIDTH < 2) begin
            $display("ERROR: async_fifo ADDR_WIDTH must be >= 2 (depth >= 4)");
            $finish;
        end
    end
`endif

    wire [ADDR_WIDTH-1:0] wr_addr, rd_addr;
    wire [ADDR_WIDTH:0]   wr_ptr_gray, rd_ptr_gray;
    wire [ADDR_WIDTH:0]   wr_ptr_gray_sync;   // write ptr seen in rd_clk domain
    wire [ADDR_WIDTH:0]   rd_ptr_gray_sync;   // read ptr seen in wr_clk domain
    wire                  wr_ce;

    // ---------------- storage ----------------
    fifo_dpram #(
        .DATA_WIDTH (DATA_WIDTH),
        .ADDR_WIDTH (ADDR_WIDTH)
    ) u_mem (
        .wr_clk  (wr_clk),
        .wr_ce   (wr_ce),
        .wr_addr (wr_addr),
        .wr_data (wr_data),
        .rd_addr (rd_addr),
        .rd_data (rd_data)
    );

    // ---------------- write side ----------------
    fifo_wr_ctrl #(
        .ADDR_WIDTH      (ADDR_WIDTH),
        .ALMOST_FULL_LVL (ALMOST_FULL_LVL)
    ) u_wr_ctrl (
        .wr_clk           (wr_clk),
        .wr_rst_n         (wr_rst_n),
        .wr_en            (wr_en),
        .rd_ptr_gray_sync (rd_ptr_gray_sync),
        .wr_ce            (wr_ce),
        .wr_addr          (wr_addr),
        .wr_ptr_gray      (wr_ptr_gray),
        .full             (full),
        .almost_full      (almost_full),
        .wr_level         (wr_level)
    );

    // ---------------- read side ----------------
    fifo_rd_ctrl #(
        .ADDR_WIDTH       (ADDR_WIDTH),
        .ALMOST_EMPTY_LVL (ALMOST_EMPTY_LVL)
    ) u_rd_ctrl (
        .rd_clk           (rd_clk),
        .rd_rst_n         (rd_rst_n),
        .rd_en            (rd_en),
        .wr_ptr_gray_sync (wr_ptr_gray_sync),
        .rd_addr          (rd_addr),
        .rd_ptr_gray      (rd_ptr_gray),
        .empty            (empty),
        .almost_empty     (almost_empty),
        .rd_level         (rd_level)
    );

    // ---------------- clock-domain crossings ----------------
    // write pointer -> read clock domain (used for `empty`)
    cdc_sync_2ff #(.WIDTH(ADDR_WIDTH+1)) u_sync_w2r (
        .clk   (rd_clk),
        .rst_n (rd_rst_n),
        .d     (wr_ptr_gray),
        .q     (wr_ptr_gray_sync)
    );

    // read pointer -> write clock domain (used for `full`)
    cdc_sync_2ff #(.WIDTH(ADDR_WIDTH+1)) u_sync_r2w (
        .clk   (wr_clk),
        .rst_n (wr_rst_n),
        .d     (rd_ptr_gray),
        .q     (rd_ptr_gray_sync)
    );

endmodule

`default_nettype wire
