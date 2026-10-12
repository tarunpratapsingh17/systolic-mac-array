// -----------------------------------------------------------------------------
// fifo_dpram.v
// Simple dual-port RAM: synchronous write on wr_clk, asynchronous
// (combinational) read. On Xilinx 7-series this infers distributed (LUT) RAM.
//
// The read is combinational so the word at the head of the FIFO is visible on
// rd_data as soon as `empty` deasserts (first-word fall-through behaviour).
// -----------------------------------------------------------------------------
`timescale 1ns/1ps
`default_nettype none

module fifo_dpram #(
    parameter DATA_WIDTH = 8,
    parameter ADDR_WIDTH = 4
) (
    // write port (wr_clk domain)
    input  wire                  wr_clk,
    input  wire                  wr_ce,
    input  wire [ADDR_WIDTH-1:0] wr_addr,
    input  wire [DATA_WIDTH-1:0] wr_data,
    // read port (combinational, used by rd_clk domain)
    input  wire [ADDR_WIDTH-1:0] rd_addr,
    output wire [DATA_WIDTH-1:0] rd_data
);

    localparam DEPTH = 1 << ADDR_WIDTH;

    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];

    always @(posedge wr_clk) begin
        if (wr_ce)
            mem[wr_addr] <= wr_data;
    end

    assign rd_data = mem[rd_addr];

endmodule

`default_nettype wire
