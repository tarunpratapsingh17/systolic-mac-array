// -----------------------------------------------------------------------------
// fifo_rd_ctrl.v
// Read-side control, runs entirely in the rd_clk domain.
//   * keeps the read pointer in binary (for addressing) and Gray (for CDC)
//   * generates a registered `empty` flag (set during reset)
//   * reports a conservative occupancy (rd_level) and `almost_empty`
// -----------------------------------------------------------------------------
`timescale 1ns/1ps
`default_nettype none

module fifo_rd_ctrl #(
    parameter ADDR_WIDTH       = 4,
    parameter ALMOST_EMPTY_LVL = 4
) (
    input  wire                  rd_clk,
    input  wire                  rd_rst_n,
    input  wire                  rd_en,
    input  wire [ADDR_WIDTH:0]   wr_ptr_gray_sync,  // write ptr, already synced to rd_clk
    output wire [ADDR_WIDTH-1:0] rd_addr,
    output reg  [ADDR_WIDTH:0]   rd_ptr_gray,       // goes to the write-side synchronizer
    output reg                   empty,
    output wire                  almost_empty,
    output wire [ADDR_WIDTH:0]   rd_level           // entries available, as seen by the reader
);

    function [ADDR_WIDTH:0] gray2bin;
        input [ADDR_WIDTH:0] g;
        integer i;
        begin
            gray2bin[ADDR_WIDTH] = g[ADDR_WIDTH];
            for (i = ADDR_WIDTH - 1; i >= 0; i = i - 1)
                gray2bin[i] = gray2bin[i+1] ^ g[i];
        end
    endfunction

    reg  [ADDR_WIDTH:0] rd_ptr_bin;
    wire                rd_ce;
    wire [ADDR_WIDTH:0] rd_ptr_bin_nxt;
    wire [ADDR_WIDTH:0] rd_ptr_gray_nxt;
    wire                empty_nxt;

    // A read only happens when requested AND there is data.
    assign rd_ce           = rd_en & ~empty;
    assign rd_ptr_bin_nxt  = rd_ptr_bin + {{ADDR_WIDTH{1'b0}}, rd_ce};
    assign rd_ptr_gray_nxt = (rd_ptr_bin_nxt >> 1) ^ rd_ptr_bin_nxt;

    // Empty: both pointers identical, including the wrap bit.
    assign empty_nxt = (rd_ptr_gray_nxt == wr_ptr_gray_sync);

    always @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin
            rd_ptr_bin  <= {(ADDR_WIDTH+1){1'b0}};
            rd_ptr_gray <= {(ADDR_WIDTH+1){1'b0}};
            empty       <= 1'b1;
        end else begin
            rd_ptr_bin  <= rd_ptr_bin_nxt;
            rd_ptr_gray <= rd_ptr_gray_nxt;
            empty       <= empty_nxt;
        end
    end

    assign rd_addr = rd_ptr_bin[ADDR_WIDTH-1:0];

    // Occupancy from the read side. The write pointer it sees is 2+ cycles
    // old, so this can only UNDER-estimate the fill level -> safe for the reader.
    assign rd_level     = gray2bin(wr_ptr_gray_sync) - rd_ptr_bin;
    wire [31:0] rd_level_ext = {{(31-ADDR_WIDTH){1'b0}}, rd_level};
    assign almost_empty = (rd_level_ext <= ALMOST_EMPTY_LVL);

endmodule

`default_nettype wire
