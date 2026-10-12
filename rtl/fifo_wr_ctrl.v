// -----------------------------------------------------------------------------
// fifo_wr_ctrl.v
// Write-side control, runs entirely in the wr_clk domain.
//   * keeps the write pointer in binary (for addressing) and Gray (for CDC)
//   * generates a registered `full` flag
//   * reports a conservative occupancy (wr_level) and `almost_full`
//
// Pointers are ADDR_WIDTH+1 bits wide. The extra MSB is a "wrap" bit that
// lets full and empty be told apart when the address bits are equal.
// -----------------------------------------------------------------------------
`timescale 1ns/1ps
`default_nettype none

module fifo_wr_ctrl #(
    parameter ADDR_WIDTH      = 4,
    parameter ALMOST_FULL_LVL = 12
) (
    input  wire                  wr_clk,
    input  wire                  wr_rst_n,
    input  wire                  wr_en,
    input  wire [ADDR_WIDTH:0]   rd_ptr_gray_sync,  // read ptr, already synced to wr_clk
    output wire                  wr_ce,             // actual write strobe to the RAM
    output wire [ADDR_WIDTH-1:0] wr_addr,
    output reg  [ADDR_WIDTH:0]   wr_ptr_gray,       // goes to the read-side synchronizer
    output reg                   full,
    output wire                  almost_full,
    output wire [ADDR_WIDTH:0]   wr_level           // entries used, as seen by the writer
);

    // Gray -> binary: each binary bit is the XOR of all Gray bits above it.
    function [ADDR_WIDTH:0] gray2bin;
        input [ADDR_WIDTH:0] g;
        integer i;
        begin
            gray2bin[ADDR_WIDTH] = g[ADDR_WIDTH];
            for (i = ADDR_WIDTH - 1; i >= 0; i = i - 1)
                gray2bin[i] = gray2bin[i+1] ^ g[i];
        end
    endfunction

    reg  [ADDR_WIDTH:0] wr_ptr_bin;
    wire [ADDR_WIDTH:0] wr_ptr_bin_nxt;
    wire [ADDR_WIDTH:0] wr_ptr_gray_nxt;
    wire                full_nxt;

    // A write only happens when requested AND there is room.
    assign wr_ce           = wr_en & ~full;
    assign wr_ptr_bin_nxt  = wr_ptr_bin + {{ADDR_WIDTH{1'b0}}, wr_ce};
    assign wr_ptr_gray_nxt = (wr_ptr_bin_nxt >> 1) ^ wr_ptr_bin_nxt;

    // Full: the writer is exactly one lap (DEPTH entries) ahead of the reader.
    // In binary that is "MSB different, rest equal". In Gray code the top TWO
    // bits are inverted and the rest equal (Gray reflects around the midpoint).
    assign full_nxt = (wr_ptr_gray_nxt ==
                       {~rd_ptr_gray_sync[ADDR_WIDTH:ADDR_WIDTH-1],
                         rd_ptr_gray_sync[ADDR_WIDTH-2:0]});

    always @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) begin
            wr_ptr_bin  <= {(ADDR_WIDTH+1){1'b0}};
            wr_ptr_gray <= {(ADDR_WIDTH+1){1'b0}};
            full        <= 1'b0;
        end else begin
            wr_ptr_bin  <= wr_ptr_bin_nxt;
            wr_ptr_gray <= wr_ptr_gray_nxt;   // registered -> glitch-free into the CDC
            full        <= full_nxt;
        end
    end

    assign wr_addr = wr_ptr_bin[ADDR_WIDTH-1:0];

    // Occupancy from the write side. The read pointer it sees is 2+ cycles
    // old, so this can only OVER-estimate the fill level -> safe for the writer.
    assign wr_level    = wr_ptr_bin - gray2bin(rd_ptr_gray_sync);
    // zero-extend to 32 bits so the compare against the integer parameter is lint-clean
    wire [31:0] wr_level_ext = {{(31-ADDR_WIDTH){1'b0}}, wr_level};
    assign almost_full = (wr_level_ext >= ALMOST_FULL_LVL);

endmodule

`default_nettype wire
