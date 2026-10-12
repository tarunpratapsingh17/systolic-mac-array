// -----------------------------------------------------------------------------
// cdc_sync_2ff.v
// Two-flop synchronizer for a multi-bit bus.
//
// Only safe for buses where AT MOST ONE bit changes per source clock
// (e.g. Gray-coded pointers). For arbitrary multi-bit data use a handshake
// or a FIFO instead.
//
// ASYNC_REG tells Vivado to place both flops in the same slice and to treat
// them as a synchronizer chain (better MTBF, excluded from SRL inference).
// -----------------------------------------------------------------------------
`timescale 1ns/1ps
`default_nettype none

module cdc_sync_2ff #(
    parameter WIDTH = 5
) (
    input  wire             clk,      // destination clock
    input  wire             rst_n,    // destination-domain reset (active low)
    input  wire [WIDTH-1:0] d,        // signal from the source domain
    output wire [WIDTH-1:0] q         // synchronized signal
);

    (* ASYNC_REG = "TRUE" *) reg [WIDTH-1:0] sync_ff1;
    (* ASYNC_REG = "TRUE" *) reg [WIDTH-1:0] sync_ff2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sync_ff1 <= {WIDTH{1'b0}};
            sync_ff2 <= {WIDTH{1'b0}};
        end else begin
            sync_ff1 <= d;          // may go metastable
            sync_ff2 <= sync_ff1;   // has a full cycle to resolve
        end
    end

    assign q = sync_ff2;

endmodule

`default_nettype wire
