// -----------------------------------------------------------------------------
// mac_pe.v - weight-stationary processing element (PE)
//
//            a_in ──►[ PE ]──► a_out        (activation moves right, 1 cycle)
//                      │
//         psum_in ─────┘
//                      ▼
//                  psum_out = psum_in + a_in * w   (partial sum moves down)
//
// The weight `w` is loaded once (w_load) and then stays in the PE while many
// activation rows stream through - that is what "weight-stationary" means.
// All datapath registers advance only when `en` is high, so the whole array
// can be stalled (e.g. when the result FIFO is full) without losing data.
// -----------------------------------------------------------------------------
`timescale 1ns/1ps
`default_nettype none

module mac_pe #(
    parameter DW  = 8,     // activation / weight width (signed)
    parameter ACC = 32     // accumulator width (signed)
) (
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  en,
    // weight load
    input  wire                  w_load,
    input  wire signed [DW-1:0]  w_in,
    // dataflow
    input  wire signed [DW-1:0]  a_in,
    input  wire signed [ACC-1:0] psum_in,
    output reg  signed [DW-1:0]  a_out,
    output reg  signed [ACC-1:0] psum_out,
    output reg  signed [DW-1:0]  w
);

    // DW x DW signed product, sign-extended to the accumulator width
    wire signed [2*DW-1:0] prod = a_in * w;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            w        <= {DW{1'b0}};
            a_out    <= {DW{1'b0}};
            psum_out <= {ACC{1'b0}};
        end else begin
            if (w_load)
                w <= w_in;
            if (en) begin
                a_out    <= a_in;
                psum_out <= psum_in + {{(ACC-2*DW){prod[2*DW-1]}}, prod};
            end
        end
    end

endmodule

`default_nettype wire
