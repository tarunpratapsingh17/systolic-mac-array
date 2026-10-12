// -----------------------------------------------------------------------------
// reset_sync.v - asynchronous assert, synchronous de-assert reset synchronizer
// -----------------------------------------------------------------------------
`timescale 1ns/1ps
`default_nettype none

module reset_sync (
    input  wire clk,
    input  wire arst_n,     // asynchronous reset in (active low)
    output wire rst_n       // reset synchronized to clk
);

    (* ASYNC_REG = "TRUE" *) reg [1:0] sync;

    always @(posedge clk or negedge arst_n) begin
        if (!arst_n)
            sync <= 2'b00;
        else
            sync <= {sync[0], 1'b1};
    end

    assign rst_n = sync[1];

endmodule

`default_nettype wire
