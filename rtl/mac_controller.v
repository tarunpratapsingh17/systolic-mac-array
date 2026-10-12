// -----------------------------------------------------------------------------
// mac_controller.v - compute-domain sequencer (runs on core_clk)
//
// Reads commands from the command FIFO (first-word fall-through) and drives
// the systolic array; pushes result rows into the result FIFO.
//
//   command word = {is_weight, row[N*DW-1:0]}
//     is_weight = 1 : row k of W (rows are taken in order 0..N-1, then wrap)
//     is_weight = 0 : one row of A to multiply by the current W
//
// Rules
//   * the array only advances (en) while the result FIFO has room, so results
//     are never dropped - a slow host simply stalls the array
//   * a weight row is only loaded when no activation is in flight, so every
//     activation row is multiplied by one consistent weight matrix
//   * activation rows are issued back-to-back: 1 row per cycle when available
// -----------------------------------------------------------------------------
`timescale 1ns/1ps
`default_nettype none

module mac_controller #(
    parameter N   = 4,
    parameter DW  = 8,
    parameter ACC = 32
) (
    input  wire              clk,
    input  wire              rst_n,
    // command FIFO, read side
    input  wire              cmd_empty,
    input  wire [N*DW:0]     cmd_data,
    output wire              cmd_rd,
    // result FIFO, write side
    input  wire              res_full,
    output wire              res_wr,
    output wire [N*ACC-1:0]  res_data,
    // systolic array
    output wire              en,
    output wire [N-1:0]      w_load_row,
    output wire [N*DW-1:0]   w_row,
    output wire              a_valid,
    output wire [N*DW-1:0]   a_row,
    input  wire              c_valid,
    input  wire [N*ACC-1:0]  c_row,
    // status
    output wire              busy
);

    localparam IW = $clog2(2*N + 1) + 1;     // in-flight counter width
    localparam RW = (N > 1) ? $clog2(N) : 1;  // weight-row index width

    reg  [IW-1:0] inflight;                   // rows issued but not yet written out
    reg  [RW-1:0] wrow;                       // next weight row to load

    wire head_is_weight = cmd_data[N*DW];
    wire array_idle     = (inflight == {IW{1'b0}});

    assign en = ~res_full;

    wire take_act    = en && !cmd_empty && !head_is_weight;
    wire take_weight = en && !cmd_empty &&  head_is_weight && array_idle;

    assign cmd_rd     = take_act | take_weight;
    assign a_valid    = take_act;
    assign a_row      = take_act ? cmd_data[N*DW-1:0] : {(N*DW){1'b0}};
    assign w_row      = cmd_data[N*DW-1:0];
    assign w_load_row = take_weight ? ({{(N-1){1'b0}}, 1'b1} << wrow) : {N{1'b0}};

    assign res_wr   = en & c_valid;
    assign res_data = c_row;
    assign busy     = ~array_idle;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            inflight <= {IW{1'b0}};
            wrow     <= {RW{1'b0}};
        end else begin
            case ({take_act, res_wr})
                2'b10:   inflight <= inflight + 1'b1;
                2'b01:   inflight <= inflight - 1'b1;
                default: inflight <= inflight;
            endcase
            if (take_weight)
                wrow <= ({{(32-RW){1'b0}}, wrow} == N - 1) ? {RW{1'b0}} : wrow + 1'b1;
        end
    end

endmodule

`default_nettype wire
