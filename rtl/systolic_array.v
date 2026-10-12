// -----------------------------------------------------------------------------
// systolic_array.v - N x N weight-stationary systolic array
//
// Computes one output row per cycle:   C[m][j] = sum_k A[m][k] * W[k][j]
//
//   * PE(k,j) holds weight W[k][j]
//   * activation A[m][k] enters array row k from the left and moves right
//   * partial sums start at 0 at the top of each column and move down
//   * row k of the input is delayed k cycles (input skew) so A[m][k] meets the
//     partial sum of column j exactly when it reaches PE(k,j)
//   * column j of the output is delayed N-1-j cycles (output de-skew) so the
//     whole result row comes out on the same cycle
//
// Latency: a row presented on a_row_in appears on c_row_out 2N-1 cycles later
// (counting only cycles with en = 1). Throughput: one row per cycle.
//
// Bus packing: element i of a row lives in bits [i*W +: W].
// -----------------------------------------------------------------------------
`timescale 1ns/1ps
`default_nettype none

module systolic_array #(
    parameter N   = 4,
    parameter DW  = 8,
    parameter ACC = 32
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               en,            // global advance (stall when low)
    // weight load: one row of W at a time
    input  wire [N-1:0]       w_load_row,    // one-hot: which array row to load
    input  wire [N*DW-1:0]    w_row,         // W[k][0..N-1]
    // activation input: one row of A per cycle
    input  wire               a_valid,
    input  wire [N*DW-1:0]    a_row,         // A[m][0..N-1]
    // result output: one row of C per cycle
    output wire               c_valid,
    output wire [N*ACC-1:0]   c_row          // C[m][0..N-1]
);

    // ------------------------------------------------------------------ input skew
    // a_skew[k] is A[m][k] delayed by k cycles (row 0 is not delayed).
    // All internal buses are flat vectors; element e of a bus is [e*W +: W].
    wire [N*DW-1:0] a_skew;

    genvar k, j;
    generate
        for (k = 0; k < N; k = k + 1) begin : g_skew
            if (k == 0) begin : g_nodelay
                assign a_skew[0 +: DW] = a_row[0 +: DW];
            end else begin : g_delay
                reg [k*DW-1:0] sr;                       // k-stage shift register
                if (k == 1) begin : g_one
                    always @(posedge clk or negedge rst_n)
                        if (!rst_n)  sr <= {DW{1'b0}};
                        else if (en) sr <= a_row[k*DW +: DW];
                end else begin : g_many
                    always @(posedge clk or negedge rst_n)
                        if (!rst_n)  sr <= {(k*DW){1'b0}};
                        else if (en) sr <= {sr[(k-1)*DW-1:0], a_row[k*DW +: DW]};
                end
                assign a_skew[k*DW +: DW] = sr[k*DW-1 -: DW];
            end
        end
    endgenerate

    // ------------------------------------------------------------------ PE grid
    // a_h: activation entering PE(k,j) is element k*(N+1)+j
    // p_v: partial sum entering PE(k,j) is element k*N+j (row N = array output)
    /* verilator lint_off UNUSEDSIGNAL */     // activations leaving the right edge are dropped
    wire [N*(N+1)*DW-1:0]  a_h;
    /* verilator lint_on UNUSEDSIGNAL */
    wire [(N+1)*N*ACC-1:0] p_v;

    generate
        for (k = 0; k < N; k = k + 1) begin : g_row
            assign a_h[(k*(N+1))*DW +: DW] = a_skew[k*DW +: DW];
            for (j = 0; j < N; j = j + 1) begin : g_col
                if (k == 0) begin : g_top
                    assign p_v[j*ACC +: ACC] = {ACC{1'b0}};
                end
                /* verilator lint_off PINCONNECTEMPTY */
                mac_pe #(.DW(DW), .ACC(ACC)) u_pe (
                    .clk      (clk),
                    .rst_n    (rst_n),
                    .en       (en),
                    .w_load   (w_load_row[k]),
                    .w_in     (w_row[j*DW +: DW]),
                    .a_in     (a_h[(k*(N+1)+j)*DW +: DW]),
                    .psum_in  (p_v[(k*N+j)*ACC +: ACC]),
                    .a_out    (a_h[(k*(N+1)+j+1)*DW +: DW]),
                    .psum_out (p_v[((k+1)*N+j)*ACC +: ACC]),
                    .w        ()
                );
                /* verilator lint_on PINCONNECTEMPTY */
            end
        end
    endgenerate

    // ------------------------------------------------------------------ output de-skew
    // column j leaves the array N-1-j cycles before column N-1: delay it to match
    generate
        for (j = 0; j < N; j = j + 1) begin : g_deskew
            localparam D = N - 1 - j;                     // delay for this column
            wire [ACC-1:0] col_out = p_v[(N*N+j)*ACC +: ACC];
            if (D == 0) begin : g_nodelay
                assign c_row[j*ACC +: ACC] = col_out;
            end else begin : g_delay
                reg [D*ACC-1:0] sr;
                if (D == 1) begin : g_one
                    always @(posedge clk or negedge rst_n)
                        if (!rst_n)  sr <= {ACC{1'b0}};
                        else if (en) sr <= col_out;
                end else begin : g_many
                    always @(posedge clk or negedge rst_n)
                        if (!rst_n)  sr <= {(D*ACC){1'b0}};
                        else if (en) sr <= {sr[(D-1)*ACC-1:0], col_out};
                end
                assign c_row[j*ACC +: ACC] = sr[D*ACC-1 -: ACC];
            end
        end
    endgenerate

    // ------------------------------------------------------------------ valid pipeline
    reg [2*N-2:0] vpipe;     // 2N-1 stages, matches the datapath latency
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            vpipe <= {(2*N-1){1'b0}};
        else if (en)
            vpipe <= {vpipe[2*N-3:0], a_valid};
    end
    assign c_valid = vpipe[2*N-2];

endmodule

`default_nettype wire
