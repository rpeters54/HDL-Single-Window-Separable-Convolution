`timescale 1ns / 1ps

module carry_save_adder #(
    parameter XLEN = 8
) (
    input  [XLEN-1:0] a,
    input  [XLEN-1:0] b,
    input  [XLEN-1:0] c,
    output [XLEN-1:0] c_out,
    output [XLEN-1:0] sum
);

    genvar i;
    generate
        for (i = 0; i < XLEN; i++) begin : gen_full_adder
            full_adder u_full_adder (
                .a     (a[i]),
                .b     (b[i]),
                .c_in  (c[i]),
                .c_out (c_out[i]),
                .sum   (sum[i])
            );
        end
    endgenerate

endmodule
