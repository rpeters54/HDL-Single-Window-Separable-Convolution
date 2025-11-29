`timescale 1ns/1ps

module row_bank #(
    parameter WIDTH = 8,
    parameter DEPTH = 640
) (
    input                          i_clk,
    input                          i_we,
    input                          i_re,
    input      [$clog2(DEPTH)-1:0] i_addr,
    input      [WIDTH-1:0]         i_data,

    output reg [WIDTH-1:0]         o_data
);

    reg [WIDTH-1:0] w_mem [0:DEPTH-1];

    always @(posedge i_clk) begin
        if (i_we) begin
            w_mem[i_addr] <= i_data;
        end
        if (i_re) begin
            o_data <= w_mem[i_addr];
        end
    end


endmodule
