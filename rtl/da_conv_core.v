`timescale 1ns/1ps
`include "defines.vh"

module da_conv_core #(
    parameter              DATA_W     = 8,
    parameter              KERNEL_H   = 7,
    parameter signed [4:0] WEIGHT_0   = 5'sd1,
    parameter signed [4:0] WEIGHT_1   = 5'sd2,
    parameter signed [4:0] WEIGHT_2   = 5'sd3,
    parameter signed [4:0] WEIGHT_3   = 5'sd4,
    parameter signed [4:0] WEIGHT_4   = 5'sd5,
    parameter signed [4:0] WEIGHT_5   = 5'sd6,
    parameter signed [4:0] WEIGHT_6   = 5'sd7,
    parameter signed       NORM_COEFF = 0
) (
    input                                 i_clk,
    input                                 i_rst,
    input                                 i_pipe_en,
    input      [KERNEL_H-1:0][DATA_W-1:0] i_vector,

    output reg [DATA_W-1:0]               o_data
);


    /////////////////////////////////////////////////////////////////
    //
    // Stage 0:
    // - transposes input values to group items at same bit position
    // - passes grouped bits through the LUT multipliers
    // - passes that data through a single stage of the CSAs
    //
    /////////////////////////////////////////////////////////////////

    reg         [DATA_W-1:0][KERNEL_H-1:0] w_lut_addrs;
    wire signed [DATA_W-1:0]               w_lut_sums  [0:DATA_W-1];

    // transpose the input vector to groups of bits at the same place
    genvar pixel_idx, bit_idx;
    generate
        for (bit_idx = 0; bit_idx < DATA_W; bit_idx++) begin
            for (pixel_idx = 0; pixel_idx < KERNEL_H; pixel_idx++) begin
                assign w_lut_addrs[bit_idx][pixel_idx] = i_vector[pixel_idx][bit_idx];
            end
        end
    endgenerate

    // pipe input data through the lut mul unit
    genvar i;
    generate
        for (i = 0; i < 8; i++) begin : gen_lut_mul
            lut_mul #(
                .DATA_W      (DATA_W),
                .KERNEL_H    (KERNEL_H),
                .WEIGHT_0    (WEIGHT_0),
                .WEIGHT_1    (WEIGHT_1),
                .WEIGHT_2    (WEIGHT_2),
                .WEIGHT_3    (WEIGHT_3),
                .WEIGHT_4    (WEIGHT_4),
                .WEIGHT_5    (WEIGHT_5),
                .WEIGHT_6    (WEIGHT_6)
            ) u_lut_mul (
                .d          (w_lut_addrs[i]),
                .sum        (w_lut_sums[i])
            );
        end
    endgenerate


    localparam XLEN_S0_TOP = 11;
    localparam XLEN_S0_MID = 14;
    localparam XLEN_S0_LOW = 15;

    wire signed [XLEN_S0_TOP-2:0] w_csa_1_out        [0:1];
    wire signed [XLEN_S0_MID-2:0] w_csa_2_out        [0:1];
    wire signed [XLEN_S0_TOP-1:0] w_stage_0_top_next [0:1];
    wire signed [XLEN_S0_MID-1:0] w_stage_0_mid_next [0:1];
    wire signed [XLEN_S0_LOW-1:0] w_stage_0_low_next [0:1];

    // create carry save adders
    carry_save_adder #(
        .XLEN(XLEN_S0_TOP-1)
    ) u_carry_save_adder_1 (
        .a      (`SEXT(w_lut_sums[0], 8, XLEN_S0_TOP-1, 0)),
        .b      (`SEXT(w_lut_sums[1], 8, XLEN_S0_TOP-1, 1)),
        .c      (`SEXT(w_lut_sums[2], 8, XLEN_S0_TOP-1, 2)),
        .sum    (w_csa_1_out[`SUM]),
        .c_out  (w_csa_1_out[`CARRY])
    );

    carry_save_adder #(
        .XLEN(XLEN_S0_MID-1)
    ) u_carry_save_adder_2 (
        .a      (`SEXT(w_lut_sums[3], 8, XLEN_S0_MID-1, 3)),
        .b      (`SEXT(w_lut_sums[4], 8, XLEN_S0_MID-1, 4)),
        .c      (`SEXT(w_lut_sums[5], 8, XLEN_S0_MID-1, 5)),
        .sum    (w_csa_2_out[`SUM]),
        .c_out  (w_csa_2_out[`CARRY])
    );

    // format output for the pipeline registers
    assign w_stage_0_top_next[0] = `SEXT(w_csa_1_out[`SUM],   XLEN_S0_TOP-1, XLEN_S0_TOP, 0);
    assign w_stage_0_top_next[1] = `SEXT(w_csa_1_out[`CARRY], XLEN_S0_TOP-1, XLEN_S0_TOP, 1);

    assign w_stage_0_mid_next[0] = `SEXT(w_csa_2_out[`SUM],   XLEN_S0_MID-1, XLEN_S0_MID, 0);
    assign w_stage_0_mid_next[1] = `SEXT(w_csa_2_out[`CARRY], XLEN_S0_MID-1, XLEN_S0_MID, 1);

    assign w_stage_0_low_next[0] = `SEXT(w_lut_sums[6], 8, XLEN_S0_LOW, 6);
    assign w_stage_0_low_next[1] = `SEXT(w_lut_sums[7], 8, XLEN_S0_LOW, 7);

    /////////////////////////////////////////////////////////////////
    //
    // Stage 1:
    // - pads and passes data through the next set of CSAs
    //
    /////////////////////////////////////////////////////////////////

    localparam XLEN_S1_TOP = 15;
    localparam XLEN_S1_LOW = 16;

    // pipeline register output of the previous stage
    reg  signed [XLEN_S0_TOP-1:0] w_stage_0_top      [0:1];
    reg  signed [XLEN_S0_MID-1:0] w_stage_0_mid      [0:1];
    reg  signed [XLEN_S0_LOW-1:0] w_stage_0_low      [0:1];
    wire signed [XLEN_S1_TOP-2:0] w_csa_3_out        [0:1];
    wire signed [XLEN_S1_LOW-2:0] w_csa_4_out        [0:1];
    wire signed [XLEN_S1_TOP-1:0] w_stage_1_top_next [0:1];
    wire signed [XLEN_S1_LOW-1:0] w_stage_1_low_next [0:1];

    // create the two CSAs
    carry_save_adder #(
        .XLEN(XLEN_S1_TOP-1)
    ) u_carry_save_adder_3 (
        .a      (`SEXT(w_stage_0_top[0], XLEN_S0_TOP, XLEN_S1_TOP-1, 0)),
        .b      (`SEXT(w_stage_0_top[1], XLEN_S0_TOP, XLEN_S1_TOP-1, 0)),
        .c      (w_stage_0_mid[0]),
        .sum    (w_csa_3_out[`SUM]),
        .c_out  (w_csa_3_out[`CARRY])
    );

    carry_save_adder #(
        .XLEN(XLEN_S1_LOW-1)
    ) u_carry_save_adder_4 (
        .a      (`SEXT(w_stage_0_mid[1], XLEN_S0_MID, XLEN_S1_LOW-1, 0)),
        .b      (w_stage_0_low[0]),
        .c      (w_stage_0_low[1]),
        .sum    (w_csa_4_out[`SUM]),
        .c_out  (w_csa_4_out[`CARRY])
    );

    // format output for the pipeline registers
    assign w_stage_1_top_next[0] = `SEXT(w_csa_3_out[`SUM],   XLEN_S1_TOP-1, XLEN_S1_TOP, 0);
    assign w_stage_1_top_next[1] = `SEXT(w_csa_3_out[`CARRY], XLEN_S1_TOP-1, XLEN_S1_TOP, 1);

    assign w_stage_1_low_next[0] = `SEXT(w_csa_4_out[`SUM],   XLEN_S1_LOW-1, XLEN_S1_LOW, 0);
    assign w_stage_1_low_next[1] = `SEXT(w_csa_4_out[`CARRY], XLEN_S1_LOW-1, XLEN_S1_LOW, 1);

    /////////////////////////////////////////////////////////////////
    //
    // Stage 9:
    // - pads and passes data through the next set of CSAs
    //
    /////////////////////////////////////////////////////////////////

    localparam XLEN_S2 = 17;

    // pipeline register output of the previous stage
    reg  signed [XLEN_S1_TOP-1:0] w_stage_1_top  [0:1];
    reg  signed [XLEN_S1_LOW-1:0] w_stage_1_low  [0:1];
    wire signed [XLEN_S2-2:0]     w_csa_5_out    [0:1];
    wire signed [XLEN_S2-1:0]     w_stage_2_next [0:2];

    carry_save_adder #(
        .XLEN(XLEN_S2-1)
    ) u_carry_save_adder_5 (
        .a      (`SEXT(w_stage_1_top[0], XLEN_S1_TOP, XLEN_S2-1, 0)),
        .b      (`SEXT(w_stage_1_top[1], XLEN_S1_TOP, XLEN_S2-1, 0)),
        .c      (w_stage_1_low[0]),
        .sum    (w_csa_5_out[`SUM]),
        .c_out  (w_csa_5_out[`CARRY])
    );

    assign w_stage_2_next[0] = `SEXT(w_csa_5_out[`CARRY], XLEN_S2-1, XLEN_S2, 1);
    assign w_stage_2_next[1] = `SEXT(w_csa_5_out[`SUM],   XLEN_S2-1, XLEN_S2, 0);
    assign w_stage_2_next[2] = `SEXT(w_stage_1_low[1],    XLEN_S2-1, XLEN_S2, 0);

    /////////////////////////////////////////////////////////////////
    //
    // Stage 2:
    // - has 1 final CSA to reduce data to two values
    // - passes these final two values through a standard adder
    //
    /////////////////////////////////////////////////////////////////

    localparam XLEN_SUMOUT = 19;

    reg  signed [XLEN_S2-1:0]     w_stage_2   [0:2];
    wire signed [XLEN_S2-1:0]     w_csa_6_out [0:1];
    wire signed [XLEN_SUMOUT-1:0] w_sumout;
    wire signed [XLEN_SUMOUT-1:0] w_sumout_norm;

    carry_save_adder #(
        .XLEN(XLEN_S2)
    ) u_carry_save_adder_6 (
        .a      (w_stage_2[0]),
        .b      (w_stage_2[1]),
        .c      (w_stage_2[2]),
        .sum    (w_csa_6_out[`SUM]),
        .c_out  (w_csa_6_out[`CARRY])
    );

    // result of the LCA (as written in the paper)
    // for the sake of simplicitly I will assume verilog add is quick enough
    assign w_sumout = `SEXT(w_csa_6_out[`CARRY], XLEN_S2, XLEN_SUMOUT, 1) + `SEXT(w_csa_6_out[`SUM], XLEN_S2, XLEN_SUMOUT, 0);

    // The next part is not described in the paper, but we need to normalize
    // the result back into an 8-bit pixel value. We can do so by right
    // shifting the sumout by some normalization value computed from the
    // weights. If the value exceeds 8-bits, we will saturate to 255 (8-bit
    // max).
    //
    // NOTE: I had to move this value external so that it would evaluate
    // properly for testing. Look at row_pe to see how it is calculated

    assign w_sumout_norm = w_sumout >>> NORM_COEFF;

    // pipeline register block
    always @(posedge i_clk) begin
        // reset the pipeline
        if (i_rst) begin

            w_stage_0_top[0] <= 0; w_stage_0_top[1] <= 0;
            w_stage_0_mid[0] <= 0; w_stage_0_mid[1] <= 0;
            w_stage_0_low[0] <= 0; w_stage_0_low[1] <= 0;

            w_stage_1_top[0] <= 0; w_stage_1_top[1] <= 0;
            w_stage_1_low[0] <= 0; w_stage_1_low[1] <= 0;

            w_stage_2[0]     <= 0; w_stage_2[1]     <= 0;
            w_stage_2[2]     <= 0;

        // update pipeline when output latches a new value
        end else if (i_pipe_en) begin
            w_stage_0_top[0] <= w_stage_0_top_next[0]; w_stage_0_top[1] <= w_stage_0_top_next[1];
            w_stage_0_mid[0] <= w_stage_0_mid_next[0]; w_stage_0_mid[1] <= w_stage_0_mid_next[1]; 
            w_stage_0_low[0] <= w_stage_0_low_next[0]; w_stage_0_low[1] <= w_stage_0_low_next[1];

            w_stage_1_top[0] <= w_stage_1_top_next[0]; w_stage_1_top[1] <= w_stage_1_top_next[1];
            w_stage_1_low[0] <= w_stage_1_low_next[0]; w_stage_1_low[1] <= w_stage_1_low_next[1];

            w_stage_2[0]     <= w_stage_2_next[0];     w_stage_2[1]     <= w_stage_2_next[1];
            w_stage_2[2]     <= w_stage_2_next[2];
        end
    end

    // truncate output to between 0-255
    always @(*) begin
        if      (w_sumout_norm > 255)  o_data = 8'd255;
        else if (w_sumout_norm < 0)    o_data = 8'd0;
        else                           o_data = w_sumout_norm[7:0];
    end



endmodule
