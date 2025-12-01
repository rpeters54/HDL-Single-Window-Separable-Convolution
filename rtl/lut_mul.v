`timescale 1ns/1ps

module lut_mul #(
    parameter              DATA_W    =  8,
    parameter              KERNEL_H  =  7,
    parameter signed [4:0] WEIGHT_0  =  5'sd1,
    parameter signed [4:0] WEIGHT_1  = -5'sd2,
    parameter signed [4:0] WEIGHT_2  =  5'sd3,
    parameter signed [4:0] WEIGHT_3  = -5'sd4,
    parameter signed [4:0] WEIGHT_4  =  5'sd5,
    parameter signed [4:0] WEIGHT_5  = -5'sd6,
    parameter signed [4:0] WEIGHT_6  =  5'sd7
) (
    input         [KERNEL_H-1:0] d,
    output signed [DATA_W-1:0]  sum
);


    // pad the input and weights with a zero to make it easier to 
    // pre-calculate and index the luts

    wire [KERNEL_H:0] d_pad = { d, 1'b0 };

    localparam               TOTAL_W    = (KERNEL_H + 1) * 5;
    localparam [TOTAL_W-1:0] W_PAD_FLAT = { 
        WEIGHT_6, WEIGHT_5, WEIGHT_4, WEIGHT_3, 
        WEIGHT_2, WEIGHT_1, WEIGHT_0, 5'd0
    };

    // populate the lookup tables with all weight combinations
    function [(DATA_W*16)-1:0] init_lut (
        input [TOTAL_W-1:0] w_flat,
        input integer       offset
    );
        integer i, j;
        integer temp_sum;
        integer idx_bit;
        begin
            init_lut = 0; 
            for (i = 0; i < 16; i = i + 1) begin
                temp_sum = 0;
                for (j = 0; j < 4; j = j + 1) begin
                    if (i[j]) begin
                        idx_bit = (offset + j) * 5;
                        temp_sum += integer'($signed(w_flat[idx_bit +: 5]));
                    end
                end
                // Pack into result using Part-Select
                init_lut[(i * DATA_W) +: DATA_W] = temp_sum[DATA_W-1:0];
            end
        end
    endfunction


    // readonly look up tables
    // since the weights are compile-time constants, these values
    // are also compile-time constants.
    localparam [(DATA_W*16)-1:0] L_ROM_FLAT = init_lut(W_PAD_FLAT, 0);
    localparam [(DATA_W*16)-1:0] U_ROM_FLAT = init_lut(W_PAD_FLAT, 4);

    wire signed [DATA_W-1:0] l_val = L_ROM_FLAT[({3'd0, d_pad[3:0]} << $clog2(DATA_W)) +: DATA_W];
    wire signed [DATA_W-1:0] u_val = U_ROM_FLAT[({3'd0, d_pad[7:4]} << $clog2(DATA_W)) +: DATA_W];

    // the output is the sum of the two lookup values
    assign sum = u_val + l_val;


`ifdef FORMAL
    // generate a formal model that emulates the behavior correctly
    reg signed [7:0] ref_sum;
    integer k;

    always @(*) begin
        ref_sum = 0;
        for (k = 0; k < 8; k++) begin
            if (d_pad[k]) begin
                ref_sum = ref_sum + DATA_W'($signed(W_PAD_FLAT[k * 5 +: 5]));
            end
        end
    end

    // assert that for all inputs they are equivalent
    always @(*) begin
        assert(sum == ref_sum);
    end

    // cover some edge cases (for bmc runs)
    always @(*) begin
        cover(sum > 50); 
        cover(sum < -50);
    end
`endif

endmodule
