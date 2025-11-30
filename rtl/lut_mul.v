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

    // readonly look up tables
    reg signed [DATA_W-1:0] l_rom [0:15];
    reg signed [DATA_W-1:0] u_rom [0:15];

    // pad the input and weights with a zero to make it easier to 
    // pre-calculate and index the luts
    wire [KERNEL_H:0]      d_pad = { d,       1'b0 };
    wire [KERNEL_H:0][4:0] w_pad = { 
        WEIGHT_6, WEIGHT_5, WEIGHT_4, WEIGHT_3,
        WEIGHT_2, WEIGHT_1, WEIGHT_0, 5'd0
    };

    // populate the lookup tables with all weight combinations
    integer temp_sum;
    initial begin
        for (integer i = 0; i < 16; i++) begin
            temp_sum = 0;
            for (integer j = 0; j < 4; j++) begin
                if (i[j]) begin
                    temp_sum += integer'($signed(w_pad[j]));
                end
            end
            l_rom[i] = temp_sum[7:0];
        end
        for (integer i = 0; i < 16; i++) begin
            temp_sum = 0;
            for (integer j = 0; j < 4; j++) begin
                if (i[j]) begin
                    temp_sum += integer'($signed(w_pad[j+4]));
                end
            end
            u_rom[i] = temp_sum[7:0];
        end
    end

    // the output is the sum of the two lookup values
    assign sum = u_rom[d_pad[7:4]] + l_rom[d_pad[3:0]];

`ifdef FORMAL

    // generate a formal model that emulates the behavior correctly
    reg signed [7:0] ref_sum;
    integer k;

    always @(*) begin
        ref_sum = 0;
        for (k = 0; k < 8; k++) begin
            if (d_pad[k]) begin
                ref_sum = ref_sum + $signed(w_pad[k]);
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
