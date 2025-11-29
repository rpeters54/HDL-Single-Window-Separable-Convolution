`timescale 1ns/1ps
`include "defines.vh"

module col_pe #(
    parameter       DATA_W    =  8,
    parameter       KERNEL_H  =  7,
    parameter [4:0] WEIGHT_0  =  5'sd1,
    parameter [4:0] WEIGHT_1  = -5'sd2,
    parameter [4:0] WEIGHT_2  =  5'sd3,
    parameter [4:0] WEIGHT_3  = -5'sd4,
    parameter [4:0] WEIGHT_4  =  5'sd5,
    parameter [4:0] WEIGHT_5  = -5'sd6,
    parameter [4:0] WEIGHT_6  =  5'sd7
) (
    input                                 i_clk,  // input clk
    input                                 i_rst,  // reset the system

    input                                 i_rdy,  // output is ready to receive
    input                                 i_vld,  // input is valid
    input                                 i_eof,  // end-of-file
    input      [KERNEL_H-1:0][DATA_W-1:0] i_data, // input column vector

    output reg                            o_rdy,  // tells sender col_pe is ready to receive
    output reg                            o_vld,  // tells receiver the current output is valid
    output reg               [DATA_W-1:0] o_data  // 8-bit column-wise convolved pixel
);

    /////////////////////////////////////////////////////////////////
    //
    // Input Skid Buffer [Stage 0]
    // - latch data whenever it is available and the shift register is moving
    // - exists to break the combinational i_rdy -> o_rdy path that would
    //   massively increase latency
    //
    /////////////////////////////////////////////////////////////////

    wire                            w_sb_vld;
    wire                            w_sb_eof;
    wire [KERNEL_H-1:0][DATA_W-1:0] w_sb_data;
    reg                             w_core_rdy;

    skid_buffer #(
        .DATA_W(KERNEL_H*DATA_W + 1)
    ) u_input_skid (
        .i_clk   (i_clk),
        .i_rst   (i_rst),

        .i_vld   (i_vld),
        .i_data  ({i_eof, i_data}),
        .o_rdy   (o_rdy), 

        .o_vld   (w_sb_vld),
        .o_data  ({w_sb_eof, w_sb_data}),
        .i_rdy   (w_core_rdy)
    );

    /////////////////////////////////////////////////////////////////
    //
    // Input Register [Stage 1]
    // - Feeds new column vector into the pixel processor
    //
    /////////////////////////////////////////////////////////////////

    localparam NORM_COEFF = $clog2(
        `ABS(WEIGHT_0) + `ABS(WEIGHT_1) + `ABS(WEIGHT_2) + `ABS(WEIGHT_3) + 
        `ABS(WEIGHT_4) + `ABS(WEIGHT_5) + `ABS(WEIGHT_6)
    );

    localparam [1:0] FILL   = 0;
    localparam [1:0] READY  = 1;
    localparam [1:0] FLUSH  = 2;

    localparam ADD_STAGES   = 3;

    reg [1:0]                      w_state, w_state_next;
    reg [3:0]                      w_vld_cnt, w_vld_cnt_next;
    reg                            w_vld_next;

    always @(posedge i_clk) begin
        if (i_rst) begin
            w_state   <= FILL;
            w_vld_cnt <= 0;
            o_vld     <= 0;
        end else begin
            w_state   <= w_state_next;
            w_vld_cnt <= w_vld_cnt_next;
            o_vld     <= w_vld_next;
        end
    end

    // FSM tracks when the output is ready and uses backpressure to
    // halt any new inputs from arriving
    always @(*) begin
        w_state_next   = w_state;
        w_vld_cnt_next = w_vld_cnt;
        w_vld_next     = o_vld;
        w_core_rdy     = 0;

        case (w_state)
            FILL : begin
                // collect new data if the skid buffer is valid
                if (w_sb_vld) begin
                    w_core_rdy     = 1;
                    w_vld_cnt_next = w_vld_cnt + 1;

                    // change state depending on if the pipeline is full or we have
                    // received an eor
                    if (w_sb_eof) begin
                        w_state_next = FLUSH;
                    end else if (w_vld_cnt >= (ADD_STAGES - 1)) begin
                        w_state_next = READY;
                    end else begin
                        w_state_next = FILL;
                    end

                    // if moving to ready (or flush in some rarer
                    // circumstances where sr is also full), assert vld
                    if (w_vld_cnt >= (ADD_STAGES - 1)) begin
                        w_vld_next = 1;
                    end else begin
                        w_vld_next = 0;
                    end
                end
            end
            READY : begin
                case ({w_sb_vld, i_rdy})
                    // if not ready, let the skid buffer catch the input
                    2'b10 : begin
                        w_state_next  = READY;
                        w_vld_next    = 1;
                        w_core_rdy    = 0;
                    end
                    // if not valid, move back to fill temporarily
                    2'b01 : begin
                        w_state_next   = FILL;
                        w_vld_cnt_next = w_vld_cnt - 1;
                        w_vld_next     = 0;
                        w_core_rdy     = 0;
                    end
                    // if both, pass through
                    2'b11 : begin
                        if (w_sb_eof) begin
                            w_state_next = FLUSH;
                        end else begin
                            w_state_next = READY;
                        end
                        w_vld_next    = 1;
                        w_core_rdy    = 1;
                    end
                    // if neither, do nothing
                    2'b00 : begin
                        w_state_next  = READY;
                        w_vld_next    = 1;
                        w_core_rdy    = 0;
                    end
                endcase
            end
            FLUSH : begin
                // flush results until the last valid result exits the pipe
                w_core_rdy   = 0;
                w_vld_next   = 1;
                w_state_next = FLUSH;

                if (!o_vld) begin
                    w_vld_cnt_next = 0;
                    w_vld_next     = 0;
                    w_state_next   = FILL;
                end else if (i_rdy) begin
                    // if the adder pipeline is empty, reset
                    if (w_vld_cnt <= 0) begin
                        w_vld_cnt_next = 0;
                        w_vld_next     = 0;
                        w_state_next   = FILL;
                    // shift the pipeline once, gating the input
                    end else begin
                        w_vld_cnt_next = w_vld_cnt - 1;
                    end
                end
            end
            default : ;
        endcase
    end

    /////////////////////////////////////////////////////////////////
    //
    // Convolutional Core Tree Adder [Stages 2 - 4]
    // - use lut multipliers and carry save adders to compute dot product
    // - computation is broken across 3 register stages to reduce
    //   critical path length
    //
    /////////////////////////////////////////////////////////////////

    wire w_internal_shift = 
        (w_state == FILL && w_sb_vld) ||
        (w_state == READY && w_sb_vld && i_rdy) ||
        (w_state == FLUSH && i_rdy);

    wire [KERNEL_H-1:0][DATA_W-1:0] w_core_input =
        (w_state == FLUSH) ? {KERNEL_H*DATA_W{1'b0}} : w_sb_data;

    da_conv_core #(
        .DATA_W     (DATA_W),
        .KERNEL_H   (KERNEL_H),
        .WEIGHT_0   (WEIGHT_0),
        .WEIGHT_1   (WEIGHT_1),
        .WEIGHT_2   (WEIGHT_2),
        .WEIGHT_3   (WEIGHT_3),
        .WEIGHT_4   (WEIGHT_4),
        .WEIGHT_5   (WEIGHT_5),
        .WEIGHT_6   (WEIGHT_6),
        .NORM_COEFF (NORM_COEFF)
    ) u_core (
        .i_clk      (i_clk),
        .i_rst      (i_rst),
        .i_pipe_en  (w_internal_shift),
        .i_vector   (w_core_input),
        .o_data     (o_data)
    );


`ifdef FORMAL

    /////////////////////////////////////////////////////////////////
    // Formal Verification
    /////////////////////////////////////////////////////////////////

    reg f_past_valid = 0;
    always @(posedge i_clk) begin
        f_past_valid <= 1;
        if(i_rst) f_past_valid <= 0;
    end

    always @(*) if (!f_past_valid) assume(i_rst);

    // 1. Recreate control signals
    wire f_shift_en;
    assign f_shift_en = 
        (w_state == FILL && w_sb_vld) ||
        (w_state == READY && w_sb_vld && i_rdy) ||
        (w_state == FLUSH && i_rdy);

    // 2. History Buffer
    // Stores the FULL vector because the core processes vectors in parallel
    reg [KERNEL_H-1:0][DATA_W-1:0] past_inputs [0:ADD_STAGES];
    integer k;

    always @(posedge i_clk) begin
        if (i_rst) begin
            for(k=0; k<=ADD_STAGES; k=k+1) past_inputs[k] <= 0;
        end else if (f_shift_en) begin
            if (w_state == FLUSH) past_inputs[0] <= 0;
            else                  past_inputs[0] <= w_sb_data;

            for(k=0; k<ADD_STAGES; k=k+1) 
                past_inputs[k+1] <= past_inputs[k];
        end
    end

    // 3. Golden Model
    reg signed [18:0] golden_sum;
    always @(*) begin
        golden_sum = 0;
        // The latency is purely ADD_STAGES (3).
        // The input at T-3 is located at past_inputs[3].
        // We act on the vector stored at that specific history slot.
        golden_sum = golden_sum + $signed({1'b0, past_inputs[3][0]}) * $signed(WEIGHT_0);
        golden_sum = golden_sum + $signed({1'b0, past_inputs[3][1]}) * $signed(WEIGHT_1);
        golden_sum = golden_sum + $signed({1'b0, past_inputs[3][2]}) * $signed(WEIGHT_2);
        golden_sum = golden_sum + $signed({1'b0, past_inputs[3][3]}) * $signed(WEIGHT_3);
        golden_sum = golden_sum + $signed({1'b0, past_inputs[3][4]}) * $signed(WEIGHT_4);
        golden_sum = golden_sum + $signed({1'b0, past_inputs[3][5]}) * $signed(WEIGHT_5);
        golden_sum = golden_sum + $signed({1'b0, past_inputs[3][6]}) * $signed(WEIGHT_6);
    end

    reg signed [18:0] golden_norm;
    reg       [7:0]   golden_pixel;
    always @(*) begin
        golden_norm = golden_sum >>> NORM_COEFF;
        if (golden_norm > 255)      golden_pixel = 255;
        else if (golden_norm < 0)   golden_pixel = 0;
        else                        golden_pixel = golden_norm[7:0];
    end

    // --- Assertions ---

    // Data Accuracy
    always @(posedge i_clk) begin
        if (f_past_valid && !i_rst && o_vld) begin
            assert(o_data == golden_pixel);
        end
    end

    // Reset Check
    always @(posedge i_clk) begin
        if (f_past_valid && $past(i_rst)) begin
            assert(o_data == 0 && !o_vld);
        end
    end

    // Backpressure Stability
    always @(posedge i_clk) begin
        if (f_past_valid && !i_rst && $past(o_vld) && !$past(i_rdy)) begin
            assert(o_data == $past(o_data));
            assert(o_vld);
        end
    end

`endif
endmodule
