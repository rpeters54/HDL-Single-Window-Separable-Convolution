`timescale 1ns/1ps
`include "defines.vh"

module row_pe #(
    parameter       DATA_W    = 8,
    parameter       KERNEL_H  = 7,
    parameter [4:0] WEIGHT_0  = 5'sd1,
    parameter [4:0] WEIGHT_1  = -5'sd2,
    parameter [4:0] WEIGHT_2  = 5'sd3,
    parameter [4:0] WEIGHT_3  = -5'sd4,
    parameter [4:0] WEIGHT_4  = 5'sd5,
    parameter [4:0] WEIGHT_5  = -5'sd6,
    parameter [4:0] WEIGHT_6  = 5'sd7
) (
    input            i_clk,  // input clk
    input            i_rst,  // reset the system
    input            i_rdy,  // output is ready to receive
    input            i_vld,  // input is valid
    input            i_eor,  // end of row flag (accompanies the last valid input pixel in a row)
    input      [7:0] i_data, // 8-bit pixel data

    output reg       o_rdy,  // tells sender row_pe is ready to receive
    output reg       o_vld,  // tells receiver the current output is valid
    output reg [7:0] o_data  // 8-bit convolved pixel
);

    /////////////////////////////////////////////////////////////////
    //
    // Shift Register + FSM [Stage 0 - 6]
    // - handles rdy-vld handshake for IO
    // - populates 7-stage shift register that feeds into pipelined adder
    // - total cycles from reset to output is 10
    //
    /////////////////////////////////////////////////////////////////

    localparam NORM_COEFF = $clog2(
        `ABS(WEIGHT_0) +
        `ABS(WEIGHT_1) +
        `ABS(WEIGHT_2) +
        `ABS(WEIGHT_3) +
        `ABS(WEIGHT_4) +
        `ABS(WEIGHT_5) +
        `ABS(WEIGHT_6)
    );

    localparam [1:0] FILL   = 0;
    localparam [1:0] READY  = 1;
    localparam [1:0] SKID   = 2;
    localparam [1:0] FLUSH  = 3;

    localparam [3:0] ADD_STAGES   = 3;
    localparam [3:0] TOTAL_STAGES = KERNEL_H + ADD_STAGES;

    reg               [1:0] w_state, w_next_state;
    reg [KERNEL_H-1:0][7:0] w_data_sr;
    reg               [7:0] w_skid_reg;
    reg               [3:0] w_vld_cnt;
    reg                     w_latch_input, w_clr_output, w_latch_skid, w_dummy_latch,
        w_load_skid, w_skid_full, w_rdy_next, w_early_rst;
    wire                    w_pipe_en;


    assign w_pipe_en = w_latch_input || w_dummy_latch || w_load_skid;

    // input shift register latches new values keeps a saturating counter
    // that informs the next stage when the output is ready
    always @(posedge i_clk) begin
        if (i_rst || w_early_rst) begin
            w_state     <= FILL;
            w_data_sr   <= 0;
            w_vld_cnt   <= 0;
            w_skid_full <= 0;
            o_rdy       <= 1;
        end else begin
            w_state <= w_next_state;
            o_rdy   <= w_rdy_next;
            case (1'b1)
                w_latch_input : begin
                    w_data_sr   <= { w_data_sr[KERNEL_H-2:0], i_data };
                end
                w_dummy_latch : begin
                    w_data_sr   <= { w_data_sr[KERNEL_H-2:0], 8'd0 };
                end
                w_latch_skid : begin
                    w_skid_reg  <= i_data;
                    w_skid_full <= 1;
                end
                w_load_skid : begin
                    w_data_sr   <= { w_data_sr[KERNEL_H-2:0], w_skid_reg };
                    w_skid_full <= 0;
                end
                default : ;
            endcase
            case (1'b1)
                w_latch_input : begin
                    w_vld_cnt <= (w_vld_cnt < TOTAL_STAGES) ? w_vld_cnt + 1 : w_vld_cnt;
                end
                w_clr_output : begin
                    w_vld_cnt <= (w_vld_cnt > 0)            ? w_vld_cnt - 1 : w_vld_cnt;
                end
                default : ;
            endcase
        end
    end


    // FSM tracks when the output is ready and uses backpressure to
    // halt any new inputs from arriving
    always @(*) begin
        w_latch_input = 0;
        w_latch_skid  = 0;
        w_dummy_latch = 0;
        w_load_skid   = 0;
        w_clr_output  = 0;
        w_early_rst   = 0;
        w_rdy_next    = 1;
        o_vld         = 0;
        w_next_state  = FILL;
        case (w_state)
            FILL : begin
                // proceed if the sender is valid
                w_next_state = FILL;
                if (i_vld) begin
                    w_latch_input = 1;
                    if (i_eor) begin
                        w_early_rst  = 1;
                        w_next_state = FILL;
                    end else if (w_vld_cnt >= (TOTAL_STAGES - 1)) begin
                        w_next_state = READY;
                    end
                end
            end
            READY : begin
                o_vld = 1;
                case ({i_vld, i_rdy})
                    2'b10 : begin
                        w_rdy_next    = 0;
                        w_latch_skid  = 1;
                        if (i_eor) begin
                            w_next_state = FLUSH;
                        end else begin
                            w_next_state = SKID;
                        end
                    end
                    2'b01 : begin
                        w_clr_output  = 1;
                        w_next_state  = FILL;
                    end
                    2'b11 : begin
                        w_latch_input = 1;
                        w_next_state  = READY;
                    end
                    2'b00 : begin
                        w_next_state  = READY;
                    end
                endcase
            end
            SKID : begin
                // proceed if receiver is ready
                o_vld = 1;
                if (i_rdy) begin
                    w_rdy_next   = 1;
                    w_load_skid  = 1;
                    w_next_state = READY;
                end else begin
                    w_rdy_next   = 0;
                    w_next_state = SKID;
                end
            end
            FLUSH : begin
                // flush results until the last valid result exits the pipe
                o_vld        = 1;
                w_next_state = FLUSH;
                if (i_rdy) begin
                    if (w_skid_full) begin
                        w_load_skid   = 1;
                    end else if (w_vld_cnt <= KERNEL_H) begin
                        o_vld         = 0;
                        w_early_rst   = 1;
                        w_next_state  = FILL;
                    end else begin
                        w_dummy_latch = 1;
                    end
                end
            end
        endcase
    end


    /////////////////////////////////////////////////////////////////
    //
    // Convolutional Core Tree Adder [Stages 7 - 9]
    // - use lut multipliers and carry save adders to compute dot product
    // - computation is broken across 3 register stages to reduce
    //   critical path length
    //
    /////////////////////////////////////////////////////////////////

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
        .i_rst      (i_rst || w_early_rst),
        .i_pipe_en  (w_pipe_en),
        .i_vector   (w_data_sr),
        .o_data     (o_data)
    );


    // ---------------- Setup Requirements ----------------

    reg f_past_valid = 0;
    always @(posedge i_clk) begin
        f_past_valid <= 1;
    end

    // ---------------- Assumptions ----------------

    // must begin by asserting reset
    always @(*) if (!f_past_valid) assume(i_rst);

    reg [7:0] past_inputs [0:TOTAL_STAGES-1];
    integer k;
    // Use w_pipe_en for ghost history update to match implementation
    always @(posedge i_clk) begin
        if (i_rst || w_early_rst) begin
            for(k=0; k<integer'(TOTAL_STAGES); k=k+1) past_inputs[k] <= 0;
        end else if (w_latch_input) begin
            past_inputs[0] <= i_data;
            for(k=0; k<integer'(TOTAL_STAGES)-1; k=k+1) past_inputs[k+1] <= past_inputs[k];
        end else if (w_load_skid) begin
            past_inputs[0] <= w_skid_reg;
            for(k=0; k<integer'(TOTAL_STAGES)-1; k=k+1) past_inputs[k+1] <= past_inputs[k];
        end else if (w_dummy_latch) begin
            past_inputs[0] <= 0;
            for(k=0; k<integer'(TOTAL_STAGES)-1; k=k+1) past_inputs[k+1] <= past_inputs[k];
        end
    end

    // Golden Model (Using unpacked parameters)
    reg signed [18:0] golden_sum;
    always @(*) begin
        golden_sum = 0;
        golden_sum = golden_sum + $signed({1'b0, past_inputs[3]}) * WEIGHT_0;
        golden_sum = golden_sum + $signed({1'b0, past_inputs[4]}) * WEIGHT_1;
        golden_sum = golden_sum + $signed({1'b0, past_inputs[5]}) * WEIGHT_2;
        golden_sum = golden_sum + $signed({1'b0, past_inputs[6]}) * WEIGHT_3;
        golden_sum = golden_sum + $signed({1'b0, past_inputs[7]}) * WEIGHT_4;
        golden_sum = golden_sum + $signed({1'b0, past_inputs[8]}) * WEIGHT_5;
        golden_sum = golden_sum + $signed({1'b0, past_inputs[9]}) * WEIGHT_6;
    end

    reg signed [18:0] golden_norm;
    reg        [7:0]  golden_pixel;
    always @(*) begin
        golden_norm = golden_sum >>> NORM_COEFF;

        if (golden_norm > 255)      golden_pixel = 255;
        else if (golden_norm < 0)   golden_pixel = 0;
        else                        golden_pixel = golden_norm[7:0];
    end

    // ensure we reach states that are valid
    always @(posedge i_clk) begin
        if (f_past_valid && $past(i_vld) && $past(i_rdy))
            cover(o_vld);
    end

    // output should always be 0 when reset
    always @(posedge i_clk) begin
        if (f_past_valid && ($past(i_rst) || $past(w_early_rst))) begin
            assert(o_data == 0 && !o_vld);
        end
    end

    // output holds steady if the receiver is not ready
    always @(posedge i_clk) begin
        if (f_past_valid && !$past(i_rst) && !$past(i_rdy) && $past(o_vld))
            assert(o_data == $past(o_data) && o_vld);
    end

    // compare output to golden model
    always @(posedge i_clk) begin
        if (f_past_valid) begin
            //assert(w_sumout      == golden_sum);
            //assert(w_sumout_norm == golden_norm);
            assert(o_data        == golden_pixel);
        end
    end


endmodule
