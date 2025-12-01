`timescale 1ns/1ps
`include "defines.vh"

module row_pe #(
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
    input                   i_clk,  // input clk
    input                   i_rst,  // reset the system

    input                   i_rdy,  // output is ready to receive
    input                   i_vld,  // input is valid
    input                   i_eor,  // end-of-row
    input                   i_eof,  // end-of-file
    input      [DATA_W-1:0] i_data, // input pixel

    output                  o_rdy,  // tells sender row_pe is ready to receive
    output reg              o_vld,  // tells receiver the current pixel is valid
    output                  o_eor,  // end-of-row
    output                  o_eof,  // end-of-file
    output     [DATA_W-1:0] o_data  // 8-bit row-wise convolved pixel
);

    /////////////////////////////////////////////////////////////////
    //
    // Input Skid Buffer [Stage 0]
    // - pass through data if it is available and the row pe is ready
    // - latch one input of data if the row pe is not ready
    // - exists to break the combinational i_rdy -> o_rdy path that would
    //   massively increase latency
    //
    /////////////////////////////////////////////////////////////////

    wire              w_sb_vld;
    wire              w_sb_eor;
    wire              w_sb_eof;
    wire [DATA_W-1:0] w_sb_data;
    // we can accept new data from skid buffer if:
    // - downstream is ready
    // - we are not flushing
    reg               w_core_rdy;

    skid_buffer #(
        .DATA_W(DATA_W + 2)
    ) u_input_skid (
        .i_clk   (i_clk),
        .i_rst   (i_rst),

        .i_vld   (i_vld),
        .i_data  ({i_eor, i_eof, i_data}),
        .o_rdy   (o_rdy), 

        .o_vld   (w_sb_vld),
        .o_data  ({w_sb_eor, w_sb_eof, w_sb_data}),
        .i_rdy   (w_core_rdy)
    );

    /////////////////////////////////////////////////////////////////
    //
    // Input Shift Register [Stages 1 - 7]
    // - shift new pixels in when the output and input are ready
    // - propragate downstream stall information upstream
    //
    /////////////////////////////////////////////////////////////////

    localparam [1:0] FILL   = 0;
    localparam [1:0] READY  = 1;
    localparam [1:0] FLUSH  = 2;

    // processing unit latency
    localparam       ADD_STAGES   = 3;
    localparam [3:0] TOTAL_STAGES = KERNEL_H + ADD_STAGES;

    reg [1:0]                      w_state, w_state_next;
    reg [KERNEL_H-1:0][DATA_W-1:0] w_data_sr, w_data_sr_next;
    reg [ADD_STAGES:0]             w_eor_sr, w_eor_sr_next;
    reg [ADD_STAGES:0]             w_eof_sr, w_eof_sr_next;
    reg [3:0]                      w_vld_cnt, w_vld_cnt_next;
    reg                            w_vld_next;


    // track the eof and eor signal
    assign o_eor = w_eor_sr[ADD_STAGES];
    assign o_eof = w_eof_sr[ADD_STAGES];

    always @(posedge i_clk) begin
        if (i_rst) begin
            w_state   <= FILL;
            w_data_sr <= 0;
            w_eor_sr  <= 0;
            w_eof_sr  <= 0;
            w_vld_cnt <= 0;
            o_vld     <= 0;
        end else begin
            w_state   <= w_state_next;
            w_data_sr <= w_data_sr_next;
            w_eor_sr  <= w_eor_sr_next;
            w_eof_sr  <= w_eof_sr_next;
            w_vld_cnt <= w_vld_cnt_next;
            o_vld     <= w_vld_next;
        end
    end

    // FSM tracks when the output is ready and uses backpressure to
    // halt any new inputs from arriving
    always @(*) begin
        w_state_next   = w_state;
        w_data_sr_next = w_data_sr;
        w_eor_sr_next  = w_eor_sr;
        w_eof_sr_next  = w_eof_sr;
        w_vld_cnt_next = w_vld_cnt;
        w_vld_next     = o_vld;
        w_core_rdy     = 0;

        case (w_state)
            FILL : begin
                // collect new data if the skid buffer is valid
                if (w_sb_vld) begin
                    w_core_rdy     = 1;
                    w_data_sr_next = { w_data_sr[KERNEL_H-2:0],  w_sb_data            };
                    w_eor_sr_next  = { w_eor_sr[ADD_STAGES-1:0], w_sb_eor             };
                    // NOTE: ignore eofs that arrive without a correpsonding eor
                    w_eof_sr_next  = { w_eof_sr[ADD_STAGES-1:0], w_sb_eof && w_sb_eor };
                    w_vld_cnt_next = w_vld_cnt + 1;

                    // change state depending on if the sr is full or we have
                    // received an eor
                    if (w_sb_eor) begin
                        w_state_next = FLUSH;
                    end else if (w_vld_cnt >= (TOTAL_STAGES - 1)) begin
                        w_state_next = READY;
                    end else begin
                        w_state_next = FILL;
                    end

                    // if moving to ready (or flush in some rarer
                    // circumstances where sr is also full), assert vld
                    if (w_vld_cnt >= (TOTAL_STAGES - 1)) begin
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
                        w_eor_sr_next  = { w_eor_sr[ADD_STAGES-1:0], w_sb_eor  };
                        w_eof_sr_next  = { w_eof_sr[ADD_STAGES-1:0], w_sb_eof  };
                        w_data_sr_next = { w_data_sr[KERNEL_H-2:0],  w_sb_data };
                        if (w_sb_eor) begin
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
                w_vld_next   = 1;
                w_core_rdy   = 0;
                w_state_next = FLUSH;

                if (!o_vld) begin
                    w_vld_cnt_next = 0;
                    w_vld_next     = 0;
                    w_eor_sr_next  = 0;
                    w_eof_sr_next  = 0;
                    w_data_sr_next = 0;
                    w_state_next   = FILL;
                end else if (i_rdy) begin
                    // if the adder pipeline is empty, reset
                    if (w_vld_cnt <= KERNEL_H) begin
                        w_vld_cnt_next = 0;
                        w_vld_next     = 0;
                        w_eor_sr_next  = 0;
                        w_eof_sr_next  = 0;
                        w_data_sr_next = 0;
                        w_state_next   = FILL;
                    // shift the pipeline once, gating the input
                    end else begin
                        w_vld_cnt_next = w_vld_cnt - 1;
                        w_eor_sr_next  = { w_eor_sr[ADD_STAGES-1:0], 1'd0 };
                        w_eof_sr_next  = { w_eof_sr[ADD_STAGES-1:0], 1'd0 };
                        w_data_sr_next = { w_data_sr[KERNEL_H-2:0],  8'd0 };
                    end
                end
            end
            default : ;
        endcase
    end


    /////////////////////////////////////////////////////////////////
    //
    // Convolutional Core Tree Adder [Stages 8 - 10]
    // - use lut multipliers and carry save adders to compute dot product
    // - computation is broken across 3 register stages to reduce
    //   critical path length
    //
    /////////////////////////////////////////////////////////////////

    wire w_internal_shift = 
        (w_state == FILL && w_sb_vld) ||
        (w_state == READY && w_sb_vld && i_rdy) ||
        (w_state == FLUSH && i_rdy);

    da_conv_core #(
        .DATA_W     (DATA_W),
        .KERNEL_H   (KERNEL_H),
        .WEIGHT_0   (WEIGHT_0),
        .WEIGHT_1   (WEIGHT_1),
        .WEIGHT_2   (WEIGHT_2),
        .WEIGHT_3   (WEIGHT_3),
        .WEIGHT_4   (WEIGHT_4),
        .WEIGHT_5   (WEIGHT_5),
        .WEIGHT_6   (WEIGHT_6)
    ) u_core (
        .i_clk      (i_clk),
        .i_rst      (i_rst),
        .i_pipe_en  (w_internal_shift),
        .i_vector   (w_data_sr),
        .o_data     (o_data)
    );



    /////////////////////////////////////////////////////////////////
    // Formal Verification
    /////////////////////////////////////////////////////////////////

    reg f_past_valid = 0;
    always @(posedge i_clk) begin
        f_past_valid <= 1;
    end

    always @(*) if (!f_past_valid) assume(i_rst);

    // normalization coefficient used by the convolution core
    localparam integer NORM_SUM = `ABS(
        integer'(WEIGHT_0) + integer'(WEIGHT_1) + integer'(WEIGHT_2) + integer'(WEIGHT_3) + 
        integer'(WEIGHT_4) + integer'(WEIGHT_5) + integer'(WEIGHT_6)
    );
    localparam integer NORM_COEFF = (NORM_SUM > 0) ? $clog2(NORM_SUM) : 0;

    // recreate the shift enable logic for the ghost buffer
    wire f_shift_en = 
        (w_state == FILL && w_sb_vld) ||
        (w_state == READY && w_sb_vld && i_rdy) ||
        (w_state == FLUSH && i_rdy);

    // tracks what entered the computation pipeline.
    reg [DATA_W-1:0] past_inputs [0:TOTAL_STAGES]; 
    integer k;
    always @(posedge i_clk) begin
        if (i_rst) begin
            for(k=0; k<=TOTAL_STAGES; k=k+1) past_inputs[k] <= 0;
        end else if (f_shift_en) begin
            if (w_state == FLUSH) past_inputs[0] <= {DATA_W{1'd0}};
            else                  past_inputs[0] <= w_sb_data;

            for(k=0; k<TOTAL_STAGES; k=k+1) 
                past_inputs[k+1] <= past_inputs[k];
        end
    end

    // golden model of da_conv_core
    reg signed [18:0] golden_sum;
    always @(*) begin
        golden_sum = 0;
        // Core latency is ADD_STAGES (3).
        // The core takes the SR vector. SR[0] is the NEWEST data.
        // SR[0] at time T corresponds to input at T.
        // The output appears at T+3.
        // So Output at T depends on SR at T-3.
        // SR[0] at T-3 is past_inputs[3].
        golden_sum = golden_sum + $signed({1'b0, past_inputs[3]}) * $signed(WEIGHT_0);
        golden_sum = golden_sum + $signed({1'b0, past_inputs[4]}) * $signed(WEIGHT_1);
        golden_sum = golden_sum + $signed({1'b0, past_inputs[5]}) * $signed(WEIGHT_2);
        golden_sum = golden_sum + $signed({1'b0, past_inputs[6]}) * $signed(WEIGHT_3);
        golden_sum = golden_sum + $signed({1'b0, past_inputs[7]}) * $signed(WEIGHT_4);
        golden_sum = golden_sum + $signed({1'b0, past_inputs[8]}) * $signed(WEIGHT_5);
        golden_sum = golden_sum + $signed({1'b0, past_inputs[9]}) * $signed(WEIGHT_6);
    end

    reg signed [18:0]       golden_norm;
    reg        [DATA_W-1:0] golden_pixel;
    always @(*) begin
        golden_norm = golden_sum >>> NORM_COEFF;
        if (golden_norm > 255)      golden_pixel = 255;
        else if (golden_norm < 0)   golden_pixel = 0;
        else                        golden_pixel = golden_norm[7:0];
    end

    // --- Assertions ---

    // Data Accuracy
    always @(posedge i_clk) begin
        if (f_past_valid && o_vld) begin
            assert(o_data == golden_pixel);
        end
    end

    // Flag History
    localparam CTRL_LATENCY = ADD_STAGES + 1; // 4
    reg [CTRL_LATENCY-1:0] f_eor_hist;
    reg [CTRL_LATENCY-1:0] f_eof_hist;

    always @(posedge i_clk) begin
        if (i_rst) begin
            f_eor_hist <= 0;
            f_eof_hist <= 0;
        end else if (f_shift_en) begin
            // If flushing, flags shift in as 0
            f_eor_hist <= {f_eor_hist[CTRL_LATENCY-2:0], (w_state == FLUSH) ? 1'b0 : w_sb_eor};
            f_eof_hist <= {f_eof_hist[CTRL_LATENCY-2:0], (w_state == FLUSH) ? 1'b0 : w_sb_eof};
        end
    end

    // Flag Assertions
    always @(posedge i_clk) begin
        if (f_past_valid && !i_rst && o_vld) begin
            assert(o_eor == f_eor_hist[CTRL_LATENCY-1]);
            assert(o_eof == f_eof_hist[CTRL_LATENCY-1]);
        end
    end

    // eof and eor checks
    always @(posedge i_clk) if (f_past_valid && !i_rst) begin
        if (!o_vld) begin
            assert(!o_eor);
            assert(!o_eof);
        end
        if (w_sb_eor && w_sb_vld && w_core_rdy) begin
            assert(w_state_next == FLUSH);
        end
        if (o_eof) assert(o_eor);
    end

    // axi assertions
    always @(posedge i_clk) begin
        // if data is not accepted it must remain steady
        if (f_past_valid && $past(!i_rst) && $past(o_vld) && !$past(i_rdy)) begin
            assert($stable(o_data));
            assert(o_vld);
        end
    end


    always @(posedge i_clk) if (f_past_valid && $past(!i_rst)) begin
        cover(i_rdy && o_vld);
        cover(!i_rdy && o_vld);
    end


endmodule
