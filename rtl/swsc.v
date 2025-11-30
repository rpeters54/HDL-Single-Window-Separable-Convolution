`timescale 1ns/1ps

module swsc #(
    parameter              DATA_W    =  8,
    parameter              KERNEL_H  =  7,
    parameter              MAX_IMG_W =  640,
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

    output                  o_rdy,  // tells sender col_pe is ready to receive
    output                  o_vld,  // tells receiver the current output is valid
    output     [DATA_W-1:0] o_data  // 8-bit column-wise convolved pixel
);

    ////////////////////////////////////////////////////
    // Row Processing Engine: manages row-wise conv
    ////////////////////////////////////////////////////

    wire              w_row_pe_vld;
    wire              w_row_pe_eor;
    wire              w_row_pe_eof;
    wire [DATA_W-1:0] w_row_pe_data;
    wire              w_line_buf_rdy;
    row_pe #(
        .DATA_W     (DATA_W),
        .KERNEL_H   (KERNEL_H),
        .WEIGHT_0   (WEIGHT_0),
        .WEIGHT_1   (WEIGHT_1),
        .WEIGHT_2   (WEIGHT_2),
        .WEIGHT_3   (WEIGHT_3),
        .WEIGHT_4   (WEIGHT_4),
        .WEIGHT_5   (WEIGHT_5),
        .WEIGHT_6   (WEIGHT_6)
    ) u_row_pe (
        .i_clk  (i_clk),  // input clk
        .i_rst  (i_rst),  // reset the system

        .i_rdy  (w_line_buf_rdy),  // output is ready to receive
        .i_vld  (i_vld),           // input is valid
        .i_eor  (i_eor),           // end-of-row
        .i_eof  (i_eof),           // end-of-file
        .i_data (i_data),          // input pixel

        .o_rdy  (o_rdy),         // tells sender row_pe is ready to receive
        .o_vld  (w_row_pe_vld),  // tells receiver the current pixel is valid
        .o_eor  (w_row_pe_eor),  // end-of-row
        .o_eof  (w_row_pe_eof),  // end-of-file
        .o_data (w_row_pe_data)  // 8-bit row-wise convolved pixel
    );


    ////////////////////////////////////////////////////
    // Line Buffer: stores row outputs and organizes
    // outputs for the column processor
    ////////////////////////////////////////////////////

    wire                            w_line_buf_vld;
    wire                            w_line_buf_eof;
    wire [KERNEL_H-1:0][DATA_W-1:0] w_line_buf_data;
    wire                            w_col_pe_rdy;

    line_buffer #(
        .DATA_W     (DATA_W),
        .KERNEL_H   (KERNEL_H),
        .MAX_IMG_W  (MAX_IMG_W)
    ) u_line_buffer (
        .i_clk  (i_clk),  // input clk
        .i_rst  (i_rst),  // reset the system

        .i_rdy  (w_col_pe_rdy),  // output is ready to receive
        .i_vld  (w_row_pe_vld),  // input is valid
        .i_eor  (w_row_pe_eor),  // end-of-row
        .i_eof  (w_row_pe_eof),  // end-of-file
        .i_data (w_row_pe_data), // 8-bit row-wise convolved pixel

        .o_rdy  (w_line_buf_rdy),  // tells sender line_buffer is ready to receive
        .o_vld  (w_line_buf_vld),  // tells receiver the current column vector is valid
        .o_eof  (w_line_buf_eof),  // end-of-file
        .o_data (w_line_buf_data)  // next column vector to be processed by the col_pe
    );


    ////////////////////////////////////////////////////
    // Column Processing Engine: handles col-wise conv
    ////////////////////////////////////////////////////


    col_pe #(
        .DATA_W     (DATA_W),
        .KERNEL_H   (KERNEL_H),
        .WEIGHT_0   (WEIGHT_0),
        .WEIGHT_1   (WEIGHT_1),
        .WEIGHT_2   (WEIGHT_2),
        .WEIGHT_3   (WEIGHT_3),
        .WEIGHT_4   (WEIGHT_4),
        .WEIGHT_5   (WEIGHT_5),
        .WEIGHT_6   (WEIGHT_6)
    ) u_col_pe (
        .i_clk  (i_clk),  // input clk
        .i_rst  (i_rst),  // reset the system

        .i_rdy  (i_rdy),           // output is ready to receive
        .i_vld  (w_line_buf_vld),  // input is valid
        .i_eof  (w_line_buf_eof),  // end-of-file
        .i_data (w_line_buf_data), // next column vector to be processed by the col_pe

        .o_rdy  (w_col_pe_rdy),  // tells sender col_pe is ready to receive
        .o_vld  (o_vld),         // tells receiver the current output is valid
        .o_data (o_data)         // 8-bit column-wise convolved pixel
    );

endmodule
