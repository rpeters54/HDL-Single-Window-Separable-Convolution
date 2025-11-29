`timescale 1ns/1ps

module line_buffer #(
    parameter DATA_W    = 8,
    parameter KERNEL_H  = 7,
    parameter MAX_IMG_W = 640
) (
    input                                 i_clk,
    input                                 i_rst,

    input                                 i_rdy,
    input                                 i_vld,
    input                                 i_eor,
    input                                 i_eof,
    input                    [DATA_W-1:0] i_data,

    output reg                            o_rdy,
    output reg                            o_vld,
    output reg                            o_eof,
    output reg [KERNEL_H-1:0][DATA_W-1:0] o_data
);

    ////////////////////////////////////////////////
    // skid buffer
    ////////////////////////////////////////////////

    wire              w_sb_vld;
    wire              w_sb_eor;
    wire              w_sb_eof;
    wire [DATA_W-1:0] w_sb_data;
    reg               w_core_rdy;

    skid_buffer #(
        .DATA_W(DATA_W + 2)
    ) u_input_skid (
        .i_clk   (i_clk),
        .i_rst   (i_rst),

        .i_rdy   (w_core_rdy),
        .i_vld   (i_vld),
        .i_data  ({i_eor, i_eof, i_data}),

        .o_rdy   (o_rdy),
        .o_vld   (w_sb_vld),
        .o_data  ({w_sb_eor, w_sb_eof, w_sb_data})
    );


    ////////////////////////////////////////////////
    // input demux
    ////////////////////////////////////////////////

    localparam [1:0] FILL   = 0;
    localparam [1:0] READY  = 1;
    localparam [1:0] STALL  = 2;
    localparam [1:0] FLUSH  = 3;

    parameter PTR_W = $clog2(KERNEL_H); 
    parameter CNT_W = $clog2(MAX_IMG_W); 

    reg [1:0]           w_state, w_state_next;
    reg [DATA_W-1:0]    w_data_dly, w_data_dly_next;
    reg [PTR_W-1:0]     w_row_ptr, w_row_ptr_next, w_row_ptr_prev;
    reg [CNT_W-1:0]     w_col_cnt, w_col_cnt_next;
    reg                 w_primed, w_primed_next;
    reg                 o_vld_next;
    reg                 o_eof_next;
    reg                 w_bank_update;

    always @(posedge i_clk) begin
        if (i_rst) begin
            w_state        <= FILL;
            w_row_ptr_prev <= 0;
            w_row_ptr      <= 0;
            w_col_cnt      <= 0;
            w_primed       <= 0;
            w_data_dly     <= 0;
            o_vld          <= 0;
            o_eof          <= 0;
        end else begin
            w_state        <= w_state_next;
            w_row_ptr_prev <= w_row_ptr;
            w_row_ptr      <= w_row_ptr_next;
            w_col_cnt      <= w_col_cnt_next;
            w_primed       <= w_primed_next;
            w_data_dly     <= w_data_dly_next;
            o_vld          <= o_vld_next;
            o_eof          <= o_eof_next;
        end
    end

    always @(*) begin
        w_state_next    = w_state;
        w_row_ptr_next  = w_row_ptr;
        w_col_cnt_next  = w_col_cnt;
        w_primed_next   = w_primed;
        w_data_dly_next = w_data_dly;
        o_vld_next      = o_vld;
        o_eof_next      = o_eof;
        
        w_core_rdy    = 0;
        w_bank_update   = 0;

        case (w_state)
            FILL : begin
                w_core_rdy = 1;

                if (w_sb_vld) begin
                    w_bank_update   = 1;
                    w_data_dly_next = w_sb_data;

                    // Row/Col Pointer Management
                    if (w_sb_eor) begin
                        w_col_cnt_next = 0;
                        if (w_sb_eof) begin
                            w_row_ptr_next = 0;
                            w_primed_next  = 0;
                        end else if (w_row_ptr == KERNEL_H - 1) begin
                            w_row_ptr_next = 0;
                            w_primed_next  = 1;
                        end else begin
                            w_row_ptr_next = w_row_ptr + 1;
                        end
                    end else begin
                        w_col_cnt_next = w_col_cnt + 1;
                    end

                    // State Transitions
                    if (w_sb_eof) begin
                        w_state_next = FLUSH;
                        o_vld_next   = (w_primed); 
                        o_eof_next   = 1;
                    end else if (w_primed || (w_sb_eor && w_row_ptr == KERNEL_H - 1)) begin
                        w_state_next = READY;
                        o_vld_next   = 1;
                        o_eof_next   = 0;
                    end else begin
                        w_state_next = FILL;
                        o_vld_next   = 0;
                    end
                end else begin
                    o_vld_next = 0; 
                end
            end

            READY : begin
                w_core_rdy = 1; 
                
                case ({w_sb_vld, i_rdy})
                    2'b10: begin // Stall: Valid input exists, downstream blocked
                        w_state_next = READY;
                        w_core_rdy = 0; // Hold inputs
                        o_vld_next   = 1; // Keep output valid
                    end
                    2'b01: begin // Drain: Downstream ready, Input dried up
                        w_state_next = STALL; // Move to STALL
                        o_vld_next   = 0;     // Output becomes invalid
                        w_core_rdy = 1;     // Keep listening for input
                    end
                    2'b11: begin // Flow: Both ready
                        w_bank_update   = 1;
                        w_data_dly_next = w_sb_data;
                        
                        // Pointer Updates (Same as FILL)
                        if (w_sb_eor) begin
                            w_col_cnt_next = 0;
                            if (w_sb_eof) begin
                                w_row_ptr_next = 0;
                                w_primed_next  = 0;
                            end else if (w_row_ptr == KERNEL_H - 1) begin
                                w_row_ptr_next = 0;
                                w_primed_next  = 1;
                            end else begin
                                w_row_ptr_next = w_row_ptr + 1;
                            end
                        end else begin
                            w_col_cnt_next = w_col_cnt + 1;
                        end

                        if (w_sb_eof) begin
                            w_state_next = FLUSH;
                            o_eof_next   = 1;
                        end else begin
                            w_state_next = READY;
                            o_eof_next   = 0;
                        end
                        o_vld_next = 1;
                    end
                    2'b00: begin // Idle: Neither ready
                         w_state_next = READY;
                         o_vld_next   = 1;
                         w_core_rdy = 0;
                    end
                endcase
            end

            STALL : begin
                w_core_rdy = 1; // Always ready to accept resumption
                
                if (w_sb_vld) begin
                    // RESUMPTION: We have new data. We must process it NOW.
                    w_bank_update   = 1;
                    w_data_dly_next = w_sb_data;

                    // Pointer Logic (Must duplicate here to process the incoming pixel)
                    if (w_sb_eor) begin
                        w_col_cnt_next = 0;
                        if (w_sb_eof) begin
                            w_row_ptr_next = 0;
                            w_primed_next  = 0;
                        end else if (w_row_ptr == KERNEL_H - 1) begin
                            w_row_ptr_next = 0;
                            w_primed_next  = 1;
                        end else begin
                            w_row_ptr_next = w_row_ptr + 1;
                        end
                    end else begin
                        w_col_cnt_next = w_col_cnt + 1;
                    end

                    // Exit Stall
                    if (w_sb_eof) begin
                        w_state_next = FLUSH;
                        o_eof_next   = 1;
                    end else begin
                        w_state_next = READY;
                        o_eof_next   = 0;
                    end
                    o_vld_next = 1; // Valid again
                end else begin
                    // Still waiting for data
                    w_state_next = STALL;
                    o_vld_next   = 0;
                end
            end

            FLUSH : begin
                w_core_rdy = 0;
                if (!o_vld || i_rdy) begin
                    o_vld_next     = 0;
                    o_eof_next     = 0;
                    w_state_next   = FILL;
                    w_row_ptr_next = 0;
                    w_col_cnt_next = 0;
                    w_primed_next  = 0;
                end
            end
        endcase
    end

    ////////////////////////////////////////////////
    // internal row bank
    ////////////////////////////////////////////////

    wire [DATA_W-1:0] w_bank_out [0:KERNEL_H-1];

    wire w_bank_read = (w_state == READY) || (w_state == FILL && w_sb_vld);

    genvar i;
    generate
        for (i = 0; i < KERNEL_H; i = i + 1) begin : gen_buffers
            // active only for the bank matching current row_ptr
            wire w_we = w_bank_update && (w_row_ptr == i);

            row_bank #(
                .WIDTH(DATA_W),
                .DEPTH(MAX_IMG_W)
            ) u_bram (
                .i_clk   (i_clk),
                .i_we    (w_we),
                .i_re    (w_bank_read),
                .i_addr  (w_col_cnt),
                .i_data  (w_sb_data), 
                .o_data  (w_bank_out[i])
            );
        end
    endgenerate

    ////////////////////////////////////////////////
    // output merge unit (barrel shifter)
    ////////////////////////////////////////////////

    integer k;
    always @(*) begin
        o_data[KERNEL_H-1] = w_data_dly;
        for (k = 0; k < KERNEL_H - 1; k = k + 1) begin
            o_data[k] = w_bank_out[(w_row_ptr_prev + 1 + PTR_W'(k)) % KERNEL_H];
        end
    end


`ifdef FORMAL

    ////////////////////////////////////////////////
    // formal verification
    ////////////////////////////////////////////////


    reg f_past_valid = 0;
    always @(posedge i_clk) f_past_valid <= 1;
    always @(*) if (!f_past_valid) assume(i_rst);

    wire [DATA_W-1:0] f_debug_top_pixel;
    assign f_debug_top_pixel = o_data[KERNEL_H-1];

    // reset state checks
    always @(posedge i_clk) begin
        if (f_past_valid && $past(i_rst)) begin
            assert(!o_vld);
            assert(w_row_ptr == 0);
            assert(w_col_cnt == 0);
            assert(w_state == FILL);
        end
    end

    // ////////////////////////////////////////////////
    // Latency & Validity Checks
    // ////////////////////////////////////////////////

    // 1. If we are in READY/STALL and accepted data last cycle, output must be valid now
    always @(posedge i_clk) begin
        if (f_past_valid && $past(!i_rst)) begin
             // If we were READY last cycle and had valid input, we must be valid now
             if ($past(w_state) == READY && $past(w_sb_vld)) begin
                 assert(o_vld);
                 // The top pixel is a pass-through of the data we just accepted
                 assert(f_debug_top_pixel == $past(w_sb_data));
                 assert(o_eof == $past(w_sb_eof));
             end
        end
    end

    // 2. If downstream backpressures (i_rdy is low), output must hold stable
    // Note: Assuming i_rdy = 0 means downstream is NOT ready
    always @(posedge i_clk) begin
        if (f_past_valid && $past(!i_rst)) begin
            if ($past(o_vld) && !$past(i_rdy)) begin
                assert(o_vld);
                assert(o_data == $past(o_data));
                assert(o_eof  == $past(o_eof));
            end
        end
    end

    // ////////////////////////////////////////////////
    // Pointer Logic Checks
    // ////////////////////////////////////////////////

    // Column counter behavior
    always @(posedge i_clk) begin
        if (f_past_valid && $past(!i_rst) && $past(w_bank_update)) begin
            if ($past(w_sb_eor)) begin
                assert(w_col_cnt == 0);
            end else begin
                assert(w_col_cnt == $past(w_col_cnt) + 1);
            end
        end
    end

    // Row pointer behavior
    always @(posedge i_clk) begin
        if (f_past_valid && $past(!i_rst) && $past(w_bank_update)) begin
            if ($past(w_sb_eor)) begin
                if ($past(w_sb_eof)) begin
                    assert(w_row_ptr == 0);
                end else if ($past(w_row_ptr) == KERNEL_H - 1) begin
                    assert(w_row_ptr == 0);
                end else begin
                    assert(w_row_ptr == $past(w_row_ptr) + 1);
                end
            end else begin
                // Row pointer should stay stable while moving across a row
                assert(w_row_ptr == $past(w_row_ptr));
            end
        end
    end

    // ////////////////////////////////////////////////
    // Golden Reference (Virtual Counters)
    // ////////////////////////////////////////////////

    integer f_virtual_row;
    integer f_virtual_col;
    integer f_virtual_row_mod;

    // Create virtual counters that are not resource constrained
    always @(posedge i_clk) begin
        if (i_rst) begin
            f_virtual_row <= 0;
            f_virtual_col <= 0;
        end else if (w_bank_update) begin
            if (w_sb_eor) begin
                f_virtual_col <= 0;
                if (w_sb_eof) f_virtual_row <= 0;
                else          f_virtual_row <= f_virtual_row + 1;
            end else begin
                f_virtual_col <= f_virtual_col + 1;
            end
        end
    end

    always @(*) begin
        f_virtual_row_mod = (f_virtual_row % KERNEL_H);
    end

    // Check against RTL counters
    always @(posedge i_clk) begin
        if (f_past_valid && !i_rst) begin
            // Only check when counters are stable (or updated)
            assert(w_col_cnt == f_virtual_col[$clog2(MAX_IMG_W)-1:0]);
            assert(w_row_ptr == f_virtual_row_mod[PTR_W-1:0]);
        end
    end
`endif

endmodule
