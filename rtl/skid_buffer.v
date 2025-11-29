`timescale 1ns/1ps

module skid_buffer #(
    parameter DATA_W = 8
)(
    input                   i_clk,
    input                   i_rst,
    input                   i_rdy,
    input                   i_vld,
    input      [DATA_W-1:0] i_data,

    output reg              o_rdy,
    output reg              o_vld,
    output reg [DATA_W-1:0] o_data
);


    localparam PASS  = 0;
    localparam LATCH = 1;

    reg              w_state, w_state_next;
    reg [DATA_W-1:0] w_data_skid, w_data_skid_next;


    always @(posedge i_clk) begin
        if (i_rst) begin
            w_state     <= PASS;
            w_data_skid <= 0;
        end else begin
            w_state     <= w_state_next;
            w_data_skid <= w_data_skid_next;
        end
    end

    always @(*) begin
        case (w_state)
            // if nothing has been latched, pass through data
            PASS : begin
                // reflect ready back to sender
                o_rdy  = i_vld;
                o_vld  = i_vld;
                o_data = i_data;

                if (i_vld && !i_rdy) begin
                    // if downstream is not ready, latch
                    w_data_skid_next = i_data;
                    w_state_next     = LATCH;
                end else begin
                    w_data_skid_next = w_data_skid;
                    w_state_next     = PASS;
                end
            end
            LATCH : begin
                w_data_skid_next = w_data_skid;
                o_rdy            = 0;
                o_vld            = 1;
                o_data           = w_data_skid;

                if (i_rdy) begin
                    // if downstream is ready move back to pass
                    w_state_next     = PASS;
                end else begin
                    w_state_next     = LATCH;
                end
            end
        endcase
    end

`ifdef FORMAL

    reg f_past_valid = 0;
    always @(posedge i_clk) begin
        f_past_valid <= 1;
        if ($past(i_rst)) f_past_valid <= 0; // Better reset handling
    end

    always @(*) if (!f_past_valid) assume(i_rst);


    // assume input is a legal AXI driver
    always @(posedge i_clk) if (f_past_valid && !i_rst) begin
        if ($past(i_vld) && $past(!o_rdy) && $past(!i_rst)) begin
            assume(i_vld);
            assume(i_data == $past(i_data));
        end
    end

    // -----------------------------------------------------------------
    // 2. Interface Assertions (AXI Stream / Valid-Ready Contract)
    // -----------------------------------------------------------------
    always @(posedge i_clk) if (f_past_valid && !i_rst) begin
        // If we were valid and not ready, we MUST stay valid and stable
        if ($past(o_vld) && $past(!i_rdy) && $past(!i_rst)) begin
            assert(o_vld);
            assert(o_data == $past(o_data));
        end
    end

    // -----------------------------------------------------------------
    // 3. Golden Model (Shadow FIFO)
    // -----------------------------------------------------------------
    // This tracks the expected state of the data flowing through.

    wire f_input_handshake  = i_vld && o_rdy;
    wire f_output_handshake = o_vld && i_rdy;

    // Depth 2 is sufficient: 1 slot for combinational pass, 1 for skid
    reg [DATA_W-1:0] f_shadow_mem [0:1]; 
    reg [1:0]        f_count;
    reg              f_wr_ptr; // 1 bit is enough for depth 2
    reg              f_rd_ptr;

    always @(posedge i_clk) begin
        if (i_rst) begin
            f_count  <= 0;
            f_rd_ptr <= 0;
            f_wr_ptr <= 0;
        end else begin
            // Write
            if (f_input_handshake) begin
                f_shadow_mem[f_wr_ptr] <= i_data;
                f_wr_ptr <= f_wr_ptr + 1;
            end
            // Read
            if (f_output_handshake) begin
                f_rd_ptr <= f_rd_ptr + 1;
            end
            // Count Update
            case ({f_input_handshake, f_output_handshake})
                2'b10: f_count <= f_count + 1;
                2'b01: f_count <= f_count - 1;
                default: f_count <= f_count;
            endcase
        end
    end

    // -----------------------------------------------------------------
    // 4. Data Consistency Assertions
    // -----------------------------------------------------------------
    always @(posedge i_clk) if (f_past_valid && !i_rst) begin
        // The skid buffer should never hold more than 1 items 
        assert(f_count < 2);

        // Valid output implies the shadow model is not empty or is currently
        // receiving
        assert(o_vld == (f_count > 0 || f_input_handshake));

        // If valid, the data must match the head of the shadow FIFO
        if (o_vld) begin
            if (f_count > 0) begin
                assert(o_data == f_shadow_mem[f_rd_ptr]);
            end else begin
                assert(o_data == i_data);
            end
        end
    end

    // -----------------------------------------------------------------
    // 5. Covers
    // -----------------------------------------------------------------

    always @(posedge i_clk) if (!i_rst) begin
        cover(w_state == LATCH);
        cover((w_state == LATCH) && !i_rdy);
    end

`endif
endmodule
