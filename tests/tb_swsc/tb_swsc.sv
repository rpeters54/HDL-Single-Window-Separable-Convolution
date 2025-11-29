`timescale 1ns/1ps

module tb_swsc;

    // ------------------------------------------------
    // 1. Parameters & Signals
    // ------------------------------------------------
    parameter DATA_W    = 8;
    parameter KERNEL_H  = 7;
    parameter MAX_IMG_W = 64; // Small for simulation speed
    
    // Weights (Separable Gaussian-like or Edge detection)
    parameter [4:0] W0 = 5'sd1;
    parameter [4:0] W1 = -5'sd2;
    parameter [4:0] W2 = 5'sd3;
    parameter [4:0] W3 = -5'sd4;
    parameter [4:0] W4 = 5'sd5;
    parameter [4:0] W5 = -5'sd6;
    parameter [4:0] W6 = 5'sd7;

    reg clk = 0;
    reg rst = 1;

    // DUT Inputs
    reg               i_rdy_stim; // From Downstream (Sink)
    reg               i_vld;      // From Upstream (Driver)
    reg               i_eor;
    reg               i_eof;
    reg  [DATA_W-1:0] i_data;

    // DUT Outputs
    wire              o_rdy;      // To Upstream (Driver)
    wire              o_vld;      // To Downstream (Sink)
    wire [DATA_W-1:0] o_data;

    // ------------------------------------------------
    // 2. DUT Instantiation
    // ------------------------------------------------
    swsc #(
        .DATA_W(DATA_W),
        .KERNEL_H(KERNEL_H),
        .MAX_IMG_W(MAX_IMG_W),
        .WEIGHT_0(W0), .WEIGHT_1(W1), .WEIGHT_2(W2),
        .WEIGHT_3(W3), .WEIGHT_4(W4), .WEIGHT_5(W5),
        .WEIGHT_6(W6)
    ) dut (
        .i_clk(clk),
        .i_rst(rst),
        .i_rdy(i_rdy_stim), // Backpressure from testbench sink
        .i_vld(i_vld),
        .i_eor(i_eor),
        .i_eof(i_eof),
        .i_data(i_data),
        .o_rdy(o_rdy),      // Backpressure to testbench driver
        .o_vld(o_vld),
        .o_data(o_data)
    );
/* verilator lint_off INITIALDLY */
    // ------------------------------------------------
    // 3. Clock & Reset
    // ------------------------------------------------
    always #5 clk = ~clk; // 10ns period

    initial begin
        rst = 1;
        i_vld = 0;
        i_data = 0;
        i_eor = 0;
        i_eof = 0;
        i_rdy_stim = 0;
        
        #100;
        @(posedge clk);
        rst = 0;
        #20;
    end

    initial begin
        repeat (1000) @(posedge clk);
        $display("\n[ERROR] Simulation Timed Out!");
        $display("The testbench stuck waiting for a signal (likely o_rdy) or the pipeline stalled.");
        $finish;
    end

    // ------------------------------------------------
    // 4. Stimulus Driver (The "Camera")
    // ------------------------------------------------
    // Sends a 20x20 image with a single "100" value in the center.
    // This is an "Impulse Test". The output should show the kernel weights.
    
    localparam IMG_H = 20;
    localparam IMG_W = 20;

    task send_image();
        integer r, c;
        begin
            $display("[Driver] Starting Image Transmission %0dx%0d", IMG_W, IMG_H);
            
            for (r = 0; r < IMG_H; r = r + 1) begin
                for (c = 0; c < IMG_W; c = c + 1) begin
                    // Random Gaps (simulate slow upstream)
                    if ($urandom_range(0, 10) > 8) begin
                        i_vld <= 0;
                        @(posedge clk);
                        while ($urandom_range(0, 10) > 5) @(posedge clk);
                    end

                    // IMPULSE: Set pixel to 10 at (10,10), else 0
                    if (r == 10 && c == 10) i_data <= 8'd10; 
                    else                    i_data <= 8'd0;

                    // Drive Data
                    i_vld <= 1;

                    // Flags
                    if (c == IMG_W - 1) i_eor <= 1;
                    else                i_eor <= 0;

                    if (r == IMG_H - 1 && c == IMG_W - 1) i_eof <= 1;
                    else                                  i_eof <= 0;

                    // Hold until accepted
                    do begin
                        @(posedge clk);
                    end while (o_rdy == 0); // Handle DUT stall

                    // Clear for next cycle (unless immediately driving)
                    i_vld <= 0;
                    i_eor <= 0;
                    i_eof <= 0;
                end
            end
            $display("[Driver] Image Complete");
        end
    endtask

    // ------------------------------------------------
    // 5. Output Sink (The "Monitor")
    // ------------------------------------------------
    // Randomly asserts Ready to stress-test the skid buffers
    
    integer out_cnt = 0;

    initial begin
        wait(!rst);
        forever begin
            @(posedge clk);
            // Random Backpressure: 30% chance of stalling downstream
            if ($urandom_range(0, 100) < 30) begin
                i_rdy_stim <= 0;
            end else begin
                i_rdy_stim <= 1;
            end
        end
    end

    // Monitor Logic
    always @(posedge clk) begin
        if (o_vld && i_rdy_stim) begin
            out_cnt <= out_cnt + 1;
            
            // Format output to look like a 2D grid
            // Since there is latency, the output image will appear shifted
            // compared to input.
            if (o_data != 0) begin
                $display("Time: %t | Valid Output: %d (Non-Zero Detected)", $time, $signed(o_data));
            end
        end
    end

    // ------------------------------------------------
    // 6. Main Test Sequence
    // ------------------------------------------------
    initial begin
        // Wait for reset release
        wait(!rst);
        
        // Run test
        send_image();

        // Wait for pipeline to drain
        repeat(200) @(posedge clk);
        
        $display("[Test] Finished. Total Output Pixels: %0d", out_cnt);
        $finish;
    end

    // Waveform dumping
    initial begin
        $dumpfile("swsc_verify.vcd");
        $dumpvars(0, tb_swsc);
        // Drill down into hierarchy for better debugging
        $dumpvars(0, tb_swsc.dut.u_line_buffer); 
    end
    /* verilator lint_on INITIALDLY */

endmodule
