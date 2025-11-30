`timescale 1ns/1ps

module tb_custom;
/* verilator lint_off INITIALDLY */
    parameter DATA_W    = 8;
    parameter KERNEL_H  = 7;
    parameter MAX_IMG_W = 1024;

    /*
    parameter [4:0] WEIGHT_0 = 5'sd1;
    parameter [4:0] WEIGHT_1 = 5'sd3;
    parameter [4:0] WEIGHT_2 = 5'sd7;
    parameter [4:0] WEIGHT_3 = 5'sd10;
    parameter [4:0] WEIGHT_4 = 5'sd7;
    parameter [4:0] WEIGHT_5 = 5'sd3;
    parameter [4:0] WEIGHT_6 = 5'sd1;
    */
    parameter [4:0] WEIGHT_0 = -5'sd1; 
    parameter [4:0] WEIGHT_1 = -5'sd2;
    parameter [4:0] WEIGHT_2 = -5'sd3;
    parameter [4:0] WEIGHT_3 =  5'sd12;
    parameter [4:0] WEIGHT_4 = -5'sd3;
    parameter [4:0] WEIGHT_5 = -5'sd2;
    parameter [4:0] WEIGHT_6 = -5'sd1;

    reg clk = 0;
    reg rst = 1;

    reg               i_rdy_stim;
    reg               i_vld;
    reg               i_eor;
    reg               i_eof;
    reg  [DATA_W-1:0] i_data;

    wire              o_rdy;
    wire              o_vld;
    wire [DATA_W-1:0] o_data;


    swsc #(
        .DATA_W(DATA_W),
        .KERNEL_H(KERNEL_H),
        .MAX_IMG_W(MAX_IMG_W),
        .WEIGHT_0(WEIGHT_0), .WEIGHT_1(WEIGHT_1), .WEIGHT_2(WEIGHT_2),
        .WEIGHT_3(WEIGHT_3), .WEIGHT_4(WEIGHT_4), .WEIGHT_5(WEIGHT_5),
        .WEIGHT_6(WEIGHT_6)
    ) dut (
        .i_clk(clk),
        .i_rst(rst),
        .i_rdy(i_rdy_stim),
        .i_vld(i_vld),
        .i_eor(i_eor),
        .i_eof(i_eof),
        .i_data(i_data),
        .o_rdy(o_rdy),
        .o_vld(o_vld),
        .o_data(o_data)
    );


    always #5 clk = ~clk; // 100MHz

    // Runtime variables
    string  input_filename;
    integer input_fd;
    integer output_fd;
    integer img_w_var;
    integer img_h_var;
    integer exp_w_var;
    integer exp_h_var;
    integer exp_pixels;
    integer out_cnt = 0;

    // parses header from input pgm file to configure how data should be
    // passed to the dut
    task load_pgm_header(
        output integer w,
        output integer h,
        output integer max_val,
        output integer fd
    );
        string magic;
        integer code, tmp_char;
        begin

            // open file
            fd = $fopen(input_filename, "rb");
            if (fd == 0) begin
                $display("[Error] Could not open input file: %s", input_filename);
                $finish;
            end

            // verify proper version
            code = $fscanf(fd, "%s", magic);
            if (magic != "P5") begin
                $display("[Error] Only PGM P5 format is supported. Found: %s", magic);
                $finish;
            end

            // read configuration info
            code = $fscanf(fd, "%d %d %d", w, h, max_val);
            $display("[Info] Loaded PGM: %0dx%0d (Max: %0d)", w, h, max_val);

            tmp_char = $fgetc(fd);
            if (w > MAX_IMG_W) begin
                $display("[Error] Image width %0d exceeds HW Max %0d", w, MAX_IMG_W);
                $finish;
            end
        end
    endtask

    // image driver
    task send_image();
        integer r, c, pixel_val, scan_code;
        begin
            for (r = 0; r < img_h_var; r++) begin
                for (c = 0; c < img_w_var; c++) begin

                    // get the next pixel
                    if (input_fd != 0) begin
                        pixel_val = $fgetc(input_fd);
                    end else begin
                        // default to single pixel impulse if nothing is given
                        if (r == img_h_var/2 && c == img_w_var/2) pixel_val = 255;
                        else pixel_val = 0;
                    end

                    // drive input
                    i_data <= pixel_val[7:0];
                    i_vld  <= 1;

                    // EOR/EOF flags
                    i_eor <= (c == img_w_var - 1);
                    i_eof <= (r == img_h_var - 1) && (c == img_w_var - 1);

                    // wait until DUT is ready to receive
                    // it may be blocked temporarily by random output
                    // backpressure
                    do begin
                        @(posedge clk);
                    end while (o_rdy == 0);

                    // Random Stall (Simulate slow upstream)
                    if ($urandom_range(0, 100) < 5) begin
                        i_vld <= 0;
                        i_eor <= 0;
                        i_eof <= 0;
                        repeat ($urandom_range(1,3)) @(posedge clk);
                    end
                end
            end
            // deassert input once done
            i_vld <= 0; 
            i_eor <= 0; 
            i_eof <= 0;
        end
    endtask

    // main
    initial begin
        int max_val;
        if ($value$plusargs("img=%s", input_filename)) begin
            load_pgm_header(img_w_var, img_h_var, max_val, input_fd);
        end else begin
            $display("[Info] No input file provided. Defaulting to 20x20 Impulse Test.");
            input_fd = 0;
            img_w_var = 20;
            img_h_var = 20;
        end
        exp_w_var = img_w_var - (KERNEL_H-1);
        exp_h_var = img_h_var - (KERNEL_H-1);
        exp_pixels = exp_w_var * exp_h_var;

        // open output file
        output_fd = $fopen("output.pgm", "w");
        $fwrite(output_fd, "P5\n%0d %0d\n255\n", exp_w_var, exp_h_var);

        // start test
        rst = 1;
        i_vld = 0;
        i_rdy_stim = 0;

        #100;
        @(posedge clk);
        rst = 0;

        // drive dut stimulus
        send_image();

        // once done sending, wait until the output has received all pixels
        wait (out_cnt >= exp_pixels);
        repeat(50) @(posedge clk);

        assert(out_cnt == exp_pixels) else $error("[Error] DUT returned different number of pixels than expected\n");
        $display("[Success] Wrote %0d pixels to 'output_processed.pgm'", out_cnt);

        if (input_fd != 0) $fclose(input_fd);
        $fclose(output_fd);
        $finish;
    end


    // randomly block output to test backpressure
    always @(posedge clk) begin
        if (rst) i_rdy_stim <= 0;
        else     i_rdy_stim <= ($urandom_range(0, 100) > 10); // 10% busy chance
    end

    // write each received pixel to the output
    always @(posedge clk) begin
        if (o_vld && i_rdy_stim) begin
            $fwrite(output_fd, "%c", o_data);
            out_cnt <= out_cnt + 1;
            $display("[Info][%0d/%0d] Wrote %0d to 'output_processed.pgm'", out_cnt, exp_pixels, o_data);
        end
    end

    // timeout check
    initial begin
        repeat(1000000) @(posedge clk);
        $display("[Error] Watchdog Timeout");
        $finish;
    end

    // dump waveform
    initial begin
        $dumpfile("swsc.vcd");
        $dumpvars(0, tb_custom);
    end

/* verilator lint_on INITIALDLY */
endmodule
