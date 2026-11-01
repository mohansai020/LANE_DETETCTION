// ============================================================================
// tb_ov7670_system.v  -  ModelSim testbench of the WHOLE design
//                        (simulation only, do NOT add it to the Quartus project)
//
//   ov7670_model (below)  ->  lane_detection_top  ->  one VGA frame
//
// The OV7670 model
//   * answers the SCCB register writes (pulls the 9th bit low) and keeps
//     the written values, so the camera set-up can be checked
//   * draws a colour "hand-drawn road" (paper, two ink lane lines, a dashed
//     centre line, a horizon line, noise) at 640 x 480
//   * sends it like the real camera: VSYNC pulse, HREF per row, 2 bytes
//     (RGB565) per pixel, data changing on the falling edge of PCLK,
//     PCLK = XCLK from the FPGA delayed by 7 ns
//
// The testbench waits for the first lane result, then records the next VGA
// frame (640 x 480) into tb_vga_frame.ppm  (open with GIMP / IrfanView).
// Display mode = SW1..0 (+SW=<decimal> on the vsim line), default 0
// (camera picture + lanes).
//
// ModelSim:  vlog *.v
//            vsim -L altera_mf_ver tb_ov7670_system
//            run -all                      (takes a few minutes)
// ============================================================================
`timescale 1ns/1ps
module tb_ov7670_system;
    reg clk50 = 1'b0;
    always #10 clk50 = ~clk50;

    reg  [1:0]  key = 2'b10;          // KEY0 pressed = reset
    reg  [17:0] sw  = 18'd0;
    wire [17:0] ledr;
    wire [8:0]  ledg;
    wire [6:0]  h0, h1, h2, h3, h4, h5, h6, h7;
    wire        vga_clk, vga_hs, vga_vs, vga_blank_n, vga_sync_n;
    wire [7:0]  vga_r, vga_g, vga_b;

    wire        pclk, href, vsync, xclk, cam_rst_n, pwdn, sioc;
    wire [7:0]  d;
    wire        siod;
    pullup (siod);                    // the pull-up resistor on SIOD

    ov7670_model cam (
        .xclk(xclk), .reset_n(cam_rst_n), .pwdn(pwdn),
        .pclk(pclk), .href(href), .vsync(vsync), .d(d),
        .sioc(sioc), .siod(siod)
    );

    lane_detection_top dut (
        .CLOCK_50(clk50), .KEY(key), .SW(sw), .LEDR(ledr), .LEDG(ledg),
        .HEX0(h0), .HEX1(h1), .HEX2(h2), .HEX3(h3), .HEX4(h4), .HEX5(h5), .HEX6(h6), .HEX7(h7),
        .VGA_CLK(vga_clk), .VGA_HS(vga_hs), .VGA_VS(vga_vs), .VGA_BLANK_N(vga_blank_n),
        .VGA_SYNC_N(vga_sync_n), .VGA_R(vga_r), .VGA_G(vga_g), .VGA_B(vga_b),
        .OV_PCLK(pclk), .OV_D(d), .OV_HREF(href), .OV_VSYNC(vsync),
        .OV_XCLK(xclk), .OV_RESET_N(cam_rst_n), .OV_PWDN(pwdn),
        .OV_SIOC(sioc), .OV_SIOD(siod)
    );

    // the camera set-up uses ms delays and a 100 kHz bus; in simulation it is
    // made 10x faster so the run is shorter (the hardware uses the real values)
    defparam dut.u_config.CLK_HZ = 5_000_000;

    function integer degrees;            // theta index -> degrees (1.79 deg steps)
        input integer t;
        degrees = (t * 29326 + 8192) / 16384;
    endfunction

    integer fd, n, v;

    initial begin
        $timeformat(-6, 0, " us", 12);           // %t prints micro-seconds
        if ($value$plusargs("SW=%d", v)) sw = v;
        // edge-memory border pixels are never written; on the FPGA the M9K RAM
        // powers up as 0, in simulation it starts as X -> clear it
        for (n = 0; n < 76800; n = n + 1) dut.u_edge_mem.mem[n] = 1'b0;
        #2000 key = 2'b11;
        $display("[tb] SW=%b", sw);

        @(posedge dut.cam_ok);
        $display("[tb] %t: camera configured: %0d SCCB writes, %0d not acknowledged",
                 $time, cam.n_writes, cam.n_bad);
        $display("[tb]   COM7=%h (04 = VGA RGB)  COM15=%h (D0 = RGB565 0..255)  COM8=%h (E7 = auto exp/gain/WB)",
                 cam.regs[8'h12], cam.regs[8'h40], cam.regs[8'h13]);

        @(posedge dut.lanes_valid);
        @(posedge dut.clk25);
        $display("[tb] %t: edge pixels in ROI = %0d", $time, dut.u_lane.h_count);
        $display("[tb] LEFT  lane: found=%0d  theta index=%0d (%0d deg)  rho=%0d",
                 dut.l_ok, dut.l_t, degrees(dut.l_t), $signed({1'b0, dut.l_r}) - 256);
        $display("[tb] RIGHT lane: found=%0d  theta index=%0d (%0d deg)  rho=%0d",
                 dut.r_ok, dut.r_t, degrees(dut.r_t), $signed({1'b0, dut.r_r}) - 256);

        // the result is taken by the VGA at the next vertical blank; then
        // record one full frame
        @(posedge dut.u_vga.new_frame);
        @(negedge vga_vs);
        @(posedge vga_vs);
        fd = $fopen("tb_vga_frame.ppm", "w");
        $fwrite(fd, "P3\n640 480\n255\n");
        n = 0;
        while (n < 640*480) begin
            @(posedge vga_clk);
            if (vga_blank_n) begin
                $fwrite(fd, "%0d %0d %0d\n", vga_r, vga_g, vga_b);
                n = n + 1;
            end
        end
        $fclose(fd);
        $display("[tb] written tb_vga_frame.ppm   LEDG=%b   FIFO overflow=%0d", ledg, dut.overflow);
        $finish;
    end

    always @(posedge vsync) $display("[tb] %t: camera frame starts", $time);
endmodule


// ============================================================================
// ov7670_model  -  behavioural model of the OV7670 (VGA, RGB565)
// ============================================================================
module ov7670_model (
    input            xclk,
    input            reset_n,
    input            pwdn,
    output           pclk,
    output reg       href,
    output reg       vsync,
    output reg [7:0] d,
    input            sioc,
    inout            siod
);
    localparam W = 640, H = 480;
    localparam LINE = 1568;              // PCLK per row  (784 pixels x 2 bytes)
    localparam ROWS = 510;               // rows per frame (3 VSYNC + 17 + 480 + 10)

    assign #7 pclk = xclk;               // PCLK follows XCLK (CLKRC / PLL setting)

    // ---------------------------------------------------------- SCCB slave
    reg  [7:0] regs [0:255];
    reg        ack_drive = 1'b0;
    reg        active = 1'b0;
    reg  [7:0] sh;
    reg  [7:0] b0, b1;
    integer    bitcnt = 0, byten = 0;
    integer    n_writes = 0, n_bad = 0;
    assign siod = ack_drive ? 1'b0 : 1'bz;

    always @(negedge siod) if (sioc === 1'b1) begin          // START
        active = 1'b1;  bitcnt = 0;  byten = 0;
    end
    always @(posedge siod) if (sioc === 1'b1 && active) begin // STOP
        active = 1'b0;
        if (byten == 3) n_writes = n_writes + 1;
    end
    always @(posedge sioc) if (active && bitcnt < 8) begin
        sh = {sh[6:0], (siod === 1'b0) ? 1'b0 : 1'b1};
        bitcnt = bitcnt + 1;
    end else if (active) bitcnt = bitcnt + 1;                  // 9th bit
    always @(negedge sioc) if (active) begin
        if (bitcnt == 8) begin                                 // byte received
            if (byten == 0) begin b0 = sh; if (sh != 8'h42) n_bad = n_bad + 1; end
            if (byten == 1) b1 = sh;
            if (byten == 2) regs[b1] = sh;
            ack_drive <= #50 (byten == 0) ? (sh == 8'h42) : 1'b1;   // answer
        end else if (bitcnt == 9) begin
            ack_drive <= #50 1'b0;
            bitcnt = 0;
            byten  = byten + 1;
        end
    end

    // ---------------------------------------------------------- test picture
    reg [15:0] img [0:W*H-1];
    integer x, y, seed, r, g, b, k;

    function real dist;                  // distance of (px,py) from a line
        input real px, py, x0, y0, x1, y1;
        real dx, dy, len, c;
        begin
            dx = x1 - x0;  dy = y1 - y0;
            len = $sqrt(dx*dx + dy*dy);
            c = ((px - x0) * dy - (py - y0) * dx) / len;
            dist = (c < 0.0) ? -c : c;
        end
    endfunction

    initial begin
        seed = 11;
        for (y = 0; y < H; y = y + 1)
            for (x = 0; x < W; x = x + 1) begin
                k = x / 80 - y / 60 + ($random(seed) % 9);          // light + noise
                r = 225 + k;  g = 220 + k;  b = 200 + k;             // paper
                if (y >= 150) begin
                    if (dist(x, y, 40.0,  479.0, 330.0, 150.0) < 6.5) begin r = 35; g = 40; b = 90; end  // left lane (blue ink)
                    if (dist(x, y, 610.0, 479.0, 330.0, 150.0) < 6.5) begin r = 30; g = 30; b = 30; end  // right lane (black)
                    if ((dist(x, y, 326.0, 479.0, 330.0, 150.0) < 3.0) && ((y / 24) % 2 == 0)) begin
                        r = 90; g = 90; b = 90;                                                         // dashed centre
                    end
                end
                if ((y >= 144) && (y <= 147)) begin r = 140; g = 140; b = 130; end                     // horizon
                if (r > 255) r = 255;  if (g > 255) g = 255;  if (b > 255) b = 255;
                img[y*W + x] = {r[7:3], g[7:2], b[7:3]};
            end
    end

    // ---------------------------------------------------------- video timing
    integer col = 0, row = 0;
    reg [15:0] pix;
    initial begin href = 1'b0; vsync = 1'b0; d = 8'd0; end

    always @(negedge pclk) begin                                 // camera changes data on falling edge
        if (!reset_n || pwdn) begin
            col <= 0;  row <= 0;
            href <= #2 1'b0;  vsync <= #2 1'b0;  d <= #2 8'd0;
        end else begin
            col <= (col == LINE - 1) ? 0 : col + 1;
            if (col == LINE - 1) row <= (row == ROWS - 1) ? 0 : row + 1;

            vsync <= #2 (row < 3);
            if ((row >= 20) && (row < 20 + H) && (col < 2 * W)) begin
                pix = img[(row - 20) * W + col / 2];
                href <= #2 1'b1;
                d    <= #2 col[0] ? pix[7:0] : pix[15:8];          // high byte first
            end else begin
                href <= #2 1'b0;
                d    <= #2 8'd0;
            end
        end
    end
endmodule
