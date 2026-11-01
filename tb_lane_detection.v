// ============================================================================
// tb_lane_detection.v  -  ModelSim testbench (simulation only, do NOT add it
//                         to the Quartus project)
//
// Draws a "hand-drawn road" test picture inside the testbench (white paper,
// two thick dark lines meeting at a vanishing point, a dashed centre line,
// a horizon line and some noise), sends it pixel by pixel into
// lane_detection (Canny -> Hough -> local maxima) and prints the lanes.
// It also writes  tb_result.ppm  (open with GIMP / IrfanView / Photoshop):
//   left half  = test picture with the detected lanes in red
//   right half = Canny edge image
//
// ModelSim:  vlog *.v   then   vsim tb_lane_detection   then   run -all
// ============================================================================
`timescale 1ns/1ps
module tb_lane_detection;
    localparam W = 320, H = 240;

    reg clk = 1'b0;
    always #20 clk = ~clk;                 // 25 MHz

    reg        rst = 1'b1;
    reg        in_valid = 1'b0;
    reg  [8:0] in_x = 9'd0;
    reg  [7:0] in_y = 8'd0;
    reg  [7:0] in_gray = 8'd0;
    reg        frame_end = 1'b0;

    wire       record, e_valid, e_bit, lanes_valid, busy;
    wire [8:0] e_x;
    wire [7:0] e_y;
    wire       l_ok, r_ok;
    wire [6:0] l_t, r_t;
    wire [8:0] l_r, r_r;

    lane_detection dut (
        .clk(clk), .rst(rst), .th(8'd30), .peak_mode(1'b0), .freeze(1'b0),
        .in_valid(in_valid), .in_x(in_x), .in_y(in_y), .in_gray(in_gray),
        .frame_end(frame_end),
        .record(record),
        .edge_valid(e_valid), .edge_x(e_x), .edge_y(e_y), .edge_bit(e_bit),
        .lanes_valid(lanes_valid),
        .left_ok(l_ok),  .left_t(l_t),  .left_r(l_r),
        .right_ok(r_ok), .right_t(r_t), .right_r(r_r),
        .hough_busy(busy)
    );

    // ---------------------------------------------------------- test picture
    reg [7:0] img   [0:W*H-1];
    reg       edges [0:W*H-1];
    integer   x, y, k, fd, seed;
    real      d, vx, vy;

    // distance of point (px,py) from the line (x0,y0)-(x1,y1)
    function real dist;
        input real px, py, x0, y0, x1, y1;
        real dx, dy, len, c;
        begin
            dx = x1 - x0;  dy = y1 - y0;
            len = $sqrt(dx*dx + dy*dy);
            c = ((px - x0) * dy - (py - y0) * dx) / len;
            dist = (c < 0.0) ? -c : c;
        end
    endfunction

    task make_picture;
        begin
            seed = 7;
            vx = 165.0;  vy = 75.0;                           // vanishing point
            for (y = 0; y < H; y = y + 1)
                for (x = 0; x < W; x = x + 1) begin
                    k = 205 + x / 40 - y / 30 + ($random(seed) % 5);   // paper + light + noise
                    if (y >= 75) begin
                        if (dist(x, y, 20.0,  239.0, vx, vy) < 3.2) k = 40;   // left lane
                        if (dist(x, y, 305.0, 239.0, vx, vy) < 3.2) k = 40;   // right lane
                        if ((dist(x, y, 163.0, 239.0, vx, vy) < 1.5) && ((y / 12) % 2 == 0))
                            k = 70;                                         // dashed centre
                    end
                    if ((y == 72) || (y == 73)) k = 150;                     // horizon line
                    img[y*W + x] = k;
                end
        end
    endtask

    // record the edge image
    always @(posedge clk)
        if (e_valid) edges[e_y*W + e_x] <= e_bit;

    // ---------------------------------------------------------- lane drawing
    // same CORDIC as the hardware: rotate (1,0) t times -> K cos, K sin
    function on_lane;
        input integer px, py, t, rbin;
        real cx, cy, tmp, dd;
        integer i;
        begin
            cx = 1.0;  cy = 0.0;
            for (i = 0; i < t; i = i + 1) begin
                tmp = cx + cy / 32.0;
                cy  = cy - cx / 32.0;
                cx  = tmp;
            end
            dd = (px - 160) * cx + (py - 120) * (-cy) - (rbin - 256);
            on_lane = (dd > -0.8) && (dd < 0.8) && (py >= 120);
        end
    endfunction

    function integer degrees;
        input integer t;
        begin
            degrees = (t * 29326 + 8192) / 16384;
        end
    endfunction

    // ---------------------------------------------------------- stimulus
    initial begin
        make_picture;
        for (k = 0; k < W*H; k = k + 1) edges[k] = 1'b0;

        repeat (5) @(posedge clk);
        rst = 1'b0;
        wait (!busy);                                  // accumulator cleared
        $display("sending the %0d x %0d test picture ...", W, H);

        for (y = 0; y < H; y = y + 1) begin
            for (x = 0; x < W; x = x + 1) begin
                @(posedge clk);
                in_valid <= 1'b1;  in_x <= x;  in_y <= y;  in_gray <= img[y*W + x];
                @(posedge clk);
                in_valid <= 1'b0;                      // one gap clock, like the camera
            end
            repeat (40) @(posedge clk);                // line blanking
        end
        @(posedge clk) frame_end <= 1'b1;
        @(posedge clk) frame_end <= 1'b0;

        @(posedge lanes_valid);
        @(posedge clk);
        $display("edge pixels in ROI  : %0d", dut.h_count);
        $display("LEFT  lane: found=%0d  theta index=%0d (%0d deg)  rho=%0d",
                 l_ok, l_t, degrees(l_t), $signed({1'b0, l_r}) - 256);
        $display("RIGHT lane: found=%0d  theta index=%0d (%0d deg)  rho=%0d",
                 r_ok, r_t, degrees(r_t), $signed({1'b0, r_r}) - 256);

        fd = $fopen("tb_result.ppm", "w");
        $fwrite(fd, "P3\n%0d %0d\n255\n", 2*W, H);
        for (y = 0; y < H; y = y + 1)
            for (x = 0; x < 2*W; x = x + 1) begin
                if (x < W) begin
                    if ((l_ok && on_lane(x, y, l_t, l_r)) || (r_ok && on_lane(x, y, r_t, r_r)))
                        $fwrite(fd, "255 0 0\n");
                    else
                        $fwrite(fd, "%0d %0d %0d\n", img[y*W+x], img[y*W+x], img[y*W+x]);
                end else begin
                    k = edges[y*W + x - W] ? 255 : 0;
                    $fwrite(fd, "%0d %0d %0d\n", k, k, k);
                end
            end
        $fclose(fd);
        $display("written tb_result.ppm");
        $finish;
    end
endmodule
