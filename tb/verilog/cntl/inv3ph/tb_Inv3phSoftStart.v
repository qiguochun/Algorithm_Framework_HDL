`timescale 1ns/1ps

module TbInv3phSoftStart;
    reg clk = 0, rst = 1, en = 0;
    always #5 clk = ~clk;
    // 目标 10V Q8 = 2560；步进 64 -> 40 拍到位
    localparam signed [31:0] TGT = 32'sd2560;
    wire signed [15:0] amp;
    wire done;
    integer fail = 0, k;

    Inv3phSoftStart #(.P_STEP(32'sd64), .P_TARGET(TGT)) U_Dut (
        .iSysClk(clk), .iSysRst(rst), .iEn(en),
        .oAmpOut(amp), .oDone(done)
    );

    initial begin
        repeat (2) @(posedge clk); rst = 0;
        for (k = 0; k < 50; k = k + 1) begin
            en = 1; @(posedge clk); #1; en = 0; @(posedge clk); #1;
        end
        if (!done || amp !== TGT[15:0]) begin
            $display("FAIL soft amp=%0d done=%0b", amp, done);
            fail = fail + 1;
        end else $display("  ok ramp to %0d", amp);

        if (fail == 0) $display("PASS Inv3phSoftStart");
        else $display("FAIL Inv3phSoftStart: %0d", fail);
        $finish;
    end
endmodule
