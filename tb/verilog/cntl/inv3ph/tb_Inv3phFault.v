`timescale 1ns/1ps
`include "inv3ph_cfg.vh"

module TbInv3phFault;
    reg clk = 0, rst = 1, en = 0, run = 1;
    always #5 clk = ~clk;

    reg [15:0] bus_ave = 800, bus_pa = 400, bus_na = 400;
    reg [15:0] bus_i = 800, bus_pi = 400, bus_ni = 400;
    reg signed [15:0] va = 0, vb = 0, vc = 0;
    reg signed [15:0] ia = 0, ib = 0, ic = 0;
    reg [15:0] va_rms = 230, vb_rms = 230, vc_rms = 230;
    wire bus_lo, bus_ov, vol_ov, cur_ov, short_c, is_err;
    integer fail = 0;

    Inv3phFault U_Dut (
        .iSysClk(clk), .iSysRst(rst), .iChkEn(en), .iEnRun(run),
        .iBusAve(bus_ave), .iBusPosAve(bus_pa), .iBusNegAve(bus_na),
        .iBusInst(bus_i), .iBusPosInst(bus_pi), .iBusNegInst(bus_ni),
        .iVaInst(va), .iVbInst(vb), .iVcInst(vc),
        .iIaInst(ia), .iIbInst(ib), .iIcInst(ic),
        .iVaRms(va_rms), .iVbRms(vb_rms), .iVcRms(vc_rms),
        .oBusVolLow(bus_lo), .oBusVolOver(bus_ov),
        .oInvVolOver(vol_ov), .oInvCurOver(cur_ov),
        .oInvShort(short_c), .oIsErr(is_err)
    );

    task pulse;
        begin en = 1; @(posedge clk); #1; en = 0; @(posedge clk); #1; end
    endtask

    initial begin
        repeat (3) @(posedge clk); rst = 0; @(posedge clk);
        pulse;
        if (is_err !== 1'b0) begin $display("FAIL normal"); fail = fail + 1; end
        else $display("  ok normal");

        bus_ave = 600; pulse;
        if (!bus_lo) begin $display("FAIL bus low"); fail = fail + 1; end
        else $display("  ok bus low latch");

        rst = 1; @(posedge clk); #1; rst = 0; bus_ave = 800; @(posedge clk);
        ia = 50; pulse;
        if (!cur_ov) begin $display("FAIL cur over"); fail = fail + 1; end
        else $display("  ok cur over");

        va_rms = 100; pulse;
        if (!short_c) begin $display("FAIL short"); fail = fail + 1; end
        else $display("  ok short");

        if (fail == 0) $display("PASS Inv3phFault");
        else $display("FAIL Inv3phFault: %0d", fail);
        $finish;
    end
endmodule
