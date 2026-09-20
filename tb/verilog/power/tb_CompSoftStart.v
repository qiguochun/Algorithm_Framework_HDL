//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   tb_CompSoftStart.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.06
/*Description       :   CompSoftStart 单元 TB: 校验斜坡按 step 每拍递增、到 target 吸附并保持,
//                       全程不过冲; 另例化反向(step<0)目标实例校验下降。
//                       控制信号在 negedge 释放(与 DUT posedge 采样解耦避免竞争),
//                       采样也在 negedge(值稳定)。上升 step=1000 target=10000:
//                       0,1000,...,10000(第 10 拍), 之后恒为 10000。
//                       下降 step=-500 target=-8000: -500,...,-8000(第 16 拍), 后恒 -8000。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.1
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
`timescale 1ns/1ps

module TbCompSoftStart;
    reg  clk = 0, rst = 1, run = 0;
    wire signed [15:0] oUp;
    reg  clkN = 0, rstN = 1, runN = 0;
    wire signed [15:0] oDown;

    CompSoftStart #(.P_STEP(32'sd1000), .P_TARGET(32'sd10000)) U_Up (
        .iSysClk(clk), .iSysRst(rst), .iRun(run), .oOut(oUp));
    CompSoftStart #(.P_STEP(-32'sd500), .P_TARGET(-32'sd8000)) U_Down (
        .iSysClk(clkN), .iSysRst(rstN), .iRun(runN), .oOut(oDown));

    always #5 clk  = ~clk;
    always #5 clkN = ~clkN;

    integer j, fail = 0, got;

    initial begin
        repeat (4) @(negedge clk);          // 复位保持若干拍
        rst = 0; run = 1;                   // negedge 释放使能
        @(negedge clk);                     // 输出饱和级 +1 拍流水: 多等一拍对齐采样
        for (j = 0; j < 30; j = j + 1) begin
            @(negedge clk);                 // 等待 j+1 次 posedge 斜坡后采样
            got = oUp;
            if (got > 10000) begin $display("FAIL 过冲 %0d", got); fail = fail + 1; end
            if (j < 10) begin
                if (got != (1000*(j+1))) begin
                    $display("FAIL 上升拍 j=%0d got=%0d exp=%0d", j, got, 1000*(j+1));
                    fail = fail + 1;
                end
            end else begin
                if (got != 10000) begin $display("FAIL 保持 j=%0d got=%0d", j, got); fail = fail + 1; end
            end
        end
        $display("上升: 末值=%0d (target 10000)", oUp);
        if (fail == 0) $display("  ok 正斜坡按步进到 target 并保持");

        repeat (4) @(negedge clkN);
        rstN = 0; runN = 1;                 // 下降实例在自身时钟 negedge 使能
        @(negedge clkN);                    // 输出饱和级 +1 拍流水: 多等一拍对齐采样
        for (j = 0; j < 30; j = j + 1) begin
            @(negedge clkN);
            got = oDown;
            if (got < -8000) begin $display("FAIL 下降过冲 %0d", got); fail = fail + 1; end
            if (j < 16) begin
                if (got != (-500*(j+1))) begin
                    $display("FAIL 下降拍 j=%0d got=%0d exp=%0d", j, got, -500*(j+1));
                    fail = fail + 1;
                end
            end else begin
                if (got != -8000) begin $display("FAIL 下降保持 j=%0d got=%0d", j, got); fail = fail + 1; end
            end
        end
        $display("下降: 末值=%0d (target -8000)", oDown);
        if (fail == 0)
            $display("PASS CompSoftStart: 斜坡上升/下降按 step 递增, 到 target 停");
        else
            $display("FAIL CompSoftStart: %0d", fail);
        $finish;
    end
endmodule
