//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   tb_CompDiff.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
/*Description       :   CompDiff 单元 TB（自包含）
                        输入 = 斜坡 x = M*n (每样本 +M)。两级滤波器稳态后微分输出应 ≈ M*FS。
                        场景: M=60/样本, FS=100 (Ts=10ms), 期望 y2 ≈ 6000 (信号/秒)。
                        【流水化适配】DUT 现为多拍流水（LATENCY=8）：
                        iEn 改为单拍脉冲逐样本喂入，oYOut 在 oValid 拉高时采样；
                        喂入样本序列与断言数值与改造前完全一致。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.1
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
`timescale 1ns/1ps

module TbCompDiff;
    localparam integer M      = 60;      // 斜坡每样本增量
    localparam integer FSV    = 100;     // 采样率(Hz)，Ts=1/FSV
    localparam integer WARM   = 150;     // 预热样本
    localparam integer MEAS   = 200;     // 测量窗样本
    localparam integer EXPECT = M * FSV; // 期望微分输出 6000

    reg clk = 0;
    reg iSysRst = 1;
    reg en = 0;
    integer s = 0;
    reg signed [15:0] x;
    wire signed [15:0] y;
    wire vld;

    CompDiff U_Dut (
        .iSysClk(clk), .iSysRst(iSysRst), .iEn(en), .iXIn(x), .oYOut(y), .oValid(vld)
    );

    always #5 clk = ~clk;

    integer i, acc, hi, lo;
    real avg;
    integer fail = 0;

    initial begin
        iSysRst = 1; en = 0; x = 0;
        repeat (3) @(posedge clk); #1;
        iSysRst = 0; #1;

        // 预热 WARM 样本 + 测量窗，逐样本 iEn 脉冲、oValid 采样（每样本 x=M*s）
        acc = 0; hi = 0; lo = 0;
        for (s = 0; s < WARM + MEAS; s = s + 1) begin
            x = M * s;
            en = 1; @(posedge clk); #1; en = 0;
            while (!vld) begin @(posedge clk); #1; end
            if (s >= WARM) begin
                acc = acc + y;
                if (y > hi) hi = y;
                if (y < lo) lo = y;
            end
        end
        avg = acc / MEAS;
        $display("DIFF avg=%0.1f (expect %0d)  range=[%0d,%0d]", avg, EXPECT, lo, hi);
        if (avg > EXPECT * 0.96 && avg < EXPECT * 1.04)
            $display("PASS diff: ramp slope ~ M*FS within 4%");
        else begin
            $display("FAIL diff: avg out of range");
            fail = 1;
        end
        if (fail) $finish;
        $finish;
    end
endmodule
