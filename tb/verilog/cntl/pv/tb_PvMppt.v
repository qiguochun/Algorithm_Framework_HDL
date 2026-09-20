//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   tb_PvMppt.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.19
/*Description       :   PvMppt 单元 TB：首拍只采样、功率升续向、功率降反向、边界反转
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.1
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
`timescale 1ns/1ps
`include "algo_types.vh"

module TbPvMppt;
    localparam W = `ALGO_W_VOLT;

    reg iSysClk = 0;
    reg iSysRst = 1;
    reg iMpptEn = 0;
    reg [W-1:0] iVAvg = 0;
    reg [W-1:0] iPAvg = 0;
    wire [W-1:0] oVRef;

    PvMppt #(
        .W(W), .VREF_INIT(15), .VREF_MIN(8), .VREF_MAX(44), .VSTEP(1)
    ) U_Dut (
        .iSysClk(iSysClk), .iSysRst(iSysRst), .iMpptEn(iMpptEn),
        .iVAvg(iVAvg), .iPAvg(iPAvg), .oVRef(oVRef)
    );

    always #5 iSysClk = ~iSysClk;

    task automatic tick;
        input [W-1:0] p;
        begin
            iPAvg = p;
            @(negedge iSysClk); iMpptEn = 1;
            @(posedge iSysClk);
            @(negedge iSysClk); iMpptEn = 0;
            @(posedge iSysClk);
        end
    endtask

    integer bad = 0;
    reg [W-1:0] v0, v1, v2;

    initial begin
        iSysRst = 1;
        repeat (4) @(posedge iSysClk);
        iSysRst = 0;
        @(posedge iSysClk);

        // 1) 首拍只采样：VRef 保持初值 15
        tick(100);
        if (oVRef != 15) begin
            bad = 1; $display("FAIL arm: oVRef=%0d expect 15", oVRef);
        end

        // 2) 功率升且默认向上：15 -> 16
        tick(120);
        if (oVRef != 16) begin
            bad = 1; $display("FAIL up: oVRef=%0d expect 16", oVRef);
        end
        v0 = oVRef;

        // 3) 功率再升：续向 16 -> 17
        tick(140);
        if (oVRef != 17) begin
            bad = 1; $display("FAIL continue: oVRef=%0d expect 17", oVRef);
        end

        // 4) 功率降：反向 17 -> 16
        tick(100);
        if (oVRef != 16) begin
            bad = 1; $display("FAIL reverse: oVRef=%0d expect 16", oVRef);
        end

        // 5) 推到上限再扰动：触 44 后应向下（功率继续升 -> 续向直至撞顶反弹）
        while (oVRef < 44) begin
            tick(iPAvg + 10);
            if (oVRef > 44) begin bad = 1; $display("FAIL overshoot max"); end
        end
        v1 = oVRef;
        tick(iPAvg + 10);   // 仍升功率但已到顶 -> 强制向下
        if (!(oVRef < v1)) begin
            bad = 1; $display("FAIL max bounce: oVRef=%0d was %0d", oVRef, v1);
        end

        // 6) 推到下限：功率持平/微升以保持向下方向，撞 8 后应向上
        while (oVRef > 8) begin
            tick(iPAvg);    // dp==0 -> 续向（向下）
            if (oVRef < 8) begin bad = 1; $display("FAIL undershoot min"); end
        end
        v2 = oVRef;
        tick(iPAvg);        // 触底强制向上
        if (!(oVRef > v2)) begin
            bad = 1; $display("FAIL min bounce: oVRef=%0d was %0d", oVRef, v2);
        end

        if (!bad) $display("PASS PvMppt: arm/climb/reverse/bounds");
        else      $display("FAIL PvMppt");
        $finish;
    end
endmodule
