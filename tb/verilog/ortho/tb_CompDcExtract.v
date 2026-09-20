//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   tb_CompDcExtract.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
/*Description       :   直流提取（SOGI/Notch 双模式）单元 TB。
                        输入: DC=15000 + 100Hz 纹波(幅1500)，概念 Fs=5kHz。
                        mode0 SOGI oDcOut≈15000(反馈对消)；mode1 Notch oDcOut≈15000(带残余, 容差放宽)。
                        oAcAmp: SOGI 含 vβ DC 偏置不断言；Notch 为瞬时残余 >0。
                        【多拍流水版】每样本一拍 iEn 单拍脉冲，输出在 oValid 后采样
                        （两路 MODE0/1 同步提交；读取时机改动，输入序列不变）。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.2
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
`timescale 1ns/1ps

module TbCompDcExtract;
    reg clk = 0, iSysRst = 1;
    reg iEn = 0;
    reg signed [15:0] x_in = 0;
    wire signed [15:0] dc0, ac0, dc1, ac1;
    wire oV0, oV1;
    integer fail = 0;
    always #5 clk = ~clk;

    CompDcExtract #(.MODE(0)) U_Sogi (.iSysClk(clk), .iSysRst(iSysRst), .iEn(iEn), .iXIn(x_in),
                                      .oDcOut(dc0), .oAcAmp(ac0), .oValid(oV0));
    CompDcExtract #(.MODE(1)) U_Nch  (.iSysClk(clk), .iSysRst(iSysRst), .iEn(iEn), .iXIn(x_in),
                                      .oDcOut(dc1), .oAcAmp(ac1), .oValid(oV1));

    reg [15:0] ang = 0;
    wire signed [15:0] cs;
    wire signed [15:0] ds;   // sin 未用
    AlgoTrig U_Ref (.iAngle(ang), .oSin(ds), .oCos(cs));

    reg signed [63:0] s0, s1;
    integer md0, md1;
    integer i;
    reg [31:0] ph32;
    reg signed [15:0] xg;
    integer tmp0, tmp1;
    integer N = 8000;
    integer SKIP = 5000;

    initial begin
        s0=0; s1=0; md0=0; md1=0; ph32=0; x_in=0; iEn=0;
        @(negedge clk); iSysRst = 0; #2;
        for (i = 0; i < N; i = i + 1) begin
            ang = ph32[31:16]; #1;
            xg = (($signed(cs) * 1500) >>> 15) + 15000;   // DC15000+纹波1500
            x_in = xg;
            @(negedge clk); iEn = 1'b1;         // 采样节拍（负沿置位）
            @(posedge clk);                      // 正沿被 DUT 接受（本样本）
            @(negedge clk); iEn = 1'b0;          // 单拍脉冲撤除
            while (!(oV0 && oV1)) @(posedge clk); // 等两路提交（本样本结果）
            #1;
            if (i >= SKIP) begin
                s0 = s0 + dc0;  s1 = s1 + dc1;
                tmp0 = dc0 - 15000; if (tmp0 < 0) tmp0 = -tmp0; if (tmp0 > md0) md0 = tmp0;
                tmp1 = dc1 - 15000; if (tmp1 < 0) tmp1 = -tmp1; if (tmp1 > md1) md1 = tmp1;
            end
            @(posedge clk);                      // 与提交错开，保证接受间隔 > LATENCY
            ph32 = ph32 + 32'd85899346;   // f/Fs = 0.02 (100Hz/5kHz)
        end

        s0 = s0 / (N - SKIP); s1 = s1 / (N - SKIP);
        $display("SOGI : dc均值=%0d 最大偏差=%0d  (ac均值略见终端) ac首末=%0d", s0, md0, ac0);
        $display("Notch: dc均值=%0d 最大偏差=%0d", s1, md1);

        // mode0 SOGI: dc≈15000 高精度
        if (s0 > 14900 && s0 < 15100 && md0 < 400) $display("  ok SOGI dc≈直流");
        else begin $display("FAIL SOGI dc 偏差 mean=%0d maxdev=%0d", s0, md0); fail = fail + 1; end
        // mode1 Notch: dc≈15000（含残余纹波）
        if (s1 > 14500 && s1 < 15500 && md1 < 1200) $display("  ok Notch dc≈直流");
        else begin $display("FAIL Notch dc 偏差 mean=%0d maxdev=%0d", s1, md1); fail = fail + 1; end
        // ac_amp 基本有效：两模式均非负
        if (ac0 >= 0 && ac1 >= 0) $display("  ok ac_amp>=0");
        else begin $display("FAIL ac_amp"); fail = fail + 1; end

        if (fail == 0) $display("PASS CompDcExtract");
        else $display("FAIL CompDcExtract: %0d", fail);
        $finish;
    end
endmodule
