//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   tb_CompSogi.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
/*Description       :   CompSogi(Tustin) 单元 TB：50Hz 正弦(幅 2000)入，Fs=5kHz 概念。
                        验证（终值窗口）: oVAlpha 与输入同相（幅≈入）、oVBeta 滞后 90 度。
                        手段: AlgoTrig 产生 cos(θ) 作输入与相位参考，对尾部窗口 dot/cross 判相位、sum² 判幅值。
                        【多拍流水版】每样本一拍 iEn 触发，输出在 oValid 采样（时机改动，输入序列不变）。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.2
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
`timescale 1ns/1ps

module TbCompSogi;
    reg clk = 0, iSysRst = 1;
    reg signed [15:0] x_in = 0;
    reg iEn = 0;
    wire signed [15:0] va, vb;
    wire oValid;
    integer fail = 0;
    always #5 clk = ~clk;

    CompSogi U_Dut (.iSysClk(clk), .iSysRst(iSysRst), .iEn(iEn),
                    .iXIn(x_in), .oVAlpha(va), .oVBeta(vb), .oValid(oValid));

    // 相位参考（与输入同一角度）
    reg [15:0] ang = 0;
    wire signed [15:0] cs, sn;
    AlgoTrig U_Ref (.iAngle(ang), .oSin(sn), .oCos(cs));

    // 累加（64 位）
    reg signed [63:0] sumVA2, sumVB2, sumX2, dotA, crsA, dotB, crsB;
    integer nMeas;

    integer N = 20000;      // 4 s @ 5000（每样本一拍 iEn）
    integer SKIP = 18000;
    integer i;
    reg [31:0] ph32;
    reg signed [15:0] xs;
    reg ok1, ok2, ok3, ok4;

    initial begin
        sumVA2=0; sumVB2=0; sumX2=0; dotA=0; crsA=0; dotB=0; crsB=0; nMeas=0;
        ph32=0;
        @(negedge clk); iSysRst = 0; #2;
        for (i = 0; i < N; i = i + 1) begin
            ang = ph32[31:16];         // 当前相位
            #1;
            xs = ($signed(cs) * 2000) >>> 15;   // 输入幅 2000（cs 组合沉降一拍）
            x_in = xs;
            @(negedge clk); iEn = 1'b1;         // 采样节拍（负沿置位）
            @(posedge clk);                      // 正沿被 DUT 采样（本样本接受）
            @(negedge clk); iEn = 1'b0;          // 下一负沿撤除（单拍脉冲）
            while (!oValid) @(posedge clk);      // 等 oValid（本样本结果提交）
            #1;
            if (i >= SKIP) begin
                sumVA2 = sumVA2 + $signed(va)*$signed(va);
                sumVB2 = sumVB2 + $signed(vb)*$signed(vb);
                sumX2  = sumX2  + $signed(x_in)*$signed(x_in);
                dotA = dotA + $signed(va)*$signed(cs);
                crsA = crsA + $signed(va)*$signed(sn);
                dotB = dotB + $signed(vb)*$signed(cs);
                crsB = crsB + $signed(vb)*$signed(sn);
                nMeas = nMeas + 1;
            end
            @(posedge clk);                      // 与提交错开，保证接受间隔 > LATENCY
            ph32 = ph32 + 32'd42949673; // 0.01*2^32：每样本 50Hz/5kHz
        end

        // --- 输出诊断 ---
        $display("va vs x: dot=%0d cross=%0d (in-phase 希望 cross~0 dot>0)", dotA, crsA);
        $display("vb vs x: dot=%0d cross=%0d (lag90 希望 dot~0 cross<0)", dotB, crsB);
        $display("sumVA2=%0d sumX2=%0d sumVB2=%0d nMeas=%0d", sumVA2, sumX2, sumVB2, nMeas);

        ok1 = (dotA > 0) && ((crsA < 0 ? -crsA : crsA) < ((dotA) / 20));
        ok2 = (crsB < 0) && ((dotB < 0 ? -dotB : dotB) < ((-crsB) / 20));
        ok3 = (sumVA2*100 > sumX2*95) && (sumVA2*100 < sumX2*105);
        ok4 = (sumVB2*100 > sumX2*95) && (sumVB2*100 < sumX2*105);
        if (!ok1) begin $display("FAIL tag1 va 相位非同相"); fail = fail + 1; end
        if (!ok2) begin $display("FAIL tag2 vb 未滞后 90"); fail = fail + 1; end
        if (!ok3) begin $display("FAIL tag3 va 幅值偏差>5%"); fail = fail + 1; end
        if (!ok4) begin $display("FAIL tag4 vb 幅值偏差>5%"); fail = fail + 1; end

        if (fail == 0) $display("PASS CompSogi");
        else $display("FAIL CompSogi: %0d", fail);
        $finish;
    end
endmodule
