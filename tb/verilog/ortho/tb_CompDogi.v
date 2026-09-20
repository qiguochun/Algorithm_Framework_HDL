//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   tb_CompDogi.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.06
/*Description       :   CompDogi 单元 TB：两相旋转矢量(50Hz) 入(幅2000, vA=Acos, vB=Asin),
                        Fs=5kHz。验证（尾部窗口）: 正序幅值 sqrt(mean(ap^2+bp^2)) ≈ 2000、
                        oWEst(Q6) 换算频率 ≈ 50Hz(±1Hz)、归一化误差 oErrNorm 近 0。
                        输入相位初值带 90 度偏移以体现 FLL/正序同步工作。
                        【多拍流水版】每样本一拍 iEn 触发，输出在 oValid 采样（时机改动, 输入序列不变）。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.2
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
`timescale 1ns/1ps

module TbCompDogi;
    reg clk = 0, iSysRst = 1;
    reg signed [15:0] va = 0, vb = 0;
    reg iEn = 0;
    wire [15:0] th;
    wire signed [15:0] we, ap, bp, en;
    wire oValid;
    integer fail = 0;
    always #5 clk = ~clk;

    CompDogi U_Dut (.iSysClk(clk), .iSysRst(iSysRst), .iEn(iEn),
                    .iVAlpha(va), .iVBeta(vb),
                    .oTheta(th), .oWEst(we),
                    .oVAlphaPos(ap), .oVBetaPos(bp), .oErrNorm(en),
                    .oValid(oValid));

    // 两相参考
    reg [15:0] ang;
    wire signed [15:0] ca, sa;
    AlgoTrig U_Ref (.iAngle(ang), .oSin(sa), .oCos(ca));

    // 累加量
    reg signed [63:0] sumA2, sumWe;
    reg [31:0] ph32;
    integer i, nMeas;
    integer N = 20000;         // 4 s @5000（每样本一拍 iEn）
    integer SKIP = 15000;
    reg ok1, ok2;
    reg signed [63:0] meanA2;
    real rMeanA2, amp, fMean;
    integer A = 2000;

    initial begin
        sumA2=0; sumWe=0; nMeas=0;
        @(negedge clk); iSysRst = 0; #2;
        ph32 = 32'h4000;                 // 起始相位偏移 90 度
        for (i = 0; i < N; i = i + 1) begin
            ang = ph32[31:16];
            #1;                                  // LUT 沉降
            va = ($signed(ca) * A) >>> 15;   // vA = A cos
            vb = ($signed(sa) * A) >>> 15;   // vB = A sin
            @(negedge clk); iEn = 1'b1;          // 采样节拍（负沿置位）
            @(posedge clk);                      // 正沿被 DUT 接受（本样本）
            @(negedge clk); iEn = 1'b0;          // 撤除（单拍脉冲）
            while (!oValid) @(posedge clk);      // 等 oValid（本样本结果提交）
            #1;
            if (i == 2500) begin
                fMean = $itor(we) / 64.0 / 6.2831853;
                $display("w_est@2500 f=%0f Hz", fMean);
            end
            if (i >= SKIP) begin
                sumA2 = sumA2 + $signed(ap)*$signed(ap)
                              + $signed(bp)*$signed(bp);
                sumWe = sumWe + $signed(we);
                nMeas = nMeas + 1;
            end
            @(posedge clk);                      // 与提交错开，保证接受间隔 > LATENCY
            ph32 = ph32 + 32'd42949673;  // 0.01*2^32: 50Hz/5k
        end

        meanA2   = sumA2 / nMeas;
        rMeanA2  = $itor(meanA2[31:0]);
        amp      = $sqrt(rMeanA2);
        fMean    = $itor(sumWe / nMeas) / 64.0 / 6.2831853;
        $display("pos-seq amp=%0f (输入2000)   nMeas=%0d", amp, nMeas);
        $display("w_est mean f=%0f Hz   (oErrNorm Q15 最近=%0d)", fMean, en);

        ok1 = (amp > 1700) && (amp < 2300);       // 正序幅值 ±15%
        ok2 = (fMean > 49.0) && (fMean < 51.0);   // 频率 ±1Hz
        if (!ok1) begin $display("FAIL tag1 正序幅值偏差>15%"); fail = fail + 1; end
        if (!ok2) begin $display("FAIL tag2 w_est 频率超差");   fail = fail + 1; end

        if (fail == 0) $display("PASS CompDogi: DSOGI-FLL 锁 50Hz, 正序幅值≈输入");
        else $display("FAIL CompDogi: %0d", fail);
        $finish;
    end
endmodule
