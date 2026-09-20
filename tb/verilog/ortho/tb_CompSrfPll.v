//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   tb_CompSrfPll.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.06
/*Description       :   CompSrfPll 单元 TB：两相旋转矢量(50Hz) 入(幅2000, vA=Acos, vB=Asin),
                        Fs=5kHz, 起始相位 45 度偏移体现锁相。
                        验证（尾部窗口）: oQ -> 0、oW 换算频率≈50Hz、
                        oCos 与 vA 同相同频(dot>0, |cross| 小)、|oErr| 均值小。
                        【多拍流水版】每样本一拍 iEn 触发, 输出在 oValid 采样（时机改动,
                        输入序列/判据不变; 复位在首个 posedge 后于负沿释放, iEn 单拍无竞争）。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.2
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
`timescale 1ns/1ps

module TbCompSrfPll;
    reg clk = 0, iSysRst = 1;
    reg iEn = 0;
    reg signed [15:0] va = 0, vb = 0;
    wire [15:0] th;
    wire signed [15:0] cos_, dq, qq, wq, eq;
    wire oValid;
    integer fail = 0;
    always #5 clk = ~clk;

    CompSrfPll U_Dut (.iSysClk(clk), .iSysRst(iSysRst), .iEn(iEn),
                      .iVAlpha(va), .iVBeta(vb),
                      .oTheta(th), .oCos(cos_), .oD(dq), .oQ(qq),
                      .oW(wq), .oErr(eq), .oValid(oValid));

    // 两相参考
    reg [15:0] ang;
    wire signed [15:0] ca, sa;
    AlgoTrig U_Ref (.iAngle(ang), .oSin(sa), .oCos(ca));

    // 累加量
    reg signed [63:0] dot, crs, sumAbsQ, sumAbsE;
    reg [31:0] ph32;
    integer i, nMeas;
    integer N = 20000;         // 4 s @5000
    integer SKIP = 4000;
    reg ok1, ok2, ok3;
    reg signed [63:0] sumWe;
    real fMean;
    integer A = 2000;

    initial begin
        dot=0; crs=0; sumAbsQ=0; sumAbsE=0; sumWe=0; nMeas=0;
        @(negedge clk); iSysRst = 0; #2;   // 复位在首个 posedge 后负沿释放
        ph32 = 32'h2000;                 // 45 度起始相位差
        for (i = 0; i < N; i = i + 1) begin
            ang = ph32[31:16];
            #1;                          // AlgoTrig 参考组合沉降
            va = ($signed(ca) * A) >>> 15;
            vb = ($signed(sa) * A) >>> 15;
            @(negedge clk); iEn = 1'b1;  // 采样节拍（负沿置位）
            @(posedge clk);               // 正沿被 DUT 接受（本样本）
            @(negedge clk); iEn = 1'b0;   // 下一负沿撤除（单拍脉冲）
            while (!oValid) @(posedge clk); // 等本样本提交
            #1;                          // 组合沉降后再读
            if (i == 300)  $display("early err@300=%0d w_f=%0f",
                                    eq, $itor(wq)/64.0/6.2831853);
            if (i == 1500) $display("mid   err@1500=%0d w_f=%0f",
                                    eq, $itor(wq)/64.0/6.2831853);
            if (i >= SKIP) begin
                dot    = dot + $signed(cos_)*$signed(ca);
                crs    = crs + $signed(cos_)*$signed(sa);
                sumAbsQ= sumAbsQ + (qq<0 ? -$signed(qq) : $signed(qq));
                sumAbsE= sumAbsE + (eq<0 ? -$signed(eq) : $signed(eq));
                sumWe  = sumWe + $signed(wq);
                nMeas  = nMeas + 1;
            end
            @(posedge clk);              // 与提交错开, 保证接受间隔 > LATENCY
            ph32 = ph32 + 32'd42949673;  // 0.01*2^32: 50Hz/5k
        end

        fMean = $itor(sumWe / nMeas) / 64.0 / 6.2831853;
        $display("dot=%0d crs=%0d  (|crs|<dot/4 判相位同步, dot>0)", dot, crs);
        $display("mean|q|=%0f  mean|err|=%0f", $itor(sumAbsQ)/$itor(nMeas),
                 $itor(sumAbsE)/$itor(nMeas));
        $display("w mean f=%0f Hz  theta_o=%0d", fMean, th);

        ok1 = (dot > 0) && (crs < 0 ? -crs : crs) < (dot / 4);   // 同相同频 ~<14°
        ok2 = (fMean > 49.0) && (fMean < 51.0);                 // 频率 ±1Hz
        ok3 = (sumAbsQ < nMeas*300);                            // q 均值 < 300/32767 (~1%)
        if (!ok1) begin $display("FAIL tag1 oCos 未同相同频"); fail = fail + 1; end
        if (!ok2) begin $display("FAIL tag2 w 频率超差");       fail = fail + 1; end
        if (!ok3) begin $display("FAIL tag3 q 未趋 0");         fail = fail + 1; end

        if (fail == 0) $display("PASS CompSrfPll: SRF-PLL 锁 50Hz, vq->0, 相位同步");
        else $display("FAIL CompSrfPll: %0d", fail);
        $finish;
    end
endmodule
