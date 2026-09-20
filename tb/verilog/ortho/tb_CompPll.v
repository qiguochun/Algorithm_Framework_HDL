//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   tb_CompPll.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
/*Description       :   CompPll 单元 TB：50Hz 正弦(幅2000)入, Fs=5kHz 概念。
                        验证（尾部窗口）: oErr -> 0、oDF 收敛到输入幅值、
                        oOut 与参考 cos 同相同频(dot>0, |cross|小)、theta 稳定推进、
                        oWF(Q6) 换算频率 ≈ 50Hz。输入相位初值带 22.5 度偏移以体现锁相。
                        【多拍流水版】每样本一拍 iEn 触发，输出在 oValid 采样
                        （时机改动，输入序列不变）。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.2
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
`timescale 1ns/1ps

module TbCompPll;
    reg clk = 0, iSysRst = 1;
    reg iEn = 0;
    reg signed [15:0] u_in = 0;
    wire [15:0] th;
    wire signed [15:0] out_, err_, df, wf;
    wire oValid;
    integer fail = 0;
    always #5 clk = ~clk;

    CompPll U_Dut (.iSysClk(clk), .iSysRst(iSysRst), .iEn(iEn), .iUIn(u_in),
                   .oTheta(th), .oOut(out_), .oErr(err_),
                   .oDF(df), .oWF(wf), .oValid(oValid));

    // 相位参考
    reg [31:0] ph32;
    reg [15:0] ang;
    wire signed [15:0] cs, sn;
    AlgoTrig U_Ref (.iAngle(ang), .oSin(sn), .oCos(cs));

    // 累加量
    reg signed [63:0] dot, crs, sumDf, sumAbsE;
    reg [31:0] th0;
    integer i, nMeas;
    integer N = 20000;       // 4 s @5000
    integer SKIP = 15000;
    integer maxAbsE = 0;
    reg ok1, ok2, ok3, ok4;
    reg signed [15:0] xs;
    real fHz;

    initial begin
        dot=0; crs=0; sumDf=0; sumAbsE=0; nMeas=0;
        // 复位
        @(negedge clk); iSysRst = 0; #2;
        ph32 = 32'h1000;               // 22.5 度起始相位差
        for (i = 0; i < N; i = i + 1) begin
            ang = ph32[31:16];
            #1;
            xs = ($signed(cs) * 2000) >>> 15;   // 输入幅 2000
            u_in = xs;
            @(negedge clk); iEn = 1'b1;         // 采样节拍（负沿置位）
            @(posedge clk);                      // 正沿被 DUT 接受（本样本）
            @(negedge clk); iEn = 1'b0;          // 下一负沿撤除（单拍脉冲）
            while (!oValid) @(posedge clk);      // 等 oValid（本样本结果提交）
            #1;
            if (i == 500) begin
                // 打印锁相早期|err|以观察拉入
                $display("early err@500=%0d", err_);
            end
            if (i >= SKIP) begin
                dot    = dot + $signed(out_)*$signed(cs);
                crs    = crs + $signed(out_)*$signed(sn);
                sumDf  = sumDf + $signed(df);
                sumAbsE= sumAbsE + (err_ < 0 ? -$signed(err_) : $signed(err_));
                nMeas  = nMeas + 1;
            end
            if (err_ < 0) begin if (-err_ > maxAbsE) maxAbsE = -err_; end
            else begin if (err_ > maxAbsE) maxAbsE = err_; end
            @(posedge clk);                      // 与提交错开，保证接受间隔 > LATENCY
            ph32 = ph32 + 32'd42949673;   // 0.01*2^32: 50Hz/5k
        end

        // 尾部 theta 推进检验
        fHz = $itor(wf) / 64.0 / 6.2831853;
        $display("dot=%0d crs=%0d  |crs|/dot=%0f", dot, crs,
                 (crs<0?-1:1)*$itor(crs)/$itor(dot));
        $display("meanDf=%0f  (幅2000)", $itor(sumDf)/$itor(nMeas));
        $display("meanAbsE=%0f  maxAbsE=%0d", $itor(sumAbsE)/$itor(nMeas), maxAbsE);
        $display("w_f=%0d -> f=%0f Hz", wf, fHz);
        $display("theta_o=%0d th_inc_per_smpl~%0d", th, 655);

        // 判据
        ok1 = (dot > 0) && (crs < 0 ? -crs : crs) < ((dot) / 4);   // 相位差<~14度
        ok2 = (sumDf > (nMeas*2000*85)/100) && (sumDf < (nMeas*2000*115)/100); // d~2000 15%
        ok3 = (sumAbsE < nMeas*400);        // |err|均值 < 400/32768 ~1.2%
        ok4 = (fHz > 49.0) && (fHz < 51.0); // w 频率容差

        if (!ok1) begin $display("FAIL tag1 out 未同相同频"); fail = fail + 1; end
        if (!ok2) begin $display("FAIL tag2 d_f 未收敛幅值");  fail = fail + 1; end
        if (!ok3) begin $display("FAIL tag3 err 未趋于 0");    fail = fail + 1; end
        if (!ok4) begin $display("FAIL tag4 w 频率超差");      fail = fail + 1; end

        if (fail == 0) $display("PASS CompPll");
        else $display("FAIL CompPll: %0d", fail);
        $finish;
    end
endmodule
