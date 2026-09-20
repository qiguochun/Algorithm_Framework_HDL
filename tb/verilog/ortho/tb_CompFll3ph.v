//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   tb_CompFll3ph.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
/*Description       :   CompFll3ph 单元 TB：50Hz 平衡三相入(幅2000), Fs=5kHz。
                        验证（尾部窗口）: 正序幅值 sqrt(mean(oVAlphaPos^2+oVBetaPos^2)) ≈ 2000、
                        oWEst(Q6) 换算频率 ≈ 50Hz(±1Hz)。
                        生成: 等幅值三相 va=Vcos, vb=Vcos(wt-120), vc=Vcos(wt+120)。
                        【多拍流水版】每样本一拍 iEn 触发，输出在 oValid 后采样
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

module TbCompFll3ph;
    reg clk = 0, iSysRst = 1, iEn = 0;
    reg signed [15:0] va = 0, vb = 0, vc = 0;
    wire [15:0] th;
    wire signed [15:0] we, vap, vbp, vd, vq;
    wire oValid;
    integer fail = 0;
    always #5 clk = ~clk;

    CompFll3ph U_Dut (.iSysClk(clk), .iSysRst(iSysRst), .iEn(iEn),
                      .iVa(va), .iVb(vb), .iVc(vc),
                      .oTheta(th), .oWEst(we),
                      .oVAlphaPos(vap), .oVBetaPos(vbp), .oValid(oValid),
                      .oVd(vd), .oVq(vq));

    // 三相参考 cos
    reg [15:0] ang, angb, angc;
    wire signed [15:0] ca, cb, cc;
    AlgoTrig U_Ra (.iAngle(ang),  .oCos(ca));
    AlgoTrig U_Rb (.iAngle(angb), .oCos(cb));
    AlgoTrig U_Rc (.iAngle(angc), .oCos(cc));

    // 累加量
    reg signed [63:0] sumA2, sumWe;
    reg [31:0] ph32;
    integer i, nMeas;
    integer N = 20000;         // 4 s @5000
    integer SKIP = 15000;
    reg ok1, ok2;
    reg signed [15:0] xa, xb, xc;
    real fHz, fMean, amp;
    integer A = 2000;
    reg signed [63:0] meanA2;
    real rMeanA2;

    initial begin
        sumA2=0; sumWe=0; nMeas=0;
        @(negedge clk); iSysRst = 0; #2;
        ph32 = 0;
        for (i = 0; i < N; i = i + 1) begin
            ang  = ph32[31:16];
            angb = ang - 16'h5555;          // -120 度
            angc = ang + 16'h5555;          // +120 度
            #1;
            xa = ($signed(ca) * A) >>> 15;
            xb = ($signed(cb) * A) >>> 15;
            xc = ($signed(cc) * A) >>> 15;
            va = xa; vb = xb; vc = xc;
            @(negedge clk); iEn = 1'b1;      // 采样节拍（负沿置位）
            @(posedge clk);                  // 正沿被 DUT 接受（本样本）
            @(negedge clk); iEn = 1'b0;      // 撤除（单拍脉冲）
            while (!oValid) @(posedge clk);  // 等 oValid（本样本结果提交）
            #1;
            if (i == 2500) begin
                fHz = $itor(we) / 64.0 / 6.2831853;
                $display("w_est@2500 f=%0f", fHz);
            end
            if (i >= SKIP) begin
                sumA2 = sumA2 + $signed(vap)*$signed(vap)
                              + $signed(vbp)*$signed(vbp);
                sumWe = sumWe + $signed(we);
                nMeas = nMeas + 1;
            end
            @(posedge clk);                  // 与提交错开，保证接受间隔 > LATENCY
            ph32 = ph32 + 32'd42949673;      // 0.01*2^32: 50Hz/5k
        end

        // 结果（$itor 对大 64 位数取低 32 位不可靠, 先在整数域求均值）
        meanA2  = sumA2 / nMeas;              // ~4e6, 落入 32 位范围
        rMeanA2 = $itor(meanA2[31:0]);
        amp     = $sqrt(rMeanA2);
        fMean   = $itor(sumWe / nMeas) / 64.0 / 6.2831853;
        $display("pos-seq amp=%0f (输入2000)   nMeas=%0d", amp, nMeas);
        $display("w_est mean f=%0f Hz", fMean);
        $display("theta_o=%0d vd=%0d vq=%0d", th, vd, vq);

        ok1 = (amp > 1700) && (amp < 2300);       // 正序幅值 ±15%
        ok2 = (fMean > 49.0) && (fMean < 51.0);   // 频率 ±1Hz
        if (!ok1) begin $display("FAIL tag1 正序幅值偏差>15%"); fail = fail + 1; end
        if (!ok2) begin $display("FAIL tag2 w_est 频率超差");   fail = fail + 1; end

        if (fail == 0) $display("PASS CompFll3ph");
        else $display("FAIL CompFll3ph: %0d", fail);
        $finish;
    end
endmodule
