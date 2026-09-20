//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   tb_CompNotch.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
/*Description       :   CompNotch 陷波器单元 TB（默认 fc=100Hz, Fs=5kHz, c1=0.5, c2=0）。
                        校验: DC 直通(增益≈1)；100Hz 中心正弦显著衰减(幅比<0.30)；
                        250Hz 非中心保留(幅比 0.90~1.10)。手段: 尾窗 sum² 判幅比、DC 均值判直通。
                        【流水化适配】DUT 现为多拍流水（LATENCY=6，新增 iEn/oValid）：
                        iEn 改为单拍脉冲逐样本喂入，oYOut(oY) 在 oValid 拉高时采样；
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

module TbCompNotch;
    reg clk = 0, iSysRst = 1;
    reg en = 0;
    reg signed [15:0] x_in = 0;
    wire signed [15:0] y_o;
    wire vld;
    integer fail = 0;
    always #5 clk = ~clk;

    CompNotch U_Dut (.iSysClk(clk), .iSysRst(iSysRst), .iEn(en),
                     .iXIn(x_in), .oY(y_o), .oValid(vld));

    reg [15:0] ang = 0;
    wire signed [15:0] cs;
    wire signed [15:0] ds;   // sin 未用
    AlgoTrig U_Ref (.iAngle(ang), .oSin(ds), .oCos(cs));

    reg signed [63:0] sumY2, sumX2, sumY;
    integer nMeas;
    integer i;
    reg [31:0] ph32;
    reg signed [15:0] xg;

    task doreinit;
        begin iSysRst = 1; en = 0; repeat(2) @(posedge clk); #1;
              iSysRst = 0; @(posedge clk); #1; end
    endtask

    // 任务: 逐样本喂 x = dc + amp*cos(2π f k/Fs)，f 由 incv/2^32 = f/Fs 决定；
    //       iEn 单拍脉冲、oValid 采样
    task run(input integer tot, input integer skp, input integer dc,
             input integer amp, input [31:0] incv);
        begin
            sumY2=0; sumX2=0; sumY=0; nMeas=0; ph32=0; x_in=0;
            doreinit;
            for (i = 0; i < tot; i = i + 1) begin
                ang = ph32[31:16]; #1;
                xg = (($signed(cs) * amp) >>> 15) + dc;
                x_in = xg;
                en = 1; @(posedge clk); #1; en = 0;
                while (!vld) begin @(posedge clk); #1; end
                if (i >= skp) begin
                    sumY2 = sumY2 + $signed(y_o)*$signed(y_o);
                    sumX2 = sumX2 + $signed(x_in)*$signed(x_in);
                    sumY  = sumY  + $signed(y_o);
                    nMeas = nMeas + 1;
                end
                ph32 = ph32 + incv;
            end
        end
    endtask

    initial begin
        // --- 阶段1: DC 15000 直通 ---
        run(3000, 1500, 15000, 0, 0);
        if (nMeas > 0 && (sumY / nMeas) > 14900 && (sumY / nMeas) < 15100)
            $display("  ok DC 直通 均值≈%0d", sumY / nMeas);
        else begin $display("FAIL DC 直通 mean=%0d", sumY / nMeas); fail = fail + 1; end

        // --- 阶段2: 中心 100Hz 正弦 (幅 2000) 显著衰减: sumY2/sumX2 < 0.09 ---
        run(6000, 4000, 0, 2000, 32'd85899346);       // f/Fs=0.02
        if (sumX2 > 0 && (sumY2*100) < (sumX2*9))
            $display("  ok 100Hz 衰减 sumY2/sumX2=%0d%% (期望<9%%)", (sumY2*100)/sumX2);
        else begin $display("FAIL 100Hz 未显著衰减 sumY2/sumX2=%0d%%",
                            sumX2>0?(sumY2*100)/sumX2:0); fail = fail + 1; end

        // --- 阶段3: 250Hz 非中心 (幅 2000) 保留: 幅度比平方 0.81~1.21 ---
        run(6000, 4000, 0, 2000, 32'd214748365);      // f/Fs=0.05
        if (sumX2 > 0 && (sumY2*100) > (sumX2*81) && (sumY2*100) < (sumX2*121))
            $display("  ok 250Hz 保留 sumY2/sumX2=%0d%%", (sumY2*100)/sumX2);
        else begin $display("FAIL 250Hz 异常 sumY2/sumX2=%0d%%",
                            sumX2>0?(sumY2*100)/sumX2:0); fail = fail + 1; end

        if (fail == 0) $display("PASS CompNotch");
        else           $display("FAIL CompNotch: %0d", fail);
        $finish;
    end
endmodule
