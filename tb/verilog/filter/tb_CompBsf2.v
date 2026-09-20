//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   tb_CompBsf2.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.06
/*Description       :   CompBsf2 二阶带阻/陷波单元 TB（f0=250Hz, fs=5kHz, Q=0.707, Q20）。
                        校验: 直流 16384 通过(均值≈16384±1%，DC 增益=1)；
                        250Hz 中心正弦(幅8000) 被深度陷波(幅比≈0.012, 期望<0.05)；
                        20Hz(f0/12.5) / 1000Hz(4*f0) 正弦 通带直通
                        (幅比≈0.994/0.952, 期望>0.85)。
                        手段: 尾窗 sum^2 判幅比、均值判直流稳态。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.1
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
`timescale 1ns/1ps

module TbCompBsf2;
    localparam integer W     = 16;
    localparam integer FS    = 5000;
    localparam integer DCV   = 16384;
    localparam integer A_SIN = 8000;
    localparam integer PER_250 = 20;    // 250Hz 周期样本
    localparam integer PER_20  = 250;   // 20Hz  周期样本
    localparam integer PER_1K  = 5;     // 1000Hz 周期样本

    reg clk = 0;
    reg iSysRst = 1;
    reg en = 1;
    reg signed [W-1:0] x = 0;
    wire signed [W-1:0] y;
    wire vld;

    CompBsf2 U_Dut (
        .iSysClk(clk), .iSysRst(iSysRst), .iEn(en), .iXIn(x), .oYOut(y), .oValid(vld)
    );

    always #5 clk = ~clk;

    integer fail = 0;
    integer n, idx, xv;
    real sv, mean, sux2, suy2, gain;

    task reinit;
        begin
            en = 0; x = 0;
            iSysRst = 1; repeat (2) @(posedge clk); #1;
            iSysRst = 0; @(posedge clk); #1;
        end
    endtask

    // 喂直流：测陷波器直流直通(DC 增益=1 -> 输出≈dc)
    task run_dc(input integer dc, input integer warm, input integer meas);
        begin
            reinit; mean = 0.0;
            for (n = 0; n < warm + meas; n = n + 1) begin
                x = dc;
                en = 1; @(posedge clk); #1; en = 0;
                while (!vld) begin @(posedge clk); #1; end
                if (n >= warm) mean = mean + $itor($signed(y));
            end
            mean = mean / $itor(meas);
        end
    endtask

    // 喂正弦，测幅比
    task run_sine(input integer amp, input integer PER,
                  input integer warm, input integer meas);
        begin
            reinit; sux2 = 0.0; suy2 = 0.0;
            for (n = 0; n < warm + meas; n = n + 1) begin
                idx = n % PER;
                sv  = $sin(2.0 * 3.141592653589793 * $itor(idx) / $itor(PER));
                xv  = $rtoi($itor(amp) * sv);
                x   = xv;
                en = 1; @(posedge clk); #1; en = 0;
                while (!vld) begin @(posedge clk); #1; end
                if (n >= warm) begin
                    sux2 = sux2 + $itor($signed(xv)) * $itor($signed(xv));
                    suy2 = suy2 + $itor($signed(y)) * $itor($signed(y));
                end
            end
            if (sux2 > 0.0) gain = $sqrt(suy2 / sux2);
            else            gain = 0.0;
        end
    endtask

    initial begin
        // ---- 测试1: 直流 16384 直通（DC 增益精确=1 -> 期望≈16384±1%） ----
        run_dc(DCV, 2500, 2000);
        if (mean > 16200.0 && mean < 16600.0)
            $display("  ok 直流直通  mean=%0.1f (DC=%0d, %+.2f%%)",
                     mean, DCV, (mean/$itor(DCV) - 1.0) * 100.0);
        else begin
            $display("FAIL 直流直通  mean=%0.1f", mean);
            fail = fail + 1;
        end

        // ---- 测试2: 250Hz 中心正弦 8000 深度陷波，幅比≈0.012（期望<0.05） ----
        run_sine(A_SIN, PER_250, 4000, 20000);
        if (gain < 0.05)
            $display("  ok 250Hz 陷波  amp-ratio=%.5f (理论≈0.012)", gain);
        else begin
            $display("FAIL 250Hz 陷波 amp-ratio=%.5f", gain);
            fail = fail + 1;
        end

        // ---- 测试3: 20Hz 正弦 8000 通带直通，幅比≈0.994（期望>0.85） ----
        run_sine(A_SIN, PER_20, 3000, 15000);
        if (gain > 0.85)
            $display("  ok 20Hz 通带直通  amp-ratio=%.5f (理论≈0.994)", gain);
        else begin
            $display("FAIL 20Hz 通带 amp-ratio=%.5f", gain);
            fail = fail + 1;
        end

        // ---- 测试4: 1000Hz(4*f0) 正弦 8000 通带直通，幅比≈0.952（期望>0.85） ----
        run_sine(A_SIN, PER_1K, 2000, 15000);
        if (gain > 0.85)
            $display("  ok 1000Hz 通带直通  amp-ratio=%.5f (理论≈0.952)", gain);
        else begin
            $display("FAIL 1000Hz 通带 amp-ratio=%.5f", gain);
            fail = fail + 1;
        end

        if (fail == 0) $display("PASS CompBsf2");
        else           $display("FAIL CompBsf2: %0d", fail);
        $finish;
    end
endmodule
