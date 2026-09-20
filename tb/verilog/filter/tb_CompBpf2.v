//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   tb_CompBpf2.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.06
/*Description       :   CompBpf2 二阶巴特沃斯带通单元 TB（f0=250Hz, fs=5kHz, Q=0.707, Q20）。
                        校验: 250Hz 中心正弦(幅8000) 通过(幅比≈1.0, 期望>0.9)；
                        直流 16384 被直阻(输出≈0)；
                        62.5Hz(f0/4) / 1000Hz(4*f0) 正弦 离中心被抑制
                        (幅比≈0.353/0.305, 期望<0.45)，带通仅通中心附近。
                        手段: 尾窗 sum^2 判幅比、均值判直流残留。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.1
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
`timescale 1ns/1ps

module TbCompBpf2;
    localparam integer W     = 16;
    localparam integer FS    = 5000;
    localparam integer DCV   = 16384;
    localparam integer A_SIN = 8000;
    localparam integer PER_250 = 20;    // 250Hz 周期样本
    localparam integer PER_625 = 80;    // 62.5Hz 周期样本
    localparam integer PER_1K  = 5;     // 1000Hz 周期样本

    reg clk = 0;
    reg iSysRst = 1;
    reg en = 1;
    reg signed [W-1:0] x = 0;
    wire signed [W-1:0] y;
    wire vld;

    CompBpf2 U_Dut (
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

    // 喂直流：测带通直阻后输出残留(应≈0)
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
        // ---- 测试1: 250Hz 中心正弦 8000 通过，幅比≈1.0（期望>0.9） ----
        run_sine(A_SIN, PER_250, 3000, 15000);
        if (gain > 0.9)
            $display("  ok 250Hz 中心通过  amp-ratio=%.5f (理论≈1.0)", gain);
        else begin
            $display("FAIL 250Hz 中心 amp-ratio=%.5f", gain);
            fail = fail + 1;
        end

        // ---- 测试2: 直流被直阻（带通 DC 增益=0，稳态输出≈0） ----
        run_dc(DCV, 3000, 2000);
        if (mean > -200.0 && mean < 200.0)
            $display("  ok 直流直阻  mean=%0.1f (应≈0)", mean);
        else begin
            $display("FAIL 直流直阻  mean=%0.1f", mean);
            fail = fail + 1;
        end

        // ---- 测试3: 62.5Hz(f0/4) 正弦 8000 被抑制，幅比≈0.353（期望<0.45） ----
        run_sine(A_SIN, PER_625, 3000, 20000);
        if (gain < 0.45)
            $display("  ok 62.5Hz 下边带抑制  amp-ratio=%.5f (理论≈0.353)", gain);
        else begin
            $display("FAIL 62.5Hz 抑制 amp-ratio=%.5f", gain);
            fail = fail + 1;
        end

        // ---- 测试4: 1000Hz(4*f0) 正弦 8000 被抑制，幅比≈0.305（期望<0.45） ----
        run_sine(A_SIN, PER_1K, 2000, 15000);
        if (gain < 0.45)
            $display("  ok 1000Hz 上边带抑制  amp-ratio=%.5f (理论≈0.305)", gain);
        else begin
            $display("FAIL 1000Hz 抑制 amp-ratio=%.5f", gain);
            fail = fail + 1;
        end

        if (fail == 0) $display("PASS CompBpf2");
        else           $display("FAIL CompBpf2: %0d", fail);
        $finish;
    end
endmodule
