//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   tb_CompHpf2.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.06
/*Description       :   CompHpf2 二阶巴特沃斯高通单元 TB（fc=250Hz, fs=5kHz, 系数 Q20）。
                        校验: 直流 16384 被直阻(输出≈0)；
                        20Hz(fc/12.5) 正弦(幅8000) 强衰减(幅比≈0.0064, 期望<0.03)；
                        500Hz(2*fc) 正弦(幅8000) 通过(幅比≈0.974, 期望>0.85)。
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

module TbCompHpf2;
    localparam integer W     = 16;
    localparam integer FS    = 5000;
    localparam integer DCV   = 16384;
    localparam integer A_SIN = 8000;
    localparam integer PER_20  = 250;    // 20Hz  周期样本
    localparam integer PER_500 = 10;     // 500Hz 周期样本

    reg clk = 0;
    reg iSysRst = 1;
    reg en = 1;
    reg signed [W-1:0] x = 0;
    wire signed [W-1:0] y;
    wire vld;

    CompHpf2 U_Dut (
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

    // 喂直流：测通带输出残留(高通应≈0)
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
        // ---- 测试1: 直流被直阻（高通 DC 增益≈0，稳态输出≈0） ----
        run_dc(DCV, 3000, 2000);
        if (mean > -200.0 && mean < 200.0)
            $display("  ok 直流直阻  mean=%0.1f (应≈0)", mean);
        else begin
            $display("FAIL 直流直阻  mean=%0.1f", mean);
            fail = fail + 1;
        end

        // ---- 测试2: 20Hz(fc/12.5) 正弦 8000 强衰减，幅比≈0.0064（期望<0.03） ----
        run_sine(A_SIN, PER_20, 3000, 15000);
        if (gain < 0.03)
            $display("  ok 20Hz 强衰减  amp-ratio=%.5f (理论≈0.0064)", gain);
        else begin
            $display("FAIL 20Hz 强衰减 amp-ratio=%.5f", gain);
            fail = fail + 1;
        end

        // ---- 测试3: 500Hz(2*fc) 正弦 8000 通过，幅比≈0.974（期望>0.85） ----
        run_sine(A_SIN, PER_500, 2000, 15000);
        if (gain > 0.85)
            $display("  ok 500Hz 通带通过  amp-ratio=%.5f (理论≈0.974)", gain);
        else begin
            $display("FAIL 500Hz 通带 amp-ratio=%.5f", gain);
            fail = fail + 1;
        end

        if (fail == 0) $display("PASS CompHpf2");
        else           $display("FAIL CompHpf2: %0d", fail);
        $finish;
    end
endmodule
