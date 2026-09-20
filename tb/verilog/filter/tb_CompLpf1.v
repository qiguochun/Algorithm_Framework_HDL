//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   tb_CompLpf1.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.06
/*Description       :   CompLpf1 一阶巴特沃斯低通单元 TB（默认 fc=10Hz, fs=5kHz）。
                        校验: 直流 16384 通过；100Hz 正弦对 fc=10 衰减≈0.1；
                        另例化 FC=100 时 100Hz 幅比≈0.707（证明 FS/FC 编译期改系数）。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.1
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
`timescale 1ns/1ps

module TbCompLpf1;
    localparam integer W     = 16;
    localparam integer FS    = 5000;    // 采样率 Hz
    localparam integer PER_100 = 50;    // 100Hz 周期样本 = FS/100
    localparam integer DCV   = 16384;   // 直流测试幅值
    localparam integer A_SIN = 8000;    // 正弦测试幅值

    reg clk = 0;
    reg iSysRst = 1;
    reg en = 0;
    reg signed [W-1:0] x = 0;
    wire signed [W-1:0] y;
    wire vld;

    CompLpf1 U_Dut (
        .iSysClk(clk), .iSysRst(iSysRst), .iEn(en), .iXIn(x), .oYOut(y), .oValid(vld)
    );
    wire signed [W-1:0] y_fc;
    wire                vld_fc;
    CompLpf1 #(.FC(100), .FS(5000)) U_DutFc (
        .iSysClk(clk), .iSysRst(iSysRst), .iEn(en), .iXIn(x), .oYOut(y_fc), .oValid(vld_fc)
    );

    always #5 clk = ~clk;

    integer fail = 0;
    integer s, idx, xv;
    real sv, mean, sux2, suy2, gain;

    task reinit;
        begin
            en = 0; x = 0;
            iSysRst = 1; repeat (2) @(posedge clk); #1;
            iSysRst = 0; @(posedge clk); #1;
        end
    endtask

    // 喂直流: x=dc，warm 个样本后测 meas 个样本均值（每样本 iEn 脉冲 + 等 oValid）
    task run_dc(input integer dc, input integer warm, input integer meas);
        begin
            reinit; mean = 0.0;
            for (s = 0; s < warm + meas; s = s + 1) begin
                x = dc;
                en = 1; @(posedge clk); #1; en = 0;
                while (!vld) begin @(posedge clk); #1; end
                if (s >= warm) mean = mean + $itor($signed(y));
            end
            mean = mean / $itor(meas);
        end
    endtask

    // 喂正弦: x = amp*sin(2*pi*s/PER)，warm 后测 meas 个样本 sum^2 幅比
    task run_sine(input integer amp, input integer PER,
                  input integer warm, input integer meas);
        begin
            reinit; sux2 = 0.0; suy2 = 0.0;
            for (s = 0; s < warm + meas; s = s + 1) begin
                idx = s % PER;
                sv  = $sin(2.0 * 3.141592653589793 * $itor(idx) / $itor(PER));
                xv  = $rtoi($itor(amp) * sv);
                x   = xv;
                en = 1; @(posedge clk); #1; en = 0;
                while (!vld) begin @(posedge clk); #1; end
                if (s >= warm) begin
                    sux2 = sux2 + $itor($signed(xv)) * $itor($signed(xv));
                    suy2 = suy2 + $itor($signed(y)) * $itor($signed(y));
                end
            end
            if (sux2 > 0.0) gain = $sqrt(suy2 / sux2);
            else            gain = 0.0;
        end
    endtask

    initial begin
        // ---- 测试1: 直流 16384 通过（增益≈1, DC 增益 1.0024 -> 期望 ≈16424） ----
        run_dc(DCV, 2500, 2000);
        if (mean > 16200.0 && mean < 16600.0)
            $display("  ok DC 通过  mean=%0.1f (DC=%0d, +%.2f%%)",
                     mean, DCV, (mean/$itor(DCV) - 1.0) * 100.0);
        else begin
            $display("FAIL DC 通过  mean=%0.1f 期望≈16424", mean);
            fail = fail + 1;
        end

        // ---- 测试2: 100Hz(10*fc) 正弦 8000 衰减，幅比≈0.0996（期望 0.04~0.20） ----
        run_sine(A_SIN, PER_100, 3000, 10000);
        if (gain > 0.04 && gain < 0.20)
            $display("  ok 100Hz 衰减  amp-ratio=%.4f (理论≈0.0996, 10*fc->-20dB)",
                     gain);
        else begin
            $display("FAIL 100Hz 衰减 amp-ratio=%.4f", gain);
            fail = fail + 1;
        end

        // ---- 测试3: 同 100Hz 激励，FC=100 的例化应接近 -3dB（证明 FS/FC 编译期改系数） ----
        begin : fc_sweep
            real sux2f, suy2f, gain_fc;
            sux2f = 0.0; suy2f = 0.0;
            reinit;
            for (s = 0; s < 3000 + 4000; s = s + 1) begin
                idx = s % PER_100;
                sv  = $sin(2.0 * 3.141592653589793 * $itor(idx) / $itor(PER_100));
                xv  = $rtoi($itor(A_SIN) * sv);
                x   = xv;
                en = 1; @(posedge clk); #1; en = 0;
                while (!vld_fc) begin @(posedge clk); #1; end
                if (s >= 3000) begin
                    sux2f = sux2f + $itor($signed(xv)) * $itor($signed(xv));
                    suy2f = suy2f + $itor($signed(y_fc)) * $itor($signed(y_fc));
                end
            end
            gain_fc = (sux2f > 0.0) ? $sqrt(suy2f / sux2f) : 0.0;
            if (gain_fc > 0.55 && gain_fc < 0.85)
                $display("  ok FC=100 编译期改系数  100Hz amp-ratio=%.4f (理论≈0.707)",
                         gain_fc);
            else begin
                $display("FAIL FC=100 编译期改系数 amp-ratio=%.4f 期望≈0.707", gain_fc);
                fail = fail + 1;
            end
        end

        if (fail == 0) $display("PASS CompLpf1");
        else           $display("FAIL CompLpf1: %0d", fail);
        $finish;
    end
endmodule
