//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   tb_CompHpFilter.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
/*Description       :   CompHpFilter 单元 TB（自包含）
                        输入 = 直流 4096 + 2.5kHz 正弦(幅度6000)，采样率抽象 50k。
                        期望: 直流被滤除，交流幅度 ≈ |H(f)|*A ≈ 5980。
                        【流水化适配】DUT 现为多拍流水（LATENCY=4）：
                        iEn 改为单拍脉冲逐样本喂入，oYOut 在 oValid 拉高时采样；
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

module TbCompHpFilter;
    localparam integer W        = 16;
    localparam integer FS       = 50000;
    localparam integer FC       = 200;          // fc 低 -> 正弦(2.5k) 全通过
    localparam integer FSHIFT   = 15;
    localparam integer PER      = 20;           // 正弦周期(样本)=FS/fsig,含波峰
    localparam integer A_SIN    = 6000;         // 正弦幅度
    localparam integer DC_OFF   = 4096;         // 直流分量
    localparam integer WARM     = 800;          // 预热样本(滤除瞬态)
    localparam integer MEAS_LEN = 200;          // 测量窗样本(=10 周期)

    reg clk = 0;
    reg iSysRst = 1;                            // 高有效复位（起始复位态）
    reg en = 0;
    integer s = 0;
    reg signed [W-1:0] x;
    wire signed [W-1:0] y;
    wire vld;

    CompHpFilter #(
        .W(W), .FC(FC), .FS(FS), .FSHIFT(FSHIFT)
    ) U_Dut (
        .iSysClk(clk), .iSysRst(iSysRst), .iEn(en), .iXIn(x), .oYOut(y), .oValid(vld)
    );

    always #5 clk = ~clk;

    integer i, vmax, vmin;
    integer acc;
    real phase, sval;
    real amp, avg;
    integer fail = 0;

    initial begin
        iSysRst = 1; en = 0; x = 0;
        repeat (3) @(posedge clk); #1;
        iSysRst = 0; #1;

        // 预热 WARM 个样本 + 测量窗，逐样本 iEn 脉冲、oValid 采样
        for (s = 0; s < WARM + MEAS_LEN; s = s + 1) begin
            phase = 2.0 * 3.141592653589793 * (s % PER) / PER;
            sval  = $itor(A_SIN) * $sin(phase);
            x     = DC_OFF + $rtoi(sval);
            en = 1; @(posedge clk); #1; en = 0;
            while (!vld) begin @(posedge clk); #1; end
            if (s == WARM) begin vmax = 0; vmin = 0; acc = 0; end
            if (s >= WARM) begin
                if (y > vmax) vmax = y;
                if (y < vmin) vmin = y;
                acc = acc + y;
            end
        end
        amp = (vmax - vmin) / 2.0;              // 半峰峰值 = 幅度
        avg = $itor(acc) / $itor(MEAS_LEN);
        $display("HPF  amp=%0.1f (expect ~5980)  avg=%0.2f", amp, avg);
        if (amp > 5800.0 && amp < 6150.0)
            $display("PASS hp_filter: DC removed, AC amplitude ~A*|H|");
        else begin
            $display("FAIL hp_filter: amp out of range");
            fail = 1;
        end
        if (avg > 100.0 || avg < -100.0) begin
            $display("FAIL hp_filter: residual DC avg=%0.2f", avg);
            fail = 1;
        end
        if (fail) $finish;
        $display("PASS hp_filter: steady-state DC ~ 0");
        $finish;
    end
endmodule
