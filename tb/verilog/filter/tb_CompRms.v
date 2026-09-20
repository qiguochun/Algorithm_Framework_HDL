//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   tb_CompRms.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
/*Description       :   CompRms 单元 TB（自包含）
                        输入 = 幅度 2000、周期 100 样本的正弦；窗口 N=100（=整周期）。
                        期望: 每个窗口 rms = A/sqrt(2) ≈ 1414（±3%）。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.1
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
`timescale 1ns/1ps

module TbCompRms;
    localparam integer PER     = 100;    // 正弦周期(样本)
    localparam integer A_SIN   = 2000;   // 幅度
    localparam integer WINDOWS = 8;      // 测量窗数
    localparam integer TOTAL   = 8 * PER + 64; // 总样本(含头尾)
    localparam integer REXP    = 1414;   // A/sqrt(2)

    reg clk = 0;
    reg iSysRst = 1;
    reg en = 0;
    integer n = 0;
    reg signed [15:0] x;
    wire signed [15:0] rms_val;
    wire rms_valid;

    always @(posedge clk) n <= n + 1;

    real ph, sv;
    always @* begin
        ph = 2.0 * 3.141592653589793 * (n % PER) / PER;
        sv = $itor(A_SIN) * $sin(ph);
        x  = (sv >= 0) ? $rtoi(sv + 0.5) : $rtoi(sv - 0.5);
    end

    CompRms #(.W(16), .N(PER)) U_Dut (
        .iSysClk(clk), .iSysRst(iSysRst), .iEn(en), .iXIn(x),
        .oRmsVal(rms_val), .oRmsValid(rms_valid)
    );

    always #5 clk = ~clk;

    integer i;
    integer win_cnt, win_sum;
    real avg_rms;
    integer fail = 0;

    initial begin
        iSysRst = 1; en = 1;
        repeat (3) @(posedge clk);
        iSysRst = 0;

        win_cnt = 0; win_sum = 0;
        for (i = 0; i < TOTAL; i = i + 1) begin
            @(posedge clk);
            if (rms_valid) begin
                win_cnt = win_cnt + 1;
                win_sum = win_sum + rms_val;
                $display("RMS  win%0d value=%0d", win_cnt, rms_val);
            end
        end
        if (win_cnt == 0) begin
            $display("FAIL rms: no window completed");
            $finish;
        end
        avg_rms = win_sum / win_cnt;
        $display("RMS  avg=%0.1f (expect ~%0d, windows=%0d)", avg_rms, REXP, win_cnt);
        if (avg_rms > REXP * 0.97 && avg_rms < REXP * 1.03)
            $display("PASS rms: windowed RMS = A/sqrt(2)");
        else begin
            $display("FAIL rms: rms out of range");
            fail = 1;
        end
        if (fail) $finish;
        $finish;
    end
endmodule
