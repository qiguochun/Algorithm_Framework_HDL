//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   tb_CompGoertzel.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.06
/*Description       :   CompGoertzel 单元 TB（N=64, k=8, 系数 Q14=23170）。
                        1) bin 精确实正弦(幅2000): 首个完整窗 oAmp ~ A*N/2 = 64000(±3%)。
                        2) 直流(幅2000): bin8 处应近 0(off-bin 拒绝, 阈值 A*N/20)。
                        输入逐拍送入, 自动分窗; 测满窗后 oValid 采样。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.1
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
`timescale 1ns/1ps

module TbCompGoertzel;
    localparam integer W = 16;
    localparam integer N = 64;
    localparam integer K = 8;

    reg clk = 0;
    reg iSysRst = 1;
    reg signed [W-1:0] iData = 0;
    wire [31:0] oAmp;
    wire oValid;
    integer fail = 0;
    always #5 clk = ~clk;

    CompGoertzel #(.W(W), .N_WINDOW(N), .K_BIN(K),
                   .COEF_Q14(16'sd23170)) U_Dut (
        .iSysClk(clk), .iSysRst(iSysRst), .iData(iData),
        .oAmp(oAmp), .oValid(oValid));

    reg signed [W-1:0] tone [0:255];
    integer n, i, c;
    real ph, sv;
    reg [31:0] amp1, amp2;
    integer expv;
    reg found;

    initial begin
        @(negedge clk); iSysRst = 1; #2;

        // 生成 bin k 精确实正弦, 幅 2000
        for (n = 0; n < 256; n = n + 1) begin
            ph = 2.0*3.14159265358979*K*n/N;
            sv = $itor(2000)*$cos(ph);
            tone[n] = (sv>=0) ? $rtoi(sv+0.5) : $rtoi(sv-0.5);
        end

        // ---------- 场景1: bin 精确正弦 ----------
        iSysRst = 1; repeat (2) @(posedge clk);
        @(negedge clk); iSysRst = 0; #2;
        for (n = 0; n < N; n = n + 1) begin
            iData = tone[n];
            @(posedge clk); #1;
        end
        c = 0; found = 0;
        for (i = 0; i < 200 && !found; i = i + 1) begin
            @(posedge clk); #1;
            if (oValid) found = 1;
        end
        amp1 = oAmp;
        expv = 2000*N/2;                       // = 64000
        $display("goertzel bin%d tone A=2000: amp=%0d (exp %0d)", K, amp1, expv);
        if (!found) begin
            $display("FAIL goertzel: 场景1超时"); fail = fail + 1;
        end else if (amp1 < (expv*97)/100 || amp1 > (expv*103)/100) begin
            $display("FAIL goertzel tag1 bin 幅度偏差>3%");
            fail = fail + 1;
        end

        // ---------- 场景2: 直流 off-bin 拒绝 ----------
        iSysRst = 1; repeat (2) @(posedge clk);
        @(negedge clk); iSysRst = 0; #2;
        for (n = 0; n < N; n = n + 1) begin
            iData = 2000;
            @(posedge clk); #1;
        end
        c = 0; found = 0;
        for (i = 0; i < 200 && !found; i = i + 1) begin
            @(posedge clk); #1;
            if (oValid) found = 1;
        end
        amp2 = oAmp;
        $display("goertzel DC A=2000: amp=%0d (off-bin 应小)", amp2);
        if (!found) begin
            $display("FAIL goertzel: 场景2超时"); fail = fail + 1;
        end else if (amp2 > (2000*N)/20) begin          // 阈值 A*N/20 = 6400
            $display("FAIL goertzel tag2 直流泄漏过大");
            fail = fail + 1;
        end

        if (fail == 0) $display("PASS CompGoertzel: 单频 bin 幅 ~ A*N/2, off-bin 拒绝");
        else $display("FAIL CompGoertzel: %0d", fail);
        $finish;
    end
endmodule
