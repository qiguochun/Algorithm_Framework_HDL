//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   tb_CompXcorr.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
/*Description       :   CompXcorr 单元 TB
                        WIN=128(2幂), x=周期16正弦, y=x延迟5；
                        填满窗口后停止 iEn，等待后台 MAC+sqrt+div 完成，
                        期望 oLagPeak==5、oRPeak 接近 1。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.2
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
`timescale 1ns/1ps

module TbCompXcorr;
    localparam integer WIN   = 128;
    localparam integer MLAG  = 16;
    localparam integer D     = 5;
    localparam integer A_SIN = 12000;
    localparam integer PER   = 16;
    localparam integer TOT   = 200;
    localparam integer WAIT  = 5000;   // MAC+sqrt+div 余量
    localparam integer RKLO  = 28000;
    localparam integer RKHI  = 32767;

    reg clk = 0;
    reg iSysRst = 1;
    reg en = 0;
    reg signed [15:0] x_in = 0;
    reg signed [15:0] y_in = 0;
    wire signed [15:0] r_peak;
    wire [7:0] lag_peak;
    wire ready;

    reg signed [15:0] XS [0:TOT-1];
    reg signed [15:0] YS [0:TOT-1];
    integer gi;
    real sv;
    initial begin
        for (gi = 0; gi < TOT; gi = gi + 1) begin
            sv = $itor(A_SIN) * $sin(2.0 * 3.141592653589793 * $itor(gi) / $itor(PER));
            XS[gi] = (sv >= 0) ? $rtoi(sv + 0.5) : $rtoi(sv - 0.5);
            if (gi >= D) YS[gi] = XS[gi-D];
            else         YS[gi] = 16'sd0;
        end
    end

    CompXcorr #(
        .W(16), .WIN_SIZE(WIN), .MAX_LAG(MLAG), .LAGW(8)
    ) U_Dut (
        .iSysClk(clk), .iSysRst(iSysRst), .iEn(en), .iXIn(x_in), .iYIn(y_in),
        .oRPeak(r_peak), .oLagPeak(lag_peak), .oReady(ready)
    );

    always #5 clk = ~clk;

    integer bad = 0;
    integer i;

    initial begin
        iSysRst = 1; en = 0;
        repeat (3) @(posedge clk);
        iSysRst = 0;

        for (i = 0; i < WIN; i = i + 1) begin
            @(negedge clk);
            x_in = XS[i];
            y_in = YS[i];
            en = 1;
            @(posedge clk);
            @(negedge clk);
            en = 0;
        end

        if (!ready) begin
            bad = 1;
            $display("Xc  FAIL: ready=0 after WIN samples");
        end

        // 等待后台计算完成（缓冲不再改写）
        repeat (WAIT) @(posedge clk);

        $display("Xc  INFO: lag_peak=%0d r_peak=%0d", lag_peak, r_peak);
        if (lag_peak != D) begin
            bad = 1;
            $display("Xc  FAIL: lag_peak=%0d expect %0d", lag_peak, D);
        end
        if (r_peak <= RKLO || r_peak > RKHI) begin
            bad = 1;
            $display("Xc  FAIL: r_peak=%0d out of [%0d,%0d]", r_peak, RKLO, RKHI);
        end

        if (!bad) $display("PASS CompXcorr: y=x延迟5, lag_peak=5 & r_peak~1");
        else      $display("FAIL CompXcorr");
        $finish;
    end
endmodule
