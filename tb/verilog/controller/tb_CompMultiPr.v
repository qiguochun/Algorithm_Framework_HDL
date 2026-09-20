//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   tb_CompMultiPr.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.06
/*Description       :   CompMultiPr 单元 TB: 一阶滞后虚拟对象(tau=2ms, DC 增益 1)一拍 ZOH 闭环,
//                       参考为 50Hz 基波(9000) + 250Hz 5 次谐波(2500) 正弦和。
//                       多谐振(基波+5次谐振支路)提供大增益, 使两频率稳态误差同时趋小。
//                       判据: 预热后窗口对误差做 DFT 投影, 基波残差幅值与 5 次残差幅值均小于阈值
//                       (默认 <300 / <150 counts), 同时打印整体峰值|e|。
//                       [多拍流水适配] CompMultiPr 为多拍流水: iEn 一拍=一采样,
//                       对象推进改在 oValid 沿(LATENCY=6 拍出一结果); 判据不变。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.2 (pipe TB)
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
`timescale 1ns/1ps

module TbCompMultiPr;
    localparam real Fs    = 20000.0;
    localparam real Ts    = 1.0/Fs;
    localparam real F0    = 50.0;
    localparam real W0    = 2.0*3.141592653589793*F0;
    localparam real W5    = 5.0*W0;
    localparam real TAU   = 0.002;
    localparam real ALC   = Ts/TAU;
    localparam real A1    = 9000.0;   // 50Hz 幅值
    localparam real A5    = 2500.0;   // 250Hz 5 次谐波幅值
    localparam integer NMAX = 20000;  // 1.0s
    localparam integer WM   = 12000;  // 预热 0.6s
    localparam integer LATENCY = 6;   // 与 CompMultiPr 默认 LATENCY 一致

    reg                iSysClk = 0;
    reg                iSysRst = 1;
    reg                iEn     = 0;
    reg  signed [15:0] iErrIn  = 0;
    wire signed [15:0] oOut;
    wire               oValid;

    CompMultiPr U_Dut (
        .iSysClk(iSysClk), .iSysRst(iSysRst),
        .iEn(iEn), .iErrIn(iErrIn), .oOut(oOut), .oValid(oValid)
    );
    always #5 iSysClk = ~iSysClk;

    real    y = 0.0;
    real    oUreal;
    integer s = -1;
    real    eR;
    real    tt;
    real    fc1 = 0.0, fs1 = 0.0, fc5 = 0.0, fs5 = 0.0;
    integer wcnt = 0;
    integer peak = 0;
    integer waitCnt = 0;
    integer eCur;

    function integer fqe16(input real x);
        integer v;
        begin
            v = (x >= 0.0) ? $rtoi(x + 0.5) : -$rtoi(0.5 - x);
            if (v >  32767) v =  32767;
            if (v < -32768) v = -32768;
            fqe16 = v;
        end
    endfunction

    real rd;

    initial begin
        iSysRst = 1; iEn = 0; iErrIn = 0;
        repeat (4) @(posedge iSysClk);
        iSysRst = 0;
        @(posedge iSysClk);
        #1;                     // 回到半周期稳定期, 避免首拍 iEn 与正沿竞态

        y = 0.0; s = -1; iErrIn = 0; peak = 0;
        fc1 = 0.0; fs1 = 0.0; fc5 = 0.0; fs5 = 0.0; wcnt = 0;
        iErrIn = 0;   // 首样本(tt=0 参考=0, 误差=-y=0)

        forever begin
            iEn = 1;
            @(posedge iSysClk);
            #1; iEn = 0;

            waitCnt = 0;
            while (!oValid) begin
                @(posedge iSysClk); #1;
                waitCnt = waitCnt + 1;
                if (waitCnt > 64) begin
                    $display("FAIL CompMultiPr: oValid 未出现");
                    $finish;
                end
            end

            // 对象一拍 ZOH(在 oValid 沿推进)
            oUreal = $signed(oOut);
            y = y + ALC * (oUreal - y);          // 虚拟对象一阶滞后
            s = s + 1;

            tt = s * Ts;
            eR = A1*$sin(W0*tt) + A5*$sin(W5*tt) - y;   // 误差
            eCur = fqe16(eR);
            iErrIn = eCur[15:0];
            if (s >= WM) begin
                fc1 = fc1 + eR*$cos(W0*tt);  fs1 = fs1 + eR*$sin(W0*tt);
                fc5 = fc5 + eR*$cos(W5*tt);  fs5 = fs5 + eR*$sin(W5*tt);
                wcnt = wcnt + 1;
                rd = eR; if (rd < 0.0) rd = -rd;
                if ($rtoi(rd) > peak) peak = $rtoi(rd);
            end
            if (s == NMAX) begin
                rd = 2.0*$sqrt(fc1*fc1 + fs1*fs1)/(1.0*wcnt);  // 50Hz 残差幅值
                $display("MultiPr 50Hz 残差幅值=%.1f  (阈值 300)", rd);
                if (rd < 300.0)
                    $display("  ok 基波误差趋小");
                else
                    $display("  FAIL 基波残差幅值 %.1f", rd);
                rd = 2.0*$sqrt(fc5*fc5 + fs5*fs5)/(1.0*wcnt);  // 250Hz 残差幅值
                $display("MultiPr 250Hz 残差幅值=%.1f  (阈值 150)", rd);
                if (rd < 150.0)
                    $display("  ok 5 次谐波误差趋小");
                else
                    $display("  FAIL 5 次残差幅值 %.1f", rd);
                $display("MultiPr 窗内峰值|e|= %0d counts", peak);
                if (rd < 150.0 && (2.0*$sqrt(fc1*fc1+fs1*fs1)/(1.0*wcnt)) < 300.0)
                    $display("PASS CompMultiPr: 基波+5 次参考跟踪, 两频率误差均趋小");
                else
                    $display("FAIL CompMultiPr");
                $finish;
            end
        end
    end
endmodule
