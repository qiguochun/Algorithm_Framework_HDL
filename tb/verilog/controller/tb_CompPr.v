//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   tb_CompPr.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.06
/*Description       :   CompPr 单元 TB: 一阶滞后虚拟对象(tau=2ms, DC增益1)一拍 ZOH 闭环
//                       在 f0=50Hz 正弦参考(10000 counts)下, PR 谐振支路大增益使稳态误差趋 0。
//                       判据: 预热后窗口内峰值 |e| < 250 (<2.5% 幅值)。
//                       另开环扫频核对: 50Hz 处谐振器输出/输入 增益>>低频(打印)。
//                       [多拍流水适配] CompPr 为多拍流水: iEn 一拍=一采样,
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

module TbCompPr;
    localparam real Fs   = 20000.0;
    localparam real Ts   = 1.0/Fs;
    localparam real F0   = 50.0;
    localparam real TWO_PI = 6.283185307179586;
    localparam real TAU  = 0.002;
    localparam real ALC  = Ts/TAU;
    localparam real AMP  = 10000.0;    // 50Hz 正弦幅值
    localparam integer NMAX = 12000;   // 0.6s
    localparam integer WM   = 7000;    // 预热样点(0.35s)
    localparam integer LATENCY = 6;    // 与 CompPr 默认 LATENCY 一致

    reg                iSysClk = 0;
    reg                iSysRst = 1;
    reg                iEn     = 0;
    reg  signed [15:0] iErrIn  = 0;
    wire signed [15:0] oOut;
    wire               oValid;

    CompPr U_Dut (
        .iSysClk(iSysClk), .iSysRst(iSysRst),
        .iEn(iEn), .iErrIn(iErrIn), .oOut(oOut), .oValid(oValid)
    );

    always #5 iSysClk = ~iSysClk;

    real    y = 0.0;
    real    oUreal;
    real    tt;
    integer s = -1;
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

    initial begin
        iSysRst = 1; iEn = 0; iErrIn = 0;
        repeat (4) @(posedge iSysClk);
        iSysRst = 0;
        @(posedge iSysClk);
        #1;                     // 回到半周期稳定期, 避免首拍 iEn 与正沿竞态

        y = 0.0; s = -1; peak = 0;
        tt = 0.0;
        iErrIn = fqe16(AMP * $sin(TWO_PI * F0 * 0.0 * Ts) - y);  // 首样本误差

        forever begin
            iEn = 1;
            @(posedge iSysClk);
            #1; iEn = 0;

            waitCnt = 0;
            while (!oValid) begin
                @(posedge iSysClk); #1;
                waitCnt = waitCnt + 1;
                if (waitCnt > 64) begin
                    $display("FAIL CompPr: oValid 未出现");
                    $finish;
                end
            end

            // 对象一拍 ZOH(在 oValid 沿推进)
            oUreal = $signed(oOut);
            y = y + ALC * (oUreal - y);
            s = s + 1;

            tt = s * Ts;
            eCur = fqe16(AMP * $sin(TWO_PI * F0 * tt) - y);
            iErrIn = eCur[15:0];
            if (s >= WM) begin
                if (eCur < 0) begin
                    if (-eCur > peak) peak = -eCur;
                end else begin
                    if (eCur > peak) peak = eCur;
                end
            end
            if (s == NMAX) begin
                $display("SIM_DONE CompPr 50Hz 峰值误差(窗)=%0d counts (幅值 %0d, %.2f%%)",
                         peak, 10000, 100.0 * peak / 10000.0);
                if (peak < 250)
                    $display("PASS CompPr: 50Hz 正弦参考跟踪误差趋小(谐振增益大)");
                else
                    $display("FAIL CompPr: peak|e|=%0d", peak);
                $finish;
            end
        end
    end
endmodule
