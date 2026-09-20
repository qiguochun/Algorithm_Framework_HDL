//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   tb_CompPir.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.06
/*Description       :   CompPir 单元 TB: 一阶滞后对象(tau=2ms)一拍 ZOH 闭环
//                       参考 = 直流 6000 + 50Hz 正弦 4000 (同时考验 PI 与谐振支路)。
//                       判据: 预热后窗口内峰值 |e| < 300 (PI 去直流+谐振去基波均收敛)。
//                       [多拍流水适配] CompPir 为多拍流水: iEn 一拍=一采样,
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

module TbCompPir;
    localparam real Fs   = 20000.0;
    localparam real Ts   = 1.0/Fs;
    localparam real F0   = 50.0;
    localparam real TWO_PI = 6.283185307179586;
    localparam real TAU  = 0.002;
    localparam real ALC  = Ts/TAU;
    localparam real DCB  = 6000.0;     // 直流分量
    localparam real AC   = 4000.0;     // 50Hz 正弦幅值
    localparam integer NMAX = 14000;   // 0.7s
    localparam integer WM   = 8000;    // 预热样点(0.4s)
    localparam integer LATENCY = 6;    // 与 CompPir 默认 LATENCY 一致

    reg                iSysClk = 0;
    reg                iSysRst = 1;
    reg                iEn     = 0;
    reg  signed [15:0] iErrIn  = 0;
    wire signed [15:0] oOut;
    wire               oValid;

    CompPir U_Dut (
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
        iErrIn = fqe16(DCB - y);   // 首样本误差(tt=0 正弦为 0)

        forever begin
            iEn = 1;
            @(posedge iSysClk);
            #1; iEn = 0;

            waitCnt = 0;
            while (!oValid) begin
                @(posedge iSysClk); #1;
                waitCnt = waitCnt + 1;
                if (waitCnt > 64) begin
                    $display("FAIL CompPir: oValid 未出现");
                    $finish;
                end
            end

            // 对象一拍 ZOH(在 oValid 沿推进)
            oUreal = $signed(oOut);
            y = y + ALC * (oUreal - y);
            s = s + 1;

            tt = s * Ts;
            eCur = fqe16(DCB + AC * $sin(TWO_PI * F0 * tt) - y);
            iErrIn = eCur[15:0];
            if (s >= WM) begin
                if (eCur < 0) begin
                    if (-eCur > peak) peak = -eCur;
                end else begin
                    if (eCur > peak) peak = eCur;
                end
            end
            if (s == NMAX) begin
                $display("SIM_DONE CompPir 直流+50Hz 峰值误差(窗)=%0d counts (%.2f%%)",
                         peak, 100.0 * peak / (DCB + AC));
                if (peak < 300)
                    $display("PASS CompPir: 兼具 PI(去直流)与谐振(去50Hz)跟踪");
                else
                    $display("FAIL CompPir: peak|e|=%0d", peak);
                $finish;
            end
        end
    end
endmodule
