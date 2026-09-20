//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   tb_CompPi.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.06
/*Description       :   CompPi 单元 TB: 一阶滞后虚拟对象(R*C 型, tau=2ms, DC 增益1)
//                       一拍 ZOH 延迟闭环, 阶跃 10000 counts 参考 -> 无静差。
//                       判据: 预热后窗口内峰值 |e| < 150 (无稳态误差)。
//                       TB 用 real 数学模型(仅仿真), 控制器接口全 S16 整型。
//                       [多拍流水适配] CompPi 为多拍流水: iEn 一拍=一采样,
//                       对象(对象每拍推进)改在 oValid(新 u 有效)沿推进; 判据不变。
//                       时钟周期数与 iEn 采样周期解耦(LATENCY 拍出一结果)。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.2 (pipe TB)
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
`timescale 1ns/1ps

module TbCompPi;
    // 采样率/对象参数(与模块系数 Fs=20kHz 一致)
    localparam real Fs   = 20000.0;
    localparam real Ts   = 1.0/Fs;
    localparam real TAU  = 0.002;      // 对象时间常数 2ms
    localparam real ALC  = Ts/TAU;     // alpha=0.025
    localparam real STEP = 10000.0;    // 阶跃参考
    localparam integer NMAX = 6000;    // 0.3s
    localparam integer WM   = 2500;    // 预热样点(0.125s)
    localparam integer LATENCY = 4;    // 与 CompPi 默认 LATENCY 一致

    reg                iSysClk = 0;
    reg                iSysRst = 1;
    reg                iEn     = 0;
    reg  signed [15:0] iErrIn  = 0;
    wire signed [15:0] oOut;
    wire               oValid;

    CompPi U_Dut (
        .iSysClk(iSysClk), .iSysRst(iSysRst),
        .iEn(iEn), .iErrIn(iErrIn), .oOut(oOut), .oValid(oValid)
    );

    always #5 iSysClk = ~iSysClk;      // 每时钟=一拍(多拍出一采样结果)

    real    y = 0.0;                   // 对象输出(实数)
    real    oUreal;
    integer s = -1;                    // 已应用的控制样本序号(对象推进次数)
    integer peak = 0;                  // 测量窗内峰值 |e|
    integer nZero = 0;                 // 测量窗内 e==0 计数
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

        y = 0.0; s = -1; peak = 0; nZero = 0;
        iErrIn = fqe16(STEP - y);       // 首样本误差

        // 闭环: 对象在 oValid(新 u 有效)沿推进一拍 Ts (原为每时钟推进)
        forever begin
            // ---- 采样一拍: iEn 打一拍, 采 iErrIn ----
            iEn = 1;
            @(posedge iSysClk);
            #1; iEn = 0;

            // ---- 等新 u 有效(oValid) ----
            waitCnt = 0;
            while (!oValid) begin
                @(posedge iSysClk); #1;
                waitCnt = waitCnt + 1;
                if (waitCnt > 64) begin
                    $display("FAIL CompPi: oValid 未出现");
                    $finish;
                end
            end

            // ---- 对象一拍 ZOH: 应用本样本 u ----
            oUreal = $signed(oOut);
            y = y + ALC * (oUreal - y);          // 一阶滞后对象
            s = s + 1;

            // ---- 误差 & 窗口统计(与对象推进同步, 逐样本) ----
            eCur = fqe16(STEP - y);
            iErrIn = eCur[15:0];
            if (s >= WM) begin
                if (eCur < 0) begin
                    if (-eCur > peak) peak = -eCur;
                end else begin
                    if (eCur > peak) peak = eCur;
                end
                if (eCur == 0) nZero = nZero + 1;
            end
            if (s == NMAX) begin
                $display("SIM_DONE CompPi y=%.1f oOut=%0d peak|e|(window)=%0d/10000", y, oOut, peak);
                if (peak < 150)
                    $display("PASS CompPi: PI 阶跃跟踪收敛, 无稳态误差");
                else
                    $display("FAIL CompPi: peak|e|=%0d 静差过大", peak);
                $finish;
            end
        end
    end
endmodule
