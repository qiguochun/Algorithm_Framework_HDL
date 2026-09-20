//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   CompPi.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.06
/*Description       :   位置式 PI 控制器（双线性梯形离散 + 条件积分抗饱和）
//                       连续:  C(s)=Kp+Ki/s
//                       离散(梯形/双线性积分): ui[n]=ui[n-1]+(Ki*Ts/2)(e[n]+e[n-1])
//                                             u[n]  = Kp*e[n]+ui[n]
//                       抗饱和: 输出越限且误差同向(继续加)时冻结积分累加。
//                       定点:  系数 Kp、Ki*Ts/2 均 S16/Q14 参数; 中间 48bit 累加,
//                               fn_sat 舍入右移(Q14)后 S16 输出; 输出限幅 [P_OMIN,P_OMAX]。
//                       default: Kp=0.8, Ki*Ts/2=0.02 (Fs=20kHz -> Ki≈800/s, 一拍=一Ts)
//  ----------------------------------------------------------------------
//  多拍流水(消除单周期组合乘加链): 每 iEn 采样一拍, 内部按 rPipe 分拍,
//  每拍只做一个乘法(乘积独立寄存)或一步求和/饱和, 禁止一拍算完多个乘加。
//    beat1(rPipe[0]) 比例乘 Kp*e                 —— 一拍(一个乘法)
//    beat2(rPipe[1]) 积分乘 KiTs2*(e+e[n-1])      —— 一拍(一个乘法)
//    beat3(rPipe[2]) 求和: 积分候选 cand、raw u   —— 一拍
//    beat4(rPipe[3], 末拍) 抗饱和冻结+fn_sat+限幅+状态回写 —— 一拍
//  LATENCY=4: iEn 采样后第 4 拍 oValid 拉高一拍, oOut 为新 u[n]。
//  约束: 相邻 iEn 间隔 >= LATENCY 拍(非重叠流水, 保证数值与原组合式逐样本一致)。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.2 (pipe)
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
`include "algo_arith.vh"

module CompPi #(
    parameter integer       W        = 16,
    parameter integer       QF       = 14,           // 系数 Q 格式(统一下缩位)
    parameter integer       LATENCY  = 4,            // 多拍流水延迟: iEn 采样 -> oValid 有效
    parameter signed [15:0] P_KP     = 16'sd13107,   // Kp       = 0.800    Q14
    parameter signed [15:0] P_KIT2   = 16'sd328,     // Ki*Ts/2  = 0.0200   Q14 (Fs=20k,Ki~800/s)
    parameter signed [31:0] P_OMAX   = 32'sd30000,   // 输出上限
    parameter signed [31:0] P_OMIN   = -32'sd30000   // 输出下限
) (
    input  wire                iSysClk,
    input  wire                iSysRst,       // 高有效复位
    input  wire                iEn,           // 采样使能: 一拍=一采样(相邻 iEn 间隔>=LATENCY)
    input  wire signed [W-1:0] iErrIn,        // 控制误差 e=ref-fbk (S16)
    output reg  signed [W-1:0] oOut,          // 控制器输出 u (S16)
    output reg                 oValid         // iEn 后 LATENCY 拍拉高一拍(新 u 有效)
);

    // ---- Q14 域输出限幅 (上限<<QF) ----
    localparam signed [47:0] L_2QF   = (48'sd1 <<< QF);
    localparam signed [47:0] L_OMaxQ = L_2QF * P_OMAX;
    localparam signed [47:0] L_OMinQ = L_2QF * P_OMIN;

    // ---- 状态/分拍寄存器 ----
    reg signed [47:0] rI;          // 积分状态 ui<<QF (S48)
    reg signed [15:0] rErrP;       // e[n-1]
    reg signed [15:0] rX;          // 本拍采样误差 e[n] (iEn 锁定, 供各分拍用)
    reg signed [47:0] rKpE;        // beat1 乘积: Kp*e      (Q14 域)
    reg signed [47:0] rIt2;        // beat2 乘积: KiTs2*(e+e[n-1]) (Q14 域)
    reg signed [47:0] rCand;       // beat3: 积分候选 rI + 增量
    reg signed [47:0] rAcc;        // beat3: raw u = Kp*e + cand (Q14 域)
    reg [LATENCY-1:0] rPipe;       // 采样脉冲移位(分拍); bit0=iEn 注入

    wire signed [47:0] wX    = rX;               // S16->S48 符号扩展
    wire signed [47:0] wESum = rX + rErrP;       // e[n]+e[n-1] (S48)
    wire signed [15:0] wOut16 = fn_sat(rAcc, QF[4:0]);   // S16 饱和输出(仅末拍用)
    wire signed [31:0] wO32   = {{16{wOut16[15]}}, wOut16};
    wire wFreeze = ((rAcc > L_OMaxQ) && ($signed(rX) > 16'sd0)) ||
                   ((rAcc < L_OMinQ) && ($signed(rX) < 16'sd0));

    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            rI     <= {48{1'b0}};
            rErrP  <= {16{1'b0}};
            rX     <= {16{1'b0}};
            rKpE   <= {48{1'b0}};
            rIt2   <= {48{1'b0}};
            rCand  <= {48{1'b0}};
            rAcc   <= {48{1'b0}};
            rPipe  <= {LATENCY{1'b0}};
            oOut   <= {W{1'b0}};
            oValid <= 1'b0;
        end else begin
            // 采样(误差一拍) + 采样脉冲移位分拍
            if (iEn) rX <= iErrIn;
            rPipe <= {rPipe[LATENCY-2:0], iEn};

            // beat1: 比例乘一拍(单乘法, 独立寄存)
            if (rPipe[0])
                rKpE <= P_KP * wX;

            // beat2: 积分乘一拍(梯形 e+e[n-1], 单乘法, 独立寄存)
            if (rPipe[1])
                rIt2 <= P_KIT2 * wESum;

            // beat3: 求和一拍(无乘法): 积分候选 & raw u
            if (rPipe[2]) begin
                rCand <= rI + rIt2;              // ui 候选(梯形增量)
                rAcc  <= rKpE + rI + rIt2;       // raw u (Q14 域)
            end

            // beat4(末拍): 抗饱和冻结 + fn_sat/限幅 + 状态回写
            if (rPipe[LATENCY-1]) begin
                if (!wFreeze) rI <= rCand;       // 越限且误差同向 -> 冻结积分
                rErrP <= rX;                     // e[n-1] <= e[n]
                if (wO32 > $signed(P_OMAX))
                    oOut <= P_OMAX[W-1:0];
                else if (wO32 < $signed(P_OMIN))
                    oOut <= P_OMIN[W-1:0];
                else
                    oOut <= wOut16;
            end

            // 完成脉冲: iEn 后 LATENCY 拍拉高一拍
            oValid <= rPipe[LATENCY-1];
        end
    end
endmodule
