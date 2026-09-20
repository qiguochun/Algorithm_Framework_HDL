//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   CompPir.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.06
/*Description       :   PIR 控制器 = PI(梯形/双线性积分) + 谐振器, 并联
//                       连续:  C(s)=Kp + Ki/s + 2*Kr*wc*s/(s^2+2*wc*s+w0^2)
//                       离散(每部分双线性):
//                         PI 支路 ui[n]=ui[n-1]+(Ki*Ts/2)(e[n]+e[n-1])      (积分状态 S48/Q14)
//                         谐振支路 yr[n]=b0*e[n]+b2*e[n-2]-a1*yr[n-1]-a2*yr[n-2]
//                         u[n]=sat16(Kp*e + ui + yr)   (S48/Q14 累加, fn_sat 舍入)
//                       抗饱和: 输出越限且误差同向时冻结 PI 积分。
//                       系数 Kp/Ki*Ts/2/a1/a2/b0/b2 均 S16/Q14(QF)参数。
//                       default: Kp=1.0, Ki*Ts/2=0.02 (Ki~800/s), f0=50Hz,
//                                wc=2*pi*2 rad/s, Fs=20kHz, Kr=400。
//  ----------------------------------------------------------------------
//  多拍流水(消除单周期组合乘加链): 每个乘法一拍且独立寄存, 求和/饱和各一拍:
//    beat1(rPipe[0]) b0*e、b2*e[n-2]            —— 一拍(2 独立乘法)
//    beat2(rPipe[1]) -a1*yr[n-1]、-a2*yr[n-2]   —— 一拍(2 独立乘法)
//    beat3(rPipe[2]) Kp*e                        —— 一拍(1 乘法)
//    beat4(rPipe[3]) KiTs2*(e+e[n-1])            —— 一拍(1 乘法)
//    beat5(rPipe[4]) resN 求和 + fn_sat + 积分候选 + 三支并入 raw u —— 一拍(无乘法)
//    beat6(rPipe[5],末拍) 抗饱和冻结+fn_sat/限幅+PI/谐振状态回写 —— 一拍
//  LATENCY=6: iEn 采样后第 6 拍 oValid 拉高一拍, oOut 为新 u[n]。
//  约束: 相邻 iEn 间隔 >= LATENCY 拍(非重叠流水, 数值与原组合式逐样本一致)。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.2 (pipe)
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
`include "algo_arith.vh"

module CompPir #(
    parameter integer       W       = 16,
    parameter integer       QF      = 14,            // 系数 Q 格式
    parameter integer       LATENCY = 6,             // 多拍流水延迟: iEn 采样 -> oValid 有效
    parameter signed [15:0] P_KP    = 16'sd16384,    // Kp        = 1.000    Q14
    parameter signed [15:0] P_KIT2  = 16'sd328,      // Ki*Ts/2   = 0.0200   Q14
    parameter signed [15:0] P_RA1   = -16'sd32743,   // a1        = -1.998498 Q14
    parameter signed [15:0] P_RA2   = 16'sd16363,    // a2        =  0.998744 Q14
    parameter signed [15:0] P_RB0   = 16'sd4115,     // b0        =  0.251160 Q14 (Kr=400)
    parameter signed [15:0] P_RB2   = -16'sd4115,    // b2        = -b0
    parameter signed [31:0] P_OMAX  = 32'sd30000,    // 输出上限
    parameter signed [31:0] P_OMIN  = -32'sd30000    // 输出下限
) (
    input  wire                iSysClk,
    input  wire                iSysRst,       // 高有效复位
    input  wire                iEn,           // 采样使能: 一拍=一采样(相邻 iEn 间隔>=LATENCY)
    input  wire signed [W-1:0] iErrIn,        // 控制误差 e=ref-fbk (S16)
    output reg  signed [W-1:0] oOut,          // 控制器输出 u (S16)
    output reg                 oValid         // iEn 后 LATENCY 拍拉高一拍(新 u 有效)
);

    localparam signed [47:0] L_2QF   = (48'sd1 <<< QF);
    localparam signed [47:0] L_OMaxQ = L_2QF * P_OMAX;
    localparam signed [47:0] L_OMinQ = L_2QF * P_OMIN;

    // ---- PI 状态 ----
    reg signed [47:0] rI;          // ui<<QF (S48)
    reg signed [15:0] rErrP;       // e[n-1]
    // ---- 谐振器状态 ----
    reg signed [15:0] rR1, rR2;    // yr[n-1], yr[n-2]
    reg signed [15:0] rE1, rE2;    // e[n-1], e[n-2]
    reg signed [15:0] rX;          // 本拍采样误差 e[n]

    // ---- 分拍乘积/中间寄存器 ----
    reg signed [47:0] rB0, rB2;    // beat1: b0*e、b2*e[n-2]
    reg signed [47:0] rA1, rA2;    // beat2: -a1*yr[n-1]、-a2*yr[n-2]
    reg signed [47:0] rKpE;        // beat3: Kp*e
    reg signed [47:0] rIt2;        // beat4: KiTs2*(e+e[n-1])
    reg signed [15:0] rRes;        // beat5: yr[n] (S16/Q0)
    reg signed [47:0] rCand;       // beat5: 积分候选
    reg signed [47:0] rAcc;        // beat5: raw u (Q14 域)
    reg [LATENCY-1:0] rPipe;

    wire signed [47:0] wX    = rX;
    wire signed [47:0] wE2   = rE2;
    wire signed [47:0] wR1   = rR1;
    wire signed [47:0] wR2   = rR2;
    wire signed [47:0] wESum = rX + rErrP;                 // e+e[n-1]
    // 组合量(仅末两拍用; 由已寄存乘积组合, 无组合环)
    wire signed [47:0] wResN  = rB0 + rB2 - rA1 - rA2;     // 谐振分子
    wire signed [15:0] wRes16 = fn_sat(wResN, QF[4:0]);
    wire signed [47:0] wResQ  = {{32{wRes16[15]}}, wRes16} <<< QF;  // 谐振并入 Q14 域
    wire signed [47:0] wCand  = rI + rIt2;                 // 积分候选
    wire signed [47:0] wAcc   = rKpE + wCand + wResQ;      // raw u
    wire signed [15:0] wOut16 = fn_sat(rAcc, QF[4:0]);     // S16 输出(末拍用)
    wire signed [31:0] wO32   = {{16{wOut16[15]}}, wOut16};
    wire wFreeze = ((rAcc > L_OMaxQ) && ($signed(rX) > 16'sd0)) ||
                   ((rAcc < L_OMinQ) && ($signed(rX) < 16'sd0));

    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            rI    <= {48{1'b0}};
            rErrP <= {16{1'b0}};
            rR1   <= {16{1'b0}}; rR2 <= {16{1'b0}};
            rE1   <= {16{1'b0}}; rE2 <= {16{1'b0}};
            rX    <= {16{1'b0}};
            rB0   <= {48{1'b0}}; rB2 <= {48{1'b0}};
            rA1   <= {48{1'b0}}; rA2 <= {48{1'b0}};
            rKpE  <= {48{1'b0}}; rIt2 <= {48{1'b0}};
            rRes  <= {16{1'b0}};
            rCand <= {48{1'b0}}; rAcc <= {48{1'b0}};
            rPipe <= {LATENCY{1'b0}};
            oOut  <= {W{1'b0}};
            oValid<= 1'b0;
        end else begin
            // 误差采样一拍 + 采样脉冲移位分拍
            if (iEn) rX <= iErrIn;
            rPipe <= {rPipe[LATENCY-2:0], iEn};

            // beat1: 谐振 b 支路乘法一拍
            if (rPipe[0]) begin
                rB0 <= P_RB0 * wX;
                rB2 <= P_RB2 * wE2;
            end
            // beat2: 谐振 a 反馈乘法一拍
            if (rPipe[1]) begin
                rA1 <= P_RA1 * wR1;
                rA2 <= P_RA2 * wR2;
            end
            // beat3: Kp 支路乘法一拍
            if (rPipe[2])
                rKpE <= P_KP * wX;
            // beat4: PI 梯形增量乘法一拍
            if (rPipe[3])
                rIt2 <= P_KIT2 * wESum;
            // beat5: 求和/饱和一拍(无乘法): 谐振 fn_sat、积分候选、三支并入 raw u
            if (rPipe[4]) begin
                rRes  <= wRes16;
                rCand <= wCand;
                rAcc  <= wAcc;
            end
            // beat6(末拍): 抗饱和冻结 + fn_sat/限幅 + PI/谐振状态回写
            if (rPipe[LATENCY-1]) begin
                if (!wFreeze) rI <= rCand;         // 越限且误差同向 -> 冻结 PI 积分
                rErrP <= rX;
                rR2   <= rR1;
                rR1   <= rRes;
                rE2   <= rE1;
                rE1   <= rX;
                if (wO32 > $signed(P_OMAX))
                    oOut <= P_OMAX[W-1:0];
                else if (wO32 < $signed(P_OMIN))
                    oOut <= P_OMIN[W-1:0];
                else
                    oOut <= wOut16;
            end

            oValid <= rPipe[LATENCY-1];
        end
    end
endmodule
