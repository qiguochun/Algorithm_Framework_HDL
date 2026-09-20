//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   CompPr.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.06
/*Description       :   准 PR(比例谐振)控制器, 用于基波/特定次谐波跟踪
//                       连续:  C(s)=Kp + 2*Kr*wc*s/(s^2+2*wc*s+w0^2)
//                       双线性离散(s=c*(z-1)/(z+1), c=2/Ts)二阶谐振器:
//                       c^2=A0 归一化后
//                         y[n]=b0*e[n]+b2*e[n-2]-a1*y[n-1]-a2*y[n-2],  b2=-b0
//                       a1=(2w0^2-2c^2)/A0, a2=(c^2-2wc*c+w0^2)/A0,
//                       b0=Kr*(2wc*c)/A0   (Kr 折入分子系数)
//                       Kp 支路并联另加: u[n]=Kp*e[n]+y[n]
//                       定点: 系数均 S16/Q14(QF)参数, 中间 48bit, fn_sat 舍入 Q14 后 S16。
//                       default: Kp=1.0, f0=50Hz(w0=2*pi*50), wc=2*pi*2 rad/s,
//                                Fs=20kHz, Kr=400。
//  ----------------------------------------------------------------------
//  多拍流水(消除单周期组合乘加链): 每 iEn 采样一拍, 谐振器 4 个乘积分两拍各自
//  独立寄存, Kp 乘积一拍, 求和一拍, 饱和/求和一拍, 末拍限幅+状态回写:
//    beat1(rPipe[0]) b0*e、b2*e[n-2]            —— 一拍(2 独立乘法, 各自寄存)
//    beat2(rPipe[1]) -a1*y[n-1]、-a2*y[n-2]     —— 一拍(2 独立乘法, 各自寄存)
//    beat3(rPipe[2]) Kp*e                        —— 一拍(1 乘法)
//    beat4(rPipe[3]) 谐振分子求和 resN           —— 一拍(无乘法)
//    beat5(rPipe[4]) fn_sat(resN)/fn_sat(Kp*e)+求和为 S32 —— 一拍
//    beat6(rPipe[5],末拍) 限幅+谐振/误差状态回写+oOut —— 一拍
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

module CompPr #(
    parameter integer       W       = 16,
    parameter integer       QF      = 14,            // 系数 Q 格式
    parameter integer       LATENCY = 6,             // 多拍流水延迟: iEn 采样 -> oValid 有效
    parameter signed [15:0] P_KP    = 16'sd16384,    // Kp        = 1.000    Q14
    parameter signed [15:0] P_RA1   = -16'sd32743,   // a1 = A1/A0= -1.998498 Q14
    parameter signed [15:0] P_RA2   = 16'sd16363,    // a2 = A2/A0=  0.998744 Q14
    parameter signed [15:0] P_RB0   = 16'sd4115,     // b0 = Kr*B0/A0=0.251160 Q14 (Kr=400)
    parameter signed [15:0] P_RB2   = -16'sd4115,    // b2 = -b0
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

    // ---- 谐振器/误差状态 ----
    reg signed [15:0] rR1, rR2;   // 谐振器输出 y[n-1], y[n-2]
    reg signed [15:0] rE1, rE2;   // e[n-1], e[n-2]
    reg signed [15:0] rX;         // 本拍采样误差 e[n] (iEn 锁定)

    // ---- 分拍乘积/中间寄存器 ----
    reg signed [47:0] rB0;        // beat1: b0*e
    reg signed [47:0] rB2;        // beat1: b2*e[n-2]
    reg signed [47:0] rA1;        // beat2: -a1*y[n-1] (系数为负, 符号入乘积)
    reg signed [47:0] rA2;        // beat2: -a2*y[n-2]
    reg signed [47:0] rKpE;       // beat3: Kp*e
    reg signed [47:0] rResN;      // beat4: 谐振分子 sum
    reg signed [15:0] rRes;       // beat5: fn_sat(resN) = y[n] (S16/Q0)
    reg signed [31:0] rSum;       // beat5: u = Kp支路 + 谐振支路 (S32)
    reg [LATENCY-1:0] rPipe;

    wire signed [47:0] wX  = rX;
    wire signed [47:0] wE2 = rE2;
    wire signed [47:0] wR1 = rR1;
    wire signed [47:0] wR2 = rR2;
    // 组合(仅用于对应分拍寄存): 各自 fn_sat 后 S16
    wire signed [15:0] wRes16 = fn_sat(rResN, QF[4:0]);
    wire signed [15:0] wKp16  = fn_sat(rKpE,  QF[4:0]);
    wire signed [31:0] wSum   = {{16{wKp16[15]}}, wKp16} + {{16{wRes16[15]}}, wRes16};

    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            rR1 <= {16{1'b0}}; rR2 <= {16{1'b0}};
            rE1 <= {16{1'b0}}; rE2 <= {16{1'b0}};
            rX  <= {16{1'b0}};
            rB0 <= {48{1'b0}}; rB2 <= {48{1'b0}};
            rA1 <= {48{1'b0}}; rA2 <= {48{1'b0}};
            rKpE<= {48{1'b0}};
            rResN<= {48{1'b0}};
            rRes <= {16{1'b0}};
            rSum <= {32{1'b0}};
            rPipe<= {LATENCY{1'b0}};
            oOut <= {W{1'b0}};
            oValid<= 1'b0;
        end else begin
            // 误差采样一拍 + 采样脉冲移位分拍
            if (iEn) rX <= iErrIn;
            rPipe <= {rPipe[LATENCY-2:0], iEn};

            // beat1: 谐振器 b 支路乘法一拍(b0*e、b2*e[n-2], 各自独立寄存)
            if (rPipe[0]) begin
                rB0 <= P_RB0 * wX;
                rB2 <= P_RB2 * wE2;
            end

            // beat2: 谐振器 a 反馈乘法一拍(-a1*y[n-1]、-a2*y[n-2])
            if (rPipe[1]) begin
                rA1 <= P_RA1 * wR1;
                rA2 <= P_RA2 * wR2;
            end

            // beat3: Kp 支路乘法一拍
            if (rPipe[2])
                rKpE <= P_KP * wX;

            // beat4: 谐振分子求和一拍(无乘法)
            if (rPipe[3])
                rResN <= rB0 + rB2 - rA1 - rA2;

            // beat5: 两路各自 fn_sat(舍入 Q14->Q0/S16)后相加一拍
            if (rPipe[4]) begin
                rRes <= wRes16;
                rSum <= wSum;
            end

            // beat6(末拍): 限幅 + 谐振/误差状态回写 + 输出
            if (rPipe[LATENCY-1]) begin
                if ($signed(rSum) > P_OMAX)
                    oOut <= P_OMAX[W-1:0];
                else if ($signed(rSum) < P_OMIN)
                    oOut <= P_OMIN[W-1:0];
                else
                    oOut <= rSum[W-1:0];
                rR2 <= rR1;
                rR1 <= rRes;
                rE2 <= rE1;
                rE1 <= rX;
            end

            oValid <= rPipe[LATENCY-1];
        end
    end
endmodule
