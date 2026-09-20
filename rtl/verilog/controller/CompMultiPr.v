//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   CompMultiPr.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.06
/*Description       :   多谐振 PR 控制器(基波 + 5/7 次谐波并联跟踪)
//                       连续: C(s)=Kp + SUM_k 2*Kr_k*wc_k*s/(s^2+2*wc_k*s+w_k^2)
//                             w_k = k*w0, k = 1,5,7
//                       每谐振项双线性离散(c=2/Ts)为二阶 IIR 后并联:
//                         yk[n]=b0_k*e[n]+b2_k*e[n-2]-a1_k*yk[n-1]-a2_k*yk[n-2], b2_k=-b0_k
//                         a1_k=(2w_k^2-2c^2)/L, a2_k=(c^2-2wc_k*c+w_k^2)/L,
//                         b0_k=2*Kr_k*wc_k*c/L,  L=c^2+2*wc_k*c+w_k^2
//                       u[n]=sat(Kp*e[n]+y1[n]+y5[n]+y7[n])
//                       定点: e/y 为 S16 计数, 系数均 S16/Q14(QF)参数(注释物理值),
//                             每项计算在 S64 累加, fn 舍入右移 QF 后 S16; 输出限幅。
//                       物理默认: Fs=20kHz, f0=50Hz(w0=2*pi*50), wc=2*pi*2 rad/s,
//                                 Kr1=Kr5=Kr7=400(折入 b0)。
//                       k=1 : a1=-1.998498 a2=0.998744 b0=0.251154
//                       k=5 : a1=-1.992591 a2=0.998746 b0=0.250783
//                       k=7 : a1=-1.986702 a2=0.998748 b0=0.250413
//  ----------------------------------------------------------------------
//  多拍流水(消除单周期组合乘加链): 各乘积一拍且独立寄存, 求和/饱和/回写各一拍:
//    beat1(rPipe[0]) b0_1*e、b0_5*e、b0_7*e              —— 一拍(3 独立乘法)
//    beat2(rPipe[1]) b2_1*e[n-2]、b2_5*e[n-2]、b2_7*e[n-2] —— 一拍(3 独立乘法)
//    beat3(rPipe[2]) -a1_1*y1[n-1]、-a1_5*y5[n-1]、-a1_7*y7[n-1] —— 一拍(3 乘法)
//    beat4(rPipe[3]) -a2_1*y1[n-2]、-a2_5*y5[n-2]、-a2_7*y7[n-2] + Kp*e —— 一拍(4 乘法)
//    beat5(rPipe[4]) 三个谐振分子求和 + fn_sat16_64 + Kp 饱和 + 四支求和 —— 一拍(无乘法)
//    beat6(rPipe[5],末拍) 限幅 + 三谐振状态回写 + oOut —— 一拍
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
module CompMultiPr #(
    parameter integer       W       = 16,
    parameter integer       QF      = 14,            // 系数 Q 格式
    parameter integer       LATENCY = 6,             // 多拍流水延迟: iEn 采样 -> oValid 有效
    // Kp 支路
    parameter signed [15:0] P_KP    = 16'sd16384,    // Kp        = 1.000    Q14
    // k=1 基波 50Hz 谐振器
    parameter signed [15:0] P_RA1_1 = -16'sd32743,   // a1        = -1.998498 Q14
    parameter signed [15:0] P_RA2_1 =  16'sd16363,   // a2        =  0.998744 Q14
    parameter signed [15:0] P_RB0_1 =  16'sd4115,    // b0(Kr=400)= 0.251154 Q14
    parameter signed [15:0] P_RB2_1 = -16'sd4115,    // b2        = -b0
    // k=5 五次 250Hz 谐振器
    parameter signed [15:0] P_RA1_5 = -16'sd32647,   // a1        = -1.992591 Q14
    parameter signed [15:0] P_RA2_5 =  16'sd16363,   // a2        =  0.998746 Q14
    parameter signed [15:0] P_RB0_5 =  16'sd4109,    // b0(Kr=400)= 0.250783 Q14
    parameter signed [15:0] P_RB2_5 = -16'sd4109,    // b2        = -b0
    // k=7 七次 350Hz 谐振器
    parameter signed [15:0] P_RA1_7 = -16'sd32550,   // a1        = -1.986702 Q14
    parameter signed [15:0] P_RA2_7 =  16'sd16363,   // a2        =  0.998748 Q14
    parameter signed [15:0] P_RB0_7 =  16'sd4103,    // b0(Kr=400)= 0.250413 Q14
    parameter signed [15:0] P_RB2_7 = -16'sd4103,    // b2        = -b0
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

    // 64 位饱和收缩: S64 右移 fb(先 +half 舍入) 后钳位到 S16
    function signed [15:0] fn_sat16_64;
        input signed [63:0] a;
        input [6:0]         fb;
        reg signed [63:0] ra, half;
        begin
            half = (fb > 0) ? (64'sd1 << (fb - 1)) : 64'sd0;
            ra  = a + half;
            ra  = ra >>> fb;
            if (ra >  32767) ra =  32767;
            if (ra < -32768) ra = -32768;
            fn_sat16_64 = ra[15:0];
        end
    endfunction

    // ---- 三谐振器状态 ----
    reg signed [15:0] rR1_1, rR2_1, rE1_1, rE2_1;   // k=1
    reg signed [15:0] rR1_5, rR2_5, rE1_5, rE2_5;   // k=5
    reg signed [15:0] rR1_7, rR2_7, rE1_7, rE2_7;   // k=7
    reg signed [15:0] rX;                           // 本拍采样误差 e[n]

    // ---- 分拍乘积(S64)/中间寄存器 ----
    reg signed [63:0] rB01, rB05, rB07;    // beat1: b0_k*e
    reg signed [63:0] rB21, rB25, rB27;    // beat2: b2_k*e[n-2]
    reg signed [63:0] rA11, rA15, rA17;    // beat3: -a1_k*yk[n-1]
    reg signed [63:0] rA21, rA25, rA27;    // beat4: -a2_k*yk[n-2]
    reg signed [63:0] rKpE;                // beat4: Kp*e
    reg signed [15:0] rRes1, rRes5, rRes7; // beat5: 各谐振 yk (S16/Q0)
    reg signed [63:0] rAcc;                // beat5: u = Kp支路+三谐振 (S64)
    reg [LATENCY-1:0] rPipe;

    wire signed [63:0] wX     = rX;
    wire signed [63:0] wE2_1  = rE2_1;
    wire signed [63:0] wE2_5  = rE2_5;
    wire signed [63:0] wE2_7  = rE2_7;
    wire signed [63:0] wR1_1  = rR1_1;
    wire signed [63:0] wR1_5  = rR1_5;
    wire signed [63:0] wR1_7  = rR1_7;
    wire signed [63:0] wR2_1  = rR2_1;
    wire signed [63:0] wR2_5  = rR2_5;
    wire signed [63:0] wR2_7  = rR2_7;
    // 组合量(beat5 用; 由已寄存乘积组合, 无组合环): 各谐振分子 -> S16/Q0
    wire signed [63:0] wResN1 = rB01 + rB21 - rA11 - rA21;
    wire signed [63:0] wResN5 = rB05 + rB25 - rA15 - rA25;
    wire signed [63:0] wResN7 = rB07 + rB27 - rA17 - rA27;
    wire signed [15:0] wRes1  = fn_sat16_64(wResN1, QF[6:0]);
    wire signed [15:0] wRes5  = fn_sat16_64(wResN5, QF[6:0]);
    wire signed [15:0] wRes7  = fn_sat16_64(wResN7, QF[6:0]);
    wire signed [15:0] wKpE   = fn_sat16_64(rKpE, QF[6:0]);
    wire signed [63:0] wAcc   = wKpE + wRes1 + wRes5 + wRes7;

    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            rR1_1 <= 16'sd0; rR2_1 <= 16'sd0; rE1_1 <= 16'sd0; rE2_1 <= 16'sd0;
            rR1_5 <= 16'sd0; rR2_5 <= 16'sd0; rE1_5 <= 16'sd0; rE2_5 <= 16'sd0;
            rR1_7 <= 16'sd0; rR2_7 <= 16'sd0; rE1_7 <= 16'sd0; rE2_7 <= 16'sd0;
            rX    <= 16'sd0;
            rB01 <= 64'sd0; rB05 <= 64'sd0; rB07 <= 64'sd0;
            rB21 <= 64'sd0; rB25 <= 64'sd0; rB27 <= 64'sd0;
            rA11 <= 64'sd0; rA15 <= 64'sd0; rA17 <= 64'sd0;
            rA21 <= 64'sd0; rA25 <= 64'sd0; rA27 <= 64'sd0;
            rKpE <= 64'sd0;
            rRes1 <= 16'sd0; rRes5 <= 16'sd0; rRes7 <= 16'sd0;
            rAcc <= 64'sd0;
            rPipe <= {LATENCY{1'b0}};
            oOut <= {W{1'b0}};
            oValid <= 1'b0;
        end else begin
            // 误差采样一拍 + 采样脉冲移位分拍
            if (iEn) rX <= iErrIn;
            rPipe <= {rPipe[LATENCY-2:0], iEn};

            // beat1: 各谐振 b0 项乘法一拍(3 独立乘法, 各自寄存)
            if (rPipe[0]) begin
                rB01 <= P_RB0_1 * wX;
                rB05 <= P_RB0_5 * wX;
                rB07 <= P_RB0_7 * wX;
            end
            // beat2: 各谐振 b2 项乘法一拍
            if (rPipe[1]) begin
                rB21 <= P_RB2_1 * wE2_1;
                rB25 <= P_RB2_5 * wE2_5;
                rB27 <= P_RB2_7 * wE2_7;
            end
            // beat3: 各谐振 a1 反馈乘法一拍
            if (rPipe[2]) begin
                rA11 <= P_RA1_1 * wR1_1;
                rA15 <= P_RA1_5 * wR1_5;
                rA17 <= P_RA1_7 * wR1_7;
            end
            // beat4: 各谐振 a2 反馈乘法 + Kp 乘法一拍
            if (rPipe[3]) begin
                rA21 <= P_RA2_1 * wR2_1;
                rA25 <= P_RA2_5 * wR2_5;
                rA27 <= P_RA2_7 * wR2_7;
                rKpE <= P_KP * wX;
            end
            // beat5: 求和/饱和一拍(无乘法): 各谐振分子->S16、Kp 饱和、四支求和
            if (rPipe[4]) begin
                rRes1 <= wRes1;
                rRes5 <= wRes5;
                rRes7 <= wRes7;
                rAcc  <= wAcc;
            end
            // beat6(末拍): 限幅 + 三谐振状态回写 + 输出
            if (rPipe[LATENCY-1]) begin
                if ($signed(rAcc) > P_OMAX)
                    oOut <= P_OMAX[W-1:0];
                else if ($signed(rAcc) < P_OMIN)
                    oOut <= P_OMIN[W-1:0];
                else
                    oOut <= rAcc[W-1:0];
                rR2_1 <= rR1_1; rR1_1 <= rRes1; rE2_1 <= rE1_1; rE1_1 <= rX;
                rR2_5 <= rR1_5; rR1_5 <= rRes5; rE2_5 <= rE1_5; rE1_5 <= rX;
                rR2_7 <= rR1_7; rR1_7 <= rRes7; rE2_7 <= rE1_7; rE1_7 <= rX;
            end

            oValid <= rPipe[LATENCY-1];
        end
    end
endmodule
