//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   CompSogi.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
/*Description       :   二阶广义积分器（SOGI）正交分量生成器（Tustin 隐式联立版）。
                        A=(Ts/2)kwn, B=(Ts/2)wn, D=1+A+B^2；
                        va[n]=[(1-A-B^2)va[n-1]+2B*vb[n-1]+A(x[n]+x[n-1])]/D；
                        vb[n]=vb[n-1]-B(va[n]+va[n-1])。
                        系数折叠为 Q15 参数（默认 fc=50Hz, Fs=5kHz, k=sqrt2）。oVAlpha 同相、oVBeta 滞后 90。
                        【多拍流水版】iEn 采样节拍触发一拍，组合乘加链拆成 LATENCY 拍：
                        拍1 va 三乘法并行 -> 拍2 求和 -> 拍3 va 饱和 -> 拍4 vb 乘法+位移
                        -> 拍5 vb 求差 -> 拍6 vb 饱和 -> 拍7 状态更新(提交)。oValid 在采样拍后 LATENCY 拍拉高一拍。
                        数值与原单周期组合版逐拍等价（同一采样序列下逐样本输出一致）。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.2  (多拍流水化)
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
`include "algo_arith.vh"

module CompSogi #(
    parameter integer W      = 16,
    parameter signed [15:0] C_VA_VA = 16'sd29921,  // (1-A-B^2)/D = 0.913114 Q15
    parameter signed [15:0] C_VA_VB = 16'sd1969,   // 2B/D        = 0.060102 Q15
    parameter signed [15:0] C_VA_X  = 16'sd1393,   // A/D         = 0.042499 Q15
    parameter signed [15:0] C_VB    = 16'sd1029,   // B           = 0.031416 Q15
    parameter integer LATENCY = 6                  // 流水拍数(>=6)
) (
    input  wire                   iSysClk,
    input  wire                   iSysRst,
    input  wire                   iEn,            // 采样节拍（一拍）
    input  wire signed [W-1:0]    iXIn,
    output reg  signed [W-1:0]    oVAlpha,   // 同相分量（带通于 wn）
    output reg  signed [W-1:0]    oVBeta,    // 正交分量（滞后 90°）
    output reg                    oValid     // 采样后 LATENCY 拍拉高一拍
);
    // ---- 采样拍移位链：rPipe[LATENCY-1] 即提交/输出拍 ----
    reg [LATENCY-1:0] rPipe;

    // 逐拍中间量（乘法/加法/饱和各自一拍，独立寄存）
    reg signed [W-1:0] rXc;      // 输入快照（本样本）
    reg signed [W-1:0] rXPrev;   // x[n-1]
    reg signed [63:0]  rP1, rP2, rP3;   // va 三个乘法结果（并行一拍）
    reg signed [63:0]  rSum;            // va 求和
    reg signed [W-1:0] rVaNew;          // va 饱和
    reg signed [63:0]  rQ;              // vb 乘法结果
    reg signed [63:0]  rSh;             // vb 位移 (vb<<15)
    reg signed [63:0]  rVbN;            // vb 求差
    reg signed [W-1:0] rVbSat;          // vb 饱和

    // ---- 提交中间量在每拍使能后处于稳定段，经一次非阻塞写入 ----
    // 饱和为组合函数，在时序块外以 wire 计算，避免 BLKSEQ
    wire signed [W-1:0] wVaSat = fn_sat(rSum[47:0], 5'd15);
    wire signed [W-1:0] wVbSat = fn_sat(rVbN[47:0], 5'd15);

    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            rPipe   <= {LATENCY{1'b0}};
            rXc     <= {W{1'b0}};
            rXPrev  <= {W{1'b0}};
            rP1     <= 64'sd0; rP2 <= 64'sd0; rP3 <= 64'sd0;
            rSum    <= 64'sd0;
            rVaNew  <= {W{1'b0}};
            rQ      <= 64'sd0; rSh <= 64'sd0;
            rVbN    <= 64'sd0;
            rVbSat  <= {W{1'b0}};
            oVAlpha <= {W{1'b0}};
            oVBeta  <= {W{1'b0}};
            oValid  <= 1'b0;
        end else begin
            // 采样拍移位
            rPipe[0] <= iEn;
            if (LATENCY > 1)
                rPipe[LATENCY-1:1] <= rPipe[LATENCY-2:0];
            // 拍1（采样拍）：快照输入 + va 三路乘法（并行）
            if (iEn) begin
                rXc  <= iXIn;
                rP1  <= $signed(oVAlpha) * C_VA_VA;
                rP2  <= $signed(oVBeta)  * C_VA_VB;
                rP3  <= ($signed(iXIn) + $signed(rXPrev)) * C_VA_X;
            end
            // 拍2：va 求和
            if (rPipe[0]) rSum <= rP1 + rP2 + rP3;
            // 拍3：va 饱和（Q15 舍入收缩）
            if (rPipe[1]) rVaNew <= wVaSat;
            // 拍4：vb 乘法 + vb<<15（并行）
            if (rPipe[2]) begin
                rQ  <= ($signed(rVaNew) + $signed(oVAlpha)) * C_VB;
                rSh <= $signed(oVBeta) <<< 15;
            end
            // 拍5：vb 求差
            if (rPipe[3]) rVbN <= rSh - rQ;
            // 拍6：vb 饱和
            if (rPipe[4]) rVbSat <= wVbSat;
            // 拍7（提交拍）：状态更新 + 输出
            if (rPipe[LATENCY-1]) begin
                oVAlpha <= rVaNew;
                oVBeta  <= rVbSat;
                rXPrev  <= rXc;
            end
            oValid <= rPipe[LATENCY-1];
        end
    end
endmodule
