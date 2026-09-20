//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   CompNotch.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
/*Description       :   II 型双线性陷波滤波器。
                        系数公式: x=2c2 wnTs, y=2c1 wnTs, z=(wnTs)^2
                        a1=2-y, a2=y-z-1, b1=x-2, b2=z-x+1
                        y[n]=a1 y[n-1]+a2 y[n-2]+x[n]+b1 x[n-1]+b2 x[n-2]
                        a/b 系数由 parameter FS/FC/C1_MILLI/C2_MILLI 在 elabor 期算成 Q14
                        （默认 fc=100Hz, Fs=5kHz, c1=0.5, c2=0）。
                        DC 增益=1，中心频率深度陷波。
                        【多拍流水】乘法一拍(4 个乘积+输入抬位并行)、逐项求和各一拍、
                        Q14 舍入+饱和+状态更新一拍，共 LATENCY=6 拍；iEn 单拍脉冲采样一帧，
                        完成后 oValid 高。数值与单周期组合实现逐位等价（仅延后 LATENCY 拍）。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.2 (pipelined LATENCY=6)
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
module CompNotch #(
    parameter integer W        = 16,
    parameter integer FS       = 5000,   // 采样率 Hz
    parameter integer FC       = 100,    // 陷波中心频率 Hz
    parameter integer C1_MILLI = 500,    // c1*1000（阻尼，默认 0.5）
    parameter integer C2_MILLI = 0,      // c2*1000（零点偏移，默认 0）
    parameter integer FSHIFT   = 14      // a/b 系数 Q 格式
) (
    input  wire                iSysClk,
    input  wire                iSysRst,
    input  wire                iEn,        // 采样节拍（单拍脉冲：一帧一个采样）
    input  wire signed [W-1:0] iXIn,
    output reg  signed [W-1:0] oY,         // 陷波后输出（=直流直通）
    output reg                 oValid      // 本帧完成标志（iEn 后 LATENCY 拍拉高一拍）
);
    `include "algo_filt_coef.vh"
    localparam signed [15:0] P_A1 = fn_notch_a1(FS, FC, C1_MILLI, FSHIFT);
    localparam signed [15:0] P_A2 = fn_notch_a2(FS, FC, C1_MILLI, FSHIFT);
    localparam signed [15:0] P_B1 = fn_notch_b1(FS, FC, C2_MILLI, FSHIFT);
    localparam signed [15:0] P_B2 = fn_notch_b2(FS, FC, C2_MILLI, FSHIFT);

    localparam integer LATENCY = 6;                        // 二阶 IIR 流水拍数
    localparam signed [63:0] HALF = (64'sd1 <<< 13);       // Q14 舍入 +0.5 LSB

    reg signed [W-1:0] rY1, rY2;   // y[n-1], y[n-2]
    reg signed [W-1:0] rX1, rX2;   // x[n-1], x[n-2]

    // ---- 流水中间量 ----
    reg signed [63:0] rT1;         // a1*y[n-1]
    reg signed [63:0] rT2;         // a2*y[n-2]
    reg signed [63:0] rT3;         // b1*x[n-1]
    reg signed [63:0] rT4;         // b2*x[n-2]
    reg signed [63:0] rXT;         // x[n]<<14
    reg signed [63:0] rS1, rS2, rS3, rS4;  // 逐步累加
    reg [LATENCY-1:0] rPipe;       // 采样脉冲移位

    // ---- Q14 舍入+算术右移+S16 饱和（等价 fn_sat(…,14)，无乘法） ----
    function automatic signed [W-1:0] f_sat14(input signed [63:0] a);
        reg signed [63:0] ra;
        begin
            ra = a + HALF;
            ra = ra >>> 14;
            if (ra > 32767)   ra = 32767;
            if (ra < -32768)  ra = -32768;
            f_sat14 = ra[W-1:0];
        end
    endfunction

    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            rY1   <= {W{1'b0}};
            rY2   <= {W{1'b0}};
            rX1   <= {W{1'b0}};
            rX2   <= {W{1'b0}};
            rT1   <= 64'sd0;
            rT2   <= 64'sd0;
            rT3   <= 64'sd0;
            rT4   <= 64'sd0;
            rXT   <= 64'sd0;
            rS1   <= 64'sd0;
            rS2   <= 64'sd0;
            rS3   <= 64'sd0;
            rS4   <= 64'sd0;
            rPipe <= {LATENCY{1'b0}};
            oY    <= {W{1'b0}};
            oValid<= 1'b0;
        end else begin
            oValid <= 1'b0;
            // ---- 拍0（iEn 采样）：4 个乘积 + 输入抬位并行一拍，x 历史后移 ----
            if (iEn && (rPipe == {LATENCY{1'b0}})) begin
                rT1  <= $signed(P_A1) * rY1;
                rT2  <= $signed(P_A2) * rY2;
                rT3  <= $signed(P_B1) * rX1;
                rT4  <= $signed(P_B2) * rX2;
                rXT  <= $signed(iXIn) << 14;
                rX2  <= rX1;
                rX1  <= iXIn;
                rPipe <= {{LATENCY-1{1'b0}}, 1'b1};
            end
            // ---- 拍1~拍4：逐项求和 ----
            if (rPipe[0]) begin
                rS1  <= rT1 + rT2;
                rPipe <= (rPipe[LATENCY-2]) ? {LATENCY{1'b0}} : (rPipe << 1);
            end
            if (rPipe[1]) begin
                rS2  <= rS1 + rT3;
                rPipe <= (rPipe[LATENCY-2]) ? {LATENCY{1'b0}} : (rPipe << 1);
            end
            if (rPipe[2]) begin
                rS3  <= rS2 + rT4;
                rPipe <= (rPipe[LATENCY-2]) ? {LATENCY{1'b0}} : (rPipe << 1);
            end
            if (rPipe[3]) begin
                rS4  <= rS3 + rXT;
                rPipe <= (rPipe[LATENCY-2]) ? {LATENCY{1'b0}} : (rPipe << 1);
            end
            // ---- 拍5（末拍）：Q14 舍入饱和 + 状态/输出更新 + oValid ----
            if (rPipe[LATENCY-2]) begin
                rY2   <= rY1;
                rY1   <= f_sat14(rS4);
                oY    <= f_sat14(rS4);
                oValid<= 1'b1;
                rPipe <= {LATENCY{1'b0}};
            end
        end
    end
endmodule
