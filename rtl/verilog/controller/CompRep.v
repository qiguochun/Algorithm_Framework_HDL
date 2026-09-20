//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   CompRep.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.06
/*Description       :   重复控制器 RC (Repetitive Controller) 周期性基波/谐波抑制
//                       连续等价:  G_RC(z)=z^(-N)/(1-Q*z^(-N)),  N=基波周期采样数 fs/f0
//                       差分(按 spec): u[n]=e[n]+Q*u[n-N]; y[n]=u[n-N]
//                       (由 u 递推 + N 拍延迟得 y, 即 y[n]=e[n-N]+Q*y[n-N])
//                       实现: N 深环形缓冲存内部量 u(S32), 每拍先读槽内 u[n-N](输出 y)
//                             再写回 u[n]=e[n]+Q*u[n-N], 指针 +1 环绕。
//                       定点: 信号 e/y 为 S16 计数; Q=P_Q/2^QF (默认 0.9500 Q14);
//                             乘积/累加在 S64 完成, 舍入右移 QF 后回 Q0 S32 存储,
//                             输出经饱和缩至 S16。N 参数化(TB 用 16 加速)。
//                       default: Q=0.95, N=16(fs/f0=16, 例 Fs=16k f0=1k)。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.1
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
module CompRep #(
    parameter integer       W       = 16,
    parameter integer       QF      = 14,          // 系数 Q 格式
    parameter integer       P_N     = 16,          // 基波周期采样数 N=fs/f0
    parameter signed [15:0] P_Q     = 16'sd15565,  // 鲁棒系数 Q = 0.9500  Q14
    parameter signed [15:0] P_RND   = 16'sd8192    // 舍入半值 = 1<<(QF-1) (Q14)
) (
    input  wire                iSysClk,
    input  wire                iSysRst,       // 高有效复位
    input  wire signed [W-1:0] iErrIn,        // 周期误差 e[n] (S16)
    output reg  signed [W-1:0] oOut           // 补偿输出 y[n]=u[n-N] (S16, 一拍=一采样)
);

    localparam integer LWA = (P_N > 1) ? $clog2(P_N) : 1;   // 指针位宽

    // 环形缓冲: 每槽为内部量 u (Q0, S32), 保留 N 拍学习历史
    reg signed [31:0] rBuf [0:P_N-1];
    reg [LWA-1:0] rWr;                                   // 写/读指针
    integer j;

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

    wire signed [63:0] wOld  = rBuf[rWr];              // u[n-N] 读出(输出 y)
    wire signed [63:0] wQ64  = P_Q;                    // Q 系数提升 S64
    wire signed [63:0] wProd = wQ64 * wOld;            // Q*u (Q14 域, S64)
    wire signed [63:0] wTerm = (wProd + P_RND) >>> QF; // 舍入回 Q0: Q*u
    wire signed [63:0] wUNew = wTerm + iErrIn;         // u[n]=e[n]+Q*u[n-N]
    wire signed [31:0] wUSat = (wUNew >  64'sd2147483647) ? 32'sh7FFFFFFF :
                               ((wUNew < -64'sd2147483648) ? 32'sh80000000 : wUNew[31:0]);
    wire signed [15:0] wYSat = fn_sat16_64(wOld, 7'd0); // y[n] = u[n-N] (S16 饱和)

    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            for (j = 0; j < P_N; j = j + 1)
                rBuf[j] <= {32{1'b0}};
            rWr  <= {LWA{1'b0}};
            oOut <= {W{1'b0}};
        end else begin
            oOut      <= wYSat;                         // y[n] = u[n-N]
            rBuf[rWr] <= wUSat;                         // S32 饱和写回，防绕回
            if (rWr == (P_N - 1))
                rWr <= {LWA{1'b0}};
            else
                rWr <= rWr + 1'b1;
        end
    end
endmodule
