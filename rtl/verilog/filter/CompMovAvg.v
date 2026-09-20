//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   CompMovAvg.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.06
/*Description       :   N 点滑动平均（FIR，N 必须为 2 的幂）。
                        y[n]=(1/N)*sum_{k=0..N-1} x[n-k]，N 默认 16。
                        移位寄存器(深度 N)缓冲窗 + 运行和 rSum 实现：
                        每拍 rSum = rSum - x[n-N] + x[n]（去最旧 rWin[N-1]，加最新 iXIn），
                        y = (rSum+舍入)>>log2N；平均除法用算术右移 log2N（无组合除法）。
                        信号 S0.(W-1) 有符号（N=2^k 平均除法=移位，不受截断偏置影响）；
                        输出饱和限幅。直流输入 N 拍后输出精确等于输入（增益=1）。
                        相位/群延时=(N-1)/2 拍；平滑噪声方差 /N。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.1
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
module CompMovAvg #(
    parameter integer W  = 16,     // 信号位宽(有符号)
    parameter integer N  = 16      // 窗长（必须为 2 的幂；平均用移位）
) (
    input  wire                iSysClk,  // 时钟
    input  wire                iSysRst,  // 复位（高有效）
    input  wire                iEn,      // 采样节拍（每拍一采样）
    input  wire signed [W-1:0] iXIn,     // 输入采样 x[n]
    output reg  signed [W-1:0] oYOut     // 滑动平均输出 y[n]
);
    localparam integer LOG2N    = $clog2(N);          // N=2^LOG2N，右移位数
    localparam integer SW       = W + LOG2N + 1;      // 运行和位宽（含极端负和余量）
    localparam signed [63:0] YMAX = (64'sd1 << (W-1)) - 1;  // 输出上限
    localparam signed [63:0] YMIN = -(64'sd1 << (W-1));     // 输出下限

    // ---- 内部状态 ----
    reg signed [SW-1:0] rSum;               // 窗内 N 点运行和
    reg signed [W-1:0]  rWin [0:N-1];       // 窗寄存器：rWin[0] 最新，rWin[N-1] 最旧

    // ---- 组合临时量（阻塞算好，末尾一次非阻塞写） ----
    reg signed [SW-1:0] t_sum;      // 更新后运行和
    reg signed [63:0]   t_avg;      // 舍入后均值(未饱和)
    reg signed [63:0]   rnd;        // +0.5 LSB 舍入加数
    reg signed [63:0]   t_o;        // 输出(饱和前)
    integer k;

    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            rSum <= {SW{1'b0}};
            oYOut <= {W{1'b0}};
            for (k = 0; k < N; k = k + 1)
                rWin[k] <= {W{1'b0}};
        end else if (iEn) begin
            // 运行和：加最新 x[n]，减最旧 x[n-N]
            t_sum = rSum + $signed(iXIn) - $signed(rWin[N-1]);
            // y=(rSum+0.5*2^LOG2N)>>LOG2N，舍入为 0.5 LSB
            rnd   = 64'sd1 << (LOG2N - 1);
            t_avg = ($signed(t_sum) + rnd) >>> LOG2N;
            if (t_avg > YMAX)      t_o = YMAX;
            else if (t_avg < YMIN) t_o = YMIN;
            else                   t_o = t_avg;
            // 窗移位：新样本入 [0]，其余整体后移（[k] <- [k-1]）
            rWin[0] <= $signed(iXIn);
            for (k = N - 1; k >= 1; k = k - 1)
                rWin[k] <= rWin[k-1];
            rSum  <= t_sum;
            oYOut <= t_o[W-1:0];
        end
    end
endmodule
