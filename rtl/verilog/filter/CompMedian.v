//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   CompMedian.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.06
/*Description       :   中值滤波器（窗口 N=3 或 5，组合比较排序，无时钟流水延迟）。
                        y[n]=median{x[n], x[n-1], ..., x[n-N+1]}，N 为奇数参数化。
                        移位寄存器(深度 N)缓冲窗；组合逻辑逐元素统计比它小/大的个数
                        (lessCnt<=N/2 且 greatCnt<=N/2 者即中位值，并列取相同值)，
                        一次性排序取中值，无迭代、无除法。
                        信号 S0.(W-1) 有符号；对单样本脉冲毛刺完全免疫
                        (窗内 N-1 个正常值占多数 -> 中值不受毛刺影响)，同时保留阶跃边沿。
                        输出为当前窗中值的纯组合映射（窗口在 iEn 更新）。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.1
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
module CompMedian #(
    parameter integer W = 16,     // 信号位宽(有符号)
    parameter integer N = 5       // 窗口长度（奇数 3 或 5）
) (
    input  wire                iSysClk,  // 时钟
    input  wire                iSysRst,  // 复位（高有效）
    input  wire                iEn,      // 采样节拍（每拍一采样）
    input  wire signed [W-1:0] iXIn,     // 输入采样 x[n]
    output wire signed [W-1:0] oYOut     // 中值输出（组合，等于窗内中位值）
);
    // ---- 内部状态：窗寄存器 [0]=最新，[N-1]=最旧 ----
    reg signed [W-1:0] rWin [0:N-1];

    // ---- 组合中值：统计比 rWin[i] 小/大的个数，满足 (N-1)/2 个以内者取中位 ----
    reg signed [W-1:0] tMed;
    integer i, j;
    integer lessCnt;
    integer greatCnt;

    always @(*) begin
        tMed = rWin[0];
        for (i = 0; i < N; i = i + 1) begin
            lessCnt  = 0;
            greatCnt = 0;
            for (j = 0; j < N; j = j + 1) begin
                if (rWin[j] < rWin[i])
                    lessCnt = lessCnt + 1;
                else if (rWin[j] > rWin[i])
                    greatCnt = greatCnt + 1;
            end
            // N 为奇数：中位值元素其两侧元素数均 <= (N-1)/2=N/2(整除)
            if (lessCnt <= (N >> 1) && greatCnt <= (N >> 1))
                tMed = rWin[i];
        end
    end

    assign oYOut = tMed;

    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            for (i = 0; i < N; i = i + 1)
                rWin[i] <= {W{1'b0}};
        end else if (iEn) begin
            rWin[0] <= $signed(iXIn);
            for (i = N - 1; i >= 1; i = i - 1)
                rWin[i] <= rWin[i-1];
        end
    end
endmodule
