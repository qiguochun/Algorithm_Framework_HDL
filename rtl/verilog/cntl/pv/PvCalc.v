//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   PvCalc.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
/*Description       :   环路数据调理：反馈量一阶EMA低通输出，供观测/滤波用
//                       多拍流水改造: 采样节拍快照输入 -> delta(一拍) ->
//                       step=delta>>EMA_SHIFT(一拍) -> 加旧状态+限幅提交(一拍)。
//                       oValid 提交拍拉高; 数值与原单周期实现一致。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.2
//modifier          :
//Modify Date       :
/*Modify Record     :
//  Rev 0.2: 组合乘加链拆分 rEv 多拍流水, 新增 oValid。
*/
//------------------------------------------------------------------------------
`include "algo_types.vh"

module PvCalc #(
    parameter integer W         = `ALGO_W_VOLT,
    parameter integer EMA_SHIFT = 9          // alpha = 1/512
) (
    input  wire         iSysClk,
    input  wire         iSysRst,       // 高有效复位
    input  wire         iSampleEn,     // 控制周期 strobe
    input  wire [W-1:0] iVIn,          // 瞬时反馈（电压）
    output reg  [W-1:0] oVFilt,        // EMA 滤波后反馈
    output reg          oValid         // 提交拍有效脉冲(采样后 LATENCY=3 拍)
);
    reg [W-1:0] rVFilt = {W{1'b0}};

    // ---- 多拍事件移位 rEv: [0]=delta, [1]=step, [2]=加+限幅提交 ----
    reg [2:0] rEv;
    reg       rBusy;
    reg [W-1:0] rVin;                              // 采样快照
    reg signed [W:0] rDelta;                       // delta(一拍)
    reg signed [W:0] rStep;                        // step(一拍)

    // 事件移位 + 输入快照
    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            rEv   <= 3'b000;
            rBusy <= 1'b0;
            rVin  <= {W{1'b0}};
        end else begin
            rEv[2] <= rEv[1];
            rEv[1] <= rEv[0];
            rEv[0] <= 1'b0;
            if (iSampleEn && !rBusy) begin
                rEv[0] <= 1'b1;
                rVin   <= iVIn;
                rBusy  <= 1'b1;
            end
            if (rEv[2]) rBusy <= 1'b0;
        end
    end

    // 一拍 delta = in - filt(旧, 提交拍前不变)
    wire signed [W:0] wDelta = $signed({1'b0, rVin}) - $signed({1'b0, rVFilt});
    // 二拍 step = delta >>> EMA_SHIFT
    // 三拍 vnext = filt + step (旧状态), 限幅 [0, 2^W-1]
    wire signed [W:0] wVNext = $signed({1'b0, rVFilt}) + rStep;

    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            rDelta <= {W+1{1'b0}};
            rStep  <= {W+1{1'b0}};
            rVFilt <= {W{1'b0}};
            oVFilt <= {W{1'b0}};
            oValid <= 1'b0;
        end else begin
            oValid <= 1'b0;
            if (rEv[0])
                rDelta <= wDelta;                  // 一拍: delta 寄存
            if (rEv[1])
                rStep  <= rDelta >>> EMA_SHIFT;    // 一拍: 缩放(移位)寄存
            if (rEv[2]) begin
                if (wVNext <= 0)
                    rVFilt <= {W{1'b0}};
                else if (wVNext >= $signed({1'b0, {W{1'b1}}}))
                    rVFilt <= {W{1'b1}};
                else
                    rVFilt <= wVNext[W-1:0];
                if (wVNext <= 0)
                    oVFilt <= {W{1'b0}};
                else if (wVNext >= $signed({1'b0, {W{1'b1}}}))
                    oVFilt <= {W{1'b1}};
                else
                    oVFilt <= wVNext[W-1:0];
                oValid <= 1'b1;                    // 一拍: 加+限幅提交
            end
        end
    end
endmodule
