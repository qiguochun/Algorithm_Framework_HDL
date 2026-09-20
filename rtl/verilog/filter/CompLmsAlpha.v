//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   CompLmsAlpha.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
/*Description       :   LMS 极性系数在线估计。
                        e[n]=iCt[n]-a[n]*iOwn[n]；a[n+1]=a[n]+mu*e[n]*iOwn[n]。
                        系数 a 存 a*2^AFRAC，步长存 MUQ=mu*2^AFRAC；
                        iOwn/iCt 为 S0.(W-1) 有符号；内部宽位防溢出。收敛 a->alpha_true。
                        多拍流水改造: 采样节拍快照 iCt/iOwn 及 MUQ*iOwn 常数 ->
                        a*iOwn(乘法一拍) -> iCt - (a*iOwn)>>AFRAC = e(一拍) ->
                        (MUQ*iOwn)*e 即 mu*e*iOwn(乘法一拍) -> a+=增量提交(一拍)。
                        每个乘法结果单独寄存, 无单周期组合乘加; oValid 提交拍拉高。
                        (整数乘法结合律 (MUQ*e)*iOwn==e*(MUQ*iOwn), 数值与原实现一致)
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.2
//modifier          :
//Modify Date       :
/*Modify Record     :
//  Rev 0.2: 组合乘加链拆 rEv 多拍流水, 新增 oValid。
*/
//------------------------------------------------------------------------------
module CompLmsAlpha #(
    parameter integer W        = 16,     // 信号位宽(有符号)
    parameter integer AFRAC    = 16,     // alpha 小数位 Q.(AFRAC)
    parameter integer MUQ      = 5,      // mu*2^AFRAC（beta=mu*E[x^2])
    parameter integer ALPHAW   = 32      // oAlphaHat 输出位宽(含符号)
) (
    input  wire                iSysClk,      // 时钟
    input  wire                iSysRst,      // 复位（高有效）
    input  wire                iEn,          // 采样节拍
    input  wire signed [W-1:0] iCt,          // CT 电流(目标)
    input  wire signed [W-1:0] iOwn,         // 本侧电流(参考)
    output reg  signed [ALPHAW-1:0] oAlphaHat, // a*2^AFRAC
    output reg                     oValid      // 提交拍有效脉冲(采样后 LATENCY=4 拍)
);
    reg signed [63:0] rAq = 64'sd0;      // a*2^AFRAC 状态

    // ---- 多拍事件 rEv[3:0]: [0]=a*iOwn [1]=e [2]=inc 乘 [3]=提交 ----
    reg [3:0] rEv;
    reg       rBusy;
    reg signed [W-1:0] rC;               // iCt 快照
    reg signed [W-1:0] rOw;              // iOwn 快照
    reg signed [95:0]  rMQI;             // MUQ*iOwn(采样拍算好, 一拍)
    reg signed [95:0]  rEst;             // a*iOwn(乘法一拍)
    reg signed [63:0]  rE;               // e(一拍)
    reg signed [95:0]  rInc;             // (MUQ*iOwn)*e(乘法一拍)

    // ---- 事件移位 + 输入快照 ----
    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            rEv   <= 4'b0000;
            rBusy <= 1'b0;
            rC    <= {W{1'b0}};
            rOw   <= {W{1'b0}};
            rMQI  <= 96'sd0;
        end else begin
            rEv[3] <= rEv[2];
            rEv[2] <= rEv[1];
            rEv[1] <= rEv[0];
            rEv[0] <= 1'b0;
            if (iEn && !rBusy) begin
                rEv[0] <= 1'b1;                // 下拍进入乘法级
                rC     <= iCt;
                rOw    <= iOwn;
                rMQI   <= $signed(MUQ) * $signed(iOwn);   // mu*iOwn(常数乘法一拍)
                rBusy  <= 1'b1;
            end
            if (rEv[3]) rBusy <= 1'b0;
        end
    end

    // ---- 提交流水 ----
    wire signed [63:0] wAi   = (rEst >>> AFRAC);       // (a*iOwn)>>AFRAC
    wire signed [63:0] wE    = $signed(rC) - wAi;      // 误差 e
    wire signed [95:0] wAExt = $signed({ {32{rAq[63]}}, rAq });
    wire signed [95:0] wNxt  = wAExt + rInc;           // a + mu*e*iOwn

    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            rAq       <= 64'sd0;
            oAlphaHat <= {ALPHAW{1'b0}};
            oValid    <= 1'b0;
            rEst      <= 96'sd0;
            rE        <= 64'sd0;
            rInc      <= 96'sd0;
        end else begin
            oValid <= 1'b0;
            if (rEv[0])
                rEst <= rAq * $signed(rOw);            // 一拍: a*iOwn 寄存
            if (rEv[1])
                rE   <= $signed(rC) - (rEst >>> AFRAC); // 一拍: e 寄存
            if (rEv[2])
                rInc <= rMQI * rE;                     // 一拍: (mu*iOwn)*e 寄存
            if (rEv[3]) begin
                rAq       <= wNxt[63:0];               // 一拍: 提交 a
                oAlphaHat <= wNxt[ALPHAW-1:0];
                oValid    <= 1'b1;
            end
        end
    end
endmodule
