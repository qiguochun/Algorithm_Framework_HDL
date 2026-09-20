//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   CompSoftStart.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.06
/*Description       :   软启动斜坡: 使能后输出从 0 每 tick 加 step, 到达 target 停止并保持。
//                       方向由 step 符号决定(>0 上升 / <0 下降); 越过 target 一拍吸附到 target,
//                       无过冲。内部累加 S32, 计算在 S64 完成, 输出饱和缩至 S16。
//                       多拍流水改造: 累加器(吸附目标, 一拍) -> 输出饱和寄存器(一拍,
//                       oOut 落后 rAcc 一拍)。纯累加(无乘法除法), oValid 在输出提交拍
//                       指示本 tick 输出有效(斜坡进行中每拍拉高)。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.2
//modifier          :
//Modify Date       :
/*Modify Record     :
//  Rev 0.2: 饱和输出拆为独立一拍流水, 新增 oValid。
*/
//------------------------------------------------------------------------------
module CompSoftStart #(
    parameter integer       W       = 16,
    parameter signed [31:0] P_STEP  = 32'sd256,     // 每 tick 步进(可为负)
    parameter signed [31:0] P_TARGET= 32'sd20000    // 目标值
) (
    input  wire                iSysClk,
    input  wire                iSysRst,       // 高有效复位
    input  wire                iRun,          // 使能斜坡(1=上升, 0=保持)
    output reg  signed [W-1:0] oOut,          // 斜坡输出 (S16)
    output reg                 oValid         // 输出提交拍有效(每 tick 一拍, 斜坡进行中拉高)
);

    reg signed [31:0] rAcc;                  // 内部累加(S32, 不溢出: 到 target 即吸附)
    reg               rOutEn;                // 对齐 oOut 的有效延迟

    wire signed [31:0] wTgt  = P_TARGET;
    wire wUp   = (P_STEP >= 0);              // 方向
    wire wDone = wUp ? (rAcc >= wTgt) : (rAcc <= wTgt);
    wire signed [63:0] wNext = rAcc + P_STEP; // S64 计算防 S32 溢出
    wire signed [63:0] wSnap = wUp ? ((wNext > wTgt) ? wTgt : wNext)
                                   : ((wNext < wTgt) ? wTgt : wNext);
    wire wRun   = iRun && !wDone;            // 本拍是否累加一步

    function signed [15:0] fn_clip16;
        input signed [31:0] a;
        begin
            if (a >  32767) fn_clip16 =  32767;
            else if (a < -32768) fn_clip16 = -32768;
            else fn_clip16 = a[15:0];
        end
    endfunction

    // ---- 拍1: 累加器 + 目标吸附 (每 tick 一步, 一拍) ----
    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            rAcc <= 32'sd0;
        end else if (wRun) begin
            rAcc <= wSnap[31:0];
        end else begin
            rAcc <= rAcc;                // 到 target 或未使能: 保持
        end
    end

    // ---- 拍2: 输出饱和寄存器(落后 rAcc 一拍)；oValid 与 oOut 对齐 ----
    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            oOut   <= {W{1'b0}};
            oValid <= 1'b0;
            rOutEn <= 1'b0;
        end else begin
            rOutEn <= wRun;              // 延迟一拍，对齐即将写出的 oOut
            oOut   <= fn_clip16(rAcc);
            oValid <= rOutEn;            // 本拍 oOut 对应上一拍的推进
        end
    end
endmodule
