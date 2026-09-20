//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   Inv3phSoftStart.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.20
/*Description       :   开环电压幅值软启斜坡。
                        例化算法库 CompSoftStart；iEn 扩展为单拍 iRun，
                        每控制拍推进一步。oDone = 已达 P_TARGET。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.2 (use CompSoftStart)
//modifier          :
//Modify Date       :   2026.09.20
/*Modify Record     :
//  Rev 0.2: 内联斜坡改为例化 CompSoftStart。
*/
//------------------------------------------------------------------------------
`include "inv3ph_cfg.vh"

module Inv3phSoftStart #(
    parameter integer W        = 16,
    parameter signed [31:0] P_STEP   = 32'sd64,      // 每控制拍步进
    parameter signed [31:0] P_TARGET = 32'sd2560     // 目标幅值（默认 TB: 10V@Q8）
) (
    input  wire                iSysClk,
    input  wire                iSysRst,
    input  wire                iEn,              // 控制拍：推进一步
    output wire signed [W-1:0] oAmpOut,
    output reg                 oDone
);
    wire wVld;
    // CompSoftStart 每时钟在 iRun=1 时累加；用 iEn 单拍驱动
    CompSoftStart #(
        .W(W), .P_STEP(P_STEP), .P_TARGET(P_TARGET)
    ) U_Ss (
        .iSysClk(iSysClk), .iSysRst(iSysRst),
        .iRun(iEn), .oOut(oAmpOut), .oValid(wVld)
    );

    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst)
            oDone <= 1'b0;
        else if (iEn)
            oDone <= (oAmpOut >= P_TARGET[W-1:0]) ||
                     ((P_STEP < 0) && (oAmpOut <= P_TARGET[W-1:0]));
    end
endmodule
