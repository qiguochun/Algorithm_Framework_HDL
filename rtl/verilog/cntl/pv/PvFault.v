//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   PvFault.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
/*Description       :   通道故障/告警检测：过流/短路/反接锁存、欠压/过压回差
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.1
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
`include "algo_types.vh"

module PvFault #(
    parameter integer W               = `ALGO_W_VOLT,
    parameter integer OVER_CURR_MA    = 17000,   // 17 A 过流
    parameter integer SHORT_V         = 5,       // 短路判定电压
    parameter integer SHORT_I_MA      = 20000,   // 20 A
    parameter integer WARN_LOW_V      = 70,      // 欠压告警置位
    parameter integer WARN_LOW_RESET  = 72,      // 欠压复位
    parameter integer WARN_HIGH_V     = 900,     // 过压告警置位
    parameter integer WARN_HIGH_RESET = 880      // 过压复位
) (
    input  wire             iSysClk,
    input  wire             iSysRst,       // 高有效复位
    input  wire             iChkEn,        // 5 kHz 检测节拍
    input  wire [W-1:0]     iVInst,        // 瞬时电压(V)
    input  wire [W-1:0]     iVAve,         // 平均电压(V, 告警用)
    input  wire [W-1:0]     iCurrInst,     // 瞬时电流 mA(带符号幅值表示)
    output reg              oOverCurrent,  // 过流(锁存)
    output reg              oShortCut,     // 短路(锁存)
    output reg              oReverse,      // 反接(锁存)
    output reg              oVolLow,       // 欠压告警(回差)
    output reg              oVolHigh,      // 过压告警(回差)
    output wire             oIsErr         // 通道故障聚合
);
    integer ci;    // 电流幅值 mA

    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            oOverCurrent <= 1'b0;
            oShortCut    <= 1'b0;
            oReverse     <= 1'b0;
            oVolLow      <= 1'b0;
            oVolHigh     <= 1'b0;
        end else if (iChkEn) begin
            ci = $signed(iCurrInst);
            if (ci < 0) ci = -ci;

            if (ci > OVER_CURR_MA)                  // 过流锁存
                oOverCurrent <= 1'b1;

            if (iVInst < SHORT_V && ci > SHORT_I_MA) begin
                oShortCut <= 1'b1;                  // 短路锁存
                if ($signed(iCurrInst) < 0)
                    oReverse <= 1'b1;               // 反接锁存
            end

            if (iVAve < WARN_LOW_V[W-1:0])
                oVolLow <= 1'b1;
            else if (iVAve > WARN_LOW_RESET[W-1:0])
                oVolLow <= 1'b0;

            if (iVAve > WARN_HIGH_V[W-1:0])
                oVolHigh <= 1'b1;
            else if (iVAve < WARN_HIGH_RESET[W-1:0])
                oVolHigh <= 1'b0;
        end
    end

    assign oIsErr = oOverCurrent | oShortCut | oReverse | oVolLow | oVolHigh;
endmodule
