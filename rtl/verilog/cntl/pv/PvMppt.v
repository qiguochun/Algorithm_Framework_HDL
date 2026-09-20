//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   PvMppt.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
/*Description       :   扰动观察(P&O)MPPT：逐次扰动电压参考并据功率增减反向/续向
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.1
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
`include "algo_types.vh"

module PvMppt #(
    parameter integer W         = `ALGO_W_VOLT,   // 16
    parameter integer VREF_INIT = 15,             // 初始电压参考(V)
    parameter integer VREF_MIN  = 8,
    parameter integer VREF_MAX  = 44,
    parameter integer VSTEP     = 1               // 每 10Hz 扰动步长(V)
) (
    input  wire         iSysClk,
    input  wire         iSysRst,       // 高有效复位
    input  wire         iMpptEn,       // 10 Hz strobe
    input  wire [W-1:0] iVAvg,         // 电压 EMA
    input  wire [W-1:0] iPAvg,         // 功率 EMA (W*10)
    output reg  [W-1:0] oVRef          // 电压参考输出
);
    reg [W-1:0] rPrevP = 0;
    reg         rArmed = 1'b0;             // 首个 mppt_en 仅采样
    reg         rUpDir = 1'b1;             // 上次扰动方向：1=增压

    integer dp;
    integer nxt_v;
    reg     nxt_up;

    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            oVRef  <= VREF_INIT[W-1:0];
            rPrevP <= 0;
            rArmed <= 1'b0;
            rUpDir <= 1'b1;
        end else if (iMpptEn) begin
            if (!rArmed) begin
                rPrevP <= iPAvg;
                rArmed <= 1'b1;
            end else begin
                dp = $signed(iPAvg) - $signed(rPrevP);
                // 功率升/持平沿原方向，降则反向（边界强制反向保证继续扰动）
                nxt_up = (dp >= 0) ? rUpDir : ~rUpDir;

                if (nxt_up) begin
                    if ($signed(oVRef) + VSTEP <= VREF_MAX)
                        nxt_v = $signed(oVRef) + VSTEP;
                    else begin
                        nxt_v  = $signed(oVRef) - VSTEP;
                        nxt_up = 1'b0;             // 触上限 → 改向下
                    end
                end else begin
                    if ($signed(oVRef) - VSTEP >= VREF_MIN)
                        nxt_v = $signed(oVRef) - VSTEP;
                    else begin
                        nxt_v  = $signed(oVRef) + VSTEP;
                        nxt_up = 1'b1;             // 触下限 → 改向上
                    end
                end
                rUpDir <= nxt_up;
                oVRef  <= nxt_v[W-1:0];
                rPrevP <= iPAvg;
            end
        end
    end
endmodule
