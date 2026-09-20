//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   CompClark.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
/*Description       :   Clarke 三相 -> 两相静止变换（Amp/Power 双模式），纯组合。
                        等幅值(Amp, MODE=0): alpha=a, beta=(a+2b)/sqrt3, zero=(a+b+c)/3
                        等功率(Power,MODE=1): alpha=(2a-b-c)/3, beta=(b-c)/sqrt3, zero=(a+b+c)/3
                        信号 S16，增益系数 Q15 命名常量。反变换见 CompInvClark.v。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.2
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
`include "algo_arith.vh"

module CompClark #(
    parameter integer W    = 16,
    parameter integer MODE = 1              // 0=等幅值(Amp), 1=等功率(Power)
) (
    input  wire signed [W-1:0] iA,
    input  wire signed [W-1:0] iB,
    input  wire signed [W-1:0] iC,
    output wire signed [W-1:0] oAlpha,
    output wire signed [W-1:0] oBeta,
    output wire signed [W-1:0] oZero
);
    localparam signed [15:0] C_INV_SQRT3 = 16'sd18919;  // 1/sqrt3 = 0.5773503 Q15
    localparam signed [15:0] C_ONE_THIRD = 16'sd10923;  // 1/3 = 0.3333333 Q15

    wire signed [47:0] wSA = iA;
    wire signed [47:0] wSB = iB;
    wire signed [47:0] wSC = iC;

    assign oZero = fn_sat((wSA + wSB + wSC) * C_ONE_THIRD, 15);

    generate
        if (MODE == 0) begin : g_amp
            assign oAlpha = iA;
            assign oBeta  = fn_sat((wSA + (wSB << 1)) * C_INV_SQRT3, 15);
        end else begin : g_pwr
            assign oAlpha = fn_sat((wSA + wSA - wSB - wSC) * C_ONE_THIRD, 15);
            assign oBeta  = fn_sat((wSB - wSC) * C_INV_SQRT3, 15);
        end
    endgenerate
endmodule
