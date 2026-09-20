//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   CompInvClark.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
/*Description       :   反 Clarke：两相静止 <-> 三相（等幅值/等功率共用同式），纯组合。
                        a=alpha+zero, b=-alpha/2+sqrt3*beta/2+zero, c=-alpha/2-sqrt3*beta/2+zero
                        信号 S16，增益系数 Q15。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.1
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
`include "algo_arith.vh"

module CompInvClark #(
    parameter integer W = 16
) (
    input  wire signed [W-1:0] iAlpha,
    input  wire signed [W-1:0] iBeta,
    input  wire signed [W-1:0] iZero,
    output wire signed [W-1:0] oA,
    output wire signed [W-1:0] oB,
    output wire signed [W-1:0] oC
);
    localparam signed [15:0] C_HALF    = 16'sd16384;  // 0.5     Q15
    localparam signed [15:0] C_SQRT3_2 = 16'sd28378;  // sqrt3/2 = 0.8660254 Q15

    wire signed [47:0] wSAl = iAlpha;
    wire signed [47:0] wSBe = iBeta;
    wire signed [47:0] wSZo = iZero;

    assign oA = fn_sat(wSAl + wSZo, 0);
    assign oB = fn_sat((wSZo << 15) - wSAl*C_HALF + wSBe*C_SQRT3_2, 15);
    assign oC = fn_sat((wSZo << 15) - wSAl*C_HALF - wSBe*C_SQRT3_2, 15);
endmodule
