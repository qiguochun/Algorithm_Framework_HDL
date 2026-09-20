//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   CompInvPark.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
/*Description       :   反 Park：dq <-> αβ 旋转坐标变换（纯组合 + 例化 AlgoTrig）。
                        alpha=d*cos-q*sin; beta=d*sin+q*cos
                        iTheta 16bit 全幅：0..65535 <-> 0..2π。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.1
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
`include "algo_arith.vh"

module CompInvPark #(
    parameter integer W = 16
) (
    input  wire signed [W-1:0] iD,
    input  wire signed [W-1:0] iQ,
    input  wire [W-1:0]        iTheta,
    output wire signed [W-1:0] oAlpha,
    output wire signed [W-1:0] oBeta
);
    wire signed [15:0] wCTh, wSTh;
    AlgoTrig U_Trig (.iAngle(iTheta), .oSin(wSTh), .oCos(wCTh));

    wire signed [47:0] wSD = iD;
    wire signed [47:0] wSQ = iQ;

    assign oAlpha = fn_sat(wSD*wCTh - wSQ*wSTh, 15);
    assign oBeta  = fn_sat(wSD*wSTh + wSQ*wCTh, 15);
endmodule
