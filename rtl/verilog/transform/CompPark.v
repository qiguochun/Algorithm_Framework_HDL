//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   CompPark.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
/*Description       :   Park αβ -> dq 旋转坐标变换（纯组合 + 例化 AlgoTrig）。
                        iTheta 16bit 全幅：0..65535 <-> 0..2π。
                        Park: d=alpha*cos+beta*sin; q=-alpha*sin+beta*cos
                        信号 S16；反变换见 CompInvPark.v。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.2
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
`include "algo_arith.vh"

module CompPark #(
    parameter integer W = 16
) (
    input  wire signed [W-1:0] iAlpha,
    input  wire signed [W-1:0] iBeta,
    input  wire [W-1:0]        iTheta,  // 16bit 全幅
    output wire signed [W-1:0] oD,
    output wire signed [W-1:0] oQ
);
    wire signed [15:0] wCTh, wSTh;
    AlgoTrig U_Trig (.iAngle(iTheta), .oSin(wSTh), .oCos(wCTh));

    wire signed [47:0] wSAl = iAlpha;
    wire signed [47:0] wSBe = iBeta;

    assign oD = fn_sat(wSAl*wCTh + wSBe*wSTh, 15);
    assign oQ = fn_sat(-wSAl*wSTh + wSBe*wCTh, 15);
endmodule
