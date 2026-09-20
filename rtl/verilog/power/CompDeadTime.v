//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   CompDeadTime.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.06
/*Description       :   死区时间补偿: 由电流符号决定占空比增量方向
//                       iCur>0(电流流入负载)   -> 输出 +comp (补回死区造成的电压损失)
//                       iCur<0(电流流出负载)   -> 输出 -comp
//                       近零/死区(|i|<P_MAGTH)不补偿, 避免极性抖动。
//                       comp 为 Q 定点补偿量(物理可注释为占空比计数, 1LSB=1/32767)。
//                       纯组合逻辑(S32 运算 + S16 饱和), 无除法。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.1
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
`include "algo_arith.vh"

module CompDeadTime #(
    parameter integer W      = 16,
    parameter integer P_MAGTH = 16'd8        // 电流幅值阈值, 低于不补偿
) (
    input  wire signed [W-1:0] iCur,        // 相电流 (S16)
    input  wire signed [W-1:0] iComp,       // 死区补偿量 comp (Q 定点, 正幅值)
    output wire signed [W-1:0] oDutyInc     // 占空比补偿增量 (S16)
);

    wire signed [15:0] wMag = fn_abs16(iCur);   // |i|
    wire wNeg = iCur[W-1];                       // 1=负电流
    wire wEn  = (wMag > P_MAGTH);                // 死区/近零不补偿

    wire signed [31:0] wC   = iComp;             // S32 提升(幅值)
    wire signed [31:0] wInc = wNeg ? -wC : wC;   // 方向: 正加负减

    assign oDutyInc = wEn ? wInc[15:0] : {W{1'b0}};
endmodule
