//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   CompFfDecouple.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.06
/*Description       :   三相 dq 同步旋转坐标系前馈解耦(纯组合乘加, 无除法)
//                       vd_ff = -w*L*iq;   vq_ff = +w*L*id
//                       w*L 内部合并为单一系数 wl = w*L (Q 定点, 共用一次乘),
//                       再用系数乘 d/q 电流得解耦电压。
//                       定点(per-unit, 位宽 64bit 中间 + S16 饱和):
//                         iId/iIq:  电流 Q15 pu (1pu = 32768 计数)
//                         iOmega:   电角速度 Q8  (1pu = 2*pi*50 rad/s)
//                         iL:       电感   Q15 pu (感抗标幺)
//                         wl = (iOmega*iL + rnd)>>8  为 Q15 (w*L pu)
//                         vd/vq = (+-)(wl*i + rnd)>>15  为 Q15 (pu)
//                       输出经 fn_sat16 饱和; 无组合除法。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.1
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
module CompFfDecouple #(
    parameter integer W = 16
) (
    input  wire signed [W-1:0] iId,      // d 轴电流 (Q15 pu)
    input  wire signed [W-1:0] iIq,      // q 轴电流 (Q15 pu)
    input  wire signed [W-1:0] iOmega,   // 电角速度 w (Q8 pu)
    input  wire signed [W-1:0] iL,       // 电感 L   (Q15 pu)
    output wire signed [W-1:0] oVdFf,    // 解耦 d 轴电压 = -wL*iq (Q15 pu)
    output wire signed [W-1:0] oVqFf     // 解耦 q 轴电压 = +wL*id (Q15 pu)
);

    // 64 位饱和收缩: S64 右移 fb(先 +half 舍入) 后钳位到 S16
    function signed [15:0] fn_sat16_64;
        input signed [63:0] a;
        input [6:0]         fb;
        reg signed [63:0] ra, half;
        begin
            half = (fb > 0) ? (64'sd1 << (fb - 1)) : 64'sd0;
            ra  = a + half;
            ra  = ra >>> fb;
            if (ra >  32767) ra =  32767;
            if (ra < -32768) ra = -32768;
            fn_sat16_64 = ra[15:0];
        end
    endfunction

    wire signed [63:0] wId = iId;
    wire signed [63:0] wIq = iIq;
    wire signed [63:0] wOm = iOmega;
    wire signed [63:0] wL  = iL;

    // w*L 合并系数: (Q8 * Q15 = Q23) >>8 -> Q15 (半值 2^7=128)
    wire signed [63:0] wWl = (wOm*wL + 64'sd128) >>> 8;

    // vq = +wl*id, vd = -wl*iq; 乘 Q15*Q15=Q30, +半 2^14 后 >>15 -> Q15
    wire signed [63:0] wVqRaw = (wWl*wId + 64'sd16384) >>> 15;
    wire signed [63:0] wVdRaw = (wWl*wIq + 64'sd16384) >>> 15;

    assign oVqFf = fn_sat16_64( wVqRaw, 7'd0);
    assign oVdFf = fn_sat16_64(-wVdRaw, 7'd0);
endmodule
