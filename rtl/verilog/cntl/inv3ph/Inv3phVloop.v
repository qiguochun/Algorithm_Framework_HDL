//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   Inv3phVloop.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.20
/*Description       :   三相逆变器 dq 电压环。
                        控制核例化算法库 CompPi x2（d/q）。
                        误差: ed = Ud_ref-Ud-wL2*I2q; eq = Uq_ref-Uq+wL2*I2d
                        输出: I1d = PI_d - wC*Uq + I2d; I1q = PI_q + wC*Ud + I2q
                        默认 FS=20kHz, Kp=0.45, Ki=42.67 (Q14)。
                        LATENCY = CompPi.LATENCY（默认 4）。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.2 (use CompPi)
//modifier          :
//Modify Date       :   2026.09.20
/*Modify Record     :
//  Rev 0.2: 内联双线性 PI 改为例化 CompPi。
*/
//------------------------------------------------------------------------------
`include "inv3ph_cfg.vh"

module Inv3phVloop #(
    parameter integer W       = 16,
    parameter integer FSHIFT  = 14,
    parameter integer LATENCY = 4,
    parameter signed [15:0] P_KP   = 16'sd7373,   // Kp=0.45 Q14
    parameter signed [15:0] P_KIT2 = 16'sd17,     // Ki*Ts/2 Q14
    parameter signed [15:0] P_WL2  = 16'sd154,    // w*L2 Q14
    parameter signed [15:0] P_WC   = 16'sd41,     // w*C1 Q14
    parameter signed [31:0] P_OMAX = 32'sd20000,
    parameter signed [31:0] P_OMIN = -32'sd20000
) (
    input  wire                iSysClk,
    input  wire                iSysRst,
    input  wire                iEn,
    input  wire signed [W-1:0] iUd,
    input  wire signed [W-1:0] iUq,
    input  wire signed [W-1:0] iI2d,
    input  wire signed [W-1:0] iI2q,
    input  wire signed [W-1:0] iUdRef,
    input  wire signed [W-1:0] iUqRef,
    output reg  signed [W-1:0] oI1dRef,
    output reg  signed [W-1:0] oI1qRef,
    output reg                 oValid
);
    localparam signed [63:0] HALF = (64'sd1 <<< (FSHIFT - 1));

    // ---- 误差组合直连 CompPi（与 iEn 同拍采样）；前馈用 iEn 快照 ----
    reg signed [W-1:0] rUd, rUq, rI2d, rI2q;
    wire signed [W-1:0] wEd = iUdRef - iUd
        - (($signed(P_WL2) * $signed(iI2q) + HALF) >>> FSHIFT);
    wire signed [W-1:0] wEq = iUqRef - iUq
        + (($signed(P_WL2) * $signed(iI2d) + HALF) >>> FSHIFT);

    wire signed [W-1:0] wPid, wPiq;
    wire                wVd, wVq;

    CompPi #(
        .W(W), .QF(FSHIFT), .LATENCY(LATENCY),
        .P_KP(P_KP), .P_KIT2(P_KIT2),
        .P_OMAX(P_OMAX), .P_OMIN(P_OMIN)
    ) U_PiD (
        .iSysClk(iSysClk), .iSysRst(iSysRst),
        .iEn(iEn), .iErrIn(wEd), .oOut(wPid), .oValid(wVd)
    );

    CompPi #(
        .W(W), .QF(FSHIFT), .LATENCY(LATENCY),
        .P_KP(P_KP), .P_KIT2(P_KIT2),
        .P_OMAX(P_OMAX), .P_OMIN(P_OMIN)
    ) U_PiQ (
        .iSysClk(iSysClk), .iSysRst(iSysRst),
        .iEn(iEn), .iErrIn(wEq), .oOut(wPiq), .oValid(wVq)
    );

    function automatic signed [W-1:0] f_satw;
        input signed [31:0] v;
        begin
            if (v > P_OMAX)      f_satw = P_OMAX[W-1:0];
            else if (v < P_OMIN) f_satw = P_OMIN[W-1:0];
            else                 f_satw = v[W-1:0];
        end
    endfunction

    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            rUd <= {W{1'b0}}; rUq <= {W{1'b0}};
            rI2d <= {W{1'b0}}; rI2q <= {W{1'b0}};
            oI1dRef <= {W{1'b0}};
            oI1qRef <= {W{1'b0}};
            oValid  <= 1'b0;
        end else begin
            oValid <= 1'b0;
            if (iEn) begin
                rUd  <= iUd;  rUq  <= iUq;
                rI2d <= iI2d; rI2q <= iI2q;
            end
            // 电容电流前馈 + 负载电流前馈（PI 提交拍）
            if (wVd && wVq) begin
                oI1dRef <= f_satw($signed(wPid)
                    - (($signed(P_WC) * $signed(rUq) + HALF) >>> FSHIFT)
                    + $signed(rI2d));
                oI1qRef <= f_satw($signed(wPiq)
                    + (($signed(P_WC) * $signed(rUd) + HALF) >>> FSHIFT)
                    + $signed(rI2q));
                oValid <= 1'b1;
            end
        end
    end
endmodule
