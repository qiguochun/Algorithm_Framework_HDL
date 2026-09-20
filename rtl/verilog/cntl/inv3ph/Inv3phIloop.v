//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   Inv3phIloop.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.20
/*Description       :   三相逆变器 dq 电流环。
                        控制核例化算法库 CompPi x2 + CompFfDecouple（ωL 解耦）。
                        ed=Id_ref-Id; eq=Iq_ref-Iq
                        Ed = PI_d + SEQ*vd_ff + Ud; Eq = PI_q + SEQ*vq_ff + Uq
                        vd_ff=-wL*Iq, vq_ff=+wL*Id（CompFfDecouple）。
                        SEQ=+1 正序 / -1 负序 / 0 零序。
                        默认 FS=20kHz, Kp=6, Ki=520, Q10。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.2 (use CompPi + CompFfDecouple)
//modifier          :
//Modify Date       :   2026.09.20
/*Modify Record     :
//  Rev 0.2: 内联 PI/解耦改为例化 CompPi、CompFfDecouple。
*/
//------------------------------------------------------------------------------
`include "inv3ph_cfg.vh"

module Inv3phIloop #(
    parameter integer W       = 16,
    parameter integer FSHIFT  = 10,
    parameter integer LATENCY = 4,
    parameter integer SEQ     = 1,
    parameter signed [15:0] P_KP   = 16'sd6144,  // Kp=6 Q10
    parameter signed [15:0] P_KIT2 = 16'sd13,    // Ki*Ts/2 Q10
    // CompFfDecouple: omega Q8 pu(=1.0->256), L Q15 使 wL≈0.2827 -> 9264
    parameter signed [15:0] P_OMEGA = 16'sd256,
    parameter signed [15:0] P_L     = 16'sd9264,
    parameter signed [31:0] P_OMAX = 32'sd20000,
    parameter signed [31:0] P_OMIN = -32'sd20000
) (
    input  wire                iSysClk,
    input  wire                iSysRst,
    input  wire                iEn,
    input  wire signed [W-1:0] iId,
    input  wire signed [W-1:0] iIq,
    input  wire signed [W-1:0] iUd,
    input  wire signed [W-1:0] iUq,
    input  wire signed [W-1:0] iIdRef,
    input  wire signed [W-1:0] iIqRef,
    output reg  signed [W-1:0] oEdRef,
    output reg  signed [W-1:0] oEqRef,
    output reg                 oValid
);
    localparam signed [15:0] SEQ_S = SEQ;

    reg signed [W-1:0] rId, rIq, rUd, rUq;

    wire signed [W-1:0] wEd = iIdRef - iId;
    wire signed [W-1:0] wEq = iIqRef - iIq;

    wire signed [W-1:0] wPid, wPiq;
    wire                wVd, wVq;
    wire signed [W-1:0] wVdFf, wVqFf;

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

    CompFfDecouple #(.W(W)) U_Ff (
        .iId(rId), .iIq(rIq),
        .iOmega(P_OMEGA), .iL(P_L),
        .oVdFf(wVdFf), .oVqFf(wVqFf)
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
            rId <= {W{1'b0}}; rIq <= {W{1'b0}};
            rUd <= {W{1'b0}}; rUq <= {W{1'b0}};
            oEdRef <= {W{1'b0}};
            oEqRef <= {W{1'b0}};
            oValid <= 1'b0;
        end else begin
            oValid <= 1'b0;
            if (iEn) begin
                rId <= iId; rIq <= iIq;
                rUd <= iUd; rUq <= iUq;
            end
            if (wVd && wVq) begin
                oEdRef <= f_satw($signed(wPid)
                    + SEQ_S * $signed(wVdFf)
                    + $signed(rUd));
                oEqRef <= f_satw($signed(wPiq)
                    + SEQ_S * $signed(wVqFf)
                    + $signed(rUq));
                oValid <= 1'b1;
            end
        end
    end
endmodule
