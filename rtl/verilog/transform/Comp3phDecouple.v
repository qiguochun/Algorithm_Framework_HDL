//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   Comp3phDecouple.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
/*Description       :   三相正/负序旋转坐标解耦（dp,qp / dn,qn）。
                        例化 CompAbc2pnz 得正序 αβ(oAlphaP/oBetaP) 与负序 αβ(oAlphaN/oBetaN)，
                        再对正序做 Park(+θ)、对负序做 Park(-θ) 得 dq 直流量。
                        oDp,oQp: 正序 dq；oDn,oQn: 负序 dq（θ 由 PLL/电网相位给定，16bit 全幅）。
                        依赖: CompAbc2pnz / CompPark / AlgoTrig（编译需一并列出）。
                        【多拍流水版】iEn 单拍脉冲接受一个采样：
                        接受拍同时把 iTheta 快照（本采样 Park 角），内部 CompAbc2pnz.iEn 取外 iEn；
                        内部 CompAbc2pnz 在其 oValid 拍提交本采样正/负序 αβ；
                        随后一拍对组合 Park(+θ / -θ 用快照 θ) 结果寄存并把 oValid 推迟一拍输出。
                        总延迟 LATENCY = 内部 CompAbc2pnz LATENCY + 1（默认 6+1=7）。
                        数值与原单周期组合版逐接受样本一致（每个接受采样的输出逐位等价）。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.2  (多拍流水化, LATENCY=7)
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
module Comp3phDecouple #(
    parameter integer W       = 16,
    parameter integer LATENCY = 7                 // 总拍数 = CompAbc2pnz.LATENCY + 1
) (
    input  wire                iSysClk,
    input  wire                iSysRst,
    input  wire                iEn,             // 采样节拍（单拍脉冲：一个采样）
    input  wire signed [W-1:0] iA,
    input  wire signed [W-1:0] iB,
    input  wire signed [W-1:0] iC,
    input  wire [W-1:0]        iTheta,          // 电角度全幅 0..65535<->0..2π
    output reg  signed [W-1:0] oDp, oQp,        // 正序 dq
    output reg  signed [W-1:0] oDn, oQn,        // 负序 dq
    output reg                 oValid           // 采样后 LATENCY 拍拉高一拍
);
    wire signed [W-1:0] wAp, wBp, wAn, wBn;          // 内部 CompAbc2pnz 提交输出
    wire signed [W-1:0] wOa, wOb, wOc, wNa, wNb, wNc; // 三相 p/n（本层未用，仅接满）
    wire signed [W-1:0] wZp, wZn;
    wire                wPzV;                        // 内部 CompAbc2pnz 完成脉冲

    CompAbc2pnz #(.W(W), .LATENCY(LATENCY-1)) U_Pnz (
        .iSysClk(iSysClk), .iSysRst(iSysRst), .iEn(iEn),
        .iA(iA), .iB(iB), .iC(iC),
        .oAlphaP(wAp), .oBetaP(wBp), .oAlphaN(wAn), .oBetaN(wBn),
        .oAp(wOa), .oBp(wOb), .oCp(wOc),
        .oAn(wNa), .oBn(wNb), .oCn(wNc),
        .oRZp(wZp), .oRZn(wZn),
        .oValid(wPzV)
    );

    // 本采样 Park 角快照（接受拍锁存，内部 CompAbc2pnz.oValid 那拍用于 Park）
    reg [W-1:0] rTh;
    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) rTh <= {W{1'b0}};
        else if (iEn) rTh <= iTheta;
    end

    wire [W-1:0] wTNeg = -rTh;                // 16bit 环绕取负（-θ mod 2^16）

    // 组合 Park（正序 +θ，负序 -θ），内部 AlgoTrig 组合读
    wire signed [W-1:0] wPkDp, wPkQp, wPkDn, wPkQn;
    CompPark #(.W(W)) U_PkP (
        .iAlpha(wAp), .iBeta(wBp), .iTheta(rTh), .oD(wPkDp), .oQ(wPkQp)
    );
    CompPark #(.W(W)) U_PkN (
        .iAlpha(wAn), .iBeta(wBn), .iTheta(wTNeg), .oD(wPkDn), .oQ(wPkQn)
    );

    // 在内部 CompAbc2pnz.oValid 那拍（其提交后一拍）寄存组合 Park 结果，oValid 推迟一拍输出
    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            oDp <= {W{1'b0}}; oQp <= {W{1'b0}};
            oDn <= {W{1'b0}}; oQn <= {W{1'b0}};
            oValid <= 1'b0;
        end else begin
            if (wPzV) begin
                oDp <= wPkDp; oQp <= wPkQp;
                oDn <= wPkDn; oQn <= wPkQn;
            end
            oValid <= wPzV;
        end
    end
endmodule
