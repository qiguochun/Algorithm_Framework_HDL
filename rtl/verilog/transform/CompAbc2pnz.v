//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   CompAbc2pnz.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
/*Description       :   三相 ABC -> 正序/负序分量分解（等功率 Clarke + AFP + 合成）。
                        Clarke(Power)->alpha,beta,zero；
                        AFP 一阶全通 90 度移相: y[n]=-0.9844x[n]+x[n-1]+0.9844y[n-1]；
                        alpha_p=(alpha-beta_AFP)/2, beta_p=(beta+alpha_AFP)/2, zero_p=(zero+zero_AFP)/2；
                        alpha_n=(alpha+beta_AFP)/2, beta_n=(beta-alpha_AFP)/2, zero_n=(zero-zero_AFP)/2；
                        InvClark -> oAp/oBp/oCp、oAn/oBn/oCn。S16；系数 -0.9844 Q15=-32257。
                        例化 CompClark(Power)/CompInvClark。
                        【多拍流水版】iEn 单拍脉冲接受一个采样，组合乘加链拆成 LATENCY(=6) 拍：
                        拍1 快照 Clarke 三路输出（本采样 x）；
                        拍2 三路 AFP 乘法（C_A*x、C_NA*y[n-1]，6 个乘法并行各自寄存）；
                        拍3 各路 AFP 求和（+x[n-1]<<15）一拍；
                        拍4 各路 AFP 饱和得到 y（组合 fn_sat + 寄存）一拍；
                        拍5 顺序分量 (x±yb 等) 相减/相加一拍；
                        拍6 0.5 缩放(>>1)饱和（fb=1）一拍；
                        末拍(LATENCY-1) 状态更新(x[n-1]<-x[n]、y[n-1]<-y[n]) + 输出 oAlphaP... + oValid 一拍。
                        数值与原单周期组合版逐接受样本一致（每一接受采样的输出逐位等价）。
                        依赖: CompClark / CompInvClark（编译需一并列出）。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.2  (多拍流水化, LATENCY=6)
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
`include "algo_arith.vh"

module CompAbc2pnz #(
    parameter integer W       = 16,
    parameter integer LATENCY = 6                 // 流水拍数（建议 6~7）
) (
    input  wire                iSysClk,
    input  wire                iSysRst,
    input  wire                iEn,             // 采样节拍（单拍脉冲：一个采样）
    input  wire signed [W-1:0] iA,
    input  wire signed [W-1:0] iB,
    input  wire signed [W-1:0] iC,
    // 分解后 αβ（供上层 Park 用）
    output reg  signed [W-1:0] oAlphaP, oBetaP,
    output reg  signed [W-1:0] oAlphaN, oBetaN,
    output reg  signed [W-1:0] oRZp, oRZn,   // 正/负序零序（内部分量）
    // 合成回三相
    output wire signed [W-1:0] oAp, oBp, oCp,   // 正序 abc
    output wire signed [W-1:0] oAn, oBn, oCn,   // 负序 abc
    output reg                 oValid           // 采样后 LATENCY 拍拉高一拍
);
    localparam signed [15:0] C_A   = -16'sd32257;   // -0.9844 Q15
    localparam signed [15:0] C_NA  =  16'sd32257;   // +0.9844 Q15

    // 每路一阶全通状态：上一接受样本的输入/输出（仅在提交拍更新）
    reg signed [W-1:0] rAXp, rAYp, rBXp, rBYp, rZXp, rZYp;

    // 采样拍移位链：rPipe[0] 在采样被接受后打 1 并逐拍前移；rPipe[LATENCY-1] 即提交/输出拍
    reg [LATENCY-1:0] rPipe;

    // Clarke(Power) 组合
    wire signed [W-1:0] wClAl, wClBe, wClZ;
    CompClark #(.MODE(1)) U_Ck (
        .iA(iA), .iB(iB), .iC(iC),
        .oAlpha(wClAl), .oBeta(wClBe), .oZero(wClZ)
    );

    // ---- 本采样快照（接受拍锁存，拍到提交）----
    reg signed [W-1:0] rSAl, rSBe, rSZ;

    // ---- 48bit 符号扩展辅助线（乘法/移位用同原组合一致位宽）----
    wire signed [47:0] wSAlE = rSAl;   // 本采样 clarke alpha
    wire signed [47:0] wSBeE = rSBe;   // 本采样 clarke beta
    wire signed [47:0] wSZE  = rSZ;    // 本采样 clarke zero
    wire signed [47:0] wAxE  = rAXp;   // x[n-1] alpha
    wire signed [47:0] wBxE  = rBXp;
    wire signed [47:0] wZxE  = rZXp;
    wire signed [47:0] wAyE  = rAYp;   // y[n-1] alpha
    wire signed [47:0] wByE  = rBYp;
    wire signed [47:0] wZyE  = rZYp;
    wire signed [47:0] wYAlE = rYA;    // 本采样 y alpha
    wire signed [47:0] wYBeE = rYB;
    wire signed [47:0] wYZE  = rYZ;

    // ---- 逐拍流水中间量（一拍一组，各自独立寄存）----
    reg signed [47:0] rPA_C, rPA_N;   // alpha 轴 C_A*x[n] / C_NA*y[n-1]
    reg signed [47:0] rPB_C, rPB_N;   // beta  轴
    reg signed [47:0] rPZ_C, rPZ_N;   // zero  轴
    reg signed [47:0] rSumA, rSumB, rSumZ;   // AFP 求和 (C_A*xn + xn1<<15 + C_NA*yp)
    reg signed [W-1:0] rYA, rYB, rYZ;        // AFP 饱和输出 y
    reg signed [47:0] rDP_Al, rDP_Be, rDP_Z; // 正序 (x -/+ y_other) 组合
    reg signed [47:0] rDN_Al, rDN_Be, rDN_Z; // 负序 (x +/- y_other) 组合
    reg signed [W-1:0] rOAlP, rOBeP, rOAlN, rOBeN, rOZp, rOZn; // 0.5 缩放饱和中间量

    // ---- 组合（assign/函数外）：饱和各一拍 ----
    wire signed [W-1:0] wYsatA = fn_sat(rSumA, 15);
    wire signed [W-1:0] wYsatB = fn_sat(rSumB, 15);
    wire signed [W-1:0] wYsatZ = fn_sat(rSumZ, 15);
    wire signed [W-1:0] wOAlP  = fn_sat(rDP_Al, 1);   // fb=1：0.5 缩放舍入饱和
    wire signed [W-1:0] wOBeP  = fn_sat(rDP_Be, 1);
    wire signed [W-1:0] wOZP   = fn_sat(rDP_Z , 1);
    wire signed [W-1:0] wOAlN  = fn_sat(rDN_Al, 1);
    wire signed [W-1:0] wOBeN  = fn_sat(rDN_Be, 1);
    wire signed [W-1:0] wOZN   = fn_sat(rDN_Z , 1);

    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            rPipe  <= {LATENCY{1'b0}};
            rSAl   <= {W{1'b0}}; rSBe <= {W{1'b0}}; rSZ <= {W{1'b0}};
            rPA_C  <= 48'sd0; rPA_N <= 48'sd0;
            rPB_C  <= 48'sd0; rPB_N <= 48'sd0;
            rPZ_C  <= 48'sd0; rPZ_N <= 48'sd0;
            rSumA  <= 48'sd0; rSumB <= 48'sd0; rSumZ <= 48'sd0;
            rYA    <= {W{1'b0}}; rYB <= {W{1'b0}}; rYZ <= {W{1'b0}};
            rDP_Al <= 48'sd0; rDP_Be <= 48'sd0; rDP_Z <= 48'sd0;
            rDN_Al <= 48'sd0; rDN_Be <= 48'sd0; rDN_Z <= 48'sd0;
            rOAlP  <= {W{1'b0}}; rOBeP <= {W{1'b0}};
            rOAlN  <= {W{1'b0}}; rOBeN <= {W{1'b0}};
            rOZp   <= {W{1'b0}}; rOZn  <= {W{1'b0}};
            rAXp   <= {W{1'b0}}; rAYp <= {W{1'b0}};
            rBXp   <= {W{1'b0}}; rBYp <= {W{1'b0}};
            rZXp   <= {W{1'b0}}; rZYp <= {W{1'b0}};
            oAlphaP <= {W{1'b0}}; oBetaP <= {W{1'b0}};
            oAlphaN <= {W{1'b0}}; oBetaN <= {W{1'b0}};
            oRZp    <= {W{1'b0}}; oRZn  <= {W{1'b0}};
            oValid  <= 1'b0;
        end else begin
            // ---- 采样拍移位链 ----
            rPipe[0] <= iEn;
            if (LATENCY > 1)
                rPipe[LATENCY-1:1] <= rPipe[LATENCY-2:0];

            // ---- 拍1（接受拍）：快照 Clarke 三路输出 ----
            if (iEn) begin
                rSAl <= wClAl;
                rSBe <= wClBe;
                rSZ  <= wClZ;
            end

            // ---- 拍2：AFP 乘法（三路 x 与 yprev 乘法并行，各单独寄存）----
            if (rPipe[0]) begin
                rPA_C <= wSAlE * C_A;   rPA_N <= wAyE * C_NA;
                rPB_C <= wSBeE * C_A;   rPB_N <= wByE * C_NA;
                rPZ_C <= wSZE  * C_A;   rPZ_N <= wZyE * C_NA;
            end

            // ---- 拍3：各路 AFP 求和（加 x[n-1]<<15）一拍 ----
            if (rPipe[1]) begin
                rSumA <= rPA_C + rPA_N + (wAxE <<< 15);
                rSumB <= rPB_C + rPB_N + (wBxE <<< 15);
                rSumZ <= rPZ_C + rPZ_N + (wZxE <<< 15);
            end

            // ---- 拍4：各路 AFP 饱和 -> y ----
            if (rPipe[2]) begin
                rYA <= wYsatA;
                rYB <= wYsatB;
                rYZ <= wYsatZ;
            end

            // ---- 拍5：顺序分量组合（本采样 x 与异轴 y）一拍 ----
            if (rPipe[3]) begin
                rDP_Al <= wSAlE - wYBeE;   // alpha_p = (alpha - beta_AFP)/2
                rDP_Be <= wSBeE + wYAlE;   // beta_p  = (beta  + alpha_AFP)/2
                rDP_Z  <= wSZE  + wYZE;    // zero_p  = (zero  + zero_AFP)/2
                rDN_Al <= wSAlE + wYBeE;   // alpha_n = (alpha + beta_AFP)/2
                rDN_Be <= wSBeE - wYAlE;   // beta_n  = (beta  - alpha_AFP)/2
                rDN_Z  <= wSZE  - wYZE;    // zero_n  = (zero  - zero_AFP)/2
            end

            // ---- 拍6：0.5 缩放(fb=1)饱和 -> 中间量 ----
            if (rPipe[4]) begin
                rOAlP <= wOAlP; rOBeP <= wOBeP; rOZp <= wOZP;
                rOAlN <= wOAlN; rOBeN <= wOBeN; rOZn <= wOZN;
            end

            // ---- 末拍（提交）：状态更新 + 输出 + oValid ----
            if (rPipe[LATENCY-1]) begin
                oAlphaP <= rOAlP; oBetaP <= rOBeP; oRZp <= rOZp;
                oAlphaN <= rOAlN; oBetaN <= rOBeN; oRZn <= rOZn;
                rAXp <= rSAl; rAYp <= rYA;
                rBXp <= rSBe; rBYp <= rYB;
                rZXp <= rSZ;  rZYp <= rYZ;
            end
            oValid <= rPipe[LATENCY-1];
        end
    end

    // 由已注册的正/负序 αβ 零序反 Clarke 得三相（组合）
    CompInvClark #(.W(W)) U_Ip (
        .iAlpha(oAlphaP), .iBeta(oBetaP), .iZero(oRZp),
        .oA(oAp), .oB(oBp), .oC(oCp)
    );
    CompInvClark #(.W(W)) U_In (
        .iAlpha(oAlphaN), .iBeta(oBetaN), .iZero(oRZn),
        .oA(oAn), .oB(oBn), .oC(oCn)
    );
endmodule
