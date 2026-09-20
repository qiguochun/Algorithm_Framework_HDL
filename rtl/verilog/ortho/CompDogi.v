//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   CompDogi.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.06
/*Description       :   双 SOGI-FLL（DSOGI-FLL）两相(α/β)频率锁定环, 对齐 C 参考结构。
                        结构: 双 SOGI(α/β 轴, 例化 CompSogi) -> 残差 eps=v-v' ->
                        频率误差 N=eps_a*v2a+eps_b*v2b (v2=滞后90正交) / Vsq 归一
                        -> FLL dw=-gamma*N/Vsq（限幅 ±2pi*10Hz）-> w_est=w_nom+dw
                        -> θ+=w_est*Ts。 正序: v_a+=0.5(v1a-v2b), v_b+=0.5(v1b+v2a)。
                        信号 S16; 角频率 S32 Q6(1LSB=1/64 rad/s); θ 32bit 累加,
                        oTheta=高16位全幅。除法例化 AlgoDiv（多拍 start/done 握手,
                        空闲即捕获 N/Vsq 快照后台运算, 完成后刷新 rNq15 Q15, 准稳态不变）。
                        注: CompSogi 系数已折入 k=1.414 / fc=50Hz / Fs=5000 (默认), K_DAMP 为文档值。
                        【多拍流水版】iEn 采样节拍一拍触发，SOGI 输出组合进 N/Vsq/dif/sum 的
                        组合乘加链拆成多拍: 每样本一拍 iEn 喂双 SOGI（各自 LATENCY=6 拍出结果）
                        -> +1 拍读 SOGI 输出与样本输入 -> +1 拍 eps/dif/sum ->
                        +1 拍乘积(N 两乘/Vsq 四乘) -> +1 拍部分和 -> +1 拍 Vsq 终和 ->
                        提交拍更新 θ/dw/wEst、正序输出、N/Vsq 快照并拉 oValid。
                        rNq15 由后台 AlgoDiv 从注册快照(rNcur/rVsqCur)周期性刷新（近 ZOH）。
  修订   :   2026-09-06 v0.2 (多拍流水化)
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.2
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
`include "algo_arith.vh"

module CompDogi #(
    parameter integer W      = 16,
    parameter integer FS     = 5000,     // 采样率 Hz（须与 CompSogi 默认 Ts 匹配）
    parameter integer F_NOM  = 50,       // 标称频率 Hz
    parameter integer GAMMA  = 100,      // FLL 增益（50~500）
    parameter integer K_DAMP = 1414,     // SOGI 阻尼 k*1000（文档值, 已折入 CompSogi 系数）
    parameter integer LATENCY = 12       // iEn -> oValid 总拍数（>=12，含 SOGI 6 拍 + 计算/提交 6 拍）
) (
    input  wire                 iSysClk,
    input  wire                 iSysRst,
    input  wire                 iEn,            // 采样节拍（一拍）
    input  wire signed [W-1:0]  iVAlpha,     // α 轴采样
    input  wire signed [W-1:0]  iVBeta,      // β 轴采样
    output wire [W-1:0]         oTheta,      // 相位 16bit 全幅 0..65535<->0..2pi
    output wire signed [W-1:0]  oWEst,       // 估计角频率 Q6(1LSB=1/64 rad/s)
    output reg  signed [W-1:0]  oVAlphaPos,  // 正序 α（注册输出）
    output reg  signed [W-1:0]  oVBetaPos,   // 正序 β（注册输出）
    output wire signed [W-1:0]  oErrNorm,    // 归一化 FLL 误差 Q15（观测/调试用）
    output reg                  oValid       // 采样后 LATENCY 拍拉高一拍（本样本结果已提交）
);
    // ---- 64bit -> 32bit 等幂归一化移位 ----
    function automatic [6:0] f_sh31;
        input [63:0] v;
        integer i;
        begin
            f_sh31 = 7'd0;
            for (i = 0; i < 64; i = i + 1)
                if (v[i]) f_sh31 = (i > 30) ? (i - 30) : 7'd0;
        end
    endfunction

    // ---- 定点常量（FS=5000 / F_NOM=50 语义）----
    localparam signed [31:0] WNOM_Q  = (F_NOM * 32'sd402124 + 32'sd500) / 32'sd1000; // ~2*pi*F_NOM*64
    localparam [31:0]       C_PH    = (32'd2136 * 32'd5000) / FS;   // ~2^32/(2*pi*FS*64)
    localparam signed [31:0] DW_CL  = 32'sd4021;     // 2*pi*10*64            10Hz
    // FLL 积分增益: 每样本 dw 增量 = -n_q15*C_FLL>>20, 系数约 gamma*0.4096
    localparam signed [31:0] C_FLL   = (GAMMA * 41 + 50) / 100;
    localparam [63:0]        MIN_V2  = 64'd32768;    // Vsq 下限(幅~90)防小信号增益
    localparam integer       SOGI_LAT = 6;           // 内部 SOGI 流水拍（固定, 拍序据此对齐）

    // ---- 双 SOGI（α/β 轴, iEn 与自身同相）----
    wire signed [W-1:0] a1a, a2a, b1a, b2a;   // sogi 输出 in/quad
    wire sVaValid, sVbValid;
    CompSogi #(.W(W), .LATENCY(SOGI_LAT)) U_Sa (.iSysClk(iSysClk), .iSysRst(iSysRst),
                    .iEn(iEn), .iXIn(iVAlpha), .oVAlpha(a1a), .oVBeta(a2a), .oValid(sVaValid));
    CompSogi #(.W(W), .LATENCY(SOGI_LAT)) U_Sb (.iSysClk(iSysClk), .iSysRst(iSysRst),
                    .iEn(iEn), .iXIn(iVBeta), .oVAlpha(b1a), .oVBeta(b2a), .oValid(sVbValid));

    // ---- C 约定 v2 = v1 滞后 90: 库 v_beta_o = v_alpha 超前90, 取负对齐 ----
    wire signed [W-1:0] v2a = -a2a;
    wire signed [W-1:0] v2b = -b2a;

    // ================================================================
    //  流水拍序（采样在 posedge 时刻 T0; rPipe[k] 在第 T0+k 拍前沿呈 1）
    //    T0+0 : rPipe[0]<=iEn, 快照输入(本样本) rVaCap/rVbCap; SOGI 接受
    //    SOGI 在 T0+SOGI_LAT(=6) 拍提交, 输出本样本结果
    //    T0+7 : rPipe[6]  读 SOGI 输出 + 样本输入 -> 64bit 扩位捕获
    //    T0+8 : rPipe[7]  eps_a/eps_b/dif_ap/sum_bp
    //    T0+9 : rPipe[8]  N 两乘积 / Vsq 四平方（并行）
    //    T0+10: rPipe[9]  N 求和 / Vsq 前两对求和
    //    T0+11: rPipe[10] Vsq 终和
    //    T0+12: rPipe[11] 提交拍: FLL 状态 + 正序输出 + N/Vsq 快照 + oValid
    //  (LATENCY=12 => rPipe[LATENCY-1] = rPipe[11])
    // ================================================================
    reg [LATENCY-1:0] rPipe;

    // ---- T0+0: 采样拍快照输入（本样本, 供 +7 拍对齐使用）----
    reg signed [W-1:0] rVaCap, rVbCap;

    // ---- T0+7: SOGI/输入 64bit 扩位捕获 ----
    reg signed [63:0] rV1a, rV2a, rV1b, rV2b, rVa, rVb;

    // ---- 上一拍(上一样本)SOGI 捕获: 复现参考中“当前输入 x[k] 与上一拍 SOGI 输出
    //      v[k-1]”配对(单周期组合读注册输出天然滞后一拍), 使 N 在锁定时保持负向冻结 ----
    reg signed [63:0] rP1a, rP2a, rP1b, rP2b;

    // ---- T0+8: eps 与正序差/和 ----
    reg signed [63:0] rEpsA, rEpsB, rDifA, rSumB;

    // ---- T0+9: 乘积（每个独立乘积一拍, 各自寄存）----
    reg signed [63:0] rNA, rNB;                    // N 的两项
    reg signed [63:0] rSq1a, rSq2a, rSq1b, rSq2b;  // Vsq 的四项

    // ---- T0+10: 求和 ----
    reg signed [63:0] rNsum, rVsqA, rVsqB;

    // ---- T0+11: Vsq 终和 ----
    reg signed [63:0] rVsq;

    // ---- 每样本除法快照（供后台 AlgoDiv 捕获）----
    reg signed [63:0] rNcur;      // 本样本 N（有符号）
    reg [63:0]        rVsqCur;    // 本样本 Vsq（幅值）

    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            rPipe  <= {LATENCY{1'b0}};
            rVaCap <= {W{1'b0}};
            rVbCap <= {W{1'b0}};
        end else begin
            rPipe[0] <= iEn;
            if (LATENCY > 1)
                rPipe[LATENCY-1:1] <= rPipe[LATENCY-2:0];
            if (iEn) begin
                rVaCap <= iVAlpha;
                rVbCap <= iVBeta;
            end
        end
    end

    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            rV1a <= 64'sd0; rV2a <= 64'sd0; rV1b <= 64'sd0; rV2b <= 64'sd0;
            rP1a <= 64'sd0; rP2a <= 64'sd0; rP1b <= 64'sd0; rP2b <= 64'sd0;
            rVa  <= 64'sd0; rVb  <= 64'sd0;
        end else if (sVaValid && sVbValid) begin   // T0+7: SOGI 提交后一拍, 输出稳定可读
            // rV* = 本样本 SOGI 输出; rP* = 上一样本 SOGI 输出(错位一拍, 供 eps/N 使用)
            rP1a <= rV1a; rP2a <= rV2a; rP1b <= rV1b; rP2b <= rV2b;
            rV1a <= a1a;               // SOGI α 同相   (16bit signed -> 64 扩位)
            rV2a <= v2a;               // SOGI α 正交(-)
            rV1b <= b1a;               // SOGI β 同相
            rV2b <= v2b;               // SOGI β 正交(-)
            rVa  <= rVaCap;            // 本样本输入 α
            rVb  <= rVbCap;            // 本样本输入 β
        end
    end

    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            rEpsA <= 64'sd0; rEpsB <= 64'sd0;
            rDifA <= 64'sd0; rSumB <= 64'sd0;
        end else if (rPipe[7]) begin   // T0+8: 用“当前输入 - 上一拍 SOGI 输出”复现参考错位
            rEpsA <= rVa  - rP1a;      // eps_a = vα - v1a(上一拍)
            rEpsB <= rVb  - rP1b;      // eps_b = vβ - v1b(上一拍)
            rDifA <= rP1a - rP2b;      // v1a - v2b（正序 α）
            rSumB <= rP1b + rP2a;      // v1b + v2a（正序 β）
        end
    end

    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            rNA <= 64'sd0; rNB <= 64'sd0;
            rSq1a <= 64'sd0; rSq2a <= 64'sd0;
            rSq1b <= 64'sd0; rSq2b <= 64'sd0;
        end else if (rPipe[8]) begin   // T0+9：每乘法独立寄存（并行一拍, 均用上一拍 SOGI 输出）
            rNA   <= rEpsA * rP2a;
            rNB   <= rEpsB * rP2b;
            rSq1a <= rP1a * rP1a;
            rSq2a <= rP2a * rP2a;
            rSq1b <= rP1b * rP1b;
            rSq2b <= rP2b * rP2b;
        end
    end

    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            rNsum <= 64'sd0;
            rVsqA <= 64'sd0; rVsqB <= 64'sd0;
        end else if (rPipe[9]) begin   // T0+10
            rNsum <= rNA + rNB;
            rVsqA <= rSq1a + rSq2a;
            rVsqB <= rSq1b + rSq2b;
        end
    end

    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) rVsq <= 64'sd0;
        else if (rPipe[10]) rVsq <= rVsqA + rVsqB;   // T0+11
    end

    // ---- FLL 提交用组合量（寄存器驱动, 每样本提交拍采样一次）----
    wire signed [63:0] wInc   = (-($signed({{48{rNq15[15]}}, rNq15}) * $signed(C_FLL))) >>> 20;
    wire signed [63:0] wDwTmp = wInc + $signed({{32{rDwQ[31]}}, rDwQ});
    wire signed [63:0] wDwMax = $signed({{32{DW_CL[31]}}, DW_CL});
    wire signed [63:0] wDwC   = (wDwTmp >  wDwMax) ?  wDwMax :
                                (wDwTmp < -wDwMax) ? -wDwMax : wDwTmp;
    wire signed [31:0] wDw32   = wDwC[31:0];
    wire [31:0]        wPhi    = $unsigned($signed(WNOM_Q) + wDw32);   // w_est Q6
    wire [63:0]        wThetaDp = wPhi * C_PH;                          // w_est*Ts
    // 正序输出: 0.5 缩放由 fn_sat fb=1 完成（组合 wire, 避免 BLKSEQ）
    wire signed [15:0] wPosA = fn_sat(rDifA[47:0], 5'd1);
    wire signed [15:0] wPosB = fn_sat(rSumB[47:0], 5'd1);

    // ---- 状态 + 提交拍 ----
    reg        [31:0] rThetaAcc;   // 相位累加 2^32<->2pi（自由环绕）
    reg signed [31:0] rDwQ;        // 频率偏差 Q6, 限幅 ±10Hz
    reg signed [31:0] rWEst;       // 估计频率 WNOM_Q+dw（供输出）

    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            rThetaAcc <= 32'h0;
            rDwQ      <= 32'sd0;
            rWEst     <= WNOM_Q;
            oVAlphaPos <= {W{1'b0}};
            oVBetaPos  <= {W{1'b0}};
            rNcur      <= 64'sd0;
            rVsqCur    <= 64'd0;
            oValid     <= 1'b0;
        end else begin
            if (rPipe[LATENCY-1]) begin   // 提交拍
                // FLL 频率积分: dw += -gamma*N/Vsq*Ts*64（每样本一次, rNq15 近 ZOH）
                rDwQ      <= wDw32;
                rWEst     <= WNOM_Q + wDw32;
                rThetaAcc <= rThetaAcc + wThetaDp[31:0];
                // 正序 (0.5*(v1a-v2b), 0.5*(v1b+v2a))
                oVAlphaPos <= wPosA;
                oVBetaPos  <= wPosB;
                // 本样本 N/Vsq 快照（供后台除法）
                rNcur      <= rNsum;
                rVsqCur    <= rVsq;
            end
            oValid <= rPipe[LATENCY-1];
        end
    end

    // ---- 除法后台流水: q15 = floor(|N|*2^15 / Vsq), 再按符号/守卫饱和 ----
    //  从每样本注册快照(rNcur/rVsqCur)捕获, 避免新组合链回到老组合量
    localparam [1:0] PS_IDLE = 2'd0, PS_START = 2'd1, PS_WAIT = 2'd2;
    reg [1:0] ps = PS_IDLE;
    reg signed [63:0] rNcap = 64'sd0;     // N 快照(有符号)
    reg [63:0]        rVsqC = 64'd0;      // Vsq 快照(无符号幅值)
    reg [63:0]        rNmag = 64'd0;      // |N|
    reg signed [15:0] rNq15 = 16'sd0;     // 结果 Q15(FLL 使用)

    wire wDivStart = (ps == PS_START);

    wire [63:0] num_mag = rNmag << 15;              // |N|*2^15
    wire [63:0] den_mag = rVsqC;
    wire [63:0] mxw     = (num_mag > den_mag) ? num_mag : den_mag;
    wire [6:0]  shw     = f_sh31(mxw);
    wire [31:0] num32   = num_mag >> shw;
    wire [31:0] den32   = den_mag >> shw;

    wire        div_done, div_busy;
    wire [31:0] div_quot;
    AlgoDiv #(.W(32)) U_Div (
        .iSysClk(iSysClk), .iSysRst(iSysRst), .iStart(wDivStart),
        .iNum(num32), .iDen(den32),
        .oBusy(div_busy), .oDone(div_done), .oQuot(div_quot));

    // ratio = sign(N)*floor(|N|*2^15/Vsq)；Vsq<MIN_V2 时置 0（与组合版一致）
    wire signed [63:0] qsign = (rNcap[63])
                             ? -$signed({32'd0, div_quot[31:0]})
                             :  $signed({32'd0, div_quot[31:0]});
    wire [63:0] ratio_bits = (rVsqC < MIN_V2) ? 64'd0 : qsign[63:0];
    wire signed [15:0] wNq15 = fn_sat(ratio_bits[47:0], 5'd0);

    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            ps     <= PS_IDLE;
            rNcap  <= 64'sd0;
            rVsqC  <= 64'd0;
            rNmag  <= 64'd0;
            rNq15  <= 16'sd0;
        end else begin
            case (ps)
                PS_IDLE: begin
                    rNcap <= rNcur;               // 空闲即捕获注册快照
                    rVsqC <= rVsqCur;
                    rNmag <= (rNcur[63]) ? (~rNcur[63:0] + 64'd1) : rNcur[63:0];  // |N|
                    ps    <= PS_START;
                end
                PS_START: ps <= PS_WAIT;      // 除法启动脉冲
                PS_WAIT: begin
                    if (div_done) begin
                        rNq15 <= wNq15;
                        ps    <= PS_IDLE;
                    end
                end
                default: ps <= PS_IDLE;
            endcase
        end
    end

    // ---- 输出 ----
    assign oTheta   = rThetaAcc[31:16];
    assign oWEst    = rWEst[15:0];
    assign oErrNorm = rNq15;
endmodule
