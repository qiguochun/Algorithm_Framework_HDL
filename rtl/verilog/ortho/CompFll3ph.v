//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   CompFll3ph.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
/*Description       :   三相 SOGI-FLL 频率锁定环（对齐 C Comp_Fll_3ph_Run_F32）。
                        结构: Clarke(Power) -> 双 SOGI(α/β) -> 正序 v_ap=0.5(v1a-v2b), v_bp=0.5(v1b+v2a)
                        -> FLL 归一化 dw=-gamma*(ea*v2a+eb*v2b)/Vsq（限幅 ±2pi*10）
                        -> w_est=w_nom+dw -> θ+=w_est*Ts -> 可选 Park(αβ->dq)。
                        信号 S16；角频率 S32 Q6(1LSB=1/64 rad/s)；θ 32bit 累加，oTheta=高16位全幅。
                        例化 CompClark / CompSogi / CompPark。
                        除法统一例化 AlgoDiv（多拍 start/done 握手，参照 ROCA，禁止组合
                        除法）：后台空闲即捕获最新 N/Vsq 快照运算，完成后刷新 rNq15（Q15
                        缩放，ZOH 滞后，准稳态环路不变）。
                        【多拍流水版】每样本一拍 iEn 触发：接受拍快照 Clarke(cl_a/cl_b)，
                        双 SOGI(SOGI_L=6)后台流水处理该样本；同拍冻结 SOGI 上一拍输出
                        v1(n-1)（接受初期读到上一样本值），随后逐拍（残差/正序差和 →
                        N=ea*v2a+eb*v2b 两乘 / Vsq=sum v² 四乘 → 求和 → Vsq 总和）计算，
                        最终 LATENCY 拍 FLL 状态提交拍 (dw 积分限幅、θ 累加、正序 0.5
                        缩放饱和注册) 并拉高 oValid 一拍。
                        数值与原组合链一致：保留系数、Q6/Q15 缩放、±10Hz 限幅、fn_sat。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.3  (多拍流水化)
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
`include "algo_arith.vh"

module CompFll3ph #(
    parameter integer W     = 16,
    parameter integer FS    = 5000,     // 采样率 Hz（须与 CompSogi 默认 Ts 匹配）
    parameter integer F_NOM = 50,       // 标称频率 Hz
    parameter integer GAMMA = 100,      // FLL 增益（50~500）
    parameter integer K_SOGI= 1414,     // SOGI 阻尼 k*1000（文档值, 已折入 CompSogi 系数）
    parameter integer LATENCY= 12       // 采样接受 -> oValid 拍数（>= 9；需容纳 SOGI_L+提交）
) (
    input  wire                 iSysClk,
    input  wire                 iSysRst,
    input  wire                 iEn,      // 采样节拍（一拍，与 SOGI 共用以接受当前样本）
    input  wire signed [W-1:0]  iVa,      // A 相采样
    input  wire signed [W-1:0]  iVb,      // B 相采样
    input  wire signed [W-1:0]  iVc,      // C 相采样
    output wire [W-1:0]         oTheta,   // 相位 16bit 全幅 0..65535<->0..2pi
    output wire signed [W-1:0]  oWEst,    // 估计角频率 Q6(1LSB=1/64 rad/s)
    output reg  signed [W-1:0]  oVAlphaPos,  // 正序 α（提交拍注册输出）
    output reg  signed [W-1:0]  oVBetaPos,   // 正序 β（提交拍注册输出）
    output reg                  oValid,   // 本样本结果在 LATENCY 拍后拉高一拍
    output wire signed [W-1:0]  oVd,      // 可选 Park d（正序幅值侧）
    output wire signed [W-1:0]  oVq       // 可选 Park q
);
    // ---- 内部 SOGI 流水拍数（与 CompSogi 提交拍一致，接受后 SOGI_L 拍提交）----
    localparam integer SOGI_L = 6;

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

    // ---- 定点常量（FS=5000 语义）----
    localparam signed [31:0] WNOM_Q  = (F_NOM * 32'sd402124 + 32'sd500) / 32'sd1000; // ~2*pi*F_NOM*64
    localparam [31:0]       C_PH    = (32'd2136 * 32'd5000) / FS;   // ~2^32/(2*pi*FS*64)
    localparam signed [31:0] DW_CL  = 32'sd4021;     // 2*pi*10*64            10Hz
    // FLL 积分增益: 每样本 dw 增量 = -n_q15*C_FLL>>20, 系数约 gamma*0.4096
    localparam signed [31:0] C_FLL   = (GAMMA * 41 + 50) / 100;
    localparam [63:0]        MIN_V2  = 64'd32768;    // Vsq 下限(幅~90)防小信号增益

    // ---- Clarke(Power): alpha=(2a-b-c)/3, beta=(b-c)/sqrt3（组合，接受拍快照）----
    wire signed [W-1:0] cl_a, cl_b, cl_z;
    CompClark #(.W(W)) U_Ck (.iA(iVa), .iB(iVb), .iC(iVc),
                             .oAlpha(cl_a), .oBeta(cl_b), .oZero(cl_z));

    // ---- 双 SOGI（α/β 轴），iEn 与自身采样节拍一致 ----
    wire signed [W-1:0] a1a, a2a, b1a, b2a;   // sogi 输出 in/quad
    wire                wSaValid, wSbValid;
    CompSogi #(.LATENCY(SOGI_L)) U_Sa (.iSysClk(iSysClk), .iSysRst(iSysRst), .iEn(iEn),
                                       .iXIn(cl_a), .oVAlpha(a1a), .oVBeta(a2a),
                                       .oValid(wSaValid));
    CompSogi #(.LATENCY(SOGI_L)) U_Sb (.iSysClk(iSysClk), .iSysRst(iSysRst), .iEn(iEn),
                                       .iXIn(cl_b), .oVAlpha(b1a), .oVBeta(b2a),
                                       .oValid(wSbValid));

    // ---- C 约定 v2 = v1 滞后 90: 库 v_beta_o = v_alpha 超前90, 取负对齐 ----
    wire signed [W-1:0] v2a = -a2a;
    wire signed [W-1:0] v2b = -b2a;

    // ---- 流水拍移位链（0=接受拍，k 号 tap 于接受后第 k+1 拍执行；提交=末拍）----
    reg [LATENCY-1:0] rPipe;

    // ---- 各拍中间量（64bit 符号扩展，乘法/加法/饱和各自一拍独立寄存）----
    reg signed [63:0] rClA;      // 接受拍 Clarke alpha 快照
    reg signed [63:0] rClB;      // 接受拍 Clarke beta  快照
    reg signed [63:0] rV1a, rV2a, rV1b, rV2b;   // 接受拍冻结 SOGI 上一拍输出 v1/v2(n-1)
    reg signed [63:0] rEpsA, rEpsB;             // 残差 eps = cl - v1
    reg signed [63:0] rDif, rSum;               // 正序 (v1a-v2b)/(v1b+v2a)
    reg signed [63:0] rMa, rMb;                 // N 两乘
    reg signed [63:0] rS1, rS2, rS3, rS4;       // Vsq 四乘
    reg signed [63:0] rNa;                      // N = rMa+rMb
    reg signed [63:0] rVs12, rVs34;             // Vsq 部分和
    reg signed [63:0] rVsq;                     // Vsq 总和

    // ---- 正序输出饱和为组合函数（避免时序内直接调 fn_sat）----
    wire signed [W-1:0] wSatAp = fn_sat(rDif[47:0], 5'd1);
    wire signed [W-1:0] wSatBp = fn_sat(rSum[47:0], 5'd1);

    // ---- FLL 频率积分链（组合线，只读状态，提交拍才写入）----
    // dw += -rNq15*C_FLL>>20，限幅 ±DW_CL
    wire signed [63:0] wFllInc  = -($signed({{48{rNq15[15]}}, rNq15}) * C_FLL) >>> 20;
    wire signed [63:0] wDwRaw   = $signed(rDwQ) + wFllInc;
    wire signed [63:0] wDwClp   = (wDwRaw > $signed({32'd0,DW_CL})) ? $signed({32'd0,DW_CL})
                                : (wDwRaw < -$signed({32'd0,DW_CL})) ? -$signed({32'd0,DW_CL})
                                : wDwRaw;
    // θ += w_est*Ts（w_est 由 WNOM_Q+dw 合成）
    wire [63:0] wThetaMul = $unsigned(WNOM_Q + wDwClp[31:0]) * C_PH;

    // ---- 除法后台流水: q15 = floor(|N|*2^15 / Vsq), 再按符号/守卫饱和（ZOH 刷新）----
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
    // rNq15 饱和收缩（组合函数，时序内仅搬运，避免 BLKSEQ）
    wire signed [15:0] wNq15 = fn_sat(ratio_bits[47:0], 5'd0);

    // ---- 除法状态机（独立 always）----
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
                    rNcap <= rNa;             // 空闲即捕获当前样本 N/Vsq
                    rVsqC <= rVsq[63:0];
                    rNmag <= (rNa[63]) ? (~rNa[63:0] + 64'd1) : rNa[63:0];  // |N|
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

    // ---- 状态 ----
    reg        [31:0] rThetaAcc;   // 相位累加 2^32<->2pi（自由环绕）
    reg signed [31:0] rDwQ;        // 频率偏差 Q6, 限幅 ±10Hz
    reg signed [31:0] rWEst;       // 估计频率 WNOM_Q+dw（供输出）

    // ---- 逐拍流水（接受 -> SOGI 提交冻结 -> 残差/正序 -> N/Vsq -> FLL 提交）----
    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            rPipe     <= {LATENCY{1'b0}};
            rClA      <= 64'sd0; rClB <= 64'sd0;
            rV1a <= 64'sd0; rV2a <= 64'sd0; rV1b <= 64'sd0; rV2b <= 64'sd0;
            rEpsA <= 64'sd0; rEpsB <= 64'sd0;
            rDif  <= 64'sd0; rSum  <= 64'sd0;
            rMa <= 64'sd0; rMb <= 64'sd0;
            rS1 <= 64'sd0; rS2 <= 64'sd0; rS3 <= 64'sd0; rS4 <= 64'sd0;
            rNa   <= 64'sd0;
            rVs12 <= 64'sd0; rVs34 <= 64'sd0;
            rVsq  <= 64'sd0;
            rThetaAcc  <= 32'h0;
            rDwQ       <= 32'sd0;
            rWEst      <= WNOM_Q;
            oVAlphaPos <= {W{1'b0}};
            oVBetaPos  <= {W{1'b0}};
            oValid     <= 1'b0;
        end else begin
            // 移位链
            rPipe[0] <= iEn;
            if (LATENCY > 1)
                rPipe[LATENCY-1:1] <= rPipe[LATENCY-2:0];

            // 接受拍：快照 Clarke + 冻结上一拍 SOGI 输出 v1(n-1)（SOGI 经同一 iEn
            // 同步接受本样本，旧输出尚未被本样本覆盖；与原组合链残差一致）
            if (iEn) begin
                rClA <= {{(64-W){cl_a[W-1]}}, cl_a};
                rClB <= {{(64-W){cl_b[W-1]}}, cl_b};
                rV1a <= {{(64-W){a1a[W-1]}}, a1a};
                rV2a <= {{(64-W){v2a[W-1]}}, v2a};
                rV1b <= {{(64-W){b1a[W-1]}}, b1a};
                rV2b <= {{(64-W){v2b[W-1]}}, v2b};
            end
            // 拍1：残差/正序 差和（各自独立加减）
            if (rPipe[1]) begin
                rEpsA <= rClA - rV1a;
                rEpsB <= rClB - rV1b;
                rDif  <= rV1a - rV2b;
                rSum  <= rV1b + rV2a;
            end
            // 拍2：N 两乘 / Vsq 四乘（每乘独立寄存）
            if (rPipe[2]) begin
                rMa  <= rEpsA * rV2a;
                rMb  <= rEpsB * rV2b;
                rS1  <= rV1a * rV1a;
                rS2  <= rV2a * rV2a;
                rS3  <= rV1b * rV1b;
                rS4  <= rV2b * rV2b;
            end
            // 拍3：N 求和 / Vsq 部分和
            if (rPipe[3]) begin
                rNa   <= rMa + rMb;
                rVs12 <= rS1 + rS2;
                rVs34 <= rS3 + rS4;
            end
            // 拍4：Vsq 总和
            if (rPipe[4]) begin
                rVsq <= rVs12 + rVs34;
            end

            // 提交拍（末拍）：FLL 状态更新 + 正序输出 + oValid
            if (rPipe[LATENCY-1]) begin
                rDwQ       <= wDwClp[31:0];
                rWEst      <= WNOM_Q + wDwClp[31:0];
                rThetaAcc  <= rThetaAcc + wThetaMul[31:0];
                oVAlphaPos <= wSatAp;
                oVBetaPos  <= wSatBp;
            end
            oValid <= rPipe[LATENCY-1];
        end
    end

    // ---- 输出 ----
    assign oTheta = rThetaAcc[31:16];
    assign oWEst  = rWEst[15:0];

    // 可选 Park（组合一层，接提交后的正序/相位）
    CompPark #(.W(W)) U_Park (.iAlpha(oVAlphaPos), .iBeta(oVBetaPos),
                              .iTheta(oTheta), .oD(oVd), .oQ(oVq));
endmodule
