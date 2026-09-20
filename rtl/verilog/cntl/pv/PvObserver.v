//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   PvObserver.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
//Description       :   PV电压混合模式标量卡尔曼观测器(10Hz)，估CCM权重alpha
//                      除法/开方统一例化多拍 AlgoDiv (§6.3 ROCA start/done 握手)
//                      z 链路(纯输入函数): 每除法一个独立 AlgoDiv, 空闲自动重启
//                      连续跟踪; KF(有状态反馈): 共享一个 AlgoDiv 顺序 alpha->P,
//                      以除法完成为步进提交(避免滞后商反复提交导致振荡)
//                      平滑 EMA / Voc 于 iMpptEn 拍启动多拍流水(每步一寄存器):
//                      accept 快照 -> (a-aSm)*A_SM 乘法(一拍) -> 舍入+钳位得 aSm_n,
//                      voc_diff 乘 VOC_UP/DN(一拍) -> voc_en/相加限幅提交(一拍)。
//                      乘加无单周期组合, oValid 于提交拍拉高。分子/分母若可能超
//                      31bit 按最大者自适应右移后喂 32bit
//------------------------------------------------------------------------------
//Version           :   Rev 0.4
//modifier          :
//Modify Date       :
//Modify Record     :   2026-09-06 组合除法改为多拍 AlgoDiv 握手化
//                      Rev 0.4 平滑 EMA/Voc 组合乘加拆多拍流水, 新增 oValid
//------------------------------------------------------------------------------

`include "algo_types.vh"

module PvObserver #(
    parameter integer W        = `ALGO_W_VOLT,   // 电压位宽(整数 V/count)
    parameter integer CW       = `ALGO_W_VOLT,   // 电流位宽(整数 mA/count)
    parameter integer DW       = 16,             // duty 位宽, Q0.16
    parameter integer OAW      = 25,             // alpha 输出位宽, Q0.24
    parameter integer VOC_INIT_V = 1000,         // Voc 估计初值(V)
    parameter integer VOC_MAX_V   = 65535,       // 电压输出上限(V)
    parameter integer V_DIFF_EPS = 2             // |Vccm-Vdcm|<=此值时 z2 不可用
) (
    input  wire             iSysClk,
    input  wire             iSysRst,
    input  wire             iMpptEn,       // 10 Hz strobe (dt=0.1s)
    input  wire [DW-1:0]    iDutyQ,        // Q0.16 duty
    input  wire [W-1:0]     iVMeas,        // PV 电压量测 (V)
    input  wire [CW-1:0]    iMeas,         // PV 电流量测 (mA)
    input  wire [W-1:0]     iBusVolt,      // 母线电压 (V)
    output wire [W-1:0]     oVccm,
    output wire [W-1:0]     oVdcm,
    output wire [OAW-1:0]   oAlphaHat,
    output wire [OAW-1:0]   oAlphaHatSmooth,
    output wire [OAW-1:0]   oAlphaHatCorrected,
    output wire [W-1:0]     oVocEst,
    output reg              oValid         // 平滑/Voc 提交拍有效(采样后 LATENCY=3 拍)
);
    // ------------------------------------------------------------------
    // 常量 (Q0.24, LSB=2^-24)
    // ------------------------------------------------------------------
    localparam integer SC       = 1 << 24;                // 2^24, Q0.24 满量程
    localparam integer HSC      = SC >> 1;                // 0.5
    localparam integer DUTY_SC  = 1 << DW;                // 2^16
    localparam integer E16      = 65536;
    localparam integer TWO_LF   = 60;
    localparam integer R1_RAW   = 419;
    localparam integer R2_RAW   = 7;
    localparam integer Q_BASE_RAW   = 335544;
    localparam integer LAMBDA_Q_RAW = 838861;
    localparam integer P_INIT_RAW   = 4194304;
    localparam integer A_INIT_RAW   = 8388608;
    localparam integer A_SM_RAW     = 55365;
    localparam integer VOC_UP_Q16   = 56690;
    localparam integer VOC_DN_Q16   = 11862;
    localparam integer D_OPEN_Q     = 6553;
    localparam integer I_OPEN_MA    = 1000;
    localparam integer VOC_MAX      = VOC_MAX_V;
    localparam [63:0] DEN_ICRIT    = TWO_LF * (1 << DW) * (1 << DW); // 60*2^32
    localparam [31:0] SC_P = SC;

    // ------------------------------------------------------------------
    // 状态寄存器 (alpha/KF 于 KF 完成拍更新; 平滑/Voc/Vccm/Vdcm 于 iMpptEn 或除法完成)
    // ------------------------------------------------------------------
    reg signed [31:0] rAlphaRaw    = A_INIT_RAW;   // alpha_hat, Q0.24
    reg signed [31:0] rPRaw        = P_INIT_RAW;   // cov_P,      Q0.24
    reg signed [31:0] rAlphaSmRaw = A_INIT_RAW;   // alpha smooth Q0.24
    reg [31:0] rVocQ16      = VOC_INIT_V * E16; // Voc<<16
    reg [31:0] rVccm      = 32'd0;              // Vccm 内部(输出取低 W bit)
    reg [31:0] rVdcm      = 32'd0;              // Vdcm 内部(输出取低 W bit)

    // ------------------------------------------------------------------
    // 自适应缩放函数: 64bit 幅度, 求使 num/den 右移后都 <2^31 的移位量
    // ------------------------------------------------------------------
    function [5:0] f_hiidx;
        input [63:0] v;
        integer i;
        reg   got;
        begin
            f_hiidx = 6'd0; got = 1'b0;
            for (i = 63; i >= 0; i = i - 1)
                if (!got && v[i]) begin f_hiidx = i; got = 1'b1; end
        end
    endfunction

    function [5:0] f_shift;
        input [63:0] num;
        input [63:0] den;
        reg [5:0] sn, sd;
        begin
            sn = f_hiidx(num);
            sd = f_hiidx(den);
            sn = (sn > 6'd30) ? (sn - 6'd30) : 6'd0;
            sd = (sd > 6'd30) ? (sd - 6'd30) : 6'd0;
            f_shift = (sn > sd) ? sn : sd;
        end
    endfunction

    // ---- 公共组合量 (幅度均为正) ----
    wire [31:0] busS = iBusVolt;
    wire [31:0] dd   = iDutyQ;
    wire [31:0] one_d = DUTY_SC - dd;
    wire [31:0] iM16 = iMeas;
    wire [31:0] iV16 = iVMeas;

    // ========== Vccm = floor(busS*one_d/DUTY_SC) : 除 2^16 同右移1bit精确 ==========
    wire [63:0] p_vccm = 64'd1 * busS * one_d;               // <= 2^32
    wire [63:0] p_den1 = 64'd1 * busS * dd * dd * 64'd1000;   // <= 2^58
    wire signed [63:0] den1_64 = $signed(p_den1 >>> 32);      // = p_den1/2^32
    wire [63:0] p_icrit = 64'd1 * dd * one_d * busS * 64'd1000;
    wire [31:0] icrit_num = p_icrit[63:32];                  // <= 2^26
    wire signed [63:0] g60i  = $signed(64'd1 * TWO_LF * iM16);
    wire signed [63:0] num_vd = g60i * $signed({32'd0, busS});   // 60I*bus
    wire signed [63:0] den2_64 = den1_64 + g60i;             // den1+60I >=0

    // ========== z 链路组合 (读商缓存 rq_*) ==========
    wire [31:0] icrit_w = rq_icrit;
    wire z1_full = (icrit_w <= 32'd0) || ($unsigned(iM16) >= $unsigned(icrit_w));
    wire [63:0] z1_num64 = $unsigned(iM16) * SC;             // I*2^24
    wire z1_ovf = (icrit_w > 32'sd0) && ($unsigned(iM16) < $unsigned(icrit_w)) &&
                  (z1_num64 >= ({32'd0, icrit_w} << 32));
    wire [31:0] vccm_cur = (rq_vccm > 32'(VOC_MAX)) ? 32'(VOC_MAX) : rq_vccm;
    wire [31:0] vdcm_cur = (rq_vdcm[31]) ? 32'd0 :
                           ((rq_vdcm > 32'(VOC_MAX)) ? 32'(VOC_MAX) : rq_vdcm);
    wire signed [31:0] z1raw = (z1_full || z1_ovf) ? 32'(SC) :
                               ((rq_z1 > SC_P) ? 32'(SC) : $signed(rq_z1));
    wire signed [31:0] vdiff = $signed(vccm_cur) - $signed(vdcm_cur);
    wire [31:0] vdabs = (vdiff < 0) ? $unsigned(-vdiff) : $unsigned(vdiff);
    wire z2_use = (vdabs > V_DIFF_EPS);
    wire signed [31:0] n2 = $signed({32'd0, iV16}) - $signed(vdcm_cur);
    wire signed [63:0] z2_num = $signed(n2) * SC;
    wire signed [63:0] z2_den = vdiff;
    wire [63:0] z2_num_mag = (z2_num < 0) ? $unsigned(-z2_num) : $unsigned(z2_num);
    wire [63:0] z2_den_mag = (z2_den < 0) ? $unsigned(-z2_den) : $unsigned(z2_den);
    wire z2_same = ((z2_num >= 0) && (z2_den >= 0)) || ((z2_num < 0) && (z2_den < 0));
    wire z2_ovf = z2_use && (z2_num_mag >= (z2_den_mag << 32));
    wire signed [31:0] z2raw = z2_use ?
                               ((!z2_same) ? 32'sd0 :
                                (z2_ovf ? 32'(SC) : ((rq_z2 > SC_P) ? 32'(SC) : $signed(rq_z2)))) :
                               32'sd0;
    wire z2ok = z2_use;

    // ========== KF 组合 (读 z/状态) ==========
    wire signed [63:0] e1 = $signed({32'd0, z1raw}) - $signed({ {32{rAlphaRaw[31]}}, rAlphaRaw });
    wire signed [63:0] e2 = $signed({32'd0, z2raw}) - $signed({ {32{rAlphaRaw[31]}}, rAlphaRaw });
    wire signed [63:0] e1sq = (e1 * e1) >>> 24;
    wire signed [63:0] Qraw64 = $signed({32'd0, Q_BASE_RAW}) +
                                (($signed({32'd0, LAMBDA_Q_RAW}) * e1sq) >>> 24);
    wire signed [63:0] Ppre = $signed(rPRaw) + Qraw64;                // >=0
    wire signed [63:0] Rsum = $signed({32'd0, R1_RAW + R2_RAW});
    wire signed [63:0] inn  = $signed({32'd0, R2_RAW}) * e1 + $signed({32'd0, R1_RAW}) * e2;
    wire signed [63:0] den_j = Ppre * Rsum + $signed({32'd0, R1_RAW * R2_RAW});
    wire signed [63:0] den_d = Ppre + $signed({32'd0, R1_RAW});
    wire signed [63:0] numA = z2ok ? (Ppre * inn) : (Ppre * e1);
    wire signed [63:0] denA = z2ok ? den_j : den_d;                   // >0
    wire signed [63:0] numP = z2ok ? (Ppre * R1_RAW * R2_RAW)
                                   : (Ppre * R1_RAW);                 // >=0
    wire [63:0] numA_mag = (numA < 0) ? $unsigned(-numA) : $unsigned(numA);

    // KF 步提交量 (读 KF 商缓存 rqA/rqP)
    wire signed [63:0] kinc = (numA < 0) ? -$signed({32'd0, rqA}) : $signed({32'd0, rqA});
    wire signed [63:0] kalpha_n64 = $signed(rAlphaRaw) + kinc;
    wire signed [31:0] kalpha_n = (kalpha_n64 > 64'(SC)) ? 32'(SC) :
                                  ((kalpha_n64 < 0) ? 32'sd0 : kalpha_n64[31:0]);
    wire signed [31:0] kPn = (rqP > SC_P) ? 32'(SC) : $signed(rqP);
    wire signed [31:0] kPn_f = (denA == 64'sd0) ? rPRaw : kPn;

    // ------------------------------------------------------------------
    // 输出组合
    // ------------------------------------------------------------------
    assign oVccm              = rVccm[W-1:0];
    assign oVdcm              = rVdcm[W-1:0];
    assign oAlphaHat           = rAlphaRaw[OAW-1:0];
    assign oAlphaHatSmooth    = rAlphaSmRaw[OAW-1:0];
    assign oAlphaHatCorrected = rAlphaSmRaw[OAW-1:0];
    assign oVocEst             = rVocQ16[16+W-1:16];

    // =====================================================================
    // 多拍除法例化
    // =====================================================================
    // ---- z 链路 (每个除法一个 AlgoDiv, 空闲自动重启) ----
    wire oBusyV, oDoneV; wire [31:0] oQuotV;
    AlgoDiv #(.W(32)) U_VccmDiv (
        .iSysClk(iSysClk), .iSysRst(iSysRst), .iStart(~oBusyV),
        .iNum(p_vccm[32:1]), .iDen(32'(DUTY_SC >> 1)),
        .oBusy(oBusyV), .oDone(oDoneV), .oQuot(oQuotV));

    wire oBusyI, oDoneI; wire [31:0] oQuotI;
    AlgoDiv #(.W(32)) U_IcritDiv (
        .iSysClk(iSysClk), .iSysRst(iSysRst), .iStart(~oBusyI),
        .iNum(icrit_num), .iDen(32'(TWO_LF)),
        .oBusy(oBusyI), .oDone(oDoneI), .oQuot(oQuotI));

    wire oBusyD, oDoneD; wire [31:0] oQuotD;
    wire [5:0] shD = f_shift($unsigned(num_vd), $unsigned(den2_64));
    wire [31:0] vd_num = ($unsigned(num_vd) >> shD);
    wire [31:0] vd_den = ($unsigned(den2_64) >> shD);
    AlgoDiv #(.W(32)) U_VdcmDiv (
        .iSysClk(iSysClk), .iSysRst(iSysRst), .iStart(~oBusyD),
        .iNum(vd_num), .iDen((vd_den == 32'd0) ? 32'd1 : vd_den),
        .oBusy(oBusyD), .oDone(oDoneD), .oQuot(oQuotD));

    wire oBusy1, oDone1; wire [31:0] oQuot1;
    wire [5:0] sh1 = f_shift(z1_num64, {32'd0, icrit_w});
    wire [31:0] z1_num = (z1_num64 >> sh1);
    wire [31:0] z1_den = ({32'd0, icrit_w} >> sh1);
    AlgoDiv #(.W(32)) U_Z1Div (
        .iSysClk(iSysClk), .iSysRst(iSysRst), .iStart(~oBusy1),
        .iNum(z1_num), .iDen((z1_den == 32'd0) ? 32'd1 : z1_den),
        .oBusy(oBusy1), .oDone(oDone1), .oQuot(oQuot1));

    wire oBusy2, oDone2; wire [31:0] oQuot2;
    wire [5:0] sh2 = f_shift(z2_num_mag, z2_den_mag);
    wire [31:0] wz2_num = (z2_num_mag >> sh2);
    wire [31:0] wz2_den = (z2_den_mag >> sh2);
    AlgoDiv #(.W(32)) U_Z2Div (
        .iSysClk(iSysClk), .iSysRst(iSysRst), .iStart(~oBusy2),
        .iNum(wz2_num), .iDen((wz2_den == 32'd0) ? 32'd1 : wz2_den),
        .oBusy(oBusy2), .oDone(oDone2), .oQuot(oQuot2));

    // ---- KF 共享除法: alpha = floor(|numA|/denA), P = floor(numP/denA) ----
    wire oBusyK, oDoneK; wire [31:0] oQuotK;
    wire [5:0] shA = f_shift(numA_mag, $unsigned(denA));
    wire [31:0] a_num = (numA_mag >> shA);
    wire [31:0] a_den = ($unsigned(denA) >> shA);
    wire [5:0] shP = f_shift(numP, $unsigned(denA));
    wire [31:0] p_num = (numP >> shP);
    wire [31:0] p_den = ($unsigned(denA) >> shP);
    AlgoDiv #(.W(32)) U_KDiv (
        .iSysClk(iSysClk), .iSysRst(iSysRst),
        .iStart(rGoA || rGoP),                          // 一拍启动脉冲
        .iNum(rGoP ? p_num : a_num),
        .iDen(rGoP ? ((p_den == 32'd0) ? 32'd1 : p_den)
                   : ((a_den == 32'd0) ? 32'd1 : a_den)),
        .oBusy(oBusyK), .oDone(oDoneK), .oQuot(oQuotK));

    // ------------------------------------------------------------------
    // z 链路商缓存 + Vccm/Vdcm 输出 (oDone 打一拍后采)
    // ------------------------------------------------------------------
    reg rDoV, rDoI, rDoD, rDo1, rDo2;
    reg [31:0] rq_vccm, rq_icrit, rq_vdcm, rq_z1, rq_z2;
    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            rq_vccm <= 32'd0; rq_icrit <= 32'd0; rq_vdcm <= 32'd0;
            rq_z1 <= 32'd0;   rq_z2 <= 32'd0;
            rDoV <= 1'b0; rDoI <= 1'b0; rDoD <= 1'b0; rDo1 <= 1'b0; rDo2 <= 1'b0;
            rVccm <= 32'd0; rVdcm <= 32'd0;
        end else begin
            rDoV <= oDoneV;  if (rDoV) begin rq_vccm <= oQuotV; rVccm <= (oQuotV > 32'(VOC_MAX)) ? 32'(VOC_MAX) : oQuotV; end
            rDoI <= oDoneI;  if (rDoI) rq_icrit <= oQuotI;
            rDoD <= oDoneD;  if (rDoD) begin rq_vdcm <= oQuotD; rVdcm <= (oQuotD > 32'(VOC_MAX)) ? 32'(VOC_MAX) : oQuotD; end
            rDo1 <= oDone1;  if (rDo1) rq_z1    <= oQuot1;
            rDo2 <= oDone2;  if (rDo2) rq_z2    <= oQuot2;
        end
    end

    // ------------------------------------------------------------------
    // KF 顺序除法 FSM: IDLE -> ALPHA -> P -> COMMIT (每次用全新商)
    // ------------------------------------------------------------------
    localparam [1:0] K_IDLE=2'd0, K_ALPHA=2'd1, K_P=2'd2, K_COMMIT=2'd3;
    reg [1:0]  rKf;
    reg        rKfKick;         // 一拍脉冲，仅由 accept always 驱动
    reg        rGoA, rGoP;       // 一拍启动脉冲
    reg [31:0] rqA, rqP;         // alpha / P 商
    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            rKf <= K_IDLE; rGoA <= 1'b0; rGoP <= 1'b0;
            rqA <= 32'd0; rqP <= 32'd0;
            rAlphaRaw <= A_INIT_RAW[31:0];
            rPRaw     <= P_INIT_RAW[31:0];
        end else begin
            rGoA <= 1'b0; rGoP <= 1'b0;      // 默认脉冲清零
            case (rKf)
                K_IDLE: begin
                    if (rKfKick) begin
                        rKf <= K_ALPHA;
                        rGoA <= 1'b1;
                    end
                end
                K_ALPHA: begin
                    if (oDoneK) begin
                        rqA <= oQuotK;
                        rKf <= K_P;
                        rGoP <= 1'b1;
                    end
                end
                K_P: begin
                    if (oDoneK) begin
                        rqP <= oQuotK;
                        rKf <= K_COMMIT;
                    end
                end
                K_COMMIT: begin
                    rAlphaRaw <= kalpha_n;
                    rPRaw     <= kPn_f;
                    rKf       <= K_IDLE;
                end
                default: rKf <= K_IDLE;
            endcase
        end
    end

    // =====================================================================
    // 平滑 EMA + Voc 多拍流水 (iMpptEn 启动, 每步一寄存器, oValid 提交拍)
    // rEv: [0]=aSm 乘法  [1]=aSm 舍入钳位 + voc 乘法  [2]=提交平滑/Voc
    // =====================================================================
    reg [2:0] rEv;
    reg       rBusy;
    reg signed [31:0] rASnap;      // alpha 快照(accept)
    reg [W-1:0] rV16Snap;          // 电压量测快照
    reg [DW-1:0] rDdSnap;          // duty 快照
    reg [CW-1:0] rMSnap;           // 电流量测快照
    reg signed [63:0] rVDiff;      // voc 差值(accept 一拍)
    reg signed [63:0] rSMul;       // (a-aSm)*A_SM(乘法一拍)
    reg signed [31:0] rSmN;        // 新平滑 alpha(舍入钳位一拍)
    reg signed [63:0] rVG;         // voc_diff*VOC_UP/DN(乘法一拍)

    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            rEv    <= 3'b000;
            rBusy  <= 1'b0;
            rKfKick<= 1'b0;
            rASnap <= 32'sd0;
            rV16Snap <= {W{1'b0}};
            rDdSnap  <= {DW{1'b0}};
            rMSnap   <= {CW{1'b0}};
            rVDiff <= 64'sd0;
        end else begin
            rEv[2] <= rEv[1];
            rEv[1] <= rEv[0];
            rEv[0] <= 1'b0;
            rKfKick<= 1'b0;                          // 默认一拍脉冲清零
            if (iMpptEn && !rBusy) begin
                rEv[0]   <= 1'b1;                    // 下拍进入平滑乘法级
                rASnap   <= rAlphaRaw;               // alpha 快照
                rV16Snap <= iVMeas;                  // 输入快照(diff/en 用)
                rDdSnap  <= iDutyQ;
                rMSnap   <= iMeas;
                // voc_diff = V*2^16 - Voc<<16 (一拍)
                rVDiff   <= $signed({32'd0, iVMeas}) * E16 - $signed({32'd0, rVocQ16});
                rKfKick  <= 1'b1;                    // 一拍请求 KF（仅本 always 驱动）
                rBusy    <= 1'b1;
            end
            if (rEv[2]) begin
                rBusy  <= 1'b0;
            end
        end
    end

    // 组合量(读流水寄存): 新平滑 alpha = aSm + ((a-aSm)*A_SM >>24), 限幅[0,SC]
    wire signed [63:0] wSmN = $signed(rAlphaSmRaw) + (rSMul >>> 24);
    wire signed [31:0] wSmCl = (wSmN > 64'(SC)) ? 32'(SC) :
                               ((wSmN < 0) ? 32'sd0 : wSmN[31:0]);
    // voc 使能(用本事件新平滑)
    wire voc_en = (rDdSnap <= D_OPEN_Q) && ($unsigned(rMSnap) < I_OPEN_MA) &&
                  (wSmCl < HSC);
    // voc 加量 = Voc + (voc_g >>16)
    wire signed [63:0] wVocSum = $signed({32'd0, rVocQ16}) + (rVG >>> 16);
    wire [31:0] wVocNxt = voc_en ? ((wVocSum < 0) ? 32'd0 : wVocSum[31:0]) : rVocQ16;

    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            rAlphaSmRaw <= A_INIT_RAW[31:0];
            rVocQ16     <= VOC_INIT_V * E16;
            oValid      <= 1'b0;
            rSMul <= 64'sd0;
            rSmN  <= 32'sd0;
            rVG   <= 64'sd0;
        end else begin
            oValid <= 1'b0;
            if (rEv[0])
                rSMul <= (rASnap - $signed(rAlphaSmRaw)) * A_SM_RAW; // 一拍: 平滑差乘法
            if (rEv[1]) begin
                rSmN <= wSmCl;                       // 一拍: 平滑舍入+限幅
                rVG  <= (rVDiff >= 0) ? (rVDiff * VOC_UP_Q16)      // voc 乘法一拍
                                      : (rVDiff * VOC_DN_Q16);
            end
            if (rEv[2]) begin
                rAlphaSmRaw <= rSmN;                 // 一拍: 提交平滑 alpha
                rVocQ16     <= wVocNxt;              // 一拍: 提交 Voc
                oValid      <= 1'b1;
            end
        end
    end
endmodule
