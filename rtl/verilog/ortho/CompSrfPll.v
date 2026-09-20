//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   CompSrfPll.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.06
/*Description       :   SRF-PLL 同步参考坐标系 dq 锁相环（两相 α/β 输入）。
                        结构: Park(d=va*cos+vb*sin, q=-va*sin+vb*cos, 用当前相位,
                        无 SOGI 故不需要预测角) -> q 幅值归一化
                        err=-q/|v|（AlgoSqrt+AlgoDiv 多拍后台求幅/倒数, 误差通路无延迟）
                        -> PI(Δw=Kp*err+Ki*∫err, 限幅 ±10Hz) -> w=2pi*f_nom+Δw
                        -> θ+=w*Ts (32bit 累加, oTheta=高16全幅) -> AlgoTrig 生成
                        cos/sin 反馈。锁定后 vq->0、oCos 与 vα 同相、w~2pi*50。
                        信号 S16; 角频率 S32 Q6(1LSB=1/64 rad/s); err Q15。
                        幅值后台刷新 invA=2^30/A(约70拍), 相位误差逐样本 = q*invA>>15。
                        符号: 依本两相 park(iBeta=+vB)约定, err=-q/A 使同相 vd>0 稳定。
                        【多拍流水版】iEn 每样本一拍触发, 原单周期组合乘加链拆成 LATENCY 拍
                        流水: 接受拍 Park 4 乘法并行 -> 求和 -> 饱和 -> err 乘法 -> err 定标
                        -> 比例/积分项乘积 -> 积分器累加+限幅 -> w 合成+限幅 -> θ 累加乘积
                        -> 提交拍(θ 累加 + 状态/输出更新)。oValid 在采样拍后 LATENCY 拍拉高一拍。
                        每个乘法结果单独寄存, 加/饱和各一拍, fn_sat 仅以组合 wire 调用;
                        θ/rIInt/rWQ 状态只在提交拍(rPipe[LATENCY-1])更新。数值公式/系数/
                        Q格式与原单周期版逐样本等价(同一采样序列逐样本 bit 一致)。
  参数   :   KP 比例增益 rad/s(每单位误差), KI 积分增益 rad/s^2(每单位误差);
            FS/F_NOM 须保持 5000/50(常量表按此生成); 建议 KP<=6。
  修订   :   2026-09-06 v0.2 (多拍流水化 + oValid)
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.2
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
`include "algo_arith.vh"

module CompSrfPll #(
    parameter integer W       = 16,
    parameter integer FS      = 5000,     // 采样率 Hz（常量表须匹配）
    parameter integer F_NOM   = 50,       // 标称频率 Hz
    parameter integer KP      = 4,        // 比例增益 rad/s/单位误差
    parameter integer KI      = 8,        // 积分增益 rad/s^2/单位误差
    parameter integer LATENCY = 9         // 流水拍数(接受拍 -> oValid 拉高)
) (
    input  wire                 iSysClk,
    input  wire                 iSysRst,
    input  wire                 iEn,           // 采样节拍（单拍脉冲）
    input  wire signed [W-1:0]  iVAlpha,       // α 轴采样
    input  wire signed [W-1:0]  iVBeta,        // β 轴采样
    output wire [W-1:0]         oTheta,        // 相位 16bit 全幅 0..65535<->0..2pi
    output reg  signed [W-1:0]  oCos,          // NCO 余弦（提交拍寄存）
    output reg  signed [W-1:0]  oD,            // Park d（提交拍寄存, 锁定后 ≈ 幅值）
    output reg  signed [W-1:0]  oQ,            // Park q（提交拍寄存, 锁定后 -> 0）
    output wire signed [W-1:0]  oW,            // 估计角频率 Q6(1LSB=1/64 rad/s)
    output reg  signed [W-1:0]  oErr,          // 归一化相位误差 Q15(提交拍寄存)
    output reg                  oValid         // 本样本结果已提交（一拍）
);
    // ---- 定点常量（FS=5000 / F_NOM=50 语义）----
    localparam signed [31:0] WNOM_Q  = (F_NOM * 32'sd402124 + 32'sd500) / 32'sd1000; // ~2*pi*F_NOM*64
    localparam [31:0]       C_PH    = (32'd2136 * 32'd5000) / FS;   // ~2^32/(2*pi*FS*64)
    localparam signed [31:0] DW_CL  = 32'sd4021;     // 2*pi*10*64            10Hz 限幅
    localparam signed [31:0] C_KP   = KP * 64;       // Kp -> Q6 系数
    localparam signed [31:0] C_KI   = (KI * 65536 + FS/2) / FS;  // Ki -> per-sample 系数
    localparam signed [31:0] W_HI   = WNOM_Q + DW_CL;
    localparam signed [31:0] W_LO   = WNOM_Q - DW_CL;
    localparam [31:0]       INV_NUM = 32'd1073741824;  // 2^30, 求 invA=2^30/A
    localparam [63:0]       RD_HALF = 64'sd16384;      // Q15 舍入半
    localparam [31:0]       MIN_A   = 32'd8;           // A 下限防除小

    // ---- 参考振荡器（组合 LUT, 由当前相位 oTheta 驱动）----
    wire signed [15:0] wCTh, wSTh;
    AlgoTrig U_Osc (.iAngle(oTheta), .oSin(wSTh), .oCos(wCTh));

    // ---- 采样拍移位链：rPipe[LATENCY-1] 即提交拍 ----
    reg [LATENCY-1:0] rPipe;

    // ---- 状态（仅在提交拍更新）----
    reg        [31:0] rThetaAcc;   // 相位累加 2^32<->2pi（自由环绕）
    reg signed [31:0] rIInt;       // 积分器 Q6, 限幅 ±10Hz
    reg signed [31:0] rWQ;         // 当前角频率 Q6

    // ---- 逐拍中间量（乘法/加法/饱和各自一拍, 独立寄存）----
    reg signed [15:0] rCth;        // 接受拍 cos(θ) 快照（供 oCos 提交）
    reg signed [47:0] rM1, rM2, rM3, rM4;   // va*cos, vb*sin, va*sin, vb*cos
    reg signed [47:0] rDsum, rQsum;         // d/q 求和（未饱和）
    reg signed [15:0] rdSat, rqSat;         // d/q 饱和(S16)
    reg signed [63:0] rErrm;                // -q*invA（Q? 原始宽）
    reg signed [15:0] rErr;                 // 归一化误差 Q15
    reg signed [63:0] rPp, rIp;             // err*C_KP, err*C_KI
    reg signed [31:0] rIIn;                 // 新积分器（限幅后, 待提交）
    reg signed [31:0] rWt;                  // w 合成（限幅后, 待提交）
    reg [63:0]        rThMul;               // w*C_PH（待提交）

    // ---- 组合定标/sat（fn_sat 为阻塞赋值函数, 只能在时序块外以 wire 调用）----
    wire signed [15:0] wDsat = fn_sat(rDsum, 5'd15);
    wire signed [15:0] wQsat = fn_sat(rQsum, 5'd15);
    wire signed [63:0] wNegQ = -$signed({{48{rqSat[15]}}, rqSat});   // -q（S64）
    wire signed [63:0] wErrs = (rErrm + RD_HALF) >>> 15;            // err 定标(Q15)
    wire signed [63:0] wPsh  = $signed(rPp) >>> 15;                 // p=err*Kp*64>>15
    wire signed [31:0] wPv   = wPsh[31:0];
    wire signed [63:0] wIsh  = $signed(rIp) >>> 25;                 // di=err*Ki>>25
    wire signed [31:0] wIv   = wIsh[31:0];
    wire signed [31:0] wIAdd = rIInt + wIv;                         // 积分器累加
    wire signed [31:0] wIAddC = (wIAdd >  DW_CL) ?  DW_CL :
                                (wIAdd < -DW_CL) ? -DW_CL : wIAdd;  // 限幅 ±10Hz
    wire signed [31:0] wWRaw = WNOM_Q + wPv + rIIn;                 // w=wnom+p+i
    wire signed [31:0] wWCl  = (wWRaw >  W_HI) ?  W_HI :
                               (wWRaw <  W_LO) ?  W_LO : wWRaw;     // 限幅 ±10Hz

    // 有符号 64bit -> S16 钳位（仅组合, 不做位移; 供 err 定标后钳位）
    function signed [15:0] f_sat16;
        input signed [63:0] v;
        reg signed [63:0] tv;
        begin
            if      (v >  64'sd32767)  tv = 64'sd32767;
            else if (v < -64'sd32768)  tv = -64'sd32768;
            else                       tv = v;
            f_sat16 = tv[15:0];
        end
    endfunction
    wire signed [15:0] wErr = f_sat16(wErrs);   // err S16 钳位

    // ---- 主流水：接受拍 ~ 提交拍 ----
    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            rPipe      <= {LATENCY{1'b0}};
            rCth       <= 16'sd0;
            rM1 <= 48'sd0; rM2 <= 48'sd0; rM3 <= 48'sd0; rM4 <= 48'sd0;
            rDsum <= 48'sd0; rQsum <= 48'sd0;
            rdSat <= 16'sd0; rqSat <= 16'sd0;
            rErrm <= 64'sd0; rErr <= 16'sd0;
            rPp <= 64'sd0; rIp <= 64'sd0;
            rIIn <= 32'sd0; rWt <= 32'sd0; rThMul <= 64'd0;
            rThetaAcc <= 32'h0;
            rIInt     <= 32'sd0;
            rWQ       <= WNOM_Q;
            oD   <= 16'sd0;
            oQ   <= 16'sd0;
            oErr <= 16'sd0;
            oCos <= 16'sd0;
            oValid <= 1'b0;
        end else begin
            // 采样拍移位
            rPipe[0] <= iEn;
            if (LATENCY > 1)
                rPipe[LATENCY-1:1] <= rPipe[LATENCY-2:0];

            // 接受拍: 快照 cos(θ_n) + Park 4 乘法并行（各单独寄存）
            if (iEn) begin
                rCth <= wCTh;
                rM1  <= $signed(iVAlpha) * wCTh;
                rM2  <= $signed(iVBeta)  * wSTh;
                rM3  <= $signed(iVAlpha) * wSTh;
                rM4  <= $signed(iVBeta)  * wCTh;
            end
            // 拍: d/q 求和（d=M1+M2, q=-va*sin+vb*cos = M4-M3）
            if (rPipe[0]) begin
                rDsum <= rM1 + rM2;
                rQsum <= rM4 - rM3;
            end
            // 拍: d/q 饱和 -> S16
            if (rPipe[1]) begin
                rdSat <= wDsat;
                rqSat <= wQsat;
            end
            // 拍: 归一化误差乘法 err_raw = -q * invA
            if (rPipe[2]) begin
                rErrm <= wNegQ * $signed(rInvA);
            end
            // 拍: err 定标 + S16 钳位（Q15）
            if (rPipe[3]) begin
                rErr <= wErr;
            end
            // 拍: 比例/积分项乘积（并行两乘法）
            if (rPipe[4]) begin
                rPp <= $signed(rErr) * C_KP;
                rIp <= $signed(rErr) * C_KI;
            end
            // 拍: 积分器累加 + 限幅（状态暂存 rIIn, 提交拍才写 rIInt）
            if (rPipe[5]) begin
                rIIn <= wIAddC;
            end
            // 拍: w 合成 + 限幅（状态暂存 rWt, 提交拍才写 rWQ）
            if (rPipe[6]) begin
                rWt <= wWCl;
            end
            // 拍: θ 累加增量乘积 w*C_PH
            if (rPipe[7]) begin
                rThMul <= $unsigned(rWt) * C_PH;
            end
            // 提交拍: θ 累加 + 状态/输出更新（oValid 由 rPipe 尾拍送出）
            if (rPipe[LATENCY-1]) begin
                rThetaAcc <= rThetaAcc + rThMul[31:0];
                rIInt     <= rIIn;
                rWQ       <= rWt;
                oD        <= rdSat;
                oQ        <= rqSat;
                oErr      <= rErr;
                oCos      <= rCth;
            end
            oValid <= rPipe[LATENCY-1];
        end
    end

    // ---- 幅值 A=sqrt(d^2+q^2) 后台多拍（AlgoSqrt）----
    wire        sqrt_done, sqrt_busy;
    wire [31:0] sqrt_root;
    wire        div_done, div_busy;
    wire [31:0] div_quot;
    wire        sqrt_start = (psBg == SBG_SQST);
    wire        div_start  = (psBg == SBG_DVST);
    AlgoSqrt #(.IW(64), .OW(32)) U_Sqrt (
        .iSysClk(iSysClk), .iSysRst(iSysRst), .iStart(sqrt_start),
        .iNum(rBgA2), .oBusy(sqrt_busy), .oDone(sqrt_done), .oRoot(sqrt_root));
    AlgoDiv #(.W(32)) U_Div (
        .iSysClk(iSysClk), .iSysRst(iSysRst),
        .iStart(div_start), .iNum(INV_NUM), .iDen(rAest),
        .oBusy(div_busy), .oDone(div_done), .oQuot(div_quot));

    // ---- 幅值/invA 后台状态机（自跑, 空闲捕获已提交 oD/oQ -> 平方 -> sqrt -> div）----
    // A2=d^2+q^2 拆成平方/加法各一拍, 无组合乘法; 平方用已提交的 oD/oQ（样本间稳定）。
    localparam [2:0] SBG_IDLE = 3'd0, SBG_SQ1 = 3'd1, SBG_SQ2 = 3'd2,
                     SBG_SQST = 3'd3, SBG_SQRW = 3'd4,
                     SBG_DVST = 3'd5, SBG_DVRW = 3'd6;
    reg [2:0]  psBg;
    reg signed [15:0] rBgD, rBgQ;   // 已提交 d/q 快照
    reg signed [63:0] rBgD2, rBgQ2; // d^2, q^2
    reg [63:0] rBgA2;               // d^2+q^2
    reg [31:0] rAest;               // A 估计
    reg [31:0] rInvA;               // 2^30/A

    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            psBg  <= SBG_IDLE;
            rBgD  <= 16'sd0; rBgQ <= 16'sd0;
            rBgD2 <= 64'sd0; rBgQ2 <= 64'sd0;
            rBgA2 <= 64'd0;
            rAest <= 32'd0;
            rInvA <= 32'd0;
        end else begin
            case (psBg)
                SBG_IDLE: begin            // 空闲捕获当前已提交 d/q
                    rBgD <= oD; rBgQ <= oQ;
                    psBg  <= SBG_SQ1;
                end
                SBG_SQ1: begin             // 平方（并行两乘法）
                    rBgD2 <= rBgD * rBgD;
                    rBgQ2 <= rBgQ * rBgQ;
                    psBg  <= SBG_SQ2;
                end
                SBG_SQ2: begin             // 求和 A2=d^2+q^2
                    rBgA2 <= $unsigned(rBgD2 + rBgQ2);
                    psBg  <= SBG_SQST;
                end
                SBG_SQST: begin            // sqrt 启动脉冲（iNum=rBgA2 有效）
                    psBg  <= SBG_SQRW;
                end
                SBG_SQRW: begin
                    if (sqrt_done) begin
                        rAest <= sqrt_root;
                        psBg  <= SBG_DVST;
                    end
                end
                SBG_DVST: begin            // div 启动脉冲（iDen=rAest 有效）
                    psBg  <= SBG_DVRW;
                end
                SBG_DVRW: begin
                    if (div_done) begin
                        if (rAest >= MIN_A) rInvA <= div_quot;   // 弱信号保持旧值
                        psBg <= SBG_IDLE;
                    end
                end
                default: psBg <= SBG_IDLE;
            endcase
        end
    end

    // ---- 输出 ----
    assign oTheta = rThetaAcc[31:16];
    assign oW     = rWQ[15:0];
endmodule
