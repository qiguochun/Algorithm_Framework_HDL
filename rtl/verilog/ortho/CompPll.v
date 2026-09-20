//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   CompPll.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
/*Description       :   单相 SOGI-PLL 锁相环。
                        结构: SOGI(50Hz Tustin 正交) -> Park(θ+w*Ts 预测角) -> 归一化误差
                        err=q/sqrt(d^2+q^2) -> PI(i+=err, dw=i*0.05+err*100, 限幅±62.8)
                        -> w=2pi*50+dw -> θ+=w*Ts -> out=cos(θ) -> kalman1d 平滑 d 与 w。
                        信号 S16；相位 32bit 累加(2^32<->2π)；w S32 Q6；err Q15。
                        例化 CompSogi / CompPark / AlgoTrig / CompKalman1d。

                        【多拍流水版】每样本一拍 iEn 触发，oValid 于提交拍拉高。
                        级联 CompSogi(LATENCY=6)：样本接受拍后第 6 拍 SOGI 才提交
                        oVAlpha/oVBeta，故本 PLL 消费的是“上一已接受样本”的 SOGI
                        输出（对齐原单周期版逐拍语义：第 n 拍用 va[n-1] 产生 θ_n）。
                        接受拍窗口内 SOGI 输出仍为上一样本结果且稳定，故 Park/误差/PI
                        无需等待本样本 SOGI，提交较快。组合开方/除法已消除：幅值
                        mg=floor(sqrt(d^2+q^2)) 用 AlgoSqrt、invA=2^30/mg 用 AlgoDiv，
                        二者后台逐样本循环刷新(ZOH 寄存器 rInvA)，误差通路仅一拍
                        乘 q*invA>>15（符号/数值与原 err=q/sqrt 接近）。
                        LATENCY(=8)：θ_pk 乘拍+加拍、Park 锁存、误差一拍、PI 三拍、
                        提交一拍。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.2  (多拍流水化)
//modifier          :
//Modify Date       :
/*Modify Record     :
//  Rev 0.1: 原单周期组合版（组合开方/除法、无 iEn/oValid）
*/
//------------------------------------------------------------------------------
`include "algo_arith.vh"

module CompPll #(
    parameter integer W     = 16,
    parameter integer FS    = 5000,   // 采样率 Hz（须与 CompSogi 默认 Ts 匹配）
    parameter integer F_NOM = 50,     // 标称频率 Hz
    parameter integer LATENCY = 8     // 提交拍相对接受拍延迟（须 >= 8）
) (
    input  wire                 iSysClk,
    input  wire                 iSysRst,
    input  wire                 iEn,        // 采样节拍（一拍）
    input  wire signed [W-1:0]  iUIn,       // 单相采样
    output wire [W-1:0]         oTheta,     // 相位 16bit 全幅 0..65535<->0..2pi
    output wire signed [W-1:0]  oOut,       // 同步正弦 cos(theta)（Q15 满量程）
    output wire signed [W-1:0]  oErr,       // 归一化相位误差 Q15（锁相后 -> 0）
    output wire signed [W-1:0]  oDF,        // kalman 平滑 d（收敛到输入幅值）
    output wire signed [W-1:0]  oWF,        // kalman 平滑 w（Q6, 1LSB=1/64 rad/s）
    output reg                  oValid      // 提交拍有效脉冲（接受后 LATENCY 拍）
);
    // ---- 定点常量（FS=5000 语义, 见头注释）----
    localparam signed [31:0] WNOM_Q  = (F_NOM * 32'sd402124 + 32'sd500) / 32'sd1000; // ~2*pi*F_NOM*64
    localparam [31:0]       C_PH    = (32'd2136 * 32'd5000) / FS;   // ~2^32/(2*pi*FS*64)
    localparam signed [31:0] I_CLAMP = 32'sd2057830; // 62.8*2^15  积分器限幅(rad)
    localparam signed [31:0] D_CLAMP = 32'sd4019;    // 62.8*64    频率限幅(w_q)
    localparam signed [31:0] W_HI    = WNOM_Q + D_CLAMP; // w 上限(60Hz)
    localparam signed [31:0] W_LO    = WNOM_Q - D_CLAMP; // w 下限(40Hz)
    localparam signed [15:0] C_KP    = 16'sd6400;    // 100*64  =>  err*100(rad/s)*64
    localparam signed [31:0] C_KI    = 32'sd3277;    // 3.2/32768*2^25 => i*0.05(rad/s)*64
    localparam [31:0]       INV_NUM = 32'd1073741824;  // 2^30, 求 invA=2^30/A
    localparam [31:0]       MIN_A   = 32'd8;           // A 下限防除小
    localparam signed [63:0] RD_HALF = 64'sd16384;     // Q15 舍入半 (2^14)

    // =====================================================================
    // 辅助饱和/限幅函数（纯组合, 避免时序块内阻塞）
    // =====================================================================
    // 64bit -> S16 钳位
    function automatic signed [15:0] f_sat16;
        input signed [63:0] v;
        reg signed [63:0] tv;
        begin
            if      (v >  64'sd32767)  tv = 64'sd32767;
            else if (v < -64'sd32768)  tv = -64'sd32768;
            else                       tv = v;
            f_sat16 = tv[15:0];
        end
    endfunction
    // 32bit 积分器限幅 ±C（C>0）
    function automatic signed [31:0] f_iclamp;
        input signed [31:0] v;
        input signed [31:0] c;
        reg signed [31:0] tv;
        begin
            if      (v >  c) tv =  c;
            else if (v < -c) tv = -c;
            else             tv = v;
            f_iclamp = tv;
        end
    endfunction
    // w 频率合成限幅（64bit 和 -> 32bit, clamp [lo,hi]）
    function automatic signed [31:0] f_wclamp;
        input signed [63:0] v;
        input signed [31:0] hi;
        input signed [31:0] lo;
        reg signed [63:0] tv;
        begin
            if      (v >  $signed(hi)) tv = $signed(hi);
            else if (v <  $signed(lo)) tv = $signed(lo);
            else                       tv = v;
            f_wclamp = tv[31:0];
        end
    endfunction

    // =====================================================================
    // SOGI 正交（固定 50Hz / FS 系数）。接受拍 A 后第 6 拍提交本样本 va/vb，
    // 因此在 [A, A+6) 窗口内 oVAlpha/oVBeta 仍为上一已接受样本结果（本 PLL 消费它）。
    // =====================================================================
    wire signed [15:0] va_s, vb_s;
    wire               wSogiValid;
    CompSogi #(.LATENCY(6)) U_Sogi (.iSysClk(iSysClk), .iSysRst(iSysRst),
                                    .iEn(iEn), .iXIn(iUIn),
                                    .oVAlpha(va_s), .oVBeta(vb_s), .oValid(wSogiValid));

    // ---- 预测角（θ_pk = θ_prev + w_prev*C_PH, 拆成乘法拍、加法/环绕拍）----
    reg signed [63:0] rWpr;      // w_prev*C_PH（64bit 积, 只取低 32）
    reg [31:0]        rThPk;     // 相位累加预测（32bit 环绕）
    wire [15:0] theta_pk = rThPk[31:16];

    // ---- Park：alpha=va, beta=-vb, theta=预测角（纯组合 + AlgoTrig, 例化保留）----
    wire signed [15:0] nb = -vb_s;
    wire signed [15:0] dC, qC;
    CompPark #(.W(W)) U_Park (.iAlpha(va_s), .iBeta(nb), .iTheta(theta_pk),
                              .oD(dC), .oQ(qC));

    // 锁存 Park 结果（一拍）
    reg signed [15:0] dreg, qreg;

    // =====================================================================
    // 归一化误差后台（ZOH）：mg=sqrt(d²+q²), invA=2^30/mg —— 多拍 AlgoSqrt/AlgoDiv
    // =====================================================================
    wire        sqrt_done, sqrt_busy;
    wire [31:0] sqrt_root;
    wire        sqrt_start = (psA == 3'd1);
    AlgoSqrt #(.IW(64), .OW(32)) U_Sqrt (
        .iSysClk(iSysClk), .iSysRst(iSysRst), .iStart(sqrt_start),
        .iNum(rA2c), .oBusy(sqrt_busy), .oDone(sqrt_done), .oRoot(sqrt_root));

    wire        div_done, div_busy;
    wire [31:0] div_quot;
    AlgoDiv #(.W(32)) U_Div (
        .iSysClk(iSysClk), .iSysRst(iSysRst),
        .iStart(div_start), .iNum(INV_NUM), .iDen(rAest),
        .oBusy(div_busy), .oDone(div_done), .oQuot(div_quot));

    // d²+q²（无符号 64bit, 组合）
    wire signed [63:0] dd64 = {{48{dreg[15]}}, dreg};
    wire signed [63:0] qq64 = {{48{qreg[15]}}, qreg};
    wire signed [63:0] A2w  = dd64*dd64 + qq64*qq64;

    reg [2:0]  psA;         // 0=idle 1=sqrt启动 2=sqrt等待 3=div启动 4=div等待
    reg [63:0] rA2c;        // A2 快照
    reg [31:0] rAest;       // mg = floor(sqrt(A2))
    reg [31:0] rInvA;       // 2^30/mg（ZOH, 正数）
    wire div_start = (psA == 3'd3);

    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            psA   <= 3'd0;
            rA2c  <= 64'd0;
            rAest <= 32'd0;
            rInvA <= 32'd0;
        end else begin
            case (psA)
                3'd0: begin rA2c <= A2w[63:0]; psA <= 3'd1; end  // 空闲捕获
                3'd1: begin                     psA <= 3'd2; end // sqrt 启动脉冲
                3'd2: begin
                    if (sqrt_done) begin
                        rAest <= sqrt_root;
                        psA   <= 3'd3;
                    end
                end
                3'd3: begin                     psA <= 3'd4; end // div 启动脉冲
                3'd4: begin
                    if (div_done) begin
                        if (rAest >= MIN_A) rInvA <= div_quot;   // 弱信号保持旧值
                        psA <= 3'd0;
                    end
                end
                default: psA <= 3'd0;
            endcase
        end
    end

    // =====================================================================
    // 状态（接受拍时仍为上一已提交值；提交拍才更新）
    // =====================================================================
    reg [31:0] rThetaAcc;   // 相位累加器 2^32<->2pi（自由环绕）
    reg signed [31:0] rWQ;  // 角频率 Q6
    reg signed [31:0] rIQ;  // 误差积分器（尺度 2^15）

    // ---- 逐拍中间量 ----
    reg [LATENCY-1:0] rPipe;        // 采样拍移位（tap k 于接受拍后 k+1 拍生效）
    reg signed [15:0] rErr16;       // 归一化误差 Q15
    reg signed [31:0] rINew;        // 积分累加限幅结果
    reg signed [63:0] rPm;          // 比例乘积 err*C_KP
    reg signed [63:0] rIm;          // 积分乘积 rINew*C_KI
    reg signed [31:0] rWNew;        // 频率合成限幅结果
    wire [63:0] thinc = $unsigned(rWNew) * C_PH;   // w*Ts 相位增量（提交拍读）

    // 归一化误差通路（一拍）: err = q*invA >>15（Q15, 符号同 q）
    wire signed [63:0] qext  = $signed({ {48{qreg[15]}}, qreg});
    wire signed [63:0] invext= $signed({32'd0, rInvA});
    wire signed [63:0] errsh = (qext * invext + RD_HALF) >>> 15;

    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            rPipe      <= {LATENCY{1'b0}};
            rWpr       <= 64'sd0;
            rThPk      <= 32'd0;
            dreg       <= {W{1'b0}};
            qreg       <= {W{1'b0}};
            rErr16     <= 16'sd0;
            rINew      <= 32'sd0;
            rPm        <= 64'sd0;
            rIm        <= 64'sd0;
            rWNew      <= 32'sd0;
            rThetaAcc  <= 32'h0;
            rWQ        <= WNOM_Q;
            rIQ        <= 32'sd0;
            oValid     <= 1'b0;
        end else begin
            // 采样拍移位
            rPipe[0] <= iEn;
            if (LATENCY > 1)
                rPipe[LATENCY-1:1] <= rPipe[LATENCY-2:0];

            // tap0（接受拍后第 1 拍）：θ_pk 乘法 w_prev*C_PH
            if (rPipe[0])
                rWpr <= $signed({{32{rWQ[31]}}, rWQ}) * $signed({32'd0, C_PH});
            // tap1（第 2 拍）：θ_pk 加法/环绕（32bit）
            if (rPipe[1])
                rThPk <= rThetaAcc + rWpr[31:0];

            // tap2（第 3 拍）：锁存 Park d/q
            // （SOGI 输出此时仍为上一已接受样本结果，本 PLL 消费它, 与原逐拍语义一致）
            if (rPipe[2])
                begin dreg <= dC; qreg <= qC; end

            // tap3（第 4 拍）：归一化误差一拍（q*invA >>15, 舍入饱和）
            if (rPipe[3])
                rErr16 <= f_sat16(errsh);

            // tap4（第 5 拍）：积分累加限幅 + 比例乘法
            if (rPipe[4]) begin
                rINew <= f_iclamp($signed(rIQ) + $signed(rErr16), I_CLAMP);
                rPm   <= $signed({{48{rErr16[15]}}, rErr16}) * $signed(C_KP);
            end
            // tap5（第 6 拍）：积分乘法 i*C_KI
            if (rPipe[5])
                rIm <= $signed({{32{rINew[31]}}, rINew}) * $signed(C_KI);
            // tap6（第 7 拍）：w 合成限幅 w = clamp(WNOM + pterm + iterm)
            if (rPipe[6])
                rWNew <= f_wclamp($signed(WNOM_Q) + (rPm >>> 15) + (rIm >>> 25),
                                  W_HI, W_LO);
            // tap7（第 8 拍 = LATENCY-1）：提交拍 θ+=w*Ts(32bit 环绕) + 状态更新
            if (rPipe[LATENCY-1]) begin
                rThetaAcc <= rThetaAcc + thinc[31:0];
                rWQ       <= rWNew;
                rIQ       <= rINew;
            end
            oValid <= rPipe[LATENCY-1];
        end
    end

    // ---- 输出：cos(theta) / 误差 / kalman 平滑 d 与 w ----
    assign oTheta = rThetaAcc[31:16];
    wire signed [15:0] cth, sth;
    AlgoTrig U_Osc (.iAngle(oTheta), .oSin(sth), .oCos(cth));
    assign oOut = cth;
    assign oErr = rErr16;

    wire wKdValid, wKwValid;
    CompKalman1d #(.W(W)) U_Kd (.iSysClk(iSysClk), .iSysRst(iSysRst), .iEn(iEn),
                                .iMIn(dreg), .oYOut(oDF), .oValid(wKdValid));
    CompKalman1d #(.W(W)) U_Kw (.iSysClk(iSysClk), .iSysRst(iSysRst), .iEn(iEn),
                                .iMIn(rWQ[15:0]), .oYOut(oWF), .oValid(wKwValid));
endmodule
