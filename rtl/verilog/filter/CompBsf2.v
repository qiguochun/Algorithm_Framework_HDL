//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   CompBsf2.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.06
/*Description       :   二阶带阻/陷波滤波器（双线性/Tustin 离散，IIR）。
                        连续 H(s)=(s^2+w0^2)/(s^2+(w0/Q)*s+w0^2)，w0=2*pi*f0，Q=品质因数。
                        s=K*(z-1)/(z+1)，K=2fs，记 C=K^2+(w0/Q)K+w0^2，Cn=K^2+w0^2：
                        分子 =(Cn)(z^2+1)+(2w0^2-2K^2)z -> b0=b2=Cn/C,
                        b1=(2w0^2-2K^2)/C；
                        分母 z^0..z^2 系数 = C,(2w0^2-2K^2),(K^2-(w0/Q)K+w0^2) ->
                        a1=(2w0^2-2K^2)/C, a2=(K^2-(w0/Q)K+w0^2)/C。
                        差分 y[n]=b0*x[n]+b1*x[n-1]+b2*x[n-2]-a1*y[n-1]-a2*y[n-2]。
                        系数由 parameter FS/F0/QREF 在 elabor 期按上式整数定点算出（默认 f0=250Hz, fs=5kHz, Q=0.707）。
                        系数 Q20; 状态带 STATEFRAC 位小数避免直流死区偏置;
                        输出 Q0 舍入+饱和 S16。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.1
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
module CompBsf2 #(
    parameter integer W          = 16,     // 信号位宽(有符号)
    parameter integer F0         = 250,    // 阻带中心(陷波)频率 Hz
    parameter integer QREF       = 707,    // 品质因数 Q*1000（默认 0.707）
    parameter integer FS         = 5000,   // 采样率 Hz，Ts=1/FS
    parameter integer FSHIFT     = 20,     // 系数 Q0.FSHIFT 小数位
    parameter integer STATEFRAC  = 15      // 状态(反馈)额外小数位，防 DC 死区
) (
    input  wire                iSysClk,  // 时钟
    input  wire                iSysRst,  // 复位（高有效）
    input  wire                iEn,      // 采样节拍（单拍脉冲：一帧一个采样）
    input  wire signed [W-1:0] iXIn,     // 输入采样 x[n]
    output reg  signed [W-1:0] oYOut,    // 带阻(陷波)输出 y[n]
    output reg                 oValid    // 本帧完成标志（iEn 后 LATENCY 拍拉高一拍）
);
    `include "algo_filt_coef.vh"
    localparam integer COEF_B0 = fn_bsf2_b0(FS, F0, QREF, FSHIFT);
    localparam integer COEF_B1 = fn_bpf2_a1(FS, F0, QREF, FSHIFT); // b1=a1=(2w0^2-2K^2)/C
    localparam integer COEF_B2 = COEF_B0;
    localparam integer COEF_A1 = COEF_B1;
    localparam integer COEF_A2 = fn_bpf2_a2(FS, F0, QREF, FSHIFT);

    localparam integer LATENCY = 6;                        // 二阶 IIR 流水拍数
    localparam signed [63:0] YMAX = (64'sd1 << (W-1)) - 1;  // 输出上限 32767
    localparam signed [63:0] YMIN = -(64'sd1 << (W-1));     // 输出下限 -32768
    localparam signed [63:0] RMAX = (YMAX << STATEFRAC);    // 状态上限
    localparam signed [63:0] RMIN = (YMIN << STATEFRAC);    // 状态下限
    localparam signed [95:0] RND_R = (96'sd1 <<< (FSHIFT - 1));    // R 舍入 +0.5 LSB
    localparam signed [63:0] RND_O = (64'sd1 <<< (STATEFRAC - 1)); // 输出舍入 +0.5 LSB

    // ---- 内部状态 ----
    reg signed [63:0] rX1;         // x[n-1]
    reg signed [63:0] rX2;         // x[n-2]
    reg signed [63:0] rR1;         // 状态 y[n-1]*2^STATEFRAC
    reg signed [63:0] rR2;         // 状态 y[n-2]*2^STATEFRAC

    // ---- 流水中间量（一拍一个结果，不复用组合临时量） ----
    reg signed [95:0] rP0;         // b0*x[n]
    reg signed [95:0] rP1;         // b1*x[n-1]
    reg signed [95:0] rP2;         // b2*x[n-2]
    reg signed [95:0] rPA1;        // a1*R1（带符号）
    reg signed [95:0] rPA2;        // a2*R2（带符号）
    reg signed [95:0] rBS;         // b 项和 b0*x+b1*x1+b2*x2
    reg signed [95:0] rNum;        // 分子 (b项<<STATEFRAC) - a1*R1 - a2*R2
    reg signed [63:0] rR;          // 新状态 y*2^STATEFRAC（舍入移位后）
    reg signed [63:0] rRS;         // 新状态（饱和后）
    reg [LATENCY-1:0] rPipe;       // 采样脉冲移位：iEn 打 1，逐拍右移(升位)

    // ---- 组合：饱和 / 输出量化（仅末拍附近调用，无乘法） ----
    function automatic signed [63:0] f_satR(input signed [63:0] v);
        begin
            if      (v > RMAX) f_satR = RMAX;
            else if (v < RMIN) f_satR = RMIN;
            else               f_satR = v;
        end
    endfunction

    function automatic signed [63:0] f_quantY(input signed [63:0] rs);
        reg signed [63:0] o;
        begin
            o = (rs + RND_O) >>> STATEFRAC;
            if      (o > YMAX) f_quantY = YMAX;
            else if (o < YMIN) f_quantY = YMIN;
            else               f_quantY = o;
        end
    endfunction

    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            rX1   <= 64'sd0;
            rX2   <= 64'sd0;
            rR1   <= 64'sd0;
            rR2   <= 64'sd0;
            rP0   <= 96'sd0;
            rP1   <= 96'sd0;
            rP2   <= 96'sd0;
            rPA1  <= 96'sd0;
            rPA2  <= 96'sd0;
            rBS   <= 96'sd0;
            rNum  <= 96'sd0;
            rR    <= 64'sd0;
            rRS   <= 64'sd0;
            rPipe <= {LATENCY{1'b0}};
            oYOut <= {W{1'b0}};
            oValid<= 1'b0;
        end else begin
            oValid <= 1'b0;                            // 默认低（脉冲）
            // ---- 拍0（iEn 采样）：5 个乘法并行一拍，各自单独寄存；x 历史后移 ----
            if (iEn && (rPipe == {LATENCY{1'b0}})) begin
                rP0  <= $signed(COEF_B0) * $signed(iXIn);
                rP1  <= $signed(COEF_B1) * rX1;
                rP2  <= $signed(COEF_B2) * rX2;
                rPA1 <= $signed(COEF_A1) * rR1;
                rPA2 <= $signed(COEF_A2) * rR2;
                rX1  <= $signed(iXIn);
                rX2  <= rX1;
                rPipe <= {{LATENCY-1{1'b0}}, 1'b1};
            end
            // ---- 拍1：b 项求和 ----
            if (rPipe[0]) begin
                rBS  <= rP0 + rP1 + rP2;
                rPipe <= (rPipe[LATENCY-2]) ? {LATENCY{1'b0}} : (rPipe << 1);
            end
            // ---- 拍2：b 项抬位 + 减反馈 ----
            if (rPipe[1]) begin
                rNum <= (rBS << STATEFRAC) - rPA1 - rPA2;
                rPipe <= (rPipe[LATENCY-2]) ? {LATENCY{1'b0}} : (rPipe << 1);
            end
            // ---- 拍3：状态量化（舍入+算术右移） ----
            if (rPipe[2]) begin
                rR   <= (rNum + RND_R) >>> FSHIFT;
                rPipe <= (rPipe[LATENCY-2]) ? {LATENCY{1'b0}} : (rPipe << 1);
            end
            // ---- 拍4：状态饱和 ----
            if (rPipe[3]) begin
                rRS  <= f_satR(rR);
                rPipe <= (rPipe[LATENCY-2]) ? {LATENCY{1'b0}} : (rPipe << 1);
            end
            // ---- 拍5（末拍）：输出量化 + 状态/输出更新 + oValid ----
            if (rPipe[LATENCY-2]) begin
                rR2   <= rR1;                          // y[n-1] -> y[n-2]
                rR1   <= rRS;                          // y[n]   -> y[n-1]
                oYOut <= f_quantY(rRS);
                oValid<= 1'b1;
                rPipe <= {LATENCY{1'b0}};
            end
        end
    end
endmodule
