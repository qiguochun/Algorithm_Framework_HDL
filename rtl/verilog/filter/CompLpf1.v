//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   CompLpf1.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.06
/*Description       :   一阶巴特沃斯低通滤波器（双线性/Tustin 离散，IIR）。
                        连续 H(s)=wc/(s+wc)，s=2fs*(z-1)/(z+1) 直接代入离散 ->
                        差分 y[n]=b0*x[n]+b1*x[n-1]-a1*y[n-1]，
                        b0=b1=wc/(2fs+wc)，a1=(wc-2fs)/(wc+2fs)，wc=2*pi*fc。
                        fc/fs 隐含于 Q 系数 localparam（默认 fc=10Hz, fs=5kHz，Python 验证）。
                        系数 Q15; 状态带 STATEFRAC 位小数避免 DC 死区偏置;
                        输出 Q0 舍入+饱和 S16。
                        【多拍流水】乘法一拍(并行)、加法一拍、量化/饱和一拍、状态更新一拍，
                        共 LATENCY=4 拍；iEn 单拍脉冲采样一帧，逐拍处理，完成后 oValid 高。
                        数值与单周期组合实现逐位等价（仅时间上延后 LATENCY 拍）。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.2 (pipelined LATENCY=4)
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
module CompLpf1 #(
    parameter integer W          = 16,     // 信号位宽(有符号)
    parameter integer FC         = 10,     // 截止频率 Hz（文献/顶层参考，wc=2*pi*FC）
    parameter integer FS         = 5000,   // 采样率 Hz（文献/顶层参考，Ts=1/FS）
    parameter integer FSHIFT     = 15,     // 系数 Q0.FSHIFT 小数位
    parameter integer STATEFRAC  = 15,     // 状态(反馈)额外小数位，防 DC 死区
    parameter integer COEF_B0    = 205,    // b0=wc/(2fs+wc)=0.00624395 *2^15  Q15
    parameter integer COEF_B1    = 205,    // b1=wc/(2fs+wc)=0.00624395 *2^15  Q15(=b0)
    parameter integer COEF_A1    = -32359  // a1=(wc-2fs)/(wc+2fs)=-0.98751209 *2^15  Q15
) (
    input  wire                iSysClk,  // 时钟
    input  wire                iSysRst,  // 复位（高有效）
    input  wire                iEn,      // 采样节拍（单拍脉冲：一帧一个采样）
    input  wire signed [W-1:0] iXIn,     // 输入采样 x[n]
    output reg  signed [W-1:0] oYOut,    // 低通输出 y[n]
    output reg                 oValid    // 本帧完成标志（iEn 后 LATENCY 拍拉高一拍）
);
    // ---- Python 验证(Q15, fc=10Hz, fs=5000) ----
    // b0=b1=0.00624395, a1=-0.98751209 -> DC 增益 (b0+b1)/(1+a1)=1.0024;
    // |H(e^{jw})| @100Hz ≈ 0.0996（理论 -3dB@10Hz，十倍频 -20dB/dec）。
    // 状态 R=y*2^STATEFRAC，分子 num 在 (2^FSHIFT*2^STATEFRAC) 域。

    localparam integer LATENCY = 4;                        // 一阶 IIR 流水拍数
    localparam signed [63:0] YMAX = (64'sd1 << (W-1)) - 1;  // 输出上限 32767
    localparam signed [63:0] YMIN = -(64'sd1 << (W-1));     // 输出下限 -32768
    localparam signed [63:0] RMAX = (YMAX << STATEFRAC);    // 状态上限
    localparam signed [63:0] RMIN = (YMIN << STATEFRAC);    // 状态下限
    localparam signed [95:0] RND_R = (96'sd1 <<< (FSHIFT - 1));    // R 舍入 +0.5 LSB
    localparam signed [63:0] RND_O = (64'sd1 <<< (STATEFRAC - 1)); // 输出舍入 +0.5 LSB

    // ---- 内部状态 ----
    reg signed [63:0] rX1;         // x[n-1]
    reg signed [63:0] rR1;         // 状态 y[n-1]*2^STATEFRAC

    // ---- 流水中间量（一拍一个结果，不复用组合临时量） ----
    reg signed [95:0] rP0;         // b0*x[n]
    reg signed [95:0] rP1;         // b1*x[n-1]
    reg signed [95:0] rPA;         // -a1? 存 a1*R1(带符号)
    reg signed [95:0] rNum;        // 分子 (b项<<STATEFRAC) - a1*R1
    reg signed [63:0] rR;          // 新状态 y*2^STATEFRAC（未饱和）
    reg [LATENCY-1:0] rPipe;       // 采样脉冲移位：iEn 打 1，逐拍右移(升位)

    // ---- 组合：R 饱和 + 输出量化（仅末拍调用，无乘法） ----
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
            rX1    <= 64'sd0;
            rR1    <= 64'sd0;
            rP0    <= 96'sd0;
            rP1    <= 96'sd0;
            rPA    <= 96'sd0;
            rNum   <= 96'sd0;
            rR     <= 64'sd0;
            rPipe  <= {LATENCY{1'b0}};
            oYOut  <= {W{1'b0}};
            oValid <= 1'b0;
        end else begin
            oValid <= 1'b0;                            // 默认低（脉冲）
            // ---- 拍0（iEn 采样）：三个乘法并行一拍，各自单独寄存 ----
            if (iEn && (rPipe == {LATENCY{1'b0}})) begin
                rP0  <= $signed(COEF_B0) * $signed(iXIn);
                rP1  <= $signed(COEF_B1) * rX1;
                rPA  <= $signed(COEF_A1) * rR1;        // a1 为负 -> rPA 即 -|a1|*R1
                rX1  <= $signed(iXIn);                 // x 历史后移（本帧已用完）
                rPipe <= {{LATENCY-1{1'b0}}, 1'b1};
            end
            // ---- 拍1：加法（b0*x + b1*x1）抬位 + 减反馈 一拍 ----
            if (rPipe[0]) begin
                rNum <= (($signed(rP0) + rP1) << STATEFRAC) - rPA;
                rPipe <= (rPipe[LATENCY-2]) ? {LATENCY{1'b0}} : (rPipe << 1);
            end
            // ---- 拍2：状态量化（舍入+算术右移） ----
            if (rPipe[1]) begin
                rR   <= (rNum + RND_R) >>> FSHIFT;
                rPipe <= (rPipe[LATENCY-2]) ? {LATENCY{1'b0}} : (rPipe << 1);
            end
            // ---- 拍3（末拍）：饱和 + 输出量化 + 状态/输出更新 + oValid ----
            if (rPipe[LATENCY-2]) begin
                rR1   <= f_satR(rR);
                oYOut <= f_quantY(f_satR(rR));
                oValid <= 1'b1;
                rPipe <= {LATENCY{1'b0}};
            end
        end
    end
endmodule
