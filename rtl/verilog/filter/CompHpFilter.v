//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   CompHpFilter.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
/*Description       :   一阶高通滤波器（Tustin/双线性，IIR）。
                        算法对齐 C Comp_HPFilter：tau=1/(2pi*fc), k=1/(pi*fc*Ts)=FS/(pi*fc)
                        coeff_a=(k-1)/(k+1), coeff_b=k/(k+1)；
                        y[n]=a*y[n-1]+b*(x[n]-x[n-1])。
                        信号 S0.(W-1) 有符号；系数 Q0.FSHIFT 整数；输出饱和限幅。直流增益=0。
                        【多拍流水】差分/前馈乘法一拍、反馈乘法一拍、求和一拍、
                        量化/饱和一拍，共 LATENCY=4 拍；iEn 单拍脉冲采样，完成后 oValid 高。
                        数值与单周期组合实现逐位等价（仅延后 LATENCY 拍）。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.2 (pipelined LATENCY=4)
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
module CompHpFilter #(
    parameter integer W        = 16,     // 信号位宽(有符号)
    parameter integer FC       = 500,    // 截止频率 Hz（文献/顶层参考）
    parameter integer FS       = 50000,  // 采样率 Hz，Ts=1/FS（文献/顶层参考）
    parameter integer FSHIFT   = 15,     // 系数 Q0.FSHIFT 小数位
    parameter integer COEF_A   = 30772,  // (k-1)/(k+1) *2^FSHIFT，k=FS/(pi*FC)
    parameter integer COEF_B   = 31770   //  k/(k+1)    *2^FSHIFT
) (
    input  wire                iSysClk,  // 时钟
    input  wire                iSysRst,  // 复位（高有效）
    input  wire                iEn,      // 采样节拍（单拍脉冲：一帧一个采样）
    input  wire signed [W-1:0] iXIn,     // 输入采样
    output reg  signed [W-1:0] oYOut,    // 高通输出
    output reg                 oValid    // 本帧完成标志（iEn 后 LATENCY 拍拉高一拍）
);
    localparam integer LATENCY = 4;                       // 一阶 IIR 流水拍数
    localparam signed [63:0] MAXS = (64'sd1 << (W-1)) - 1;  // 上限
    localparam signed [63:0] MINS = -(64'sd1 << (W-1));     // 下限
    localparam signed [95:0] RND  = (96'sd1 <<< (FSHIFT - 1));  // +0.5 LSB 舍入

    // ---- 内部状态（宽位，带小数） ----
    reg signed [63:0] rXP = 64'sd0;      // x[n-1]
    reg signed [63:0] rYP = 64'sd0;      // y[n-1]

    // ---- 流水中间量 ----
    reg signed [95:0] rD;                // x[n] - x[n-1]
    reg signed [95:0] rTa;               // a*y[n-1]
    reg signed [95:0] rTb;               // b*(x-xprev)
    reg signed [95:0] rSum;              // t_a + t_b
    reg [LATENCY-1:0] rPipe;             // 采样脉冲移位

    function automatic signed [63:0] f_sat(input signed [63:0] v);
        begin
            if      (v > MAXS) f_sat = MAXS;
            else if (v < MINS) f_sat = MINS;
            else               f_sat = v;
        end
    endfunction

    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            rXP    <= 64'sd0;
            rYP    <= 64'sd0;
            rD     <= 96'sd0;
            rTa    <= 96'sd0;
            rTb    <= 96'sd0;
            rSum   <= 96'sd0;
            rPipe  <= {LATENCY{1'b0}};
            oYOut  <= {W{1'b0}};
            oValid <= 1'b0;
        end else begin
            oValid <= 1'b0;                            // 默认低（脉冲）
            // ---- 拍0（iEn 采样）：差分 + 前馈乘法；x 历史后移 ----
            if (iEn && (rPipe == {LATENCY{1'b0}})) begin
                rD   <= $signed(iXIn) - rXP;
                rTa  <= $signed(COEF_A) * rYP;         // 反馈乘法（rYP 保持旧值至末拍）
                rXP  <= $signed(iXIn);
                rPipe <= {{LATENCY-1{1'b0}}, 1'b1};
            end
            // ---- 拍1：后项乘法 b*(x-xprev) ----
            if (rPipe[0]) begin
                rTb  <= $signed(COEF_B) * rD;
                rPipe <= (rPipe[LATENCY-2]) ? {LATENCY{1'b0}} : (rPipe << 1);
            end
            // ---- 拍2：求和 ----
            if (rPipe[1]) begin
                rSum <= rTa + rTb;
                rPipe <= (rPipe[LATENCY-2]) ? {LATENCY{1'b0}} : (rPipe << 1);
            end
            // ---- 拍3（末拍）：舍入移位 + 饱和 + 状态/输出更新 + oValid ----
            if (rPipe[LATENCY-2]) begin
                rYP   <= f_sat((rSum + RND) >>> FSHIFT);
                oYOut <= f_sat((rSum + RND) >>> FSHIFT);
                oValid<= 1'b1;
                rPipe <= {LATENCY{1'b0}};
            end
        end
    end
endmodule
