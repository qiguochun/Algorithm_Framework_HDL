//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   CompDiff.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
/*Description       :   带低通的微分器（两级 Tustin，C Comp_Diff_Execute 语义）。
                        Stage1 LPF(wc): a1=wc_d/(2+wc_d), b1=(2-wc_d)/(2+wc_d), y1=a1(u[k]+u[k-1])+b1*y1[k-1]
                        Stage2 Diff(N): a2=2N/(2+N*Ts), b2=(2-N*Ts)/(2+N*Ts), y2=a2(y1-y1[k-1])+b2*y2[k-1]
                        信号 S0.(W-1) 有符号；a2 量值可能>1 用 A2Q/A2SH 单独表示。
                        输出为对采样时刻求导结果（单位=信号/秒）；斜坡输入稳态≈斜率。
                        【多拍流水】两级各 4 拍串行（级间依赖 y1 先行算出），共 LATENCY=8 拍：
                        Stage1 加/乘法(并行)、乘法、加法、量化一拍；
                        Stage2 差/乘(并行)、乘法、移位(并行)、求和+饱和+状态更新一拍。
                        iEn 单拍脉冲采样一帧，完成后 oValid 高；数值与单周期实现逐位等价。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.2 (pipelined LATENCY=8)
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
module CompDiff #(
    parameter integer W        = 16,     // 信号位宽(有符号)
    parameter integer FSHIFT   = 15,     // a1/b1/b2 的 Q0.FSHIFT 小数位
    parameter integer A1Q      = 12288,  // a1*2^FSHIFT (wc_d=1.2 -> a1=0.375)
    parameter integer B1Q      = 8192,   // b1*2^FSHIFT (b1=0.25)
    parameter integer A2Q      = 40,     // a2*2^A2SH  (a2 = 2N/(2+N*Ts))
    parameter integer A2SH     = 0,      // a2 的小数位
    parameter integer B2Q      = 19661   // b2*2^FSHIFT (b2=0.6)
) (
    input  wire                iSysClk,  // 时钟
    input  wire                iSysRst,  // 复位（高有效）
    input  wire                iEn,      // 采样节拍（单拍脉冲：一帧一个采样）
    input  wire signed [W-1:0] iXIn,     // 输入采样 u[k]
    output reg  signed [W-1:0] oYOut,    // 微分输出
    output reg                 oValid    // 本帧完成标志（iEn 后 LATENCY 拍拉高一拍）
);
    localparam integer LATENCY = 8;                        // 两级各4拍串行
    localparam signed [63:0] MAXS = (64'sd1 << (W-1)) - 1;  // 上限
    localparam signed [63:0] MINS = -(64'sd1 << (W-1));     // 下限
    localparam signed [95:0] RND  = (96'sd1 <<< (FSHIFT - 1));  // +0.5 LSB 舍入

    // ---- 内部状态 ----
    reg signed [63:0] rUP  = 64'sd0;      // u[k-1]
    reg signed [63:0] rY1P = 64'sd0;      // 低通输出 y1[k-1]
    reg signed [63:0] rY2P = 64'sd0;      // 微分输出 y2[k-1]

    // ---- 流水中间量 ----
    reg signed [95:0] rSsum;              // u[k]+u[k-1]
    reg signed [95:0] rPb1;               // b1*y1[k-1]
    reg signed [95:0] rPa1;               // a1*(u[k]+u[k-1])
    reg signed [95:0] rN1;                // n1 = a1项+b1项
    reg signed [63:0] rY1n;               // y1[k]（低通输出，量化后）
    reg signed [95:0] rD;                 // y1[k]-y1[k-1]
    reg signed [95:0] rT2a;               // a2*d
    reg signed [95:0] rT2b;               // b2*y2[k-1]
    reg signed [63:0] rPa, rPb;           // (t2a>>>A2SH), (t2b+rnd)>>>FSHIFT
    reg [LATENCY-1:0] rPipe;              // 采样脉冲移位

    function automatic signed [63:0] f_sat(input signed [63:0] v);
        begin
            if      (v > MAXS) f_sat = MAXS;
            else if (v < MINS) f_sat = MINS;
            else               f_sat = v;
        end
    endfunction

    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            rUP   <= 64'sd0;
            rY1P  <= 64'sd0;
            rY2P  <= 64'sd0;
            rSsum <= 96'sd0;
            rPb1  <= 96'sd0;
            rPa1  <= 96'sd0;
            rN1   <= 96'sd0;
            rY1n  <= 64'sd0;
            rD    <= 96'sd0;
            rT2a  <= 96'sd0;
            rT2b  <= 96'sd0;
            rPa   <= 64'sd0;
            rPb   <= 64'sd0;
            rPipe <= {LATENCY{1'b0}};
            oYOut <= {W{1'b0}};
            oValid<= 1'b0;
        end else begin
            oValid <= 1'b0;
            // ---- 拍0：s_sum = u[k]+u[k-1]（加）；b1*y1[k-1]（乘，并行）；u 历史后移 ----
            if (iEn && (rPipe == {LATENCY{1'b0}})) begin
                rSsum <= $signed(iXIn) + rUP;
                rPb1  <= $signed(B1Q) * rY1P;
                rUP   <= $signed(iXIn);
                rPipe <= {{LATENCY-1{1'b0}}, 1'b1};
            end
            // ---- 拍1：a1*(u[k]+u[k-1]) 乘法 ----
            if (rPipe[0]) begin
                rPa1  <= $signed(A1Q) * rSsum;
                rPipe <= (rPipe[LATENCY-2]) ? {LATENCY{1'b0}} : (rPipe << 1);
            end
            // ---- 拍2：n1 = a1项 + b1项 ----
            if (rPipe[1]) begin
                rN1  <= rPa1 + rPb1;
                rPipe <= (rPipe[LATENCY-2]) ? {LATENCY{1'b0}} : (rPipe << 1);
            end
            // ---- 拍3：Stage1 量化 y1[k]=(n1+rnd)>>>FSHIFT ----
            if (rPipe[2]) begin
                rY1n <= (rN1 + RND) >>> FSHIFT;
                rPipe <= (rPipe[LATENCY-2]) ? {LATENCY{1'b0}} : (rPipe << 1);
            end
            // ---- 拍4：d = y1[k]-y1[k-1]（减）；b2*y2[k-1]（乘，并行） ----
            if (rPipe[3]) begin
                rD   <= rY1n - rY1P;
                rT2b <= $signed(B2Q) * rY2P;
                rPipe <= (rPipe[LATENCY-2]) ? {LATENCY{1'b0}} : (rPipe << 1);
            end
            // ---- 拍5：a2*d 乘法 ----
            if (rPipe[4]) begin
                rT2a <= $signed(A2Q) * rD;
                rPipe <= (rPipe[LATENCY-2]) ? {LATENCY{1'b0}} : (rPipe << 1);
            end
            // ---- 拍6：两路移位并行 t2a>>>A2SH 与 (t2b+rnd)>>>FSHIFT ----
            if (rPipe[5]) begin
                rPa  <= rT2a >>> A2SH;
                rPb  <= (rT2b + RND) >>> FSHIFT;
                rPipe <= (rPipe[LATENCY-2]) ? {LATENCY{1'b0}} : (rPipe << 1);
            end
            // ---- 拍7（末拍）：求和 + 输出饱和 + 状态/输出更新 + oValid ----
            if (rPipe[LATENCY-2]) begin
                rY1P <= rY1n;                          // y1[k] -> y1[k-1]
                rY2P <= rPa + rPb;                     // 状态存未饱和 y2[k]（与原始一致）
                oYOut <= f_sat(rPa + rPb);             // 输出取饱和 y2[k]
                oValid<= 1'b1;
                rPipe <= {LATENCY{1'b0}};
            end
        end
    end
endmodule
