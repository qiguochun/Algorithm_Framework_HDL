//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   CompGoertzel.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.06
/*Description       :   Goertzel 单频 bin 幅度检测器（分块窗口，非重叠测量）。
                        bin k 对应物理频率 f = k*fs/N。逐样本递推（自动分窗 N）:
                          s[n] = x[n] + 2cos(2pi*k/N)*s[n-1] - s[n-2]
                        N 样本满窗时 |X[k]|^2 = s1^2+s2^2 - 2cos(2pi*k/N)*s1*s2，
                        实正弦(整数周期, 幅 A) 时 |X[k]| ~ A*N/2。
                        幅度经 AlgoSqrt 多拍库求根（无组合开方/除法）。
  定点   :  数据输入 S16(Q15 满幅 32767≈1); 状态 S32 饱和 ±2^30；
            系数 COEF_Q14 为 Q14 定点, 即 round(2cos(2pi*k/N)*2^14)；
            |X|^2 在 S64 内用 64bit 乘积组合, 饱和钳制; 输出幅度 oAmp U32。
  窗口   :  每 iSysClk 消耗一个样本; 每 N 个样本完成一次幅度测量并复位状态。
            测量(AlgoSqrt ~33 拍)与采样重叠; 若测量未完成而下一窗结束则丢该窗
            (仅幅度漏测, 输入不丢), 故建议 N_WINDOW >= 48。
  修订   :   2026-09-06 v0.1
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.1
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------

module CompGoertzel #(
    parameter integer W        = 16,     // 输入数据位宽 (S16)
    parameter integer N_WINDOW = 64,     // 窗口样本数 N (建议 >= 48)
    parameter integer K_BIN    = 8,      // 目标 bin k (0<k<N)
    parameter signed [15:0] COEF_Q14 = 16'sd23170  // round(2cos(2pi*8/64)*2^14)=round(sqrt2*16384)
) (
    input  wire                iSysClk,     // 系统时钟(每拍=一个样本)
    input  wire                iSysRst,     // 复位(高有效)
    input  wire signed [W-1:0] iData,       // 输入采样 S16
    output reg  [31:0]         oAmp,        // 最近一个完整窗口的 |X[k]|（U32, ~ A*N/2）
    output reg                 oValid       // oAmp 刷新脉冲(一拍)
);

    localparam signed [63:0] SAT_P = 64'sd1073741824;      // +2^30 (状态饱和上限)
    localparam signed [63:0] SAT_N = -64'sd1073741824;     // -2^30
    localparam signed [63:0] HALF  = 64'sd8192;            // 2^(14-1) Q14 舍入半

    // ---- 状态 ----
    reg signed [31:0] rS1;        // s[n-1]
    reg signed [31:0] rS2;        // s[n-2]
    reg        [31:0] rCnt;       // 窗内样本计数 0..N_WINDOW-1

    // ---- AlgoSqrt 求根 (IW=64 -> OW=32, ~33 拍) ----
    wire        sqrt_done, sqrt_busy;
    wire [31:0] sqrt_root;
    wire        sqrt_start;
    AlgoSqrt #(.IW(64), .OW(32)) U_Sqrt (
        .iSysClk(iSysClk), .iSysRst(iSysRst), .iStart(sqrt_start),
        .iNum(mag_uns), .oBusy(sqrt_busy), .oDone(sqrt_done), .oRoot(sqrt_root));

    // ---- 系数 (Q14 = 2cos*2^14) 扩展到 64 位 ----
    wire signed [63:0] cF64 = {{48{COEF_Q14[15]}}, COEF_Q14};

    // 有符号 64bit -> S32 饱和 ±2^30（避免三元/part-select 无符号提升）
    function signed [31:0] f_sat32;
        input signed [63:0] v;
        begin
            if (v >  SAT_P)      f_sat32 =  32'sd1073741824;
            else if (v <  SAT_N) f_sat32 = -32'sd1073741824;
            else                 f_sat32 = v[31:0];
        end
    endfunction

    // ---- 递推 s_new = x + 2c*s1 - s2 ----
    wire signed [63:0] sr1  = rS1;                    // 符号扩展
    wire signed [63:0] sr2  = rS2;
    wire signed [63:0] sxw  = iData;
    wire signed [63:0] cprd = cF64 * sr1;             // 2c*2^14*s1
    wire signed [63:0] csh  = (cprd + HALF) >>> 14;   // round(2c*s1)
    wire signed [63:0] snew_w = sxw + csh - sr2;
    wire signed [31:0] snew = f_sat32(snew_w);

    // ---- 满窗幅度 |X|^2 = s1^2+s2^2 - 2c*s1*s2  (s1=s_new,s2=rS1 于满窗拍) ----
    wire signed [63:0] wa  = snew;
    wire signed [63:0] wb  = rS1;
    wire signed [63:0] pa  = wa * wa;                 // <= 2^60
    wire signed [63:0] pb  = wb * wb;
    wire signed [63:0] pab = wa * wb;
    wire signed [63:0] pabq = (pab + HALF) >>> 14;    // round(s1*s2/2^14)
    wire signed [63:0] trm = cF64 * pabq;             // 2c*s1*s2  (去掉 1/2^14)
    wire signed [63:0] magw = pa + pb - trm;          // >=0 (数值保护取 0)
    wire [63:0] mag_uns = magw[63] ? 64'd0 : magw[63:0];

    // 满窗拍启动 sqrt（测量空闲时）
    assign sqrt_start = (rCnt == N_WINDOW - 1) && (~sqrt_busy);

    // ---- 主时序: 递推 + 分窗复位 + sqrt 完成刷新输出 ----
    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            rS1    <= 32'sd0;
            rS2    <= 32'sd0;
            rCnt   <= 32'd0;
            oAmp   <= 32'd0;
            oValid <= 1'b0;
        end else begin
            oValid <= 1'b0;
            if (sqrt_done) begin
                oAmp   <= sqrt_root;
                oValid <= 1'b1;
            end
            if (rCnt == (N_WINDOW - 1)) begin
                // 本拍已把第 N 个样本并入 s_new(mag 已捕获), 复位开始新窗
                rS1  <= 32'sd0;
                rS2  <= 32'sd0;
                rCnt <= 32'd0;
            end else begin
                rS1  <= snew;
                rS2  <= rS1;
                rCnt <= rCnt + 1'b1;
            end
        end
    end
endmodule
