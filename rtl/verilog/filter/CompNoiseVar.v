//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   CompNoiseVar.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
/*Description       :   噪声方差递归估计（EWMA，C Comp_NoiseVar method=1）。
                        mean[n]=a*x+(1-a)*mean[n-1]；err[n]=x-mean[n]；
                        var[n]=a*err[n]^2+(1-a)*var[n-1]；std[n]=sqrt(var[n])。
                        信号 S0.(W-1) 有符号；alpha 用 Q0.FSHIFT 整数 ALPHAQ。
                        多拍流水改造: 采样节拍快照 x, a*x/(1-a)*mean 两乘法并行(一拍)
                        -> mean=舍入右移、err=x-mean(一拍) -> err^2(一拍)
                        -> a*err^2/(1-a)*var 两乘法并行(一拍) -> var=舍入右移+提交(一拍)。
                        每个乘法结果单独寄存, 无单周期组合乘加; oValid 提交拍拉高。
                        开方改用多拍 AlgoSqrt(start/done 握手), 空闲自动重启连续跟踪,
                        取消组合二进制开方。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.2
//modifier          :
//Modify Date       :
/*Modify Record     :
//  Rev 0.2: EWMA 乘加拆 rEv 多拍流水 + isqrt 改 AlgoSqrt, 新增 oValid。
*/
//------------------------------------------------------------------------------
module CompNoiseVar #(
    parameter integer W        = 16,      // 信号位宽(有符号)
    parameter integer FSHIFT   = 15,      // alpha 的小数位 Q0.FSHIFT
    parameter integer ALPHAQ   = 1024,    // alpha*2^FSHIFT (alpha=1/32)
    parameter integer OVW      = 40,      // 方差输出位宽
    parameter integer OSW      = 32       // 标准差输出位宽
) (
    input  wire                iSysClk,   // 时钟
    input  wire                iSysRst,   // 复位（高有效）
    input  wire                iEn,       // 采样节拍
    input  wire signed [W-1:0] iXIn,      // 输入采样
    output wire [OVW-1:0]      oVar,      // 方差估计
    output wire [OSW-1:0]      oStd,      // 标准差估计
    output reg                 oValid     // 提交拍有效脉冲(采样后 LATENCY=4 拍)
);
    localparam [FSHIFT:0] ONE_Q = (1 << FSHIFT);      // 1 的定点
    localparam [FSHIFT:0] ONE_A = ONE_Q - ALPHAQ;     // (1-alpha) 定点

    // ---- 内部状态 ----
    reg signed [63:0] rMean = 64'sd0;     // 递归均值
    reg signed [63:0] rVar  = 64'sd0;     // 递归方差(>=0)

    // ---- 多拍事件 rEv[3:0]: [0]=mean/err [1]=err^2 [2]=a*e^2,(1-a)v [3]=提交 ----
    reg [3:0] rEv;
    reg       rBusy;
    reg signed [W-1:0] rX;                // x 快照
    reg signed [95:0]  rMu;               // a*x(乘法一拍)
    reg signed [95:0]  rMd;               // (1-a)*mean(乘法一拍)
    reg signed [95:0]  rRnd;              // 舍入加数
    reg signed [95:0]  rEsq;              // err^2(乘法一拍)
    reg signed [95:0]  rVu;               // a*err^2(乘法一拍)
    reg signed [95:0]  rVd;               // (1-a)*var(乘法一拍)

    // ---- 开方 (多拍, 空闲自动重启) ----
    wire        sqrt_done, sqrt_busy;
    wire [31:0] sqrt_root;
    reg  [31:0] rStd;
    AlgoSqrt #(.IW(64), .OW(32)) U_Sqrt (
        .iSysClk(iSysClk), .iSysRst(iSysRst), .iStart(~sqrt_busy),
        .iNum((rVar < 0) ? 64'd0 : $unsigned(rVar)),
        .oBusy(sqrt_busy), .oDone(sqrt_done), .oRoot(sqrt_root));

    // ---- 事件移位 + 采样快照 + a*x/(1-a)*mean 两乘法(并行一拍) ----
    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            rEv   <= 4'b0000;
            rBusy <= 1'b0;
            rX    <= {W{1'b0}};
            rMu   <= 96'sd0;
            rMd   <= 96'sd0;
            rRnd  <= 96'sd0;
        end else begin
            rEv[3] <= rEv[2];
            rEv[2] <= rEv[1];
            rEv[1] <= rEv[0];
            rEv[0] <= 1'b0;
            rRnd   <= 96'sd1 <<< (FSHIFT - 1);       // 舍入加数
            if (iEn && !rBusy) begin
                rEv[0] <= 1'b1;                      // 下拍进入 mean 级
                rX     <= iXIn;
                rMu    <= $signed(ALPHAQ) * $signed(iXIn); // a*x 乘法一拍
                rMd    <= $signed(ONE_A)  * rMean;         // (1-a)*mean 乘法一拍
                rBusy  <= 1'b1;
            end
            if (rEv[3]) rBusy <= 1'b0;               // 提交完成后空闲
        end
    end

    // ---- mean/var 组合(读 rMu/rMd 寄存): mean_n=(a*x+(1-a)mean+rnd)>>>FSHIFT ----
    wire signed [95:0] wMeanSum = rMu + rMd + rRnd;
    wire signed [63:0] wMean    = wMeanSum >>> FSHIFT;      // 低 64 即 mean[n]
    wire signed [95:0] wVarSum  = rVu + rVd + rRnd;         // var 分子
    wire signed [63:0] wVarSh   = wVarSum >>> FSHIFT;       // var[n] 低 64

    // ---- 提交流水 ----
    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            rMean  <= 64'sd0;
            rVar   <= 64'sd0;
            oValid <= 1'b0;
            rEsq   <= 96'sd0;
            rVu    <= 96'sd0;
            rVd    <= 96'sd0;
            rStd   <= 32'd0;
        end else begin
            oValid <= 1'b0;
            if (rEv[0]) begin
                rEsq <= ($signed(rX) - wMean) * ($signed(rX) - wMean); // 一拍: err^2
            end
            if (rEv[1]) begin
                rVu  <= $signed(ALPHAQ) * rEsq;      // a*err^2 乘法一拍(并行)
                rVd  <= $signed(ONE_A)  * rVar;      // (1-a)*var 乘法一拍
            end
            if (rEv[2]) begin
                rMean <= wMean;                      // 一拍: 提交 mean[n]
                if (wVarSh < 0)
                    rVar <= 64'sd0;                  // 截零(数值保护)
                else
                    rVar <= wVarSh;
                oValid <= 1'b1;
            end
            if (sqrt_done) rStd <= sqrt_root;        // 开方完成刷新标准差
        end
    end

    assign oVar = rVar[OVW-1:0];          // 方差(>=0 低截)
    assign oStd = rStd[OSW-1:0];          // 标准差
endmodule
