//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   CompGcc.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
/*Description       :   广义互相关方向检测, 递推 EWMA 版 (C Comp_GCC_Recursive, lag=0)。
                        dx=x-mean_x, dy=y-mean_y；mean<-a*mean+b*x；var<-a*var+b*dx^2；
                        cov<-a*cov+b*dx*dy；rho=cov/sqrt(var_x*var_y) 夹到 [-1,1]。
                        |rho|<thr->UNKNOWN(0)；rho>0->SAME(1)；rho<0->OPPOSITE(-1)。
                        信号 S0.(W-1) 有符号。平方根/除法统一例化 AlgoSqrt/AlgoDiv
                        （多拍 start/done 握手，参照 ROCA，禁止组合除法）。EWMA 状态逐拍
                        更新，rho 后台计算完成即刷新（结果滞后若干拍，准静态段方向不变）。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.3   (多拍握手化)
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
module CompGcc #(
    parameter integer W          = 16,      // 输入信号位宽(有符号), 亦为 rho 输出位宽(Q0.15)
    parameter integer FSHIFT     = 15,      // alpha 小数位 Q0.FSHIFT
    parameter integer ALPHAQ     = 30720,   // alpha*2^FSHIFT (alpha=15/16, 等效窗 N~16)
    parameter integer CORR_TH_Q  = 9830,    // 判定阈值 Q0.15 (0.30)
    parameter integer WARMUP     = 128      // 预热样本数, 未达到前 valid=0
) (
    input  wire                iSysClk,     // 时钟
    input  wire                iSysRst,     // 复位（高有效）
    input  wire                iEn,         // 采样节拍(每拍处理一对新样本)
    input  wire signed [W-1:0] iXIn,        // 传感器 X 采样
    input  wire signed [W-1:0] iYIn,        // 传感器 Y 采样
    output reg  signed [W-1:0] oRho,        // 归一化互相关 (Q0.15)
    output reg  signed [1:0]   oDirection,  // 0=UNKNOWN, 1=SAME, -1=OPPOSITE
    output reg                 oValid       // 有效标志(预热/退化时为 0)
);
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

    // ---- EWMA 系数定点常量 ----
    localparam integer ONE_Q = (1 << FSHIFT);   // 1 的定点
    localparam integer BETAQ = ONE_Q - ALPHAQ;  // (1-alpha) 定点

    // ---- 状态 ----
    reg signed [63:0] rMeanX;       // x 的 EWMA 均值(代码域)
    reg signed [63:0] rMeanY;       // y 的 EWMA 均值
    reg signed [63:0] rVarX;        // x 的 EWMA 方差(>=0, 代码域平方)
    reg signed [63:0] rVarY;        // y 的 EWMA 方差
    reg signed [63:0] rCovXY;       // xy 的 EWMA 协方差(可负)
    reg [31:0]        rSampleCnt;   // 已处理样本数

    // ---- 组合 EWMA 更新量(由当前状态+本拍样本推出, 阻塞计算) ----
    reg signed [95:0] c_rndq;                   // 舍入加半
    reg signed [95:0] c_tt;                     // EWMA 分子
    reg signed [63:0] c_dx, c_dy;               // 偏差(前一时刻均值)
    reg signed [63:0] c_esqx, c_esqy, c_esqc;   // dx^2, dy^2, dx*dy
    reg signed [63:0] c_mx, c_my;               // 新均值
    reg signed [63:0] c_vx, c_vy;               // 新方差
    reg signed [63:0] c_cv;                     // 新协方差
    reg [63:0]        c_vxcl, c_vycl;           // 钳位后方差(>=0)
    reg [31:0]        c_cnt;                    // 新样本计数

    always @(*) begin
        c_rndq = 96'sd1 <<< (FSHIFT - 1);

        // ---- 用前一时刻均值计算偏差 ----
        c_dx   = iXIn - rMeanX;
        c_dy   = iYIn - rMeanY;
        c_esqx = c_dx * c_dx;
        c_esqy = c_dy * c_dy;
        c_esqc = c_dx * c_dy;

        // ---- 递推更新(均值/方差/协方差) ----
        c_tt  = $signed(ALPHAQ) * rMeanX + $signed(BETAQ) * iXIn + c_rndq;
        c_mx  = c_tt >>> FSHIFT;
        c_tt  = $signed(ALPHAQ) * rMeanY + $signed(BETAQ) * iYIn + c_rndq;
        c_my  = c_tt >>> FSHIFT;

        c_tt  = $signed(ALPHAQ) * rVarX + $signed(BETAQ) * c_esqx + c_rndq;
        c_vx  = c_tt >>> FSHIFT;
        c_tt  = $signed(ALPHAQ) * rVarY + $signed(BETAQ) * c_esqy + c_rndq;
        c_vy  = c_tt >>> FSHIFT;
        c_tt  = $signed(ALPHAQ) * rCovXY + $signed(BETAQ) * c_esqc + c_rndq;
        c_cv  = c_tt >>> FSHIFT;

        c_cnt  = rSampleCnt + 32'd1;
        c_vxcl = (c_vx < 0) ? 64'd0 : c_vx;
        c_vycl = (c_vy < 0) ? 64'd0 : c_vy;
    end

    // 分母非零代理: sx=floor(sqrt(var)), 仅当 var>=1 才可能 denom!=0
    wire c_denOk  = (c_vxcl >= 64'd1) && (c_vycl >= 64'd1);
    wire c_valid  = (c_cnt >= WARMUP) && c_denOk;

    // ---- 后台计算流水: 快照(新)varX/varY/cov -> sqrt -> 除法 -> rho/方向 ----
    localparam [2:0] PS_IDLE = 3'd0;
    localparam [2:0] PS_SQ   = 3'd1;
    localparam [2:0] PS_WSQ  = 3'd2;
    localparam [2:0] PS_DIV  = 3'd3;
    localparam [2:0] PS_WDIV = 3'd4;
    localparam [2:0] PS_RHO  = 3'd5;
    reg [2:0] ps = PS_IDLE;

    reg [63:0] rVarXc, rVarYc;    // 快照方差(无符号幅值)
    reg [63:0] rCovc;             // 快照协方差(有符号)
    reg [31:0] rSx, rSy;          // sqrt 结果
    reg [15:0] rQcl;              // 除法商 Q0.15 夹取(|rho|)
    reg        rNeg;              // cov 符号
    reg signed [15:0] rRhoVal;    // 计算得到 rho(Q0.15 有符号)
    reg signed [1:0]  rDirVal;    // 计算得到方向

    wire wSqrtStart = (ps == PS_SQ);
    wire wDivStart  = (ps == PS_DIV);

    wire        sqx_done, sqy_done, sqx_busy, sqy_busy;
    wire [31:0] sqx_root, sqy_root;
    AlgoSqrt #(.IW(64), .OW(32)) U_SqX (
        .iSysClk(iSysClk), .iSysRst(iSysRst), .iStart(wSqrtStart),
        .iNum(rVarXc), .oBusy(sqx_busy), .oDone(sqx_done), .oRoot(sqx_root));
    AlgoSqrt #(.IW(64), .OW(32)) U_SqY (
        .iSysClk(iSysClk), .iSysRst(iSysRst), .iStart(wSqrtStart),
        .iNum(rVarYc), .oBusy(sqy_busy), .oDone(sqy_done), .oRoot(sqy_root));

    wire [63:0] denom_w = rSx * rSy;
    wire [63:0] covabs  = rNeg ? (~rCovc + 64'd1) : rCovc;
    wire [63:0] numrd   = (covabs << 15) + (denom_w >> 1);
    wire [63:0] mxw     = (numrd > denom_w) ? numrd : denom_w;
    wire [6:0]  shw     = f_sh31(mxw);
    wire [31:0] num32   = numrd >> shw;
    wire [31:0] den32   = denom_w >> shw;

    wire        div_done, div_busy;
    wire [31:0] div_quot;
    AlgoDiv #(.W(32)) U_Div (
        .iSysClk(iSysClk), .iSysRst(iSysRst), .iStart(wDivStart),
        .iNum(num32), .iDen(den32),
        .oBusy(div_busy), .oDone(div_done), .oQuot(div_quot));

    // 商 -> |rho| Q0.15 夹取 / 方向 (未夹取前比较用同一值)
    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            ps       <= PS_IDLE;
            rVarXc   <= 64'd0; rVarYc <= 64'd0; rCovc <= 64'd0;
            rSx      <= 32'd0; rSy    <= 32'd0;
            rQcl     <= 16'd0; rNeg   <= 1'b0;
            rRhoVal  <= 16'sd0;
            rDirVal  <= 2'sd0;
        end else begin
            case (ps)
                PS_IDLE: begin
                    if (iEn) begin
                        rVarXc <= c_vxcl;          // 快照本拍更新后的方差/协方差
                        rVarYc <= c_vycl;
                        rCovc  <= c_cv;
                        rNeg   <= c_cv[63];
                        ps     <= PS_SQ;
                    end
                end
                PS_SQ:   ps <= PS_WSQ;              // sqrt 启动脉冲
                PS_WSQ: begin
                    if (sqx_done) begin
                        rSx <= sqx_root;
                        rSy <= sqy_root;            // 双 sqrt 同延迟
                        ps  <= PS_DIV;
                    end
                end
                PS_DIV:  ps <= PS_WDIV;              // 除法启动脉冲
                PS_WDIV: begin
                    if (div_done) begin
                        rQcl <= (div_quot > 32'd32767) ? 16'h7FFF : div_quot[15:0];
                        ps   <= PS_RHO;
                    end
                end
                PS_RHO: begin
                    // 依据快照 cov 符号还原 rho, 并做方向判定
                    if (rNeg) rRhoVal <= -$signed({1'b0, rQcl});
                    else      rRhoVal <=  $signed({1'b0, rQcl});
                    if (rQcl < CORR_TH_Q)          rDirVal <= 2'sd0;
                    else if (!rNeg)                rDirVal <= 2'sd1;
                    else                           rDirVal <= -2'sd1;
                    ps <= PS_IDLE;
                end
                default: ps <= PS_IDLE;
            endcase
        end
    end

    // ---- EWMA 状态 + 输出（输出仅本 always 驱动，避免多驱动）----
    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            rMeanX     <= 64'sd0;
            rMeanY     <= 64'sd0;
            rVarX      <= 64'sd0;
            rVarY      <= 64'sd0;
            rCovXY     <= 64'sd0;
            rSampleCnt <= 32'd0;
            oRho       <= {W{1'b0}};
            oDirection <= 2'sd0;
            oValid     <= 1'b0;
        end else if (iEn) begin
            rMeanX     <= c_mx;
            rMeanY     <= c_my;
            rVarX      <= c_vxcl;
            rVarY      <= c_vycl;
            rCovXY     <= c_cv;
            rSampleCnt <= c_cnt;
            oRho       <= c_valid ? rRhoVal : 16'sd0;
            oDirection <= c_valid ? rDirVal : 2'sd0;
            oValid     <= c_valid;
        end
    end
endmodule
