//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   CompXcorr.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
/*Description       :   滑动窗口归一化互相关 (C Comp_XCorr_F32 滑动窗口核心)。
                        环形缓冲存最近 WIN_SIZE 对 (x,y)；窗口满后后台逐拍 MAC：
                        先累加 rxx/ryy，再按 tau=0..MAX_LAG 累加 Rxy[tau]，
                        再 AlgoSqrt/AlgoDiv 得 r[tau]=Rxy/sqrt(rxx*ryy)，
                        oRPeak=|r|max、oLagPeak=对应 tau。
                        信号 S0.(W-1) 有符号；能量 64bit；输出 r 为 Q0.15。
                        禁止组合整窗乘加核；WIN_SIZE 建议 2 的幂（用位掩码取模）。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.4   (流水 MAC，消组合核/多驱动)
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
module CompXcorr #(
    parameter integer W        = 16,       // 信号位宽(有符号), 亦为 r_peak 位宽(Q0.15)
    parameter integer WIN_SIZE = 128,      // 窗口长度(样本), 1<WIN_SIZE<=256，建议 2 的幂
    parameter integer MAX_LAG  = 16,       // 最大滞后, MAX_LAG<WIN_SIZE 且 <=128
    parameter integer LAGW     = 8         // lag_peak 位宽(>= clog2(MAX_LAG+1))
) (
    input  wire                iSysClk,    // 时钟
    input  wire                iSysRst,    // 复位（高有效）
    input  wire                iEn,        // 采样节拍(每拍压入一对新样本)
    input  wire signed [W-1:0] iXIn,       // x 信号采样
    input  wire signed [W-1:0] iYIn,       // y 信号采样
    output reg  signed [W-1:0] oRPeak,     // |r| 最大处的归一化相关值 (Q0.15)
    output reg  [LAGW-1:0]     oLagPeak,   // oRPeak 对应滞后 tau
    output reg                 oReady      // 窗口已满标志(此后保持)
);
    function automatic [6:0] f_sh31;
        input [63:0] v;
        integer i;
        begin
            f_sh31 = 7'd0;
            for (i = 0; i < 64; i = i + 1)
                if (v[i]) f_sh31 = (i > 30) ? (i - 30) : 7'd0;
        end
    endfunction

    localparam integer IDXW = $clog2(WIN_SIZE);
    // 非 2 幂时退回 %；2 幂用掩码（综合友好）
    localparam [IDXW-1:0] WIN_MASK = WIN_SIZE - 1;
    localparam integer POW2 = (WIN_SIZE == (1 << IDXW));

    reg signed [W-1:0] rBufX [0:WIN_SIZE-1];
    reg signed [W-1:0] rBufY [0:WIN_SIZE-1];
    reg [IDXW-1:0]     rWrIdx;
    reg [IDXW:0]       rSampleCnt;   // 0..WIN_SIZE

    function [IDXW-1:0] f_wrap;
        input [IDXW:0] a;
        begin
            if (POW2)
                f_wrap = a[IDXW-1:0] & WIN_MASK;
            else
                f_wrap = a % WIN_SIZE;
        end
    endfunction

    wire [IDXW-1:0] wNextWr = f_wrap({1'b0, rWrIdx} + 1'b1);
    wire [IDXW:0]   wNextCnt = (rSampleCnt < WIN_SIZE) ? (rSampleCnt + 1'b1) : rSampleCnt;

    // ---- 后台 FSM: 空闲捕获写指针 -> 累加能量 -> 按 lag MAC -> sqrt -> div -> 峰值 ----
    localparam [3:0] ST_IDLE  = 4'd0;
    localparam [3:0] ST_EACC  = 4'd1;   // 累加 rxx/ryy
    localparam [3:0] ST_LACC  = 4'd2;   // 累加某一 lag 的 Rxy
    localparam [3:0] ST_LNXT  = 4'd3;   // 切下一 lag
    localparam [3:0] ST_SQ    = 4'd4;
    localparam [3:0] ST_WSQ   = 4'd5;
    localparam [3:0] ST_DIV   = 4'd6;
    localparam [3:0] ST_WDIV  = 4'd7;
    localparam [3:0] ST_PK    = 4'd8;

    reg [3:0]          rSt;
    reg [IDXW-1:0]     rBase;          // 快照：满窗最旧样本线性起点(=压入后写指针)
    reg [IDXW:0]       rK;             // 窗内线性下标
    reg [LAGW-1:0]     rTau;
    reg [63:0]         rRxx, rRyy;
    reg signed [63:0]  rRxyAcc;
    reg signed [63:0]  rRxyCap [0:MAX_LAG];
    reg [31:0]         rRxRoot, rRyRoot;
    reg [15:0]         rAbsCap [0:MAX_LAG];
    reg                rNegCap [0:MAX_LAG];
    integer tcap;

    wire        wSqrtStart = (rSt == ST_SQ);
    wire        wDivStart  = (rSt == ST_DIV);

    wire        sqx_done, sqy_done, sqx_busy, sqy_busy;
    wire [31:0] sqx_root, sqy_root;
    AlgoSqrt #(.IW(64), .OW(32)) U_SqX (
        .iSysClk(iSysClk), .iSysRst(iSysRst), .iStart(wSqrtStart),
        .iNum(rRxx), .oBusy(sqx_busy), .oDone(sqx_done), .oRoot(sqx_root));
    AlgoSqrt #(.IW(64), .OW(32)) U_SqY (
        .iSysClk(iSysClk), .iSysRst(iSysRst), .iStart(wSqrtStart),
        .iNum(rRyy), .oBusy(sqy_busy), .oDone(sqy_done), .oRoot(sqy_root));

    wire [63:0] denom_w = rRxRoot * rRyRoot;

    wire [31:0] qDiv [0:MAX_LAG];
    wire        dDone [0:MAX_LAG];
    wire        dBusy [0:MAX_LAG];
    genvar gt;
    generate
        for (gt = 0; gt <= MAX_LAG; gt = gt + 1) begin : gDiv
            wire [63:0] rabs_gt  = (rRxyCap[gt][63])
                                 ? (~rRxyCap[gt] + 64'd1) : rRxyCap[gt];
            wire [63:0] numrd_gt = (rabs_gt << 15) + (denom_w >> 1);
            wire [63:0] mx_gt    = (numrd_gt > denom_w) ? numrd_gt : denom_w;
            wire [6:0]  sh_gt    = f_sh31(mx_gt);
            wire [31:0] num32_gt = numrd_gt >> sh_gt;
            wire [31:0] den32_gt = denom_w  >> sh_gt;
            AlgoDiv #(.W(32)) U_Div (
                .iSysClk(iSysClk), .iSysRst(iSysRst), .iStart(wDivStart),
                .iNum(num32_gt), .iDen(den32_gt),
                .oBusy(dBusy[gt]), .oDone(dDone[gt]), .oQuot(qDiv[gt]));
        end
    endgenerate
    wire all_done = dDone[0];

    reg signed [15:0] peak_sig;
    reg [15:0]        peak_abs;
    reg [LAGW-1:0]    peak_lag;
    integer tp;
    always @(*) begin
        peak_abs = 16'd0;
        peak_sig = 16'sd0;
        peak_lag = {LAGW{1'b0}};
        for (tp = 0; tp <= MAX_LAG; tp = tp + 1) begin
            if (rAbsCap[tp] > peak_abs) begin
                peak_abs = rAbsCap[tp];
                peak_lag = tp[LAGW-1:0];
                peak_sig = rNegCap[tp]
                         ? -$signed({1'b0, rAbsCap[tp]})
                         :  $signed({1'b0, rAbsCap[tp]});
            end
        end
    end

    // 缓冲只驱动 ready/写指针（峰值由计算 FSM 独占）
    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            rWrIdx     <= {IDXW{1'b0}};
            rSampleCnt <= {(IDXW+1){1'b0}};
            oReady     <= 1'b0;
        end else if (iEn) begin
            if (wNextCnt == WIN_SIZE) oReady <= 1'b1;
            rBufX[rWrIdx] <= iXIn;
            rBufY[rWrIdx] <= iYIn;
            rWrIdx        <= wNextWr;
            rSampleCnt    <= wNextCnt;
        end
    end

    // 线性下标 k -> 环形地址；显式 signed 临时量，避免变址读丢失符号导致无符号乘
    wire [IDXW-1:0] wAddrK  = f_wrap({1'b0, rBase} + rK);
    wire [IDXW-1:0] wAddrKt = f_wrap({1'b0, rBase} + rK + rTau);
    wire signed [W-1:0] wXk  = rBufX[wAddrK];
    wire signed [W-1:0] wYk  = rBufY[wAddrK];
    wire signed [W-1:0] wYkt = rBufY[wAddrKt];
    wire signed [31:0]  wX2  = wXk * wXk;
    wire signed [31:0]  wY2  = wYk * wYk;
    wire signed [31:0]  wXY  = wXk * wYkt;

    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            rSt      <= ST_IDLE;
            rBase    <= {IDXW{1'b0}};
            rK       <= {(IDXW+1){1'b0}};
            rTau     <= {LAGW{1'b0}};
            rRxx     <= 64'd0;
            rRyy     <= 64'd0;
            rRxyAcc  <= 64'sd0;
            rRxRoot  <= 32'd0;
            rRyRoot  <= 32'd0;
            for (tcap = 0; tcap <= MAX_LAG; tcap = tcap + 1) begin
                rRxyCap[tcap] <= 64'sd0;
                rAbsCap[tcap] <= 16'd0;
                rNegCap[tcap] <= 1'b0;
            end
            oRPeak   <= {W{1'b0}};
            oLagPeak <= {LAGW{1'b0}};
        end else begin
            case (rSt)
                ST_IDLE: begin
                    if (iEn && (wNextCnt == WIN_SIZE)) begin
                        rBase   <= wNextWr;
                        rRxx    <= 64'd0;
                        rRyy    <= 64'd0;
                        rK      <= {(IDXW+1){1'b0}};
                        rTau    <= {LAGW{1'b0}};
                        rSt     <= ST_EACC;
                    end
                end
                ST_EACC: begin
                    rRxx <= rRxx + $unsigned(wX2);
                    rRyy <= rRyy + $unsigned(wY2);
                    if (rK == WIN_SIZE - 1) begin
                        rK      <= {(IDXW+1){1'b0}};
                        rRxyAcc <= 64'sd0;
                        rSt     <= ST_LACC;
                    end else begin
                        rK <= rK + 1'b1;
                    end
                end
                ST_LACC: begin
                    rRxyAcc <= rRxyAcc + wXY;
                    if (rK == (WIN_SIZE - 1 - rTau)) begin
                        rSt <= ST_LNXT;
                    end else begin
                        rK <= rK + 1'b1;
                    end
                end
                ST_LNXT: begin
                    rRxyCap[rTau] <= rRxyAcc;
                    if (rTau == MAX_LAG) begin
                        rSt <= ST_SQ;
                    end else begin
                        rTau    <= rTau + 1'b1;
                        rK      <= {(IDXW+1){1'b0}};
                        rRxyAcc <= 64'sd0;
                        rSt     <= ST_LACC;
                    end
                end
                ST_SQ:  rSt <= ST_WSQ;
                ST_WSQ: begin
                    if (sqx_done) begin
                        rRxRoot <= sqx_root;
                        rRyRoot <= sqy_root;
                        rSt     <= ST_DIV;
                    end
                end
                ST_DIV: rSt <= ST_WDIV;
                ST_WDIV: begin
                    if (all_done) begin
                        for (tcap = 0; tcap <= MAX_LAG; tcap = tcap + 1) begin
                            rAbsCap[tcap] <= (qDiv[tcap] > 32'd32767)
                                           ? 16'h7FFF : qDiv[tcap][15:0];
                            rNegCap[tcap] <= rRxyCap[tcap][63];
                        end
                        rSt <= ST_PK;
                    end
                end
                ST_PK: begin
                    oRPeak   <= peak_sig[W-1:0];
                    oLagPeak <= peak_lag[LAGW-1:0];
                    rSt      <= ST_IDLE;
                end
                default: rSt <= ST_IDLE;
            endcase
        end
    end
endmodule
