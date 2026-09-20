//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   CompRms.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
/*Description       :   窗口 RMS 均方根（C Comp_Rms 平方累加核心，固定 N 样本窗）。
                        acc += x^2 每样本；每累计 N 个样本输出 rms = sqrt(acc/N) 并复位。
                        信号 S0.(W-1) 有符号，平方累加用 64bit；N 样本窗由参数决定。
                        rms = sqrt(acc/N) 统一例化 AlgoDiv/AlgoSqrt（多拍 start/done 握手，
                        参照 ROCA，禁止组合除法）。除运算在窗口边界起算，忙期持续多拍，
                        完成后 oRmsValid 拉一拍并更新 oRmsVal（结果滞后若干拍，逐窗刷新）。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.3   (多拍握手化)
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
module CompRms #(
    parameter integer W        = 16,     // 信号/输出位宽
    parameter integer N        = 100     // RMS 窗口样本数
) (
    input  wire                iSysClk,      // 时钟
    input  wire                iSysRst,      // 复位（高有效）
    input  wire                iEn,          // 采样节拍
    input  wire signed [W-1:0] iXIn,         // 输入采样
    output reg  signed [W-1:0] oRmsVal,      // RMS 输出(窗口完成计算后更新)
    output reg                 oRmsValid     // 结果有效脉冲(一拍)
);
    // ---- 64bit 能量 -> 32bit 有符号除数的归一化移位 ----
    // 返回最小移位 s，使 v>>s < 2^31（商事后左移回补，分子为 64bit 也安全）。
    function automatic [6:0] f_sh31;
        input [63:0] v;
        integer i;
        begin
            f_sh31 = 7'd0;
            for (i = 0; i < 64; i = i + 1)
                if (v[i]) f_sh31 = (i > 30) ? (i - 30) : 7'd0;
        end
    endfunction

    // ---- 内部状态：窗口累加（独立于计算状态机，持续随 iEn 运行）----
    reg [63:0] rSumSq = 64'd0;        // 平方累加(无符号)
    reg [15:0] rCnt   = 16'd0;        // 窗内样本计数

    wire [63:0] sq_x  = iXIn * iXIn;      // x^2（平方恒非负）
    wire [63:0] acc_w = rSumSq + sq_x;    // 含本拍样本的窗内累加
    wire        win_done = iEn && (rCnt == N - 1);   // 本拍为窗内第 N 个样本

    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            rSumSq <= 64'd0;
            rCnt   <= 16'd0;
        end else if (iEn) begin
            if (rCnt == N - 1) begin
                rSumSq <= 64'd0;
                rCnt   <= 16'd0;
            end else begin
                rSumSq <= acc_w;
                rCnt   <= rCnt + 16'd1;
            end
        end
    end

    // ---- 计算状态机：div(acc,N) -> sqrt —— 多拍 start/done 握手 ----
    localparam [2:0] ST_ACC  = 3'd0;   // 等待窗口完成并捕获能量
    localparam [2:0] ST_DIVP = 3'd1;   // AlgoDiv 启动脉冲(1 拍)
    localparam [2:0] ST_WDIV = 3'd2;   // 等待除法 done
    localparam [2:0] ST_SQP  = 3'd3;   // AlgoSqrt 启动脉冲(1 拍)
    localparam [2:0] ST_WSQ  = 3'd4;   // 等待开方 done
    localparam [2:0] ST_OUT  = 3'd5;   // 输出 RMS 并脉冲 oRmsValid

    reg [2:0]  st = ST_ACC;
    reg [63:0] rAccCap = 64'd0;        // 捕获的整窗能量(稳定保持至计算结束)
    reg [31:0] rQuot   = 32'd0;        // AlgoDiv 商
    reg [31:0] rRoot   = 32'd0;        // AlgoSqrt 根(= rms)

    wire [6:0]  wSh  = f_sh31(rAccCap);              // 归一化移位
    wire [63:0] wNum = rAccCap >> wSh;               // <2^31 的除数域被除数
    wire [63:0] wQft = {32'd0, rQuot} << wSh;        // 恢复尺度后的商 floor(acc/N)

    wire wDivStart = (st == ST_DIVP);
    wire wSqStart  = (st == ST_SQP);

    localparam [31:0] L_N = N;                 // 窗长作为 32bit 除数

    // 例化：除 floor(acc/N)，再开方 -> rms
    wire [31:0] U_DivQuot, U_SqrtRoot;
    wire        U_DivDone, U_SqrtDone, U_DivBusy, U_SqrtBusy;
    AlgoDiv #(.W(32)) U_Div (
        .iSysClk(iSysClk), .iSysRst(iSysRst), .iStart(wDivStart),
        .iNum (wNum[31:0]), .iDen (L_N),
        .oBusy(U_DivBusy), .oDone(U_DivDone), .oQuot(U_DivQuot)
    );
    AlgoSqrt #(.IW(64), .OW(32)) U_Sqrt (
        .iSysClk(iSysClk), .iSysRst(iSysRst), .iStart(wSqStart),
        .iNum(wQft),
        .oBusy(U_SqrtBusy), .oDone(U_SqrtDone), .oRoot(U_SqrtRoot)
    );

    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            st       <= ST_ACC;
            rAccCap  <= 64'd0;
            rQuot    <= 32'd0;
            rRoot    <= 32'd0;
            oRmsVal  <= {W{1'b0}};
            oRmsValid <= 1'b0;
        end else begin
            oRmsValid <= 1'b0;                 // 默认低
            case (st)
                ST_ACC: begin
                    if (win_done) begin
                        rAccCap <= acc_w;      // 捕获整窗能量(含本拍)
                        st      <= ST_DIVP;
                    end
                end
                ST_DIVP: begin                 // 除法启动脉冲(本拍高, 除法于末沿采到)
                    st <= ST_WDIV;
                end
                ST_WDIV: begin
                    if (U_DivDone) begin
                        rQuot <= U_DivQuot;
                        st    <= ST_SQP;
                    end
                end
                ST_SQP: begin                  // 开方启动脉冲
                    st <= ST_WSQ;
                end
                ST_WSQ: begin
                    if (U_SqrtDone) begin
                        rRoot   <= U_SqrtRoot;
                        st      <= ST_OUT;
                    end
                end
                ST_OUT: begin
                    oRmsVal   <= rRoot[W-1:0];
                    oRmsValid <= 1'b1;
                    st        <= ST_ACC;
                end
                default: st <= ST_ACC;
            endcase
        end
    end
endmodule
