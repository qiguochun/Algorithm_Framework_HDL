//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   CompKalman1d.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
/*Description       :   一维卡尔曼滤波器（1D Kalman_Filter_s 语义，kalman.h）。
                        P<-P+Q；G<-P/(P+R)；Out<-Out+G*(m-Out)；P<-(1-G)*P。
                        量测/估计为 S0.(W-1) 有符号；P/Q/R 为整数协方差参数；
                        增益 G=floor(P_aug*2^KFRAC/(P_aug+R))，分子 Q 定标由本模块左移
                        KFRAC 完成。除法统一例化 AlgoDiv（多拍 start/done 握手），
                        空闲且节拍时捕获 P_aug 快照起算，完成后刷新寄存器增益 rG。
                        状态更新多拍流水(3 级): accept 快照量测 -> rEv[0] innov/Paug
                        (一拍) -> rEv[1] G*innov / G*Paug 两乘法并行(一拍, 各单寄存)
                        -> rEv[2] 舍入右移+提交(一拍)。每个乘法结果单独寄存,
                        无单周期组合乘加; oValid 于提交拍拉高。
                        常值量测 m 下: Out -> m。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.4   (多拍流水化)
//modifier          :
//Modify Date       :
/*Modify Record     :
//  Rev 0.3: 除法多拍握手化
//  Rev 0.4: Out/P 更新组合乘加拆 rEv 多拍流水, 新增 oValid。
*/
//------------------------------------------------------------------------------
module CompKalman1d #(
    parameter integer W        = 16,     // 量测/估计位宽(有符号)
    parameter integer QRAW     = 200,    // 过程噪声协方差 Q(整数尺度)
    parameter integer RRAW     = 3000,   // 量测噪声协方差 R(整数尺度)
    parameter integer KFRAC    = 15      // 增益 G 的小数位 Q0.KFRAC
) (
    input  wire                iSysClk,  // 时钟
    input  wire                iSysRst,  // 复位（高有效）
    input  wire                iEn,      // 采样节拍
    input  wire signed [W-1:0] iMIn,     // 量测
    output reg  signed [W-1:0] oYOut,    // 滤波估计
    output reg                 oValid    // 提交拍有效脉冲(采样后 LATENCY=3 拍)
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

    // ---- 内部状态 ----
    reg signed [63:0] rP   = 64'sd0;     // 协方差 P
    reg signed [63:0] rOut = 64'sd0;     // 估计 Out
    reg signed [63:0] rG   = 64'sd0;     // 增益 Q0.KFRAC（正数, 后台计算刷新）

    wire signed [63:0] p_aug_w = rP + $signed(QRAW);    // 预测协方差 P+Q(>=Q>0)

    // ---- 增益除法后台流水: G = floor((P+Q)<<KFRAC / ((P+Q)+R)) ----
    localparam [1:0] PS_IDLE = 2'd0, PS_START = 2'd1, PS_WAIT = 2'd2;
    reg [1:0] ps = PS_IDLE;
    reg [63:0] rPaug = 64'd0;            // 快照 P_aug(用于分子/分母, 与 rP 演进解耦)

    wire wDivStart = (ps == PS_START);

    wire [63:0] pnum = (rPaug <<< KFRAC);                 // 分子: p_aug*2^KFRAC
    wire [63:0] pden = $unsigned(rPaug) + $unsigned(RRAW); // 分母: p_aug+R(>0)
    wire [63:0] mxw  = (pnum > pden) ? pnum : pden;
    wire [6:0]  shw  = f_sh31(mxw);
    wire [31:0] num32 = pnum >> shw;
    wire [31:0] den32 = pden >> shw;

    wire        div_done, div_busy;
    wire [31:0] div_quot;
    AlgoDiv #(.W(32)) U_Div (
        .iSysClk(iSysClk), .iSysRst(iSysRst), .iStart(wDivStart),
        .iNum(num32), .iDen(den32),
        .oBusy(div_busy), .oDone(div_done), .oQuot(div_quot));

    // =====================================================================
    // 多拍状态更新事件 rEv[2:0]: [0]=innov  [1]=mult  [2]=commit
    // =====================================================================
    reg [2:0] rEv;
    reg       rBusy;
    reg signed [W-1:0] rM;                       // 量测快照
    reg signed [63:0] rInnov;                    // 新息 m-Out(一拍)
    reg signed [63:0] rPaugS;                    // P+Q 快照(一拍)
    reg signed [95:0] rCu;                       // G*innov(乘法一拍)
    reg signed [95:0] rCp;                       // G*Paug (乘法一拍, 并行)

    wire signed [95:0] wCuA  = $signed({ {32{rCu[95]}},   rCu });
    wire signed [95:0] wRnd  = 96'sd1 <<< (KFRAC - 1);   // 舍入加数
    wire signed [95:0] wOutE = $signed({ {32{rOut[63]}}, rOut });
    wire signed [95:0] wPaugE= $signed({ {32{rPaugS[63]}}, rPaugS });
    // 提交量 o_n = Out + ((G*innov + rnd)>>>KFRAC)
    wire signed [95:0] wON   = wOutE + ((wCuA + wRnd) >>> KFRAC);
    // 提交量 p_n = Paug - ((G*Paug + rnd)>>>KFRAC) (>=0 截零)
    wire signed [95:0] wPN   = wPaugE - ((rCp + wRnd) >>> KFRAC);

    // ---- 事件移位 + 量测快照 ----
    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            rEv   <= 3'b000;
            rBusy <= 1'b0;
            rM    <= {W{1'b0}};
        end else begin
            rEv[2] <= rEv[1];
            rEv[1] <= rEv[0];
            rEv[0] <= 1'b0;
            if (iEn && !rBusy) begin
                rEv[0] <= 1'b1;                // 下拍进入 innov 级
                rM     <= iMIn;                // 量测快照
                rBusy  <= 1'b1;
            end
            if (rEv[2]) rBusy <= 1'b0;         // 提交完成后空闲
        end
    end

    // ---- 增益除法后台流水 (除法完成后刷新 rG) ----
    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            ps     <= PS_IDLE;
            rPaug  <= 64'd0;
            rG     <= 64'sd0;
        end else begin
            case (ps)
                PS_IDLE: begin
                    if (iEn) begin
                        rPaug <= p_aug_w;
                        ps    <= PS_START;
                    end
                end
                PS_START: ps <= PS_WAIT;      // 除法启动脉冲
                PS_WAIT: begin
                    if (div_done) begin
                        rG <= $signed({32'd0, div_quot});
                        ps <= PS_IDLE;
                    end
                end
                default: ps <= PS_IDLE;
            endcase
        end
    end

    // ---- 状态提交流水 (rEv[0] innov -> rEv[1] mult -> rEv[2] commit) ----
    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            rP     <= 64'sd0;
            rOut   <= 64'sd0;
            oYOut  <= {W{1'b0}};
            oValid <= 1'b0;
            rInnov <= 64'sd0;
            rPaugS <= 64'sd0;
            rCu    <= 96'sd0;
            rCp    <= 96'sd0;
        end else begin
            oValid <= 1'b0;
            if (rEv[0]) begin
                rInnov <= $signed(rM) - rOut;    // 一拍: 新息
                rPaugS <= p_aug_w;               // 一拍: P+Q
            end
            if (rEv[1]) begin
                rCu    <= rG * rInnov;           // 乘法一拍(两乘并行)
                rCp    <= rG * rPaugS;
            end
            if (rEv[2]) begin
                rOut  <= wON[63:0];              // 一拍: 舍入右移 + 提交
                if (wPN < 0)
                    rP <= 64'sd0;
                else
                    rP <= wPN[63:0];
                oYOut <= wON[W-1:0];
                oValid<= 1'b1;
            end
        end
    end
endmodule
