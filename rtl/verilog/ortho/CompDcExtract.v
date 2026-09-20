//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   CompDcExtract.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
/*Description       :   直流提取观测器（SOGI/Notch 双模式，纹波 fc=100Hz Fs=5kHz）。
                        mode=0 (SOGI): oDcOut = x - v_alpha（反馈对消）；oAcAmp = sqrt(va^2+vb^2)
                        mode=1 (Notch): oDcOut = 陷波输出（IIR 递归内联，直流直通）；
                                        oAcAmp = |x - oDcOut|（瞬时残余近似）
                        【多拍流水版】iEn 单拍采样一帧；SOGI 路径例化 CompSogi(多拍, fc=100)，
                        幅值改后台 AlgoSqrt(IW=32,OW=16) 周期刷新（近 ZOH）；Notch 路径将 CompNotch
                        的 IIR 递归内联进本模块，按每样本一拍推进（系数/舍入与原模块一致）。
                        外层提交拍 = iEn 后 LATENCY 拍，oValid 同拍拉高一拍。
                        约束：相邻 iEn 间隔 >= LATENCY（单帧在飞）；LATENCY 需 > CompSogi.LATENCY(6)。
                        数值与原组合实现逐样本等价（仅整体延后 LATENCY 拍）。
                        信号 S16；oAcAmp 为近 ZOH 幅值（mode0 周期刷新、mode1 逐样本）。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.2  (多拍流水化 + iEn/oValid)
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
`include "algo_arith.vh"

module CompDcExtract #(
    parameter integer W      = 16,
    parameter integer MODE   = 0,               // 0=SOGI 对消, 1=Notch 陷波(内联)
    parameter integer LATENCY = 8               // 外层提交拍（需 > CompSogi.LATENCY）
) (
    input  wire                iSysClk,
    input  wire                iSysRst,
    input  wire                iEn,             // 采样节拍（单拍脉冲，一帧一样本）
    input  wire signed [W-1:0] iXIn,            // 混合输入：直流 + AC 纹波
    output reg  signed [W-1:0] oDcOut,
    output reg  signed [W-1:0] oAcAmp,
    output reg                 oValid           // 提交拍（iEn 后 LATENCY 拍拉高一拍）
);

    // ---- 采样拍移位链（每样本一拍推进）----
    reg [LATENCY-1:0] rPipe;
    reg signed [W-1:0] rX0;                     // 当前在飞样本 x[n]

    generate
        if (MODE == 0) begin : G_SOGI
            // ===================== mode=0：SOGI 反馈对消 =====================
            // SOGI（fc=100Hz, Fs=5kHz, k=sqrt2）Tustin Q15 折叠系数；LATENCY=6（内部默认）
            wire signed [W-1:0] wVA, wVB;
            wire                wSogiValid;
            CompSogi #(
                .C_VA_VA(16'sd27202),   // (1-A-B^2)/D = 0.830152
                .C_VA_VB(16'sd3768),    // 2B/D        = 0.114992
                .C_VA_X (16'sd2664),    // A/D         = 0.081312
                .C_VB   (16'sd2059)     // B           = 0.062832
            ) U_Sogi (
                .iSysClk(iSysClk), .iSysRst(iSysRst), .iEn(iEn),
                .iXIn(iXIn), .oVAlpha(wVA), .oVBeta(wVB), .oValid(wSogiValid)
            );

            // 直流对消：x[n]-va[n]，单减法组合、fn_sat 组合 wire、提交拍寄存
            wire signed [15:0] wDc0 = fn_sat($signed(rX0) - $signed(wVA), 5'd0);

            // ---- 外层提交：oDcOut/oValid ----
            always @(posedge iSysClk or posedge iSysRst) begin
                if (iSysRst) begin
                    rPipe  <= {LATENCY{1'b0}};
                    rX0    <= {W{1'b0}};
                    oDcOut <= {W{1'b0}};
                    oValid <= 1'b0;
                end else begin
                    rPipe[0] <= iEn;
                    if (LATENCY > 1)
                        rPipe[LATENCY-1:1] <= rPipe[LATENCY-2:0];
                    if (iEn) rX0 <= iXIn;
                    if (rPipe[LATENCY-1]) oDcOut <= wDc0;
                    oValid <= rPipe[LATENCY-1];
                end
            end

            // ---- 正交幅值：va^2+vb^2 分拍寄存 -> 后台 AlgoSqrt 周期刷新（近 ZOH）----
            reg signed [15:0] rMa, rMb;         // 幅值 FSM 快照
            reg signed [31:0] rMa2, rMb2;       // 平方（一拍）
            reg signed [47:0] rMag2;            // 平方和（一拍）
            reg [2:0] rMSt;                     // 幅值 FSM 状态

            localparam [2:0] MS_GRAB = 3'd0, MS_MUL = 3'd1, MS_ADD = 3'd2,
                             MS_START = 3'd3, MS_WAIT = 3'd4;

            wire         wMagStart = (rMSt == MS_START);
            wire         wMagBusy, wMagDone;
            wire [15:0]  wMagRoot;
            wire [31:0]  wMagNum  = rMag2[31:0];

            AlgoSqrt #(.IW(32), .OW(16)) U_Sqrt (
                .iSysClk(iSysClk), .iSysRst(iSysRst), .iStart(wMagStart),
                .iNum(wMagNum), .oBusy(wMagBusy), .oDone(wMagDone), .oRoot(wMagRoot)
            );

            always @(posedge iSysClk or posedge iSysRst) begin
                if (iSysRst) begin
                    rMSt   <= MS_GRAB;
                    rMa    <= {W{1'b0}};
                    rMb    <= {W{1'b0}};
                    rMa2   <= 32'sd0;
                    rMb2   <= 32'sd0;
                    rMag2  <= 48'sd0;
                    oAcAmp <= {W{1'b0}};
                end else begin
                    case (rMSt)
                        MS_GRAB: begin
                            rMa  <= wVA;
                            rMb  <= wVB;
                            rMSt <= MS_MUL;
                        end
                        MS_MUL: begin
                            rMa2 <= $signed(rMa) * $signed(rMa);
                            rMb2 <= $signed(rMb) * $signed(rMb);
                            rMSt <= MS_ADD;
                        end
                        MS_ADD: begin
                            rMag2 <= $signed(rMa2) + $signed(rMb2);
                            rMSt  <= MS_START;
                        end
                        MS_START: begin
                            rMSt <= MS_WAIT;     // 此拍 wMagStart=1，AlgoSqrt 捕获 rMag2
                        end
                        default: begin            // MS_WAIT：等 sqrt 完成（busy 在 done 前先低，勿提前离开）
                            if (wMagDone) begin
                                oAcAmp <= $signed(wMagRoot);
                                rMSt   <= MS_GRAB;
                            end
                        end
                    endcase
                end
            end
        end
        else begin : G_NCH
            // ===================== mode=1：Notch 递归内联 =====================
            // y[n]=sat(A1*y1 + A2*y2 + (x<<14) + B1*x1 + B2*x2, 14)；状态同原 CompNotch 例化。
            localparam signed [15:0] N_A1 = 16'sd30709;
            localparam signed [15:0] N_A2 = -16'sd14584;
            localparam signed [15:0] N_B1 = -16'sd32768;
            localparam signed [15:0] N_B2 = 16'sd16643;
            localparam integer NL = 6;                      // IIR 流水拍数（同原 CompNotch）

            reg signed [W-1:0] nY1, nY2;   // y[n-1], y[n-2]
            reg signed [W-1:0] nX1, nX2;   // x[n-1], x[n-2]
            reg signed [W-1:0] nY;         // 本帧陷波输出（DC 直通）
            reg signed [63:0]  nT1, nT2, nT3, nT4, nXT;
            reg signed [63:0]  nS1, nS2, nS3, nS4;
            reg [NL-1:0]       nPipe;

            // Q14 舍入+算术右移+S16 饱和（等价 fn_sat(…,14)，输入 64bit 截 48bit 足够）
            wire signed [15:0] wYn = fn_sat($signed(nS4[47:0]), 5'd14);

            // 直流直通 dc=notch y；残余幅值 |x-y|（单减法/abs 组合、提交拍寄存）
            wire signed [15:0] wDc1 = fn_sat($signed(nY), 5'd0);
            wire signed [15:0] wAc1 = fn_abs16(fn_sat($signed(rX0) - $signed(nY), 5'd0));

            // ---- 外层提交：oDcOut/oAcAmp/oValid ----
            always @(posedge iSysClk or posedge iSysRst) begin
                if (iSysRst) begin
                    rPipe  <= {LATENCY{1'b0}};
                    rX0    <= {W{1'b0}};
                    oDcOut <= {W{1'b0}};
                    oAcAmp <= {W{1'b0}};
                    oValid <= 1'b0;
                end else begin
                    rPipe[0] <= iEn;
                    if (LATENCY > 1)
                        rPipe[LATENCY-1:1] <= rPipe[LATENCY-2:0];
                    if (iEn) rX0 <= iXIn;
                    if (rPipe[LATENCY-1]) begin
                        oDcOut <= wDc1;
                        oAcAmp <= wAc1;
                    end
                    oValid <= rPipe[LATENCY-1];
                end
            end

            // ---- Notch IIR 递归（每样本一拍，流水推进，数值与原 CompNotch 逐位一致）----
            always @(posedge iSysClk or posedge iSysRst) begin
                if (iSysRst) begin
                    nY1   <= {W{1'b0}};
                    nY2   <= {W{1'b0}};
                    nX1   <= {W{1'b0}};
                    nX2   <= {W{1'b0}};
                    nY    <= {W{1'b0}};
                    nT1   <= 64'sd0; nT2 <= 64'sd0; nT3 <= 64'sd0; nT4 <= 64'sd0;
                    nXT   <= 64'sd0;
                    nS1   <= 64'sd0; nS2 <= 64'sd0; nS3 <= 64'sd0; nS4 <= 64'sd0;
                    nPipe <= {NL{1'b0}};
                end else begin
                    // 拍0（iEn 采样）：4 乘积 + x<<14 并行一拍，x 历史后移
                    if (iEn && (nPipe == {NL{1'b0}})) begin
                        nT1  <= $signed(N_A1) * nY1;
                        nT2  <= $signed(N_A2) * nY2;
                        nT3  <= $signed(N_B1) * nX1;
                        nT4  <= $signed(N_B2) * nX2;
                        nXT  <= $signed(iXIn) << 14;
                        nX2  <= nX1;
                        nX1  <= iXIn;
                        nPipe <= {{NL-1{1'b0}}, 1'b1};
                    end
                    // 拍1~拍4：逐项求和
                    if (nPipe[0]) begin
                        nS1  <= nT1 + nT2;
                        nPipe <= (nPipe[NL-2]) ? {NL{1'b0}} : (nPipe << 1);
                    end
                    if (nPipe[1]) begin
                        nS2  <= nS1 + nT3;
                        nPipe <= (nPipe[NL-2]) ? {NL{1'b0}} : (nPipe << 1);
                    end
                    if (nPipe[2]) begin
                        nS3  <= nS2 + nT4;
                        nPipe <= (nPipe[NL-2]) ? {NL{1'b0}} : (nPipe << 1);
                    end
                    if (nPipe[3]) begin
                        nS4  <= nS3 + nXT;
                        nPipe <= (nPipe[NL-2]) ? {NL{1'b0}} : (nPipe << 1);
                    end
                    // 拍5（末拍）：Q14 舍入饱和 + 状态/输出更新
                    if (nPipe[NL-2]) begin
                        nY2   <= nY1;
                        nY1   <= wYn;
                        nY    <= wYn;
                        nPipe <= {NL{1'b0}};
                    end
                end
            end
        end
    endgenerate
endmodule
