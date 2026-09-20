//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   CompFft.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.06
/*Description       :   基-2 时域抽取(DIT)定点复数 FFT（原地 in-place）。
  接口   : iStart 脉冲启动 -> 装载+log2(N)级蝶形 -> oDone 拉一拍,oRe/oIm 给出频谱。
          输入为 N 点复数: 第 n 点实/虚部在 iRe/iIm 总线 [n*W +: W](W=16,输入 Q15)。
          输出亦按同序打包 oRe/oIm (Q15); 采样点沿用 bit[N]: 即第 n 点 oRe[n*W +: W]。
  算法   : Cooley-Tukey DIT,输入按位序反转装载 mem[bitrev(n)]=x[n],输出自然序。
          第 s 级块长 len=2^(s+1),半长 half=len/2,蝶形对 (a,a+half);
          旋转因子 W=exp(-j2pi*m/LUTN) 存 Q15 表 (LUTN=64 常数,cos/sin 函数查表,
          本表按 LUTN=64 预生成; m = j*(LUTN/len), j 为块内蝶形号)。
  缩放   : 每级蝶形后舍入算术右移 1 位(防溢出),共 log2(N) 级 -> 总归一化 1/N。
          故输出 = 精确 DFT / N: A 幅实正弦 X[k0]=X[N-k0]=A/2,直流 X[0]=A(Q15)。
  定点   : 输入/旋转因子/输出均 Q15(满幅 32767≈1); 复数乘 = 4 实乘+2 加减,
          乘积累加 Q30 舍入右移 15 回 Q15,每级蝶形和/差右移 1 位,无除法无浮点。
  修订   : 2026-09-06 v0.1
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.1
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------

module CompFft #(
    parameter integer W    = 16,   // 数据/系数位宽 (Q15, 须为 16)
    parameter integer N    = 64,   // FFT 点数 (2 的幂, 2<=N<=LUTN)
    parameter integer LUTN = 64    // 旋转因子 LUT 基点数 (2 的幂, 常数表按 64 生成)
) (
    input  wire                    iSysClk,     // 系统时钟
    input  wire                    iSysRst,     // 复位(高有效)
    input  wire                    iStart,      // 启动脉冲(一拍)
    input  wire signed [N*W-1:0]   iRe,         // 输入实部, 第 n 点在 [n*W +: W]
    input  wire signed [N*W-1:0]   iIm,         // 输入虚部
    output wire signed [N*W-1:0]   oRe,         // 输出实部(Q15, DFT/N)
    output wire signed [N*W-1:0]   oIm,         // 输出虚部
    output reg                     oDone        // 完成脉冲(一拍, 此后 oRe/oIm 有效)
);
    localparam integer LN    = $clog2(N);       // log2(N) = 蝶形级数
    localparam integer LLN   = $clog2(LUTN);    // log2(LUTN)
    localparam integer ADW   = LLN;             // 内部计数/地址位宽(足够覆盖表索引与 N 地址)
    localparam integer NPW   = N*W;             // 打包总线位宽

    localparam [1:0] ST_IDLE = 2'd0;
    localparam [1:0] ST_LOAD = 2'd1;
    localparam [1:0] ST_FFT  = 2'd2;
    localparam [1:0] ST_DONE = 2'd3;

    // 数据缓冲 (复数 x2)
    reg signed [W-1:0] rReMem[0:N-1];
    reg signed [W-1:0] rImMem[0:N-1];

    // 状态/流程寄存器
    reg [1:0]      rState;                        // FSM 状态
    reg [ADW-1:0]  rLoad;                         // 装载样本号 0..N-1
    reg [ADW-1:0]  rStage;                        // 当前级 0..LN-1
    reg [ADW-1:0]  rHalf;                         // 半块长 half = len/2
    reg [ADW-1:0]  rLen;                          // 块长 len = 2^(s+1)
    reg [ADW-1:0]  rBlocks;                       // 本级块数 N/len
    reg [ADW-1:0]  rStride;                       // 旋转因子表步进 LUTN/len
    reg [ADW-1:0]  rB;                            // 块号
    reg [ADW-1:0]  rJ;                            // 块内蝶形号 j
    reg [ADW-1:0]  rBase;                         // 当前块首址 rB*len
    reg [ADW-1:0]  rM;                            // 旋转因子表索引 = j*LUTN/len

    // ---- 蝶形计算临时量(阻塞, 仅 FSM 内使用) ----
    reg [ADW-1:0]  tI0, tI1, tBr;
    reg signed [W-1:0] aR, aI, bR, bI, tR, tI, cC, cS;
    reg signed [2*W-1:0] p1, p2;
    reg signed [2*W:0]   qA, qS;
    reg signed [W:0]  s0r, s0i, s1r, s1i;         // 蝶形和/差(含舍入)
    reg signed [W-1:0] y0r, y0i, y1r, y1i;        // 缩放后写回值

    // ---- 位序反转: 对 LN 位索引做比特颠倒 ----
    function [LN-1:0] f_brev;
        input [LN-1:0] v;
        integer k;
        begin
            f_brev = {LN{1'b0}};
            for (k = 0; k < LN; k = k + 1)
                f_brev[LN-1-k] = v[k];
        end
    endfunction

    // ---- 旋转因子 Q15 查找表: W_MAXN^m = exp(-j2pi*m/LUTN), LUTN=64 ----
    //   返回 cos 分量 (c = cos(2pi m / LUTN), Q15)
    function signed [W-1:0] f_tw_c;
        input [ADW-1:0] m;
        begin
            case (m)
                 0 : f_tw_c = 16'sd32767;
                 1 : f_tw_c = 16'sd32609;
                 2 : f_tw_c = 16'sd32137;
                 3 : f_tw_c = 16'sd31356;
                 4 : f_tw_c = 16'sd30273;
                 5 : f_tw_c = 16'sd28898;
                 6 : f_tw_c = 16'sd27245;
                 7 : f_tw_c = 16'sd25329;
                 8 : f_tw_c = 16'sd23170;
                 9 : f_tw_c = 16'sd20787;
                10 : f_tw_c = 16'sd18204;
                11 : f_tw_c = 16'sd15446;
                12 : f_tw_c = 16'sd12539;
                13 : f_tw_c = 16'sd9512;
                14 : f_tw_c = 16'sd6393;
                15 : f_tw_c = 16'sd3212;
                16 : f_tw_c = 16'sd0;
                17 : f_tw_c = -16'sd3212;
                18 : f_tw_c = -16'sd6393;
                19 : f_tw_c = -16'sd9512;
                20 : f_tw_c = -16'sd12539;
                21 : f_tw_c = -16'sd15446;
                22 : f_tw_c = -16'sd18204;
                23 : f_tw_c = -16'sd20787;
                24 : f_tw_c = -16'sd23170;
                25 : f_tw_c = -16'sd25329;
                26 : f_tw_c = -16'sd27245;
                27 : f_tw_c = -16'sd28898;
                28 : f_tw_c = -16'sd30273;
                29 : f_tw_c = -16'sd31356;
                30 : f_tw_c = -16'sd32137;
                31 : f_tw_c = -16'sd32609;
                default: f_tw_c = 16'sd0;
            endcase
        end
    endfunction

    //   返回 sin 分量 (s = sin(2pi m / LUTN), Q15)
    function signed [W-1:0] f_tw_s;
        input [ADW-1:0] m;
        begin
            case (m)
                 0 : f_tw_s = 16'sd0;
                 1 : f_tw_s = 16'sd3212;
                 2 : f_tw_s = 16'sd6393;
                 3 : f_tw_s = 16'sd9512;
                 4 : f_tw_s = 16'sd12539;
                 5 : f_tw_s = 16'sd15446;
                 6 : f_tw_s = 16'sd18204;
                 7 : f_tw_s = 16'sd20787;
                 8 : f_tw_s = 16'sd23170;
                 9 : f_tw_s = 16'sd25329;
                10 : f_tw_s = 16'sd27245;
                11 : f_tw_s = 16'sd28898;
                12 : f_tw_s = 16'sd30273;
                13 : f_tw_s = 16'sd31356;
                14 : f_tw_s = 16'sd32137;
                15 : f_tw_s = 16'sd32609;
                16 : f_tw_s = 16'sd32767;
                17 : f_tw_s = 16'sd32609;
                18 : f_tw_s = 16'sd32137;
                19 : f_tw_s = 16'sd31356;
                20 : f_tw_s = 16'sd30273;
                21 : f_tw_s = 16'sd28898;
                22 : f_tw_s = 16'sd27245;
                23 : f_tw_s = 16'sd25329;
                24 : f_tw_s = 16'sd23170;
                25 : f_tw_s = 16'sd20787;
                26 : f_tw_s = 16'sd18204;
                27 : f_tw_s = 16'sd15446;
                28 : f_tw_s = 16'sd12539;
                29 : f_tw_s = 16'sd9512;
                30 : f_tw_s = 16'sd6393;
                31 : f_tw_s = 16'sd3212;
                default: f_tw_s = 16'sd0;
            endcase
        end
    endfunction

    // ---- 输出: 组合打包 mem 内容 (自然序) ----
    genvar g;
    generate
        for (g = 0; g < N; g = g + 1) begin : GF_OUT
            assign oRe[g*W +: W] = rReMem[g];
            assign oIm[g*W +: W] = rImMem[g];
        end
    endgenerate

    // ---- 主状态机 ----
    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            rState  <= ST_IDLE;
            oDone   <= 1'b0;
            rLoad   <= {ADW{1'b0}};
            rStage  <= {ADW{1'b0}};
            rHalf   <= {ADW{1'b0}};
            rLen    <= {ADW{1'b0}};
            rBlocks <= {ADW{1'b0}};
            rStride <= {ADW{1'b0}};
            rB      <= {ADW{1'b0}};
            rJ      <= {ADW{1'b0}};
            rBase   <= {ADW{1'b0}};
            rM      <= {ADW{1'b0}};
        end else begin
            oDone <= 1'b0;
            case (rState)
                ST_IDLE: begin
                    if (iStart) begin
                        rState <= ST_LOAD;
                        rLoad  <= {ADW{1'b0}};  // 每次运行装载计数从 0 开始
                    end
                end
                ST_LOAD: begin
                    tBr = f_brev(rLoad);         // 输入位序反转存储
                    rReMem[tBr] <= iRe[rLoad*W +: W];
                    rImMem[tBr] <= iIm[rLoad*W +: W];
                    if (rLoad == (N - 1)) begin
                        rState  <= ST_FFT;       // 装载完毕进入蝶形
                        rStage  <= {ADW{1'b0}};
                        rHalf   <= 1;            // stage0: half=1
                        rLen    <= 2;            // stage0: len=2
                        rBlocks <= N/2;          // 本级块数
                        rStride <= LUTN/2;       // 步进 LUTN/len
                        rB      <= {ADW{1'b0}};
                        rJ      <= {ADW{1'b0}};
                        rBase   <= {ADW{1'b0}};
                        rM      <= {ADW{1'b0}};
                    end else begin
                        rLoad <= rLoad + 1'b1;
                    end
                end
                ST_FFT: begin
                    // ---- 蝶形: 地址 / 读数据 / 旋转因子 ----
                    tI0 = rBase + rJ;
                    tI1 = tI0 + rHalf;
                    aR  = rReMem[tI0];  aI = rImMem[tI0];
                    bR  = rReMem[tI1];  bI = rImMem[tI1];
                    cC  = f_tw_c(rM);
                    cS  = f_tw_s(rM);
                    // t = b*W, W = cC - j*cS
                    //   t_re = b_re*cC + b_im*cS ; t_im = b_im*cC - b_re*cS
                    p1 = bR * cC;
                    p2 = bI * cS;
                    qA = p1 + p2;                                  // Q30
                    qS = (qA + 33'sd16384) >>> (W - 1);            // 舍入右移 15 回 Q15
                    if (qS >  32'sd32767) qS = 32'sd32767;
                    if (qS < -32'sd32768) qS = -32'sd32768;
                    tR = qS[W-1:0];
                    p1 = bI * cC;
                    p2 = bR * cS;
                    qA = p1 - p2;                                  // Q30
                    qS = (qA + 33'sd16384) >>> (W - 1);
                    if (qS >  32'sd32767) qS = 32'sd32767;
                    if (qS < -32'sd32768) qS = -32'sd32768;
                    tI = qS[W-1:0];
                    // ---- 蝶形和/差并右移 1 位(缩放防溢出), 舍入加 1 ----
                    s0r = aR + tR + 17'sd1;  y0r = s0r[W:1];       // y0=(u+t)>>1
                    s0i = aI + tI + 17'sd1;  y0i = s0i[W:1];
                    s1r = aR - tR + 17'sd1;  y1r = s1r[W:1];       // y1=(u-t)>>1
                    s1i = aI - tI + 17'sd1;  y1i = s1i[W:1];
                    // ---- 写回(原地) ----
                    rReMem[tI0] <= y0r;  rImMem[tI0] <= y0i;
                    rReMem[tI1] <= y1r;  rImMem[tI1] <= y1i;
                    // ---- 计数器推进 ----
                    if (rJ == (rHalf - 1)) begin                   // 块内最后一个蝶形
                        if (rB == (rBlocks - 1)) begin             // 本级最后一块
                            if (rStage == (LN - 1))
                                rState <= ST_DONE;
                            else begin
                                rStage  <= rStage + 1'b1;
                                rHalf   <= rHalf  << 1;            // len 翻倍
                                rLen    <= rLen   << 1;
                                rBlocks <= rBlocks >> 1;
                                rStride <= rStride >> 1;
                                rB      <= {ADW{1'b0}};
                                rJ      <= {ADW{1'b0}};
                                rBase   <= {ADW{1'b0}};
                                rM      <= {ADW{1'b0}};
                            end
                        end else begin
                            rB    <= rB + 1'b1;
                            rJ    <= {ADW{1'b0}};
                            rBase <= rBase + rLen;                 // 下一块首址
                            rM    <= {ADW{1'b0}};
                        end
                    end else begin
                        rJ <= rJ + 1'b1;
                        rM <= rM + rStride;                        // 下一蝶形旋转因子索引
                    end
                end
                ST_DONE: begin
                    oDone  <= 1'b1;
                    rState <= ST_IDLE;
                end
                default: rState <= ST_IDLE;
            endcase
        end
    end
endmodule
