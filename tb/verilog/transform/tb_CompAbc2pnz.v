//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   tb_CompAbc2pnz.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
/*Description       :   CompAbc2pnz ABC->正/负序分解单元 TB（Fs=20kHz, fc=50Hz, 幅1000）。
                        纯正序三相(b 滞后 a 120°) -> oAlphaP≈1000、oAlphaN≈0、重建 oAp≈输入 a；
                        纯负序三相(b 超前 a 120°) -> 反向成立。手段: 尾窗 sum² 求幅度比。
                        【流水化适配】DUT 现为多拍流水（LATENCY=6）：iEn 单拍脉冲逐样本喂入，
                        样本在 oValid 拉高时采样；喂入样本序列与改造前完全一致。
                        另含内联定点参考模型（原单周期公式逐位复算）：
                        对每个被接受样本将 DUT 全部输出与参考逐位比对，必须全等才算通过。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.2  (流水化适配 + 逐样本参考比对)
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
`timescale 1ns/1ps

module TbCompAbc2pnz;
    localparam integer TOT = 20000;      // 每轮喂入样本数（与原 TB 一致）
    localparam integer SKP = 8000;       // 窗口统计起始（跳过暂态）

    // ---- DUT 参考模型定点常量（与原模块 fn_sat/系数完全一致）----
    localparam signed [63:0] CA    = -64'sd32257;   // -0.9844 Q15
    localparam signed [63:0] CNA   =  64'sd32257;   // +0.9844 Q15
    localparam signed [63:0] CTHIRD = 64'sd10923;   // 1/3  Q15 (等功率 Clarke)
    localparam signed [63:0] CIR3   = 64'sd18919;   // 1/sqrt3 Q15
    localparam signed [63:0] CHALF  = 64'sd16384;   // 0.5  Q15 (InvClark)
    localparam signed [63:0] CSQ32  = 64'sd28378;   // sqrt3/2 Q15 (InvClark)

    reg clk = 0, iSysRst = 1, en = 0;
    reg signed [15:0] ia, ib, ic;
    wire signed [15:0] ap, bp, an, bn, rp, rn;
    wire signed [15:0] oap, obp, ocp, oan, obn, ocn;
    wire vld;
    integer fail = 0;
    always #5 clk = ~clk;

    CompAbc2pnz U_Dut (
        .iSysClk(clk), .iSysRst(iSysRst), .iEn(en),
        .iA(ia), .iB(ib), .iC(ic),
        .oAlphaP(ap), .oBetaP(bp), .oAlphaN(an), .oBetaN(bn),
        .oRZp(rp), .oRZn(rn),
        .oAp(oap), .oBp(obp), .oCp(ocp),
        .oAn(oan), .oBn(obn), .oCn(ocn),
        .oValid(vld)
    );

    // 三相对相参考（A 相 cos、B=cos(θ-120)、C=cos(θ+120)）
    reg [15:0] angA = 0, angB = 0, angC = 0;
    wire signed [15:0] csA, csB, csC;
    wire signed [15:0] dnA, dnB, dnC;
    AlgoTrig U_A (.iAngle(angA), .oSin(dnA), .oCos(csA));
    AlgoTrig U_B (.iAngle(angB), .oSin(dnB), .oCos(csB));
    AlgoTrig U_C (.iAngle(angC), .oSin(dnC), .oCos(csC));

    // ---- 参考模型状态（每接受样本一拍更新）----
    reg signed [15:0] rfAX = 0, rfAY = 0, rfBX = 0, rfBY = 0, rfZX = 0, rfZY = 0;

    // 窗口统计
    reg signed [63:0] sAp2, sAn2, sA2, sDiff2;
    integer mism;                 // 逐样本 bit 精确比对失败数
    integer i;
    reg [31:0] ph;
    reg signed [15:0] xa, xb, xc;

    // ---- 参考函数：S16 舍入右移 + 饱和（复刻 fn_sat）----
    function automatic signed [15:0] sat16;
        input signed [63:0] v;
        input integer fb;
        reg signed [63:0] ra;
        integer half;
        begin
            half = (fb > 0) ? (1 << (fb - 1)) : 0;
            ra = v + half;
            ra = ra >>> fb;
            if (ra > 32767)   ra = 32767;
            if (ra < -32768)  ra = -32768;
            sat16 = ra[15:0];
        end
    endfunction

    function automatic signed [63:0] sxt16;
        input signed [15:0] v;
        begin
            sxt16 = v;     // 16bit 符号扩展到 64
        end
    endfunction

    task reinit;
        begin
            en = 0; ia = 0; ib = 0; ic = 0;
            angA = 0; angB = 0; angC = 0;
            ph = 0;
            rfAX = 0; rfAY = 0; rfBX = 0; rfBY = 0; rfZX = 0; rfZY = 0;
            iSysRst = 1; @(negedge clk); iSysRst = 0;
        end
    endtask

    // 喂一个采样（给定三相角度所量化出的 abc）并等 DUT 完成
    task feed;
        input integer seq;
        begin
            angA = ph[31:16];
            if (seq == 0) begin angB = angA - 21845; angC = angA + 21845; end
            else          begin angB = angA + 21845; angC = angA - 21845; end
            #1;   // 等 AlgoTrig 组合沉降后再取 cos
            xa = ($signed(csA) * 1000) >>> 15;
            xb = ($signed(csB) * 1000) >>> 15;
            xc = ($signed(csC) * 1000) >>> 15;
            ia = xa; ib = xb; ic = xc;
            // iEn 单拍脉冲：负沿置 1，跨一个正沿（DUT 接受）后在负沿清 0
            @(negedge clk); en = 1;
            @(posedge clk);           // 接受
            @(negedge clk); en = 0;
            while (!vld) @(posedge clk);   // 等完成（防死循环：vld 必定在 LATENCY 拍内出现）
            #1;
        end
    endtask

    // 对当前被接受样本：参考复算 + 逐位比对 + 窗口统计 + 参考状态更新
    reg signed [63:0] tt;
    reg signed [15:0] eAl, eBe, eZ, yA, yB, yZ;
    reg signed [15:0] eAlP, eBeP, eZp, eAlN, eBeN, eZn;
    reg signed [15:0] eAp, eBp, eCp;
    reg signed [15:0] clAl, clBe, clZ;

    task cmp_sample(input integer sIdx);
        begin
            // 等功率 Clarke
            tt = (sxt16(ia) + sxt16(ia) - sxt16(ib) - sxt16(ic)) * CTHIRD; clAl = sat16(tt, 15);
            tt = (sxt16(ib) - sxt16(ic)) * CIR3;                        clBe = sat16(tt, 15);
            tt = (sxt16(ia) + sxt16(ib) + sxt16(ic)) * CTHIRD;          clZ  = sat16(tt, 15);
            // 三路 AFP
            tt = CA*sxt16(clAl) + (sxt16(rfAX) <<< 15) + CNA*sxt16(rfAY); yA = sat16(tt, 15);
            tt = CA*sxt16(clBe) + (sxt16(rfBX) <<< 15) + CNA*sxt16(rfBY); yB = sat16(tt, 15);
            tt = CA*sxt16(clZ)  + (sxt16(rfZX) <<< 15) + CNA*sxt16(rfZY); yZ = sat16(tt, 15);
            // 顺序分量合成（0.5 缩放 fb=1）
            tt = sxt16(clAl) - sxt16(yB); eAlP = sat16(tt, 1);
            tt = sxt16(clBe) + sxt16(yA); eBeP = sat16(tt, 1);
            tt = sxt16(clZ)  + sxt16(yZ); eZp  = sat16(tt, 1);
            tt = sxt16(clAl) + sxt16(yB); eAlN = sat16(tt, 1);
            tt = sxt16(clBe) - sxt16(yA); eBeN = sat16(tt, 1);
            tt = sxt16(clZ)  - sxt16(yZ); eZn  = sat16(tt, 1);
            // InvClark 正序重建
            tt = sxt16(eAlP) + sxt16(eZp);                             eAp = sat16(tt, 0);
            tt = (sxt16(eZp) <<< 15) - sxt16(eAlP)*CHALF + sxt16(eBeP)*CSQ32; eBp = sat16(tt, 15);
            tt = (sxt16(eZp) <<< 15) - sxt16(eAlP)*CHALF - sxt16(eBeP)*CSQ32; eCp = sat16(tt, 15);

            // 逐位比对
            if (ap !== eAlP || bp !== eBeP || an !== eAlN || bn !== eBeN ||
                rp !== eZp  || rn !== eZn) mism = mism + 1;
            if (oap !== eAp || obp !== eBp || ocp !== eCp) mism = mism + 1;

            // 原窗口统计（采样时机已对齐：本接受样本输出 与 其自身输入）
            if (sIdx >= SKP) begin
                sAp2 = sAp2 + $signed(ap)*$signed(ap);
                sAn2 = sAn2 + $signed(an)*$signed(an);
                sA2  = sA2  + $signed(ia)*$signed(ia);
                sDiff2 = sDiff2 + ($signed(oap) - $signed(ia)) * ($signed(oap) - $signed(ia));
            end

            // 参考状态更新
            rfAX = clAl; rfAY = yA; rfBX = clBe; rfBY = yB; rfZX = clZ; rfZY = yZ;
        end
    endtask

    task run(input integer seq);
        integer s;
        begin
            sAp2 = 0; sAn2 = 0; sA2 = 0; sDiff2 = 0; mism = 0;
            reinit;
            for (s = 0; s < TOT; s = s + 1) begin
                feed(seq);
                cmp_sample(s);
                ph = ph + 32'd10737418;   // 0.0025*2^32: 50Hz/20kHz
            end
        end
    endtask

    initial begin
        // 正序测试
        run(0);
        if (mism != 0) begin $display("FAIL 正序 逐样本参考比对不一致=%0d", mism); fail = fail + 1; end
        else $display("  ok 正序 逐样本参考比对 bit 精确一致");
        if (sA2 > 0) begin
            if ((sAp2*100) > (sA2*93) && (sAp2*100) < (sA2*107))
                $display("  ok 正序 alpha/beta 幅²/%0d%%", (sAp2*100)/sA2);
            else begin $display("FAIL 正序幅度 sAp2/sA2=%0d%%", (sAp2*100)/sA2); fail = fail + 1; end
            if ((sAn2*100) < (sA2*2))
                $display("  ok 正序 负序≈0 (%0d%%)", (sAn2*100)/sA2);
            else begin $display("FAIL 正序 负序残留 sAn2/sA2=%0d%%", (sAn2*100)/sA2); fail = fail + 1; end
            if ((sDiff2*100) < (sA2*2))
                $display("  ok 正序 重建oAp≈a");
            else begin $display("FAIL 正序重建偏差 sDiff2/sA2=%0d%%", (sDiff2*100)/sA2); fail = fail + 1; end
        end else begin $display("FAIL 正序 无信号"); fail = fail + 1; end

        // 负序测试（重新统计）
        run(1);
        if (mism != 0) begin $display("FAIL 负序 逐样本参考比对不一致=%0d", mism); fail = fail + 1; end
        else $display("  ok 负序 逐样本参考比对 bit 精确一致");
        if (sA2 > 0) begin
            if ((sAn2*100) > (sA2*93) && (sAn2*100) < (sA2*107))
                $display("  ok 负序 alpha/beta 幅²/%0d%%", (sAn2*100)/sA2);
            else begin $display("FAIL 负序幅度 sAn2/sA2=%0d%%", (sAn2*100)/sA2); fail = fail + 1; end
            if ((sAp2*100) < (sA2*2))
                $display("  ok 负序 正序≈0 (%0d%%)", (sAp2*100)/sA2);
            else begin $display("FAIL 负序 正序残留 sAp2/sA2=%0d%%", (sAp2*100)/sA2); fail = fail + 1; end
        end else begin $display("FAIL 负序 无信号"); fail = fail + 1; end

        if (fail == 0) $display("PASS CompAbc2pnz");
        else $display("FAIL CompAbc2pnz: %0d", fail);
        $finish;
    end
endmodule
