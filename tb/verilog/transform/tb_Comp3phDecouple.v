//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   tb_Comp3phDecouple.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
/*Description       :   三相正/负序 dq 解耦单元 TB（Fs=20kHz, fc=50Hz, A=1000）。
                        纯正序三相 -> oDp≈幅值(1000)、oQp≈0，oDn/oQn≈0；
                        纯负序三相 -> oDn≈幅值、oDp/oQp≈0。手段: 尾窗均值/最大偏差。
                        【流水化适配】DUT 现为多拍流水（LATENCY=7）：iEn 单拍脉冲逐样本喂入，
                        dq 输出在 oValid 拉高时采样；喂入样本序列与改造前完全一致。
                        另含内联定点参考模型（原单周期 CompAbc2pnz + Park(+θ/-θ) 逐位复算）：
                        对每个被接受样本将 DUT 的 oDp/oQp/oDn/oQn 与参考逐位比对，必须全等。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.2  (流水化适配 + 逐样本参考比对)
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
`timescale 1ns/1ps

module TbComp3phDecouple;
    localparam integer TOT = 20000;      // 每轮喂入样本数（与原 TB 一致）
    localparam integer SKP = 8000;       // 窗口统计起始（跳过暂态）

    // ---- 参考模型定点常量（与原模块 fn_sat/系数完全一致）----
    localparam signed [63:0] CA    = -64'sd32257;
    localparam signed [63:0] CNA   =  64'sd32257;
    localparam signed [63:0] CTHIRD = 64'sd10923;
    localparam signed [63:0] CIR3   = 64'sd18919;

    reg clk = 0, iSysRst = 1, en = 0;
    reg signed [15:0] ia, ib, ic;
    reg [15:0] th;
    wire signed [15:0] dp, qp, dn, qn;
    wire vld;
    integer fail = 0;
    always #5 clk = ~clk;

    Comp3phDecouple U_Dut (
        .iSysClk(clk), .iSysRst(iSysRst), .iEn(en),
        .iA(ia), .iB(ib), .iC(ic), .iTheta(th),
        .oDp(dp), .oQp(qp), .oDn(dn), .oQn(qn),
        .oValid(vld)
    );

    reg [15:0] angA = 0, angB = 0, angC = 0;
    wire signed [15:0] csA, csB, csC;
    wire signed [15:0] dnA, dnB, dnC;
    AlgoTrig U_A (.iAngle(angA), .oSin(dnA), .oCos(csA));
    AlgoTrig U_B (.iAngle(angB), .oSin(dnB), .oCos(csB));
    AlgoTrig U_C (.iAngle(angC), .oSin(dnC), .oCos(csC));
    // Park 参考：正序用 +θ(=angA) 的 cos/sin（csA/dnA）；负序用 -θ 的 cos/sin
    wire [15:0] angN = -angA;
    wire signed [15:0] csN, snN;
    AlgoTrig U_N (.iAngle(angN), .oSin(snN), .oCos(csN));

    // ---- 参考模型状态 ----
    reg signed [15:0] rfAX = 0, rfAY = 0, rfBX = 0, rfBY = 0, rfZX = 0, rfZY = 0;

    reg signed [63:0] sDp, sQp, sDn, sQn;
    integer mdDp, mdQp, mdDn, mdX;
    integer mism;
    integer i;
    reg [31:0] ph;
    reg signed [15:0] xa, xb, xc;
    integer tmp;

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
            sxt16 = v;
        end
    endfunction

    task reinit;
        begin
            en = 0; ia = 0; ib = 0; ic = 0; th = 0;
            angA = 0; angB = 0; angC = 0;
            ph = 0;
            rfAX = 0; rfAY = 0; rfBX = 0; rfBY = 0; rfZX = 0; rfZY = 0;
            iSysRst = 1; @(negedge clk); iSysRst = 0;
        end
    endtask

    task feed;
        input integer seq;
        begin
            angA = ph[31:16];
            if (seq == 0) begin angB = angA - 21845; angC = angA + 21845; end
            else          begin angB = angA + 21845; angC = angA - 21845; end
            th = angA;                 // Park 角 = A 相电角度
            #1;   // 等 AlgoTrig 组合沉降后再取 cos
            xa = ($signed(csA) * 1000) >>> 15;
            xb = ($signed(csB) * 1000) >>> 15;
            xc = ($signed(csC) * 1000) >>> 15;
            ia = xa; ib = xb; ic = xc;
            @(negedge clk); en = 1;
            @(posedge clk);           // 接受
            @(negedge clk); en = 0;
            while (!vld) @(posedge clk);
            #1;
        end
    endtask

    reg signed [63:0] tt;
    reg signed [15:0] clAl, clBe, clZ, yA, yB, yZ;
    reg signed [15:0] aP, bP, aN, bN;
    reg signed [15:0] eDp, eQp, eDn, eQn;

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
            // 顺序分量（只用 αβ 两路供 Park）
            tt = sxt16(clAl) - sxt16(yB); aP = sat16(tt, 1);
            tt = sxt16(clBe) + sxt16(yA); bP = sat16(tt, 1);
            tt = sxt16(clAl) + sxt16(yB); aN = sat16(tt, 1);
            tt = sxt16(clBe) - sxt16(yA); bN = sat16(tt, 1);
            // Park(+θ): d=αcos+βsin; q=-αsin+βcos
            tt = sxt16(aP)*sxt16(csA) + sxt16(bP)*sxt16(dnA); eDp = sat16(tt, 15);
            tt = -sxt16(aP)*sxt16(dnA) + sxt16(bP)*sxt16(csA); eQp = sat16(tt, 15);
            // Park(-θ): 用 -θ 的 cos/sin（U_N，与 DUT 内部 AlgoTrig 同 LUT）
            tt = sxt16(aN)*sxt16(csN) + sxt16(bN)*sxt16(snN); eDn = sat16(tt, 15);
            tt = -sxt16(aN)*sxt16(snN) + sxt16(bN)*sxt16(csN); eQn = sat16(tt, 15);

            // 逐位比对
            if (dp !== eDp || qp !== eQp || dn !== eDn || qn !== eQn) mism = mism + 1;

            // 原窗口统计
            if (sIdx >= SKP) begin
                sDp = sDp + dp;  sQp = sQp + qp;  sDn = sDn + dn;  sQn = sQn + qn;
                tmp = dp - 1000; if (tmp < 0) tmp = -tmp; if (tmp > mdDp) mdDp = tmp;
                tmp = dn - 1000; if (tmp < 0) tmp = -tmp; if (tmp > mdDn) mdDn = tmp;
                tmp = qp;        if (tmp < 0) tmp = -tmp; if (tmp > mdQp) mdQp = tmp;
                tmp = dn;        if (tmp < 0) tmp = -tmp; if (tmp > mdX)  mdX = tmp;
            end

            rfAX = clAl; rfAY = yA; rfBX = clBe; rfBY = yB; rfZX = clZ; rfZY = yZ;
        end
    endtask

    task run(input integer seq);
        integer s;
        begin
            sDp = 0; sQp = 0; sDn = 0; sQn = 0;
            mdDp = 0; mdQp = 0; mdDn = 0; mdX = 0;
            mism = 0;
            reinit;
            for (s = 0; s < TOT; s = s + 1) begin
                feed(seq);
                cmp_sample(s);
                ph = ph + 32'd10737418;   // 0.0025*2^32: 50Hz/20kHz
            end
            sDp = sDp/(TOT-SKP); sQp = sQp/(TOT-SKP); sDn = sDn/(TOT-SKP); sQn = sQn/(TOT-SKP);
        end
    endtask

    initial begin
        // 正序 -> dp≈1000, qp≈0
        run(0);
        if (mism != 0) begin $display("FAIL 正序 逐样本参考比对不一致=%0d", mism); fail = fail + 1; end
        else $display("  ok 正序 逐样本参考比对 bit 精确一致");
        $display("pos: dp均值=%0d qp均值=%0d dn均值=%0d mdX=%0d", sDp, sQp, sDn, mdX);
        if (sDp > 900 && sDp < 1100 && mdDp < 300) $display("  ok 正序 dp≈幅值");
        else begin $display("FAIL 正序 dp=%0d", sDp); fail = fail + 1; end
        if (sQp > -80 && sQp < 80 && mdQp < 300) $display("  ok 正序 qp≈0");
        else begin $display("FAIL 正序 qp=%0d max=%0d", sQp, mdQp); fail = fail + 1; end
        if (sDn > -80 && sDn < 80 && mdX < 300) $display("  ok 正序 dn≈0");
        else begin $display("FAIL 正序 dn=%0d max=%0d", sDn, mdX); fail = fail + 1; end

        // 负序 -> dn≈1000, dp≈0
        run(1);
        if (mism != 0) begin $display("FAIL 负序 逐样本参考比对不一致=%0d", mism); fail = fail + 1; end
        else $display("  ok 负序 逐样本参考比对 bit 精确一致");
        $display("neg: dn均值=%0d (dev=%0d) dp均值=%0d qn均值=%0d", sDn, mdDn, sDp, sQn);
        if (sDn > 900 && sDn < 1100 && mdDn < 300) $display("  ok 负序 dn≈幅值");
        else begin $display("FAIL 负序 dn=%0d maxdev=%0d", sDn, mdDn); fail = fail + 1; end
        if (sDp > -80 && sDp < 80) $display("  ok 负序 dp≈0");
        else begin $display("FAIL 负序 dp=%0d", sDp); fail = fail + 1; end
        if (sQn > -300 && sQn < 300) $display("  ok 负序 qn≈0");
        else begin $display("FAIL 负序 qn=%0d", sQn); fail = fail + 1; end

        if (fail == 0) $display("PASS Comp3phDecouple");
        else $display("FAIL Comp3phDecouple: %0d", fail);
        $finish;
    end
endmodule
