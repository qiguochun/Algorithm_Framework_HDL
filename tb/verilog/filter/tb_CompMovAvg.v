//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   tb_CompMovAvg.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.06
/*Description       :   CompMovAvg N=16 滑动平均单元 TB。
                        校验: (1) 直流 7777：窗填满后输出精确等于输入(平均增益精确=1)；
                              (2) 平滑噪声：基值 7777 上叠加均匀噪声(幅 2000)，
                                  输出均值≈7777±1%，输出 std 明显小于输入 std(<0.6x)；
                              (3) 阶跃 6000 后输出线性爬升(每拍 +S/N)，单调不跳变，
                                  第 N 拍精确到 6000。
                        手段: 精确比较 / 均方根 std 判平滑 / 爬升单调判斜坡。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.1
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
`timescale 1ns/1ps

module TbCompMovAvg;
    localparam integer W = 16;
    localparam integer N = 16;
    localparam integer DCV = 7777;

    reg clk = 0;
    reg iSysRst = 1;
    reg en = 1;
    reg signed [W-1:0] x = 0;
    wire signed [W-1:0] y;

    CompMovAvg #(.W(W), .N(N)) U_Dut (
        .iSysClk(clk), .iSysRst(iSysRst), .iEn(en), .iXIn(x), .oYOut(y)
    );

    always #5 clk = ~clk;

    integer fail = 0;
    integer i;
    integer lfsr;
    reg signed [W-1:0] xn;
    real sxn, sx2n, syn, sy2n, meanx, meano, stdx, stdo, ratio;
    real base, amp;
    integer ramp [0:N];   // 阶跃后 N 拍输出
    integer monotone;

    task reinit;
        begin
            en = 1; x = 0;
            iSysRst = 1; repeat (2) @(posedge clk); #1;
            iSysRst = 0; @(posedge clk); #1;
        end
    endtask

    // 确定性噪声（16-bit LFSR -> 均匀 [-A, A]，写入 xn）
    task noise_gen(input integer A);
        begin
            lfsr = {lfsr[14:0], lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10]};
            xn   = $rtoi((lfsr % (2 * A + 1)) - A);
        end
    endtask

    initial begin
        // ---- 测试1: 直流 7777，窗填满后输出精确等于输入 ----
        reinit;
        for (i = 0; i < 40; i = i + 1) begin
            x = DCV; @(posedge clk); #1;
        end
        monotone = 1;
        for (i = 0; i < 20; i = i + 1) begin
            x = DCV; @(posedge clk); #1;
            if (y !== DCV) monotone = 0;
        end
        if (monotone == 1)
            $display("  ok 直流精确直通  填满后输出恒=%0d", DCV);
        else begin
            $display("FAIL 直流直通");
            fail = fail + 1;
        end

        // ---- 测试2: 平滑噪声（均值≈基值，std 降低 ~sqrt(N) 倍） ----
        reinit;
        lfsr = 16'hACE1; base = $itor(DCV); amp = $itor(2000);
        // 先填窗（基值上叠加噪声，输出统计态）
        for (i = 0; i < 64; i = i + 1) begin
            noise_gen(2000); x = DCV + xn;
            @(posedge clk); #1;
        end
        sxn = 0.0; sx2n = 0.0; syn = 0.0; sy2n = 0.0;
        for (i = 0; i < 2000; i = i + 1) begin
            noise_gen(2000);
            x = DCV + xn;
            @(posedge clk); #1;
            sxn  = sxn  + $itor($signed(xn));
            sx2n = sx2n + $itor($signed(xn)) * $itor($signed(xn));
            syn  = syn  + $itor($signed(y))  - base;
            sy2n = sy2n + ($itor($signed(y)) - base) * ($itor($signed(y)) - base);
        end
        meanx = sxn / 2000.0;
        stdx  = $sqrt(sx2n / 2000.0 - meanx * meanx);
        meano = base + syn / 2000.0;
        stdo  = $sqrt(sy2n / 2000.0);
        if (stdo > 0.0) ratio = stdo / stdx; else ratio = 0.0;
        if (meano > base * 0.99 && meano < base * 1.01 && ratio < 0.6)
            $display("  ok 噪声平滑  输出均值=%0.1f(基%0d), std 比=%0.3f (理论≈0.25)",
                     meano, DCV, ratio);
        else begin
            $display("FAIL 噪声平滑 mean=%0.1f base=%0d ratio=%0.3f", meano, DCV, ratio);
            fail = fail + 1;
        end

        // ---- 测试3: 阶跃 6000：输出线性爬升，第 N 拍精确到 6000，无跳变 ----
        reinit;
        // 先以 0 填满窗
        for (i = 0; i < N; i = i + 1) begin
            x = 0; @(posedge clk); #1;
        end
        for (i = 0; i < N; i = i + 1) begin
            x = 6000; @(posedge clk); #1;
            ramp[i] = $signed(y);
        end
        monotone = 1;
        for (i = 0; i < N - 1; i = i + 1)
            if (ramp[i] >= ramp[i+1]) monotone = 0;
        if (monotone == 1 && ramp[0] > 0 && ramp[0] < 6000 && ramp[N-1] == 6000)
            $display("  ok 阶跃线性爬升  y[0]=%0d(>0) 单调 y[N-1]=%0d(==6000)",
                     ramp[0], ramp[N-1]);
        else begin
            $display("FAIL 阶跃爬升  y0=%0d y15=%0d mon=%0d", ramp[0], ramp[N-1], monotone);
            fail = fail + 1;
        end

        if (fail == 0) $display("PASS CompMovAvg");
        else           $display("FAIL CompMovAvg: %0d", fail);
        $finish;
    end
endmodule
