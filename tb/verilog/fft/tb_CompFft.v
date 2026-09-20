//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   tb_CompFft.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.06
/*Description       :   CompFft 单元 TB(自包含, 例化 N=16 与 N=64 两个 DUT)
  验证(输出=DFT/N, 见模块头):
    1) 实数正弦 k=1: bin1 与 bin(N-1) 谱峰 |mag|≈A/2, 其余 bin 近 0(Q 容差)。
    2) 复指数(解析信号) k0: 仅在 bin k0 谱峰 |mag|≈A(单边), 检验复数乘路径。
    3) 直流: bin0 ≈ A(=N*A/N), 其余近 0。
  每级右移 1 位缩放已在期望公式中折算; PASS/FAIL 打印。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.1
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
`timescale 1ns/1ps

module TbCompFft;
    localparam integer W = 16;

    reg clk = 0;
    reg iSysRst = 1;
    always #5 clk = ~clk;

    // ---- N=16 DUT ----
    localparam integer N16 = 16;
    reg  [N16*W-1:0]      iRe16, iIm16;
    reg  iStart16;
    wire [N16*W-1:0]      oRe16, oIm16;
    wire oDone16;
    CompFft #(.W(W), .N(N16), .LUTN(64)) U_Dut16 (
        .iSysClk(clk), .iSysRst(iSysRst), .iStart(iStart16),
        .iRe(iRe16), .iIm(iIm16),
        .oRe(oRe16), .oIm(oIm16), .oDone(oDone16)
    );

    // ---- N=64 DUT ----
    localparam integer N64 = 64;
    reg  [N64*W-1:0]      iRe64, iIm64;
    reg  iStart64;
    wire [N64*W-1:0]      oRe64, oIm64;
    wire oDone64;
    CompFft #(.W(W), .N(N64), .LUTN(64)) U_Dut64 (
        .iSysClk(clk), .iSysRst(iSysRst), .iStart(iStart64),
        .iRe(iRe64), .iIm(iIm64),
        .oRe(oRe64), .oIm(oIm64), .oDone(oDone64)
    );

    // ---- 输入样本缓冲 ----
    reg signed [15:0] xr [0:1023];   // 按需取前 N 个
    reg signed [15:0] xi [0:1023];

    integer err = 0;

    // 按输入数组组装打包总线 (把样本 n 写入 bus[n*W +: W])
    task t_pack;
        input integer N;
        output [1023:0] br;
        output [1023:0] bi;
        integer n;
        begin
            for (n = 0; n < N; n = n + 1) begin
                br[n*W +: W] = xr[n];
                bi[n*W +: W] = xi[n];
            end
        end
    endtask

    // 等待 done, 超时置错: N16 / N64 各一
    task t_wait16;
        integer c;
        begin
            c = 0;
            while (!oDone16) begin
                @(posedge clk);
                c = c + 1;
                if (c > 4096) begin
                    $display("FAIL fft: N=16 timeout");
                    err = err + 1;
                    disable t_wait16;
                end
            end
        end
    endtask
    task t_wait64;
        integer c;
        begin
            c = 0;
            while (!oDone64) begin
                @(posedge clk);
                c = c + 1;
                if (c > 8192) begin
                    $display("FAIL fft: N=64 timeout");
                    err = err + 1;
                    disable t_wait64;
                end
            end
        end
    endtask

    // 脉冲启动(数据已就绪并保持): 在下一拍采样到 iStart
    task t_go16;  begin @(negedge clk); iStart16 = 1; @(negedge clk); iStart16 = 0; end endtask
    task t_go64;  begin @(negedge clk); iStart64 = 1; @(negedge clk); iStart64 = 0; end endtask

    // 填单频实余弦 x[n]=A*cos(2pi*k*n/N)
    task t_fill_cos;
        input integer N;
        input integer A;
        input integer k;
        integer n;
        real ph, sv;
        begin
            for (n = 0; n < N; n = n + 1) begin
                ph = 2.0 * 3.141592653589793 * k * n / N;
                sv = $itor(A) * $cos(ph);
                xr[n] = (sv >= 0) ? $rtoi(sv + 0.5) : $rtoi(sv - 0.5);
                xi[n] = 16'sd0;
            end
        end
    endtask

    // 填复指数 x[n]=A*e^{j*2pi*k*n/N} (实=cos, 虚=sin)
    task t_fill_exp;
        input integer N;
        input integer A;
        input integer k;
        integer n;
        real ph, sv, cv;
        begin
            for (n = 0; n < N; n = n + 1) begin
                ph = 2.0 * 3.141592653589793 * k * n / N;
                sv = $itor(A) * $sin(ph);
                cv = $itor(A) * $cos(ph);
                xr[n] = (cv >= 0) ? $rtoi(cv + 0.5) : $rtoi(cv - 0.5);
                xi[n] = (sv >= 0) ? $rtoi(sv + 0.5) : $rtoi(sv - 0.5);
            end
        end
    endtask

    // 填直流 x[n]=A (实部)
    task t_fill_dc;
        input integer N;
        input integer A;
        integer n;
        begin
            for (n = 0; n < N; n = n + 1) begin
                xr[n] = A;
                xi[n] = 16'sd0;
            end
        end
    endtask

    // ---- 测试执行: 单频检验 (real cosine) ----
    task t_test_tone;
        input integer N;      // 点数
        input integer which;  // 0=U16 1=U64
        input integer A;
        input integer k;
        integer n, rv, iv;
        real mg, re, im, expv, maxo;
        integer rex, iix;
        integer bin, kk, img;
        begin
            t_fill_cos(N, A, k);
            if (which == 0) begin
                t_pack(N, iRe16, iIm16);   // 实际只读低 N 点, 输出 16 点
                t_go16;  t_wait16;
            end else begin
                t_pack(N, iRe64, iIm64);
                t_go64;  t_wait64;
            end
            expv = $itor(A) / 2.0;        // 输出 = DFT/N -> A/2
            // 读 k 与 N-k bin
            bin = k;
            if (which == 0) begin
                rex = $signed(oRe16[bin*W +: W]); iix = $signed(oIm16[bin*W +: W]);
            end else begin
                rex = $signed(oRe64[bin*W +: W]); iix = $signed(oIm64[bin*W +: W]);
            end
            re = $itor(rex); im = $itor(iix);
            mg = $sqrt(re*re + im*im);
            $display("FFT N=%0d k=%0d tone A=%0d: bin%0d mag=%0.1f (exp %0.1f)",
                     N, k, A, bin, mg, expv);
            if (mg < expv * 0.96 || mg > expv * 1.04) begin
                $display("FAIL fft: N=%0d peak bin%0d mag off", N, bin);
                err = err + 1;
            end
            // 镜像 bin N-k
            if (k != 0 && k != N/2) begin
                kk = N - k;
                if (which == 0) begin
                    rex = $signed(oRe16[kk*W +: W]); iix = $signed(oIm16[kk*W +: W]);
                end else begin
                    rex = $signed(oRe64[kk*W +: W]); iix = $signed(oIm64[kk*W +: W]);
                end
                re = $itor(rex); im = $itor(iix);
                mg = $sqrt(re*re + im*im);
                $display("FFT N=%0d k=%0d tone A=%0d: bin%0d mag=%0.1f (exp %0.1f)",
                         N, k, A, kk, mg, expv);
                if (mg < expv * 0.96 || mg > expv * 1.04) begin
                    $display("FAIL fft: N=%0d mirror bin%0d mag off", N, kk);
                    err = err + 1;
                end
            end
            // 其余 bin 应近 0
            maxo = 0.0;
            for (n = 0; n < N; n = n + 1) begin
                if (n == bin || (k != 0 && n == (N - k))) begin
                    // skip peak bins (N-k==k handled by k!=0)
                end else begin
                    if (which == 0) begin
                        rv = $signed(oRe16[n*W +: W]); iv = $signed(oIm16[n*W +: W]);
                    end else begin
                        rv = $signed(oRe64[n*W +: W]); iv = $signed(oIm64[n*W +: W]);
                    end
                    re = $itor(rv); im = $itor(iv);
                    mg = $sqrt(re*re + im*im);
                    if (mg > maxo) maxo = mg;
                end
            end
            $display("FFT N=%0d k=%0d tone A=%0d: max other-bin mag=%0.1f (A=%0d)",
                     N, k, A, maxo, A);
            if (maxo > ($itor(A) * 0.05)) begin
                $display("FAIL fft: N=%0d excessive leakage %.1f > 5%%A", N, maxo);
                err = err + 1;
            end
        end
    endtask

    // 复指数单频 (检验复数乘): bin k0 单边谱峰 mag≈A
    task t_test_exp;
        input integer N;
        input integer which;
        input integer A;
        input integer k;
        integer n, rv, iv, rex, iix;
        real mg, re, im, expv, maxo;
        integer bin;
        begin
            t_fill_exp(N, A, k);
            if (which == 0) begin
                t_pack(N, iRe16, iIm16);
                t_go16;  t_wait16;
            end else begin
                t_pack(N, iRe64, iIm64);
                t_go64;  t_wait64;
            end
            expv = $itor(A);               // DFT/N -> A (单边)
            bin = k;
            if (which == 0) begin
                rex = $signed(oRe16[bin*W +: W]); iix = $signed(oIm16[bin*W +: W]);
            end else begin
                rex = $signed(oRe64[bin*W +: W]); iix = $signed(oIm64[bin*W +: W]);
            end
            re = $itor(rex); im = $itor(iix);
            mg = $sqrt(re*re + im*im);
            $display("FFT N=%0d k=%0d complexexp A=%0d: bin%0d mag=%0.1f (exp %0.1f)",
                     N, k, A, bin, mg, expv);
            if (mg < expv * 0.96 || mg > expv * 1.04) begin
                $display("FAIL fft: N=%0d complex peak bin%0d mag off", N, bin);
                err = err + 1;
            end
            maxo = 0.0;
            for (n = 0; n < N; n = n + 1) begin
                if (n == bin) begin
                end else begin
                    if (which == 0) begin
                        rv = $signed(oRe16[n*W +: W]); iv = $signed(oIm16[n*W +: W]);
                    end else begin
                        rv = $signed(oRe64[n*W +: W]); iv = $signed(oIm64[n*W +: W]);
                    end
                    re = $itor(rv); im = $itor(iv);
                    mg = $sqrt(re*re + im*im);
                    if (mg > maxo) maxo = mg;
                end
            end
            $display("FFT N=%0d k=%0d complexexp A=%0d: max other-bin mag=%0.1f",
                     N, k, A, maxo);
            if (maxo > ($itor(A) * 0.05)) begin
                $display("FAIL fft: N=%0d complex leakage %.1f > 5%%A", N, maxo);
                err = err + 1;
            end
        end
    endtask

    // 直流: bin0 ≈ A
    task t_test_dc;
        input integer N;
        input integer which;
        input integer A;
        integer n, rv, iv, rex, iix;
        real mg, re, im, expv, maxo;
        begin
            t_fill_dc(N, A);
            if (which == 0) begin
                t_pack(N, iRe16, iIm16);
                t_go16;  t_wait16;
            end else begin
                t_pack(N, iRe64, iIm64);
                t_go64;  t_wait64;
            end
            expv = $itor(A);
            if (which == 0) begin
                rex = $signed(oRe16[0*W +: W]); iix = $signed(oIm16[0*W +: W]);
            end else begin
                rex = $signed(oRe64[0*W +: W]); iix = $signed(oIm64[0*W +: W]);
            end
            re = $itor(rex); im = $itor(iix);
            mg = $sqrt(re*re + im*im);
            $display("FFT N=%0d DC A=%0d: bin0 mag=%0.1f (exp %0.1f)", N, A, mg, expv);
            if (mg < expv * 0.96 || mg > expv * 1.04) begin
                $display("FAIL fft: N=%0d DC bin0 off (%.1f vs %0.1f)", N, mg, expv);
                err = err + 1;
            end
            maxo = 0.0;
            for (n = 1; n < N; n = n + 1) begin
                if (which == 0) begin
                    rv = $signed(oRe16[n*W +: W]); iv = $signed(oIm16[n*W +: W]);
                end else begin
                    rv = $signed(oRe64[n*W +: W]); iv = $signed(oIm64[n*W +: W]);
                end
                re = $itor(rv); im = $itor(iv);
                mg = $sqrt(re*re + im*im);
                if (mg > maxo) maxo = mg;
            end
            $display("FFT N=%0d DC A=%0d: max non-DC bin mag=%0.1f", N, A, maxo);
            if (maxo > 32.0) begin
                $display("FAIL fft: N=%0d DC leakage %.1f too large", N, maxo);
                err = err + 1;
            end
        end
    endtask

    initial begin
        iSysRst = 1; iStart16 = 0; iStart64 = 0; iRe16 = 0; iIm16 = 0; iRe64 = 0; iIm64 = 0;
        repeat (4) @(posedge clk);
        iSysRst = 0;
        repeat (2) @(posedge clk);

        // --- N=16 ---
        t_test_tone(16, 0, 12000, 1);
        t_test_tone(16, 0, 12000, 3);
        t_test_exp (16, 0, 8000, 2);
        t_test_dc  (16, 0, 10000);
        // --- N=64 ---
        t_test_tone(64, 1, 12000, 1);
        t_test_exp (64, 1, 6000, 5);
        t_test_dc  (64, 1, 8000);

        if (err == 0)
            $display("PASS fft: CompFft N=16/N=64 基-2 DIT 谱峰验证 (peak=A/2, DC=bin0=A)");
        else
            $display("FAIL fft: %0d checks failed", err);
        $finish;
    end
endmodule
