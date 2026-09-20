// ---------------------------------------------------------------------------
// algo_filt_coef.vh —— IIR 系数编译期(elabor)整数计算（模块内 include）
//   仅整数四则; pi=314159/100000, sqrt2=141421/100000, 中间量 Q16。
//   双线性直接代入 s=2*fs*(z-1)/(z+1)，与现有 Python 黄金系数同公式。
//   在模块内 `include 后，用 FS/FC/F0/QREF 等 parameter 生成 localparam 系数。
// 修订: 2026-09-20 v0.1
// ---------------------------------------------------------------------------

function automatic signed [63:0] fn_filt_divr64;
    input signed [63:0] num;
    input signed [63:0] den;
    reg signed [63:0] n;
    reg signed [63:0] d;
    reg signed [63:0] half;
    begin
        n = num;
        d = den;
        if (d < 0) begin
            n = -n;
            d = -d;
        end
        if (d == 64'sd0) begin
            fn_filt_divr64 = 64'sd0;
        end else begin
            half = d >>> 1;
            if (n >= 0) fn_filt_divr64 = (n + half) / d;
            else        fn_filt_divr64 = -((-n + half) / d);
        end
    end
endfunction

function automatic signed [63:0] fn_filt_mulshr;
    input signed [63:0] a;
    input signed [63:0] b;
    input integer sh;
    reg signed [127:0] p;
    begin
        p = $signed({{64{a[63]}}, a}) * $signed({{64{b[63]}}, b});
        fn_filt_mulshr = p >>> sh;
    end
endfunction

function automatic signed [63:0] fn_filt_rq;
    input integer fs;
    input integer fc;
    begin
        if (fc <= 0)
            fn_filt_rq = 64'sd0;
        else
            fn_filt_rq = fn_filt_divr64((64'sd100000 * fs) <<< 16,
                                        64'sd314159 * fc);
    end
endfunction

function automatic signed [63:0] fn_filt_s2r;
    input signed [63:0] r;
    reg signed [63:0] s2;
    begin
        s2 = fn_filt_divr64(64'sd141421 <<< 16, 64'sd100000);
        fn_filt_s2r = fn_filt_mulshr(s2, r, 16);
    end
endfunction

function automatic signed [63:0] fn_filt_den_bw;
    input signed [63:0] r;
    begin
        fn_filt_den_bw = (64'sd1 <<< 16) + fn_filt_s2r(r)
                       + fn_filt_mulshr(r, r, 16);
    end
endfunction

function automatic signed [63:0] fn_filt_den_q;
    input signed [63:0] r;
    input integer qref;
    begin
        fn_filt_den_q = (64'sd1 <<< 16)
                      + fn_filt_divr64(r * 1000, qref)
                      + fn_filt_mulshr(r, r, 16);
    end
endfunction

function automatic integer fn_lpf1_b;
    input integer fs;
    input integer fc;
    input integer fshift;
    begin
        fn_lpf1_b = fn_filt_divr64((64'sd314159 * fc) <<< fshift,
                                   64'sd100000 * fs + 64'sd314159 * fc);
    end
endfunction

function automatic integer fn_lpf1_a;
    input integer fs;
    input integer fc;
    input integer fshift;
    begin
        fn_lpf1_a = fn_filt_divr64((64'sd314159 * fc - 64'sd100000 * fs) <<< fshift,
                                   64'sd100000 * fs + 64'sd314159 * fc);
    end
endfunction

function automatic integer fn_lpf2_b0;
    input integer fs;
    input integer fc;
    input integer fshift;
    reg signed [63:0] r;
    begin
        r = fn_filt_rq(fs, fc);
        fn_lpf2_b0 = fn_filt_divr64((64'sd1 <<< 16) <<< fshift, fn_filt_den_bw(r));
    end
endfunction

function automatic integer fn_lpf2_b1;
    input integer fs;
    input integer fc;
    input integer fshift;
    begin
        fn_lpf2_b1 = fn_lpf2_b0(fs, fc, fshift) * 2;
    end
endfunction

function automatic integer fn_lpf2_a1;
    input integer fs;
    input integer fc;
    input integer fshift;
    reg signed [63:0] r;
    begin
        r = fn_filt_rq(fs, fc);
        fn_lpf2_a1 = fn_filt_divr64((((64'sd1 <<< 16) - fn_filt_mulshr(r, r, 16)) <<< 1)
                                    <<< fshift,
                                    fn_filt_den_bw(r));
    end
endfunction

function automatic integer fn_lpf2_a2;
    input integer fs;
    input integer fc;
    input integer fshift;
    reg signed [63:0] r;
    begin
        r = fn_filt_rq(fs, fc);
        fn_lpf2_a2 = fn_filt_divr64(((64'sd1 <<< 16) - fn_filt_s2r(r)
                                   + fn_filt_mulshr(r, r, 16)) <<< fshift,
                                   fn_filt_den_bw(r));
    end
endfunction

function automatic integer fn_hpf2_b0;
    input integer fs;
    input integer fc;
    input integer fshift;
    reg signed [63:0] r;
    begin
        r = fn_filt_rq(fs, fc);
        fn_hpf2_b0 = fn_filt_divr64(fn_filt_mulshr(r, r, 16) <<< fshift,
                                   fn_filt_den_bw(r));
    end
endfunction

function automatic integer fn_hpf2_b1;
    input integer fs;
    input integer fc;
    input integer fshift;
    begin
        fn_hpf2_b1 = -2 * fn_hpf2_b0(fs, fc, fshift);
    end
endfunction

function automatic integer fn_hpf2_a1;
    input integer fs;
    input integer fc;
    input integer fshift;
    begin
        fn_hpf2_a1 = fn_lpf2_a1(fs, fc, fshift);
    end
endfunction

function automatic integer fn_hpf2_a2;
    input integer fs;
    input integer fc;
    input integer fshift;
    begin
        fn_hpf2_a2 = fn_lpf2_a2(fs, fc, fshift);
    end
endfunction

function automatic integer fn_bpf2_b0;
    input integer fs;
    input integer f0;
    input integer qref;
    input integer fshift;
    reg signed [63:0] r;
    begin
        r = fn_filt_rq(fs, f0);
        fn_bpf2_b0 = fn_filt_divr64(fn_filt_divr64(r * 1000, qref) <<< fshift,
                                   fn_filt_den_q(r, qref));
    end
endfunction

function automatic integer fn_bpf2_a1;
    input integer fs;
    input integer f0;
    input integer qref;
    input integer fshift;
    reg signed [63:0] r;
    begin
        r = fn_filt_rq(fs, f0);
        fn_bpf2_a1 = fn_filt_divr64((((64'sd1 <<< 16) - fn_filt_mulshr(r, r, 16)) <<< 1)
                                    <<< fshift,
                                    fn_filt_den_q(r, qref));
    end
endfunction

function automatic integer fn_bpf2_a2;
    input integer fs;
    input integer f0;
    input integer qref;
    input integer fshift;
    reg signed [63:0] r;
    begin
        r = fn_filt_rq(fs, f0);
        fn_bpf2_a2 = fn_filt_divr64(((64'sd1 <<< 16) - fn_filt_divr64(r * 1000, qref)
                                   + fn_filt_mulshr(r, r, 16)) <<< fshift,
                                   fn_filt_den_q(r, qref));
    end
endfunction

function automatic integer fn_bsf2_b0;
    input integer fs;
    input integer f0;
    input integer qref;
    input integer fshift;
    reg signed [63:0] r;
    begin
        r = fn_filt_rq(fs, f0);
        fn_bsf2_b0 = fn_filt_divr64(((64'sd1 <<< 16) + fn_filt_mulshr(r, r, 16)) <<< fshift,
                                   fn_filt_den_q(r, qref));
    end
endfunction

function automatic integer fn_hpf1_a;
    input integer fs;
    input integer fc;
    input integer fshift;
    begin
        fn_hpf1_a = fn_filt_divr64((64'sd100000 * fs - 64'sd314159 * fc) <<< fshift,
                                  64'sd100000 * fs + 64'sd314159 * fc);
    end
endfunction

function automatic integer fn_hpf1_b;
    input integer fs;
    input integer fc;
    input integer fshift;
    begin
        fn_hpf1_b = fn_filt_divr64((64'sd100000 * fs) <<< fshift,
                                  64'sd100000 * fs + 64'sd314159 * fc);
    end
endfunction

function automatic integer fn_notch_a1;
    input integer fs;
    input integer fc;
    input integer c1_milli;
    input integer fshift;
    reg signed [63:0] w;
    reg signed [63:0] y;
    begin
        w = fn_filt_divr64((64'sd2 * 314159 * fc) <<< 16, 64'sd100000 * fs);
        y = fn_filt_divr64(64'sd2 * c1_milli * w, 64'sd1000);
        fn_notch_a1 = fn_filt_divr64(((64'sd2 <<< 16) - y) <<< fshift, 64'sd1 <<< 16);
    end
endfunction

function automatic integer fn_notch_a2;
    input integer fs;
    input integer fc;
    input integer c1_milli;
    input integer fshift;
    reg signed [63:0] w;
    reg signed [63:0] y;
    reg signed [63:0] z;
    begin
        w = fn_filt_divr64((64'sd2 * 314159 * fc) <<< 16, 64'sd100000 * fs);
        y = fn_filt_divr64(64'sd2 * c1_milli * w, 64'sd1000);
        z = fn_filt_mulshr(w, w, 16);
        fn_notch_a2 = fn_filt_divr64((y - z - (64'sd1 <<< 16)) <<< fshift, 64'sd1 <<< 16);
    end
endfunction

function automatic integer fn_notch_b1;
    input integer fs;
    input integer fc;
    input integer c2_milli;
    input integer fshift;
    reg signed [63:0] w;
    reg signed [63:0] x;
    begin
        w = fn_filt_divr64((64'sd2 * 314159 * fc) <<< 16, 64'sd100000 * fs);
        x = fn_filt_divr64(64'sd2 * c2_milli * w, 64'sd1000);
        fn_notch_b1 = fn_filt_divr64((x - (64'sd2 <<< 16)) <<< fshift, 64'sd1 <<< 16);
    end
endfunction

function automatic integer fn_notch_b2;
    input integer fs;
    input integer fc;
    input integer c2_milli;
    input integer fshift;
    reg signed [63:0] w;
    reg signed [63:0] x;
    reg signed [63:0] z;
    begin
        w = fn_filt_divr64((64'sd2 * 314159 * fc) <<< 16, 64'sd100000 * fs);
        x = fn_filt_divr64(64'sd2 * c2_milli * w, 64'sd1000);
        z = fn_filt_mulshr(w, w, 16);
        fn_notch_b2 = fn_filt_divr64((z - x + (64'sd1 <<< 16)) <<< fshift, 64'sd1 <<< 16);
    end
endfunction

function automatic integer fn_diff_a1;
    input integer fs;
    input integer fc;
    input integer fshift;
    begin
        fn_diff_a1 = fn_filt_divr64((64'sd2 * 314159 * fc) <<< fshift,
                                   64'sd2 * 100000 * fs + 64'sd2 * 314159 * fc);
    end
endfunction

function automatic integer fn_diff_b1;
    input integer fs;
    input integer fc;
    input integer fshift;
    begin
        fn_diff_b1 = fn_filt_divr64((64'sd2 * 100000 * fs - 64'sd2 * 314159 * fc) <<< fshift,
                                   64'sd2 * 100000 * fs + 64'sd2 * 314159 * fc);
    end
endfunction

function automatic integer fn_diff_a2;
    input integer fs;
    input integer n;
    begin
        fn_diff_a2 = fn_filt_divr64(64'sd2 * n * fs, 64'sd2 * fs + n);
    end
endfunction

function automatic integer fn_diff_b2;
    input integer fs;
    input integer n;
    input integer fshift;
    begin
        fn_diff_b2 = fn_filt_divr64((64'sd2 * fs - n) <<< fshift, 64'sd2 * fs + n);
    end
endfunction

function automatic integer fn_ewma_alpha;
    input integer fs;
    input integer fc;
    input integer fshift;
    begin
        fn_ewma_alpha = fn_filt_divr64((64'sd2 * 314159 * fc) <<< fshift,
                                      64'sd100000 * fs);
        if (fn_ewma_alpha < 1) fn_ewma_alpha = 1;
        if (fn_ewma_alpha >= (1 <<< fshift)) fn_ewma_alpha = (1 <<< fshift) - 1;
    end
endfunction
