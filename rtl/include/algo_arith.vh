// ---------------------------------------------------------------------------
// algo_arith.vh —— 通用 S16 定点辅助函数（模块内 include 用，非独立文件）
//   fn_sat  : 对 48 位累加做 fb 位舍入右移 + S16 饱和钳位
//   fn_abs16: 求 S16 绝对值（饱和到 32767）
// 注: 开方/除法禁止用组合 function 摊开(§6.3)，统一例化多拍库 math/AlgoSqrt.v / math/AlgoDiv.v。
// 修订: 2026-09-05 v0.1
// ---------------------------------------------------------------------------
`ifndef ALGO_ARITH_VH
`define ALGO_ARITH_VH

function signed [15:0] fn_sat;
    input signed [47:0] a;   // 待收缩累加值
    input [4:0]          fb; // 右移位数（Q 格式 14/15；fb=0 仅饱和不缩放）
    reg signed [47:0] ra;
    integer half;
    begin
        half = (fb > 0) ? (1 << (fb - 1)) : 0;   // 舍入加半
        ra = a + half;
        ra = ra >>> fb;                          // 算术右移
        if (ra > 32767)   ra = 32767;            // S16 饱和
        if (ra < -32768)  ra = -32768;
        fn_sat = ra[15:0];
    end
endfunction

function signed [15:0] fn_abs16;
    input signed [15:0] x;
    begin
        fn_abs16 = (x[15]) ? ((x == -16'sd32768) ? 16'sd32767 : -x) : x;
    end
endfunction

`endif
