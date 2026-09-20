//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   tb_CompRep.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.06
/*Description       :   CompRep 单元 TB: 喂基波+3 次谐波周期误差(周期=N=16 拍),
//                       用独立浮点/整数模型镜像差分 u[n]=e[n]+Q*u[n-N], y[n]=u[n-N],
//                       逐拍与硬件 oOut 比对(饱和对齐 S16), 全程应位级一致。
//                       判据: 比对 4096 拍全部相等 -> PASS(验证环形缓冲 N 拍延迟与 Q 反馈)。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.1
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
`timescale 1ns/1ps

module TbCompRep;
    localparam integer N   = 16;             // 基波周期采样数(TB 用小 N 加速)
    localparam integer QF  = 14;
    localparam integer PQ  = 15565;          // Q=0.95 Q14
    localparam integer M   = 4096;           // 比对拍数
    localparam integer RND = (1 << (QF-1));  // 8192

    reg                clk = 0, iSysRst = 1;
    reg  signed [15:0] iE = 0;
    wire signed [15:0] oY;

    CompRep #(.P_N(N), .QF(QF), .P_Q(PQ)) U_Dut (
        .iSysClk(clk), .iSysRst(iSysRst), .iErrIn(iE), .oOut(oY)
    );
    always #5 clk = ~clk;

    // ---- 定点参考模型（TB only）----
    integer moBuf [0:N-1];
    integer moP;
    integer s;
    integer err, eCur;

    // 周期误差: 基波 + 3 次谐波(均以 N 拍为基波周期)
    function integer genE(input integer k);
        integer v; real tt;
        begin
            tt = 6.283185307179586 * k / (N * 1.0);
            v  = $rtoi(300.0*$sin(tt) + 120.0*$sin(3.0*tt));
            if (v >  32767) v = 32767;
            if (v < -32768) v = -32768;
            genE = v;
        end
    endfunction

    integer m, tmp, moY, uNew, clip;

    initial begin
        for (tmp = 0; tmp < N; tmp = tmp + 1) moBuf[tmp] = 0;
        moP  = 0; s = 0; err = 0; eCur = 0;
        repeat (3) @(posedge clk);      // 复位保持
        iSysRst = 0;
        eCur = genE(0);
        iE   = eCur[15:0];              // 第一拍误差
        for (m = 0; m < M; m = m + 1) begin
            @(posedge clk); #1;         // DUT 以 eCur 处理第 m 拍
            // 模型: y[m]=u[m-N]=moBuf[moP] 与 oY 比对(饱和对齐)
            moY = moBuf[moP];
            clip = moY;
            if (clip >  32767) clip =  32767;
            if (clip < -32768) clip = -32768;
            if (clip !== oY) begin
                err = err + 1;
                if (err < 6)
                    $display("  MISMATCH m=%0d oY=%0d model=%0d", m, oY, clip);
            end
            // 模型更新 u[m]=e[m]+Q*u[m-N], 存储, 指针 +1
            uNew = ((PQ*moY + RND) >>> QF) + eCur;
            moBuf[moP] = uNew;
            moP = (moP == N-1) ? 0 : moP + 1;
            // 准备下一拍误差
            s = s + 1;
            eCur = genE(s);
            iE   = eCur[15:0];
        end
        if (err == 0)
            $display("PASS CompRep: N=%0d 拍环形延迟 + Q=0.95 反馈逐拍镜像一致 (共 %0d 拍)",
                     N, M);
        else
            $display("FAIL CompRep: %0d/%0d 拍不匹配", err, M);
        $finish;
    end
endmodule
