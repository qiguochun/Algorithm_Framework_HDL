//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   tb_CompGcc.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
/*Description       :   CompGcc(递推 EWMA) 单元 TB(自包含)。
                        分段测: x=y 同向正弦 -> SAME(+1), 预热前 valid=0;
                        x=-y 反向 -> OPPOSITE(-1)；x,y 无关噪声 -> |rho| 小, 多判 UNKNOWN(0)。
                        信号幅 12000、周期 40；alpha=15/16(ALPHAQ=30720), 阈值 0.4, 预热 200。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.1
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
`timescale 1ns/1ps

module TbCompGcc;
    localparam integer A_SIN  = 12000;  // 信号幅度
    localparam integer PER    = 40;     // 正弦周期(样本)
    localparam integer ALPHAQ = 30720;  // alpha=15/16 (Q0.15)
    localparam integer CORR   = 13107;  // 判定阈值 0.40 (Q0.15)
    localparam integer WARM   = 200;    // 预热样本数
    localparam integer POST   = 40;     // SAME/OPP 首段后测样本数
    localparam integer NNOISE = 300;    // 噪声后测样本数

    reg clk = 0;
    reg iSysRst = 1;
    reg en = 0;
    reg signed [15:0] x_in = 0;
    reg signed [15:0] y_in = 0;
    wire signed [15:0] rho;
    wire signed [1:0]  direction;
    wire valid;

    CompGcc #(
        .W(16), .FSHIFT(15), .ALPHAQ(ALPHAQ), .CORR_TH_Q(CORR), .WARMUP(WARM)
    ) U_Dut (
        .iSysClk(clk), .iSysRst(iSysRst), .iEn(en), .iXIn(x_in), .iYIn(y_in),
        .oRho(rho), .oDirection(direction), .oValid(valid)
    );

    always #5 clk = ~clk;

    integer bad = 0;
    integer i;
    integer cnt = 0;
    real sv;

    // 复位脉冲
    task reset_dut;
        begin
            iSysRst = 1;
            x_in = 16'sd0; y_in = 16'sd0;
            repeat (2) @(posedge clk);
            iSysRst = 0;
        end
    endtask

    // ---- 断言: 预热期间 valid=0, 且首个有效出现在第 WARM 次喂入(样本计数=WARM)
    task chk_warm_gate;
        input integer nfeed_so_far;
        begin
            if (valid) begin
                bad = 1;
                $display("Gcc FAIL: valid high before warmup (feed #%0d, cnt<=%0d)",
                         nfeed_so_far, WARM - 1);
            end
        end
    endtask

    // 幅度四舍五入转整数(供正/余弦样本)
    function signed [15:0] qs;
        input real v;
        begin
            qs = (v >= 0) ? $rtoi(v + 0.5) : $rtoi(v - 0.5);
        end
    endfunction

    initial begin
        en = 1;
        repeat (3) @(posedge clk);

        // ============ 1) x = y (同向) : SAME ============
        reset_dut();
        for (i = 0; i < WARM + POST; i = i + 1) begin
            sv  = $itor(A_SIN) * $sin(2.0 * 3.141592653589793 * $itor(i) / $itor(PER));
            x_in = qs(sv);
            y_in = x_in;
            @(posedge clk);
            cnt = i + 1;                     // 喂入后样本计数
            if (cnt < WARM) begin
                chk_warm_gate(i);
            end else begin
                if (!valid || direction != 2'sd1) begin
                    bad = 1;
                    $display("Gcc FAIL same: feed#%0d valid=%0d dir=%0d (expect 1/SAME)", i, valid, direction);
                end
            end
        end
        $display("Gcc  INFO: SAME done (last rho=%0d, dir=%0d)", rho, direction);

        // ============ 2) x = -y (反向) : OPPOSITE ============
        reset_dut();
        for (i = 0; i < WARM + POST; i = i + 1) begin
            sv  = $itor(A_SIN) * $sin(2.0 * 3.141592653589793 * $itor(i) / $itor(PER));
            x_in = qs(sv);
            y_in = -x_in;
            @(posedge clk);
            cnt = i + 1;
            if (cnt < WARM) begin
                chk_warm_gate(i);
            end else begin
                if (!valid || direction != -2'sd1) begin
                    bad = 1;
                    $display("Gcc FAIL opp: feed#%0d valid=%0d dir=%0d (expect 1/-1)", i, valid, direction);
                end
            end
        end
        $display("Gcc  INFO: OPPOSITE done (last rho=%0d, dir=%0d)", rho, direction);

        // ============ 3) x,y 无关噪声: |rho| 小 -> UNKNOWN ============
        begin : noise_blk
            integer rngx, rngy, n;
            integer sum_abs, nmeas, nunk;
            real    avg_abs;
            rngx = 1234567;
            rngy = 7654321;
            sum_abs = 0; nmeas = 0; nunk = 0;
            reset_dut();
            for (n = 0; n < WARM + NNOISE; n = n + 1) begin
                rngx = (rngx * 1103515245 + 12345) & 32'h7fffffff;
                rngy = (rngy * 1103515245 + 67890) & 32'h7fffffff;
                x_in = (rngx % (2 * A_SIN + 1)) - A_SIN;
                y_in = (rngy % (2 * A_SIN + 1)) - A_SIN;
                @(posedge clk);
                cnt = n + 1;
                if (cnt < WARM) chk_warm_gate(n);
                else if (valid) begin
                    sum_abs = sum_abs + ((rho < 0) ? (-rho) : rho);
                    if (direction == 2'sd0) nunk = nunk + 1;
                    nmeas = nmeas + 1;
                end
            end
            avg_abs = $itor(sum_abs) / $itor(nmeas);
            $display("Gcc  INFO: noise measured=%0d avg|rho|Q=%0.1f (thr=%0d) unknown=%0d/%0d",
                     nmeas, avg_abs, CORR, nunk, nmeas);
            if (nmeas == 0) begin
                bad = 1;
                $display("Gcc FAIL noise: no valid samples measured");
            end
            if (nmeas > 0 && avg_abs > $itor(CORR)) begin
                bad = 1;
                $display("Gcc FAIL noise: avg|rho| above threshold");
            end
            if (nmeas > 0 && $itor(nunk) < $itor(nmeas) * 0.5) begin
                bad = 1;
                $display("Gcc FAIL noise: majority not UNKNOWN");
            end
        end

        if (!bad)
            $display("PASS comp_gcc: x=y->SAME, x=-y->OPPOSITE, 噪声->|rho|小/UNKNOWN, 预热前 valid=0");
        else
            $display("FAIL comp_gcc");
        $finish;
    end
endmodule
