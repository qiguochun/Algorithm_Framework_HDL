//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   tb_CompRateLimit.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.06
/*Description       :   CompRateLimit 限幅速率单元 TB（RATE=100/拍）。
                        校验: (1) 阶跃 0->5000：每拍上升 <=RATE(=100)，
                              上升期逐拍恰 +100，最终精确到 5000（速率受限）；
                              (2) 阶跃 0->-5000 对称下降，最终 -5000；
                              (3) 每拍 +30 的缓变斜坡(<RATE) 直通，逐拍 y==x。
                        手段: 逐拍 dy 比较 + 终值精确比较。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.1
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
`timescale 1ns/1ps

module TbCompRateLimit;
    localparam integer W    = 16;
    localparam integer RATE = 100;

    reg clk = 0;
    reg iSysRst = 1;
    reg en = 1;
    reg signed [W-1:0] x = 0;
    wire signed [W-1:0] y;

    CompRateLimit #(.W(W), .RATE(RATE)) U_Dut (
        .iSysClk(clk), .iSysRst(iSysRst), .iEn(en), .iXIn(x), .oYOut(y)
    );

    always #5 clk = ~clk;

    integer fail = 0;
    integer i, cur, prev, d, maxrise, maxfall, bad;
    integer reach_up, reach_dn;
    integer r;

    task reinit;
        begin
            en = 1; x = 0;
            iSysRst = 1; repeat (2) @(posedge clk); #1;
            iSysRst = 0; @(posedge clk); #1;
        end
    endtask

    // 固定输入值跑 nrun 拍，结果写模块级变量 maxrise/maxfall/reach_*/cur
    task run_step(input integer step, input integer nrun, input integer which);
        begin
            reach_up = -1; reach_dn = -1; maxrise = 0; maxfall = 0;
            // 稳态 0 起跑
            for (r = 0; r < 5; r = r + 1) begin
                x = 0; @(posedge clk); #1;
            end
            prev = 0;
            for (r = 0; r < nrun; r = r + 1) begin
                x = step; @(posedge clk); #1;
                cur = $signed(y);
                d   = cur - prev;
                if (d > maxrise) maxrise = d;
                if (d < maxfall) maxfall = d;
                if (which == 0 && cur == step) begin
                    if (reach_up == -1) reach_up = r;
                end else begin
                    if (reach_dn == -1 && cur == step) reach_dn = r;
                end
                prev = cur;
            end
            cur = prev;
        end
    endtask

    initial begin
        // ---- 测试1: 阶跃 0->5000，上升逐拍 +RATE(100)，终值精确 5000 ----
        reinit;
        run_step(5000, 120, 0);
        if (maxrise <= RATE && maxfall >= 0 && cur == 5000 &&
            reach_up > 30 && reach_up <= 60)
            $display("  ok 上升限速  maxrise=%0d(<=%0d) 第%0d拍到 5000, 终值=%0d",
                     maxrise, RATE, reach_up, cur);
        else begin
            $display("FAIL 上升限速 maxrise=%0d reach=%0d last=%0d", maxrise, reach_up, cur);
            fail = fail + 1;
        end

        // ---- 测试2: 阶跃 0->-5000 对称下降，终值精确 -5000 ----
        reinit;
        run_step(-5000, 120, 1);
        if (maxfall >= -RATE && maxrise <= 0 && cur == -5000 &&
            reach_dn > 30 && reach_dn <= 60)
            $display("  ok 下降限速  maxfall=%0d(>=-%0d) 第%0d拍到 -5000, 终值=%0d",
                     maxfall, RATE, reach_dn, cur);
        else begin
            $display("FAIL 下降限速 maxfall=%0d reach=%0d last=%0d", maxfall, reach_dn, cur);
            fail = fail + 1;
        end

        // ---- 测试3: 缓变斜坡每拍 +30(<RATE) 直通，逐拍 y==x ----
        reinit;
        bad = 0;
        for (i = 0; i < 100; i = i + 1) begin
            x = 30 * (i + 1); @(posedge clk); #1;
            if (y !== 30 * (i + 1)) bad = 1;
        end
        if (bad == 0)
            $display("  ok 缓变直通  100 拍斜坡(y==x) 无误");
        else begin
            $display("FAIL 缓变直通");
            fail = fail + 1;
        end

        if (fail == 0) $display("PASS CompRateLimit");
        else           $display("FAIL CompRateLimit: %0d", fail);
        $finish;
    end
endmodule
