//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   tb_CompMedian.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.06
/*Description       :   CompMedian N=5 中值滤波单元 TB。
                        校验: (1) 基线 2000 上注入隔离尖峰脉冲(+20000/-15000)，
                              输出恒为 2000（中值完全去除单样本毛刺）；
                              (2) 阶跃 1000->5000 保留：至多 (N+1)/2 拍内
                              输出达新值 5000 并保持（中值不清边沿）。
                        手段: 每拍精确比较 y 是否为基线/新稳态。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.1
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
`timescale 1ns/1ps

module TbCompMedian;
    localparam integer W = 16;
    localparam integer N = 5;

    reg clk = 0;
    reg iSysRst = 1;
    reg en = 1;
    reg signed [W-1:0] x = 0;
    wire signed [W-1:0] y;

    CompMedian #(.W(W), .N(N)) U_Dut (
        .iSysClk(clk), .iSysRst(iSysRst), .iEn(en), .iXIn(x), .oYOut(y)
    );

    always #5 clk = ~clk;

    integer fail = 0;
    integer i;
    integer bad;
    integer reach;

    task reinit;
        begin
            en = 1; x = 0;
            iSysRst = 1; repeat (2) @(posedge clk); #1;
            iSysRst = 0; @(posedge clk); #1;
        end
    endtask

    initial begin
        // ---- 测试1: 隔离尖峰毛刺被完全去除，输出恒=基线 2000 ----
        reinit;
        // 填窗：基线 2000
        for (i = 0; i < 30; i = i + 1) begin
            x = 2000; @(posedge clk); #1;
        end
        bad = 0;
        for (i = 0; i < 400; i = i + 1) begin
            if ((i % 13) == 0)      x =  20000;   // 正尖峰
            else if ((i % 13) == 7) x = -15000;   // 负尖峰
            else                    x =  2000;
            @(posedge clk); #1;
            if (y !== 2000) bad = 1;
        end
        if (bad == 0)
            $display("  ok 脉冲毛刺去除  400 拍含 +/- 尖峰输出恒=2000");
        else begin
            $display("FAIL 脉冲去除");
            fail = fail + 1;
        end

        // ---- 测试2: 阶跃 1000->5000 保留（不破坏边沿，达新稳态） ----
        reinit;
        for (i = 0; i < 30; i = i + 1) begin
            x = 1000; @(posedge clk); #1;
        end
        reach = -1;
        for (i = 0; i < 30; i = i + 1) begin
            x = 5000; @(posedge clk); #1;
            if (reach == -1 && y == 5000) reach = i;
        end
        // 之后保持新值
        bad = 0;
        for (i = 0; i < 20; i = i + 1) begin
            x = 5000; @(posedge clk); #1;
            if (y !== 5000) bad = 1;
        end
        if (reach >= 0 && reach <= ((N + 1) / 2) + 1 && bad == 0)
            $display("  ok 阶跃保留  第%0d 拍到 5000 并保持 (期望 <=%0d)",
                     reach, ((N + 1) / 2) + 1);
        else begin
            $display("FAIL 阶跃保留 reach=%0d bad=%0d", reach, bad);
            fail = fail + 1;
        end

        if (fail == 0) $display("PASS CompMedian");
        else           $display("FAIL CompMedian: %0d", fail);
        $finish;
    end
endmodule
