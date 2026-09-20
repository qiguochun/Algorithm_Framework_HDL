//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   tb_AlgoDiv.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.06
/*Description       :   AlgoDiv 多拍握手 TB：核对有符号除法/除零/延迟
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.1
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------

`timescale 1ns/1ps
`include "algo_types.vh"

module TbAlgoDiv;
    reg clk = 0;
    reg rst = 0;
    always #5 clk = ~clk;

    reg [31:0] num = 0, den = 1;
    reg start = 0;
    wire busy, done;
    wire [31:0] quot;
    integer fail = 0;

    AlgoDiv #(.W(32)) U_Dut (
        .iSysClk(clk), .iSysRst(rst), .iStart(start),
        .iNum(num), .iDen(den), .oBusy(busy), .oDone(done), .oQuot(quot));

    // 发一次除法，等 done，返回商
    task do_div(input [31:0] a, input [31:0] b);
        begin
            num = a; den = b; start = 1;
            @(posedge clk); start = 0;
            wait (done == 1'b1);
            @(posedge clk);   // done 拉一拍后采
        end
    endtask

    task chk(input integer tag, input integer got, input integer want);
        if (got !== want[31:0]) begin
            $display("FAIL tag%0d: %0d/%0d -> %0d want %0d", tag, num, den, got, want);
            fail = fail + 1;
        end else
            $display("  ok tag%0d: %0d/%0d = %0d", tag, num, den, got);
    endtask

    integer q;
    initial begin
        rst = 1; repeat (3) @(posedge clk); rst = 0;

        do_div(10, 3);    q = quot; chk(1, q, 3);       // 10/3=3
        do_div(-10, 3);   q = quot; chk(2, q, -3);      // -10/3=-3
        do_div(10, -3);   q = quot; chk(3, q, -3);      // 10/-3=-3
        do_div(-10, -3);  q = quot; chk(4, q, 3);       // -10/-3=3
        do_div(0, 5);     q = quot; chk(5, q, 0);       // 0/5=0
        do_div(100, 7);   q = quot; chk(6, q, 14);      // 100/7=14
        do_div(5, 0);     q = quot; chk(7, q, 0);       // 除零 -> 0

        if (fail == 0) begin $display("PASS AlgoDiv"); $finish; end
        $display("FAIL AlgoDiv: %0d", fail); $finish;
    end
endmodule
