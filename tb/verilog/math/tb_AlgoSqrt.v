//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   tb_AlgoSqrt.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.06
/*Description       :   AlgoSqrt 多拍握手 TB：核对整数开方/延迟
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

module TbAlgoSqrt;
    reg clk = 0;
    reg rst = 0;
    always #5 clk = ~clk;

    reg [63:0] num = 0;
    reg start = 0;
    wire busy, done;
    wire [31:0] root;
    integer fail = 0;

    AlgoSqrt #(.IW(64), .OW(32)) U_Dut (
        .iSysClk(clk), .iSysRst(rst), .iStart(start),
        .iNum(num), .oBusy(busy), .oDone(done), .oRoot(root));

    task do_sqrt(input [63:0] a);
        begin
            num = a; start = 1;
            @(posedge clk); start = 0;
            wait (done == 1'b1);
            @(posedge clk);
        end
    endtask

    task chk(input integer tag, input integer got, input integer want);
        if (got !== want[31:0]) begin
            $display("FAIL tag%0d: sqrt=%0d want %0d", tag, got, want);
            fail = fail + 1;
        end else
            $display("  ok tag%0d: sqrt(%0d)=%0d", tag, num, got);
    endtask

    integer r;
    initial begin
        rst = 1; repeat (3) @(posedge clk); rst = 0;

        do_sqrt(0);       r = root; chk(1, r, 0);
        do_sqrt(1);       r = root; chk(2, r, 1);
        do_sqrt(4);       r = root; chk(3, r, 2);
        do_sqrt(9);       r = root; chk(4, r, 3);
        do_sqrt(100);     r = root; chk(5, r, 10);
        do_sqrt(10000);   r = root; chk(6, r, 100);
        do_sqrt(12345);   r = root; chk(7, r, 111);
        do_sqrt(123456789); r = root; chk(8, r, 11111);

        if (fail == 0) begin $display("PASS AlgoSqrt"); $finish; end
        $display("FAIL AlgoSqrt: %0d", fail); $finish;
    end
endmodule
