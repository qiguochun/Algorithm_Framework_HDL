//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   tb_AlgoTrig.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
/*Description       :   AlgoTrig 单元 TB：核对若干已知角（纯 inline）
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

module TbAlgoTrig;
    reg [15:0] ang = 0;
    wire signed [15:0] s, c;
    integer fail = 0;

    AlgoTrig U_Dut (.iAngle(ang), .oSin(s), .oCos(c));

    task chk(input integer tag, input integer got, input integer want, input integer tol);
        if (got < want - tol || got > want + tol) begin
            $display("FAIL tag%0d: got=%0d want~%0d", tag, got, want);
            fail = fail + 1;
        end else
            $display("  ok tag%0d=%0d", tag, got);
    endtask

    initial begin
        ang = 16'h0000; #1;
        chk(1, s, 0, 32);      chk(2, c, 32767, 32);
        ang = 16'h4000; #1;
        chk(3, s, 32767, 40);  chk(4, c, 0, 40);
        ang = 16'h8000; #1;
        chk(5, s, 0, 32);      chk(6, c, -32768, 48);
        ang = 16'hC000; #1;
        chk(7, s, -32767, 48); chk(8, c, 0, 40);

        if (fail == 0) begin $display("PASS AlgoTrig"); $finish; end
        $display("FAIL AlgoTrig: %0d", fail); $finish;
    end
endmodule
