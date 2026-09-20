//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   tb_CompPark.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
/*Description       :   CompPark/CompInvPark 单元 TB（公式级数值校验，θ 16bit 全幅）。
                        d=alpha*cos+beta*sin ; q=-alpha*sin+beta*cos（与 C Comp_Park 一致）
                        反 Park 回绕还原验证。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.1
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
`timescale 1ns/1ps

module TbCompPark;
    reg signed [15:0] al, be, dd, qq;
    reg [15:0] th;
    wire signed [15:0] d, q;
    wire signed [15:0] a2, b2;
    integer fail = 0;

    CompPark U_P (.iAlpha(al), .iBeta(be), .iTheta(th), .oD(d), .oQ(q));
    CompInvPark U_I (.iD(d), .iQ(q), .iTheta(th), .oAlpha(a2), .oBeta(b2));

    task chk(input integer tag, input integer got, input integer want, input integer tol);
        if (got < want - tol || got > want + tol) begin
            $display("FAIL tag%0d: got=%0d want~%0d", tag, got, want);
            fail = fail + 1;
        end else
            $display("  ok tag%0d=%0d", tag, got);
    endtask

    initial begin
        // --- θ=0 : (1000,0) -> (d,q)=(1000,0) ---
        al = 1000; be = 0; th = 16'h0000; #1;
        chk(1, d, 1000, 4);  chk(2, q, 0, 4);
        // --- θ=0 : (0,1000) -> (d,q)=(0,1000)（β 轴即 q 轴，α 旋 +90 落在 +q）---
        al = 0; be = 1000; #1;
        chk(3, d, 0, 4);  chk(4, q, 1000, 4);
        // --- θ=90°(0x4000) : (1000,0) -> d≈0, q=-1000（q=-alpha sin 符号）---
        al = 1000; be = 0; th = 16'h4000; #1;
        chk(5, d, 0, 4);  chk(6, q, -1000, 6);
        // --- θ=-90°(0xC000) : (1000,0) -> d≈0, q=+1000 ---
        th = 16'hC000; #1;
        chk(7, d, 0, 4);  chk(8, q, 1000, 6);
        // --- θ=90°: (0,1000) -> d=1000,q=0 ---
        al = 0; be = 1000; th = 16'h4000; #1;
        chk(9, d, 1000, 6);  chk(10, q, 0, 4);
        // --- 45°：旋转角 45°(0x2000)，(1414,0) -> d≈1000,q≈-1000 ---
        al = 1414; be = 0; th = 16'h2000; #1;
        chk(11, d, 1000, 10); chk(12, q, -1000, 10);
        // --- 反 Park 回绕：给定输入任意角回绕还原 ---
        al = 1000; be = -400; th = 16'h3555; #1;
        chk(20, a2, al, 8); chk(21, b2, be, 8);
        al = -700; be = 500; th = 16'h7AAB; #1;
        chk(22, a2, al, 8); chk(23, b2, be, 8);

        if (fail == 0) $display("PASS CompPark");
        else $display("FAIL CompPark: %0d", fail);
        $finish;
    end
endmodule
