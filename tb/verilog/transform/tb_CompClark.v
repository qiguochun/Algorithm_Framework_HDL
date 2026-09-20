//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   tb_CompClark.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
/*Description       :   CompClark(Amp/Power)/CompInvClark 单元 TB（纯组合校验）。
                        平衡正序 A=1000: Power 模式 alpha/beta 幅值≈A、zero≈0；Amp 模式 alpha 直通。
                        校验点按精确理想值取整，容差 ±4 LSB。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.1
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
`timescale 1ns/1ps

module TbCompClark;
    reg signed [15:0] a, b, c;
    wire signed [15:0] alp_a, bet_a, zo_a;   // Amp 输出
    wire signed [15:0] alp_p, bet_p, zo_p;   // Power 输出
    integer fail = 0;

    CompClark #(.MODE(0)) U_Amp (.iA(a), .iB(b), .iC(c), .oAlpha(alp_a), .oBeta(bet_a), .oZero(zo_a));
    CompClark #(.MODE(1)) U_Pwr (.iA(a), .iB(b), .iC(c), .oAlpha(alp_p), .oBeta(bet_p), .oZero(zo_p));

    // CompInvClark 独立例化，直接给 (alpha,beta,zero)
    reg signed [15:0] ial, ibe, izo;
    wire signed [15:0] ia_, ib_, ic_;
    CompInvClark U_Inv (.iAlpha(ial), .iBeta(ibe), .iZero(izo), .oA(ia_), .oB(ib_), .oC(ic_));

    task chk(input integer tag, input integer got, input integer want, input integer tol);
        if (got < want - tol || got > want + tol) begin
            $display("FAIL tag%0d: got=%0d want~%0d", tag, got, want);
            fail = fail + 1;
        end else
            $display("  ok tag%0d=%0d", tag, got);
    endtask

    task set(input integer x, integer y, integer z);
        begin a = x[15:0]; b = y[15:0]; c = z[15:0]; #1; end
    endtask

    initial begin
        // --- 平衡三相（幅值1000）θ=0 : (1000, -500, -500) ---
        set(1000, -500, -500);
        chk(101, alp_a, 1000, 3);   // Amp alpha=a
        chk(102, bet_a, 0, 4);      // (a+2b)/sqrt3 = 0
        chk(103, zo_a, 0, 3);
        chk(111, alp_p, 1000, 3);   // Power alpha=(2a-b-c)/3
        chk(112, bet_p, 0, 4);      // (b-c)/sqrt3 = 0
        chk(113, zo_p, 0, 3);

        // --- 平衡 θ=90° : (0, 866, -866) ---
        set(0, 866, -866);
        chk(201, alp_p, 0, 3);      // alpha = A cos90
        chk(202, bet_p, 1000, 4);   // (866+866)/sqrt3 = 1000
        chk(203, alp_a, 0, 3);
        chk(204, bet_a, 1000, 4);   // (0+2*866)/sqrt3 = 1000
        chk(205, zo_a, 0, 3);

        // --- 不平衡（含零序）: (300,100,-200) ---
        set(300, 100, -200);
        chk(301, alp_p, 233, 3);    // (600-100+200)/3 = 233.3
        chk(302, bet_p, 173, 3);    // 300/sqrt3 = 173.2
        chk(303, zo_p, 67, 3);      // 200/3 = 66.7
        chk(304, alp_a, 300, 3);
        chk(305, bet_a, 289, 3);    // (300+200)/sqrt3 = 288.7

        // --- InvClark: (alpha,beta,zero) = (1000,0,0) ---
        ial = 1000; ibe = 0; izo = 0; #1;
        chk(401, ia_, 1000, 4);
        chk(402, ib_, -500, 4);
        chk(403, ic_, -500, 4);
        // --- InvClark 带零序: (1000,0,100) -> (1100,-400,-400) ---
        ial = 1000; ibe = 0; izo = 100; #1;
        chk(411, ia_, 1100, 4);
        chk(412, ib_, -400, 5);
        chk(413, ic_, -400, 5);

        if (fail == 0) $display("PASS CompClark");
        else $display("FAIL CompClark: %0d", fail);
        $finish;
    end
endmodule
