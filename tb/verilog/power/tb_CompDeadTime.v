//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   tb_CompDeadTime.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.06
/*Description       :   CompDeadTime 单元 TB: 纯组合。正电流 -> +comp, 负电流 -> -comp,
//                       近零(|i|<阈值 8)不补偿输出 0。验证正/负电流补偿方向与幅值(comp=200)。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.1
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
`timescale 1ns/1ps

module TbCompDeadTime;
    reg  signed [15:0] iCur  = 0;
    reg  signed [15:0] iComp = 200;
    wire signed [15:0] oDutyInc;
    integer fail = 0, g;

    CompDeadTime U_Dut (
        .iCur(iCur), .iComp(iComp), .oDutyInc(oDutyInc)
    );

    task chk(input integer cur, input integer expd);
        begin
            iCur = cur;
            #2;
            g = oDutyInc;
            if (g !== expd) begin
                $display("FAIL iCur=%0d -> oDutyInc=%0d exp=%0d", cur, g, expd);
                fail = fail + 1;
            end
        end
    endtask

    initial begin
        chk(  5000,  200);   // 正电流 -> +comp
        chk(-  5000, -200);  // 负电流 -> -comp
        chk( 32767,  200);
        chk(-32768, -200);   // 负满量程(abs 饱和处理)
        chk(     5,    0);   // 近零(阈值 8)不补偿
        chk(     0,    0);   // 零电流不补偿
        chk(    -6,    0);
        chk( -4000, -200);
        if (fail == 0)
            $display("PASS CompDeadTime: 电流为正加 comp, 为负减 comp, 近零不补偿");
        else
            $display("FAIL CompDeadTime: %0d", fail);
        $finish;
    end
endmodule
