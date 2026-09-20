//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   tb_CompKalman1d.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
/*Description       :   CompKalman1d 单元 TB（自包含）
                        量测 = 常数 22000（含固定初值偏差）。期望: Out 收敛到 m（末拍 |err|<50）。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.1
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
`timescale 1ns/1ps

module TbCompKalman1d;
    localparam integer M_CONST = 22000;  // 常数量测
    localparam integer N_STEP  = 1600;   // 步数

    reg clk = 0;
    reg iSysRst = 1;
    reg en = 0;
    reg signed [15:0] m = 0;
    wire signed [15:0] y;

    CompKalman1d U_Dut (
        .iSysClk(clk), .iSysRst(iSysRst), .iEn(en), .iMIn(m), .oYOut(y)
    );

    always #5 clk = ~clk;

    integer i;
    integer fail = 0;
    reg signed [31:0] err;

    initial begin
        iSysRst = 1; en = 1; m = M_CONST;
        repeat (3) @(posedge clk);
        iSysRst = 0;
        for (i = 0; i < N_STEP; i = i + 1) @(posedge clk);
        err = M_CONST - y;
        $display("KALM  y=%0d (m=%0d) err=%0d", y, M_CONST, err);
        if (err > -50 && err < 50)
            $display("PASS kalman1d: estimate converges to constant m");
        else begin
            $display("FAIL kalman1d: err too large");
            fail = 1;
        end
        if (fail) $finish;
        $finish;
    end
endmodule
