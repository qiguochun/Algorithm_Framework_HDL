//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   tb_PvStateCh.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
/*Description       :   PvStateCh 单元 TB：NO_PV→STARTING→WORKING→掉压回
//                      NO_PV，以及 FAULT 恢复路径(小参数加速)
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

module TbPvStateCh;
    reg iSysClk = 0;
    reg iSysRst = 0;
    always #5 iSysClk = ~iSysClk;

    reg tick = 0, is_err = 0;
    reg [15:0] v = 0, vave = 0;
    wire [1:0] state;
    wire o_valid;
    integer i, fail = 0;

    PvStateCh #(.W(16), .CH_MIN_V(15), .CH_VALID_TICKS(10)) U_Dut (
        .iSysClk(iSysClk), .iSysRst(iSysRst), .iTick(tick), .iIsErr(is_err),
        .iVInst(v), .iVAve(vave), .oState(state), .oValid(o_valid));

    task pulse(input integer n);
        integer j;
        for (j = 0; j < n; j = j + 1) begin
            @(posedge iSysClk); tick = 1;
            @(posedge iSysClk); tick = 0;
        end
    endtask

    task chk(input integer step, input integer want);
        if (state !== want[1:0]) begin
            $display("FAIL step%0d state=%0d want=%0d", step, state, want);
            fail = fail + 1;
        end else
            $display("  step%0d state=%0d ok (valid=%0d)", step, state, o_valid);
    endtask

    initial begin
        iSysRst = 1;
        for (i = 0; i < 3; i = i + 1) @(posedge iSysClk);
        iSysRst = 0;

        v = 5; vave = 5;
        pulse(1); chk(0, 0);          // NO_PV (v<min)

        v = 20; vave = 20;
        pulse(1); chk(1, 1);          // → STARTING
        pulse(10); chk(2, 2);         // 停留满 → WORKING
        if (o_valid !== 1'b1) begin $display("FAIL o_valid not set"); fail = fail + 1; end

        v = 5;
        pulse(1); chk(3, 0);          // 掉压 → NO_PV

        v = 20;
        pulse(1); chk(4, 1);          // → STARTING
        is_err = 1;
        pulse(1); chk(5, 3);          // → FAULT
        is_err = 0;
        pulse(1); chk(6, 1);          // 故障清 & v 够 → STARTING

        if (fail == 0) begin
            $display("PASS PvStateCh traversal");
            $finish;
        end
        $display("FAIL PvStateCh: %0d", fail);
        $finish;
    end
endmodule
