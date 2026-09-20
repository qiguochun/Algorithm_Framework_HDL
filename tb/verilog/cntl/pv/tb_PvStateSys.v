//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   tb_PvStateSys.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
/*Description       :   PvStateSys 单元 TB：IDLE→SELF_CHK→START→SELF_CHK→
//                      NORMAL→FAULT→IDLE 全遍历(小参数加速)
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

module TbPvStateSys;
    reg iSysClk = 0;
    reg iSysRst = 0;
    always #5 iSysClk = ~iSysClk;

    reg tick = 0, en_run = 0, is_err = 0, bus_fault = 0;
    reg [15:0] bus = 0;
    wire [2:0] state;
    integer i, fail = 0;

    PvStateSys #(.W(16), .BUS_OK_V(30), .BUS_LO_V(20),
                 .SELF_HOLD(5), .START_HOLD(5)) U_Dut (
        .iSysClk(iSysClk), .iSysRst(iSysRst), .iTick(tick), .iEnRun(en_run),
        .iIsErr(is_err), .iBusFault(bus_fault), .iBusVolt(bus),
        .oState(state));

    // 打 n 拍
    task pulse(input integer n);
        integer j;
        for (j = 0; j < n; j = j + 1) begin
            @(posedge iSysClk); tick = 1;
            @(posedge iSysClk); tick = 0;
        end
    endtask

    task chk(input integer step, input integer want);
        if (state !== want[2:0]) begin
            $display("FAIL step%0d state=%0d want=%0d", step, state, want);
            fail = fail + 1;
        end else
            $display("  step%0d state=%0d ok", step, state);
    endtask

    initial begin
        iSysRst = 1;
        for (i = 0; i < 3; i = i + 1) @(posedge iSysClk);
        iSysRst = 0;
        chk(0, 0);                       // IDLE

        en_run = 1; bus = 10;
        pulse(1); chk(1, 1);             // → SELF_CHK
        pulse(1); chk(2, 2);             // bus<=BUS_LO → START
        bus = 35;
        pulse(6); chk(3, 1);             // START hold 满 → SELF_CHK
        pulse(6); chk(4, 3);             // SELF_CHK bus ok 满 → NORMAL
        is_err = 1;
        pulse(1); chk(5, 4);             // → FAULT
        is_err = 0; en_run = 0;
        pulse(1); chk(6, 0);             // 故障清 → IDLE

        if (fail == 0) begin
            $display("PASS PvStateSys traversal");
            $finish;
        end
        $display("FAIL PvStateSys: %0d", fail);
        $finish;
    end
endmodule
