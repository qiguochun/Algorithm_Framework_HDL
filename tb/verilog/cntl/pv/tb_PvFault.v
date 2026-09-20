//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   tb_PvFault.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
/*Description       :   PvFault 单元 TB：正常→过流锁存→短路锁存(含反接)→
//                      欠压告警回差，复位不撤锁存
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

module TbPvFault;
    reg iSysClk = 0;
    reg iSysRst = 0;
    always #5 iSysClk = ~iSysClk;

    reg chk_en = 0;
    reg [15:0] v_inst = 0;
    reg [15:0] v_ave  = 0;
    reg [15:0] curr_inst = 0;
    wire over_current, short_cut, reverse, vol_low, vol_high, is_err;

    integer fail = 0;

    PvFault #(.W(16)) U_Dut (
        .iSysClk(iSysClk), .iSysRst(iSysRst), .iChkEn(chk_en),
        .iVInst(v_inst), .iVAve(v_ave), .iCurrInst(curr_inst),
        .oOverCurrent(over_current), .oShortCut(short_cut),
        .oReverse(reverse), .oVolLow(vol_low), .oVolHigh(vol_high),
        .oIsErr(is_err));

    // 打 n 个 chk 节拍
    task pulse(input integer n);
        integer i;
        for (i = 0; i < n; i = i + 1) begin
            @(posedge iSysClk); chk_en <= 1;
            @(posedge iSysClk); chk_en <= 0;
        end
    endtask

    initial begin
        iSysRst = 1;
        repeat (3) @(posedge iSysClk);
        iSysRst = 0;

        // 1) 正常点 V=80 I=5A
        v_inst = 80; v_ave = 80; curr_inst = 5000;
        pulse(2);
        $display("1) normal  is_err=%0b (want 0)", is_err);
        if (is_err !== 1'b0) fail = fail + 1;

        // 2) 过流 18A → 锁存，回落仍保持
        curr_inst = 18000;
        pulse(2);
        $display("2) overcur is_err=%0b oc=%0b (want 1,1)", is_err, over_current);
        if (over_current !== 1'b1) fail = fail + 1;
        curr_inst = 5000;
        pulse(2);
        $display("2b) latched oc=%0b (want 1)", over_current);
        if (over_current !== 1'b1) fail = fail + 1;

        // 3) 短路反接：v=2、I 幅值>20A 且为负(0xB000=-20480)
        v_inst = 2; curr_inst = 16'hB000;
        pulse(2);
        $display("3) short+reverse sc=%0b rev=%0b (want 1,1)", short_cut, reverse);
        if (short_cut !== 1'b1 || reverse !== 1'b1) fail = fail + 1;

        // 4) 欠压回差：60→71(保)→75(清)
        v_inst = 80; curr_inst = 0;
        v_ave = 60; pulse(2);
        $display("4a) low set vl=%0b (want 1)", vol_low);
        if (vol_low !== 1'b1) fail = fail + 1;
        v_ave = 71; pulse(2);
        $display("4b) low keep(71) vl=%0b (want 1)", vol_low);
        if (vol_low !== 1'b1) fail = fail + 1;
        v_ave = 75; pulse(2);
        $display("4c) low reset(75) vl=%0b (want 0)", vol_low);
        if (vol_low !== 1'b0) fail = fail + 1;

        if (fail == 0) begin
            $display("PASS PvFault scenario");
            $finish;
        end
        $display("FAIL PvFault: %0d checks", fail);
        $finish;
    end
endmodule
