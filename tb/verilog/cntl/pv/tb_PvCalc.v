//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   tb_PvCalc.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
/*Description       :   PvCalc(EMA) 单元 TB：阶跃输入→输出逼近输入
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

module TbPvCalc;
    localparam W = `ALGO_W_VOLT;
    reg iSysClk = 0;
    reg iSysRst = 0;
    always #5 iSysClk = ~iSysClk;
    reg [31:0] div_cnt = 0;
    reg iSampleEn = 0;
    reg [W-1:0] iVIn = 0;
    wire [W-1:0] oVFilt;

    PvCalc #(.W(W), .EMA_SHIFT(6)) U_Dut (   // alpha=1/64 收敛快些
        .iSysClk(iSysClk), .iSysRst(iSysRst), .iSampleEn(iSampleEn),
        .iVIn(iVIn), .oVFilt(oVFilt)
    );

    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin div_cnt <= 0; iSampleEn <= 0; end
        else begin
            if (div_cnt == 9) begin div_cnt <= 0; iSampleEn <= 1; end
            else begin div_cnt <= div_cnt + 1; iSampleEn <= 0; end
        end
    end

    initial begin
        iSysRst = 1;
        repeat (5) @(posedge iSysClk);
        iSysRst = 0;
        iVIn = 20000;
        repeat (5000) @(posedge iSysClk);        // 500 个 sample_en
        $display("v_in=%0d v_filt=%0d", iVIn, oVFilt);
        if (oVFilt > 16'd19000 && oVFilt < 16'd21000)
            $display("PASS PvCalc EMA tracks input");
        else begin
            $display("FAIL v_filt=%0d", oVFilt);
            $finish;
        end
        $finish;
    end
endmodule
