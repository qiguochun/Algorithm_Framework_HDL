//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   tb_PvLoop.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
/*Description       :   PvLoop 单元 TB：内联一阶虚拟对象闭环收敛校验
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.1
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
`timescale 1ns/1ps
`include "pv_cfg.vh"
`include "algo_types.vh"

module TbPvLoop;
    localparam W        = `ALGO_W_VOLT;
    localparam VREF     = `PV_SCAFFOLD_VREF;   // 20000
    localparam CTRL_DIV = 100;                 // 虚拟节拍

    reg iSysClk = 0;
    reg iSysRst = 0;
    always #5 iSysClk = ~iSysClk;

    reg [31:0] div_cnt = 0;
    reg iSampleEn = 0;
    wire [7:0] oDuty;
    reg [W-1:0] iVMeas = 0;

    // 虚拟对象（组合）+ 时序更新 —— 与 demo rtl/plant.v 相同数学
    wire [W-1:0] tgt = {1'b0, oDuty, 7'b0};                       // duty*128
    wire signed [W:0] perr = $signed({1'b0, tgt}) - $signed({1'b0, iVMeas});
    wire signed [W:0] pnext = $signed({1'b0, iVMeas}) + (perr >>> 3);

    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst)
            iVMeas <= {W{1'b0}};
        else if (iSampleEn) begin
            if (pnext <= 0) iVMeas <= {W{1'b0}};
            else if (pnext >= $signed({1'b0, {W{1'b1}}})) iVMeas <= {W{1'b1}};
            else iVMeas <= pnext[W-1:0];
        end
    end

    PvLoop #(.W(W), .VREF(VREF), .KP_SHIFT(9), .KI_SHIFT(10)) U_Dut (
        .iSysClk(iSysClk), .iSysRst(iSysRst), .iSampleEn(iSampleEn),
        .iVMeas(iVMeas), .oDuty(oDuty)
    );

    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            div_cnt <= 0; iSampleEn <= 0;
        end else begin
            if (div_cnt == CTRL_DIV-1) begin div_cnt <= 0; iSampleEn <= 1; end
            else begin div_cnt <= div_cnt + 1; iSampleEn <= 0; end
        end
    end

    initial begin
        iSysRst = 1;
        repeat (5) @(posedge iSysClk);
        iSysRst = 0;
        repeat (CTRL_DIV * 3000) @(posedge iSysClk);   // 3000 个控制周期
        $display("SIM_DONE duty=%0d v_meas=%0d (VREF=%0d)", oDuty, iVMeas, VREF);
        if (iVMeas > 16'd19500 && iVMeas < 16'd20500)
            $display("PASS PvLoop converged to VREF (no steady-state offset)");
        else begin
            $display("FAIL v_meas=%0d expect ~%0d", iVMeas, VREF);
            $finish;
        end
        $finish;
    end
endmodule
