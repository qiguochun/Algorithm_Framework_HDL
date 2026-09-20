//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   Inv3phFault.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.20
/*Description       :   三相逆变器故障快检（母线/相电压电流过限锁存）。
                        母线欠压/过压(均值+峰值)、相电压瞬时过压、相电流瞬时过流、
                        短路(过流且相电压 RMS 偏低)。故障位锁存，复位清零。
                        量纲: 电压 V 整数，电流 A 整数（与 inv3ph_cfg 阈值一致）。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.1
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
`include "inv3ph_cfg.vh"

module Inv3phFault #(
    parameter integer W                = 16,
    parameter integer BUS_LOW          = `INV3PH_ERR_BUS_VOL_LOW,
    parameter integer BUS_OVER         = `INV3PH_ERR_BUS_VOL_OVER,
    parameter integer BUS_PEAK         = `INV3PH_ERR_BUS_PEAK_OVER,
    parameter integer VOL_INST_OVER    = `INV3PH_ERR_INV_VOL_INST_OVER,
    parameter integer CUR_INST_OVER    = `INV3PH_ERR_INV_CUR_INST_OVER,
    parameter integer VOL_RMS_LOW      = `INV3PH_ERR_INV_VOL_RMS_LOW,
    parameter integer BUS_OVER_DLY     = 20
) (
    input  wire            iSysClk,
    input  wire            iSysRst,
    input  wire            iChkEn,          // 快检节拍
    input  wire            iEnRun,          // 运行使能（母线类故障需使能）
    input  wire [W-1:0]    iBusAve,         // 总母线均值 V
    input  wire [W-1:0]    iBusPosAve,
    input  wire [W-1:0]    iBusNegAve,
    input  wire [W-1:0]    iBusInst,
    input  wire [W-1:0]    iBusPosInst,
    input  wire [W-1:0]    iBusNegInst,
    input  wire signed [W-1:0] iVaInst,
    input  wire signed [W-1:0] iVbInst,
    input  wire signed [W-1:0] iVcInst,
    input  wire signed [W-1:0] iIaInst,
    input  wire signed [W-1:0] iIbInst,
    input  wire signed [W-1:0] iIcInst,
    input  wire [W-1:0]    iVaRms,
    input  wire [W-1:0]    iVbRms,
    input  wire [W-1:0]    iVcRms,
    output reg             oBusVolLow,
    output reg             oBusVolOver,
    output reg             oInvVolOver,
    output reg             oInvCurOver,
    output reg             oInvShort,
    output wire            oIsErr
);
    function automatic [W-1:0] f_abs16;
        input signed [W-1:0] x;
        begin
            if (x < 0) f_abs16 = -x;
            else       f_abs16 = x;
        end
    endfunction

    reg [15:0] rBusOverCnt;

    wire wBusLow = iEnRun && (
        (iBusAve < BUS_LOW[W-1:0]) ||
        (iBusPosAve < (BUS_LOW[W-1:0] >> 1)) ||
        (iBusNegAve < (BUS_LOW[W-1:0] >> 1)));

    wire wBusPeak = iEnRun && (
        (iBusInst > BUS_PEAK[W-1:0]) ||
        (iBusPosInst > (BUS_PEAK[W-1:0] >> 1)) ||
        (iBusNegInst > (BUS_PEAK[W-1:0] >> 1)));

    wire wBusAveOver = iEnRun && (
        (iBusAve > BUS_OVER[W-1:0]) ||
        (iBusPosAve > (BUS_OVER[W-1:0] >> 1)) ||
        (iBusNegAve > (BUS_OVER[W-1:0] >> 1)));

    wire wVolOver =
        (f_abs16(iVaInst) > VOL_INST_OVER[W-1:0]) ||
        (f_abs16(iVbInst) > VOL_INST_OVER[W-1:0]) ||
        (f_abs16(iVcInst) > VOL_INST_OVER[W-1:0]);

    wire wCurOver =
        (f_abs16(iIaInst) > CUR_INST_OVER[W-1:0]) ||
        (f_abs16(iIbInst) > CUR_INST_OVER[W-1:0]) ||
        (f_abs16(iIcInst) > CUR_INST_OVER[W-1:0]);

    wire wVolLowRms =
        (iVaRms < VOL_RMS_LOW[W-1:0]) ||
        (iVbRms < VOL_RMS_LOW[W-1:0]) ||
        (iVcRms < VOL_RMS_LOW[W-1:0]);

    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            oBusVolLow  <= 1'b0;
            oBusVolOver <= 1'b0;
            oInvVolOver <= 1'b0;
            oInvCurOver <= 1'b0;
            oInvShort   <= 1'b0;
            rBusOverCnt <= 16'd0;
        end else if (iChkEn) begin
            if (wBusLow) oBusVolLow <= 1'b1;
            if (wBusPeak) oBusVolOver <= 1'b1;
            if (wBusAveOver) begin
                if (rBusOverCnt >= BUS_OVER_DLY[15:0])
                    oBusVolOver <= 1'b1;
                else
                    rBusOverCnt <= rBusOverCnt + 16'd1;
            end else
                rBusOverCnt <= 16'd0;
            if (wVolOver) oInvVolOver <= 1'b1;
            if (wCurOver) oInvCurOver <= 1'b1;
            if (wCurOver && wVolLowRms) oInvShort <= 1'b1;
        end
    end

    assign oIsErr = oBusVolLow | oBusVolOver | oInvVolOver | oInvCurOver | oInvShort;
endmodule
