//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   Inv3phState.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.20
/*Description       :   三相逆变器系统状态机（IDLE…CONNECTED…ERR）。
                        状态: IDLE/ERR/SELFCHECK/STARTING/DISCONNECTED/
                              CONNECTING/CONNECTED/DISCONNECTING。
                        三段式：现态寄存器 + 次态组合 + 输出寄存。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.1
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
`include "inv3ph_cfg.vh"

module Inv3phState #(
    parameter integer SELF_HOLD  = 1000,   // 自检最短停留拍（@控制节拍）
    parameter integer START_HOLD = 20000   // 软启最短停留拍（可用 soft_done 提前）
) (
    input  wire        iSysClk,
    input  wire        iSysRst,
    input  wire        iTick,              // 控制节拍
    input  wire        iEnRun,
    input  wire        iEnGrid,
    input  wire        iEnGridRun,
    input  wire        iIsErr,
    input  wire        iSelfCheckPass,
    input  wire        iSoftStartDone,
    input  wire        iSyncOk,            // 并网同步完成
    output reg  [2:0]  oState,
    output reg         oPwmEn,
    output reg         oRlyEn,
    output reg         oGridMode           // 1=并网电流控制, 0=离网 V/F
);
    localparam [2:0] ST_IDLE          = `INV3PH_ST_IDLE;
    localparam [2:0] ST_ERR           = `INV3PH_ST_ERR;
    localparam [2:0] ST_SELFCHECK     = `INV3PH_ST_SELFCHECK;
    localparam [2:0] ST_STARTING      = `INV3PH_ST_STARTING;
    localparam [2:0] ST_DISCONNECTED  = `INV3PH_ST_DISCONNECTED;
    localparam [2:0] ST_CONNECTING    = `INV3PH_ST_CONNECTING;
    localparam [2:0] ST_CONNECTED     = `INV3PH_ST_CONNECTED;
    localparam [2:0] ST_DISCONNECTING = `INV3PH_ST_DISCONNECTING;

    reg [2:0]  rCur;
    reg [31:0] rHold;
    reg [2:0]  rNxt;
    reg [31:0] rNxtHold;

    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            rCur  <= ST_IDLE;
            rHold <= 32'd0;
        end else if (iTick) begin
            rCur  <= rNxt;
            rHold <= rNxtHold;
        end
    end

    always @(*) begin
        rNxt     = rCur;
        rNxtHold = rHold + 32'd1;
        case (rCur)
            ST_IDLE: begin
                rNxtHold = 32'd0;
                if (iIsErr)        rNxt = ST_ERR;
                else if (iEnRun)   rNxt = ST_SELFCHECK;
            end
            ST_ERR: begin
                rNxtHold = 32'd0;
                if (!iIsErr && !iEnRun) rNxt = ST_IDLE;
            end
            ST_SELFCHECK: begin
                if (iIsErr) begin
                    rNxt = ST_ERR; rNxtHold = 32'd0;
                end else if (!iEnRun) begin
                    rNxt = ST_IDLE; rNxtHold = 32'd0;
                end else if (iSelfCheckPass && rHold >= SELF_HOLD) begin
                    rNxt = ST_STARTING; rNxtHold = 32'd0;
                end
            end
            ST_STARTING: begin
                if (iIsErr) begin
                    rNxt = ST_ERR; rNxtHold = 32'd0;
                end else if (!iEnRun) begin
                    rNxt = ST_IDLE; rNxtHold = 32'd0;
                end else if (iSoftStartDone || rHold >= START_HOLD) begin
                    rNxt = ST_DISCONNECTED; rNxtHold = 32'd0;
                end
            end
            ST_DISCONNECTED: begin
                if (iIsErr) begin
                    rNxt = ST_ERR; rNxtHold = 32'd0;
                end else if (!iEnRun) begin
                    rNxt = ST_IDLE; rNxtHold = 32'd0;
                end else if (iEnGrid) begin
                    rNxt = ST_CONNECTING; rNxtHold = 32'd0;
                end
            end
            ST_CONNECTING: begin
                if (iIsErr) begin
                    rNxt = ST_ERR; rNxtHold = 32'd0;
                end else if (!iEnGrid || !iEnRun) begin
                    rNxt = ST_DISCONNECTED; rNxtHold = 32'd0;
                end else if (iEnGridRun && iSyncOk) begin
                    rNxt = ST_CONNECTED; rNxtHold = 32'd0;
                end
            end
            ST_CONNECTED: begin
                if (iIsErr) begin
                    rNxt = ST_ERR; rNxtHold = 32'd0;
                end else if (!iEnGridRun || !iEnGrid) begin
                    rNxt = ST_DISCONNECTING; rNxtHold = 32'd0;
                end else if (!iEnRun) begin
                    rNxt = ST_IDLE; rNxtHold = 32'd0;
                end
            end
            ST_DISCONNECTING: begin
                if (iIsErr) begin
                    rNxt = ST_ERR; rNxtHold = 32'd0;
                end else if (!iEnRun) begin
                    rNxt = ST_IDLE; rNxtHold = 32'd0;
                end else begin
                    rNxt = ST_DISCONNECTED; rNxtHold = 32'd0;
                end
            end
            default: begin
                rNxt = ST_IDLE; rNxtHold = 32'd0;
            end
        endcase
    end

    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            oState    <= ST_IDLE;
            oPwmEn    <= 1'b0;
            oRlyEn    <= 1'b0;
            oGridMode <= 1'b0;
        end else if (iTick) begin
            oState <= rNxt;
            case (rNxt)
                ST_STARTING, ST_DISCONNECTED, ST_CONNECTING,
                ST_CONNECTED, ST_DISCONNECTING: oPwmEn <= 1'b1;
                default: oPwmEn <= 1'b0;
            endcase
            oRlyEn    <= (rNxt == ST_CONNECTED) || (rNxt == ST_DISCONNECTING);
            oGridMode <= (rNxt == ST_CONNECTED) || (rNxt == ST_DISCONNECTING);
        end
    end
endmodule
