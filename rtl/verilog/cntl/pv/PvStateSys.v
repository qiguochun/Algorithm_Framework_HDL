//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   PvStateSys.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
/*Description       :   系统级FSM(IDLE/SELF_CHK/START/NORMAL/FAULT)，母线判定
//                      三段式（§8）：现态寄存器 + 次态组合 + 输出逻辑
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.2
//modifier          :   HDL-Auto
//Modify Date       :   2026.09.05
/*Modify Record     :
//  Rev 0.2: 单 always 状态机重构为 §8 三段式（状态/编码/转移条件不变）。
*/
//------------------------------------------------------------------------------
`include "algo_types.vh"

module PvStateSys #(
    parameter integer W             = `ALGO_W_VOLT,
    parameter integer BUS_OK_V      = 670,    // 母线“可工作”阈值 (C: 700-30)
    parameter integer BUS_LO_V      = 620,    // 母线“偏低”阈值 (C)
    parameter integer SELF_HOLD     = 1000,   // SELF_CHK 停留拍 (C 0.2*FS@5k)
    parameter integer START_HOLD    = 10000   // START 停留拍 (C 2*FS@5k)
) (
    input  wire             iSysClk,
    input  wire             iSysRst,       // 高有效复位
    input  wire             iTick,         // 5 kHz 节拍
    input  wire             iEnRun,        // 系统使能
    input  wire             iIsErr,        // 通道故障聚合(任一通道)
    input  wire             iBusFault,     // 母线故障(锁存于 pv_fault/bus)
    input  wire [W-1:0]     iBusVolt,      // 母线电压(V)
    output reg  [2:0]       oState,        // 0=IDLE 1=SELF_CHK 2=START 3=NORMAL 4=FAULT
    output reg  [31:0]      oHoldCnt       // 当前状态停留拍(观测)
);
    localparam [2:0] IDLE    = 3'd0;
    localparam [2:0] SELF_CHK= 3'd1;
    localparam [2:0] START   = 3'd2;
    localparam [2:0] NORMAL  = 3'd3;
    localparam [2:0] FAULT   = 3'd4;

    wire wFaultNow = iIsErr | iBusFault;

    // ================= 1) 现态寄存器（含停留计数，iTick 节拍推进） =================
    reg [2:0]  rCurrentState;
    reg [31:0] rHoldCnt;
    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            rCurrentState <= IDLE;
            rHoldCnt      <= 32'd0;
        end else if (iTick) begin
            rCurrentState <= rNextState;
            rHoldCnt      <= rNextCnt;
        end
    end

    // ================= 2) 次态组合逻辑（完备 default，异常回 IDLE） =================
    reg [2:0]  rNextState;
    reg [31:0] rNextCnt;
    always @(*) begin
        rNextState = rCurrentState;             // 默认保持
        rNextCnt   = rHoldCnt;
        case (rCurrentState)
            IDLE: begin
                rNextCnt = 32'd0;
                if (wFaultNow)              rNextState = FAULT;
                else if (iEnRun)            rNextState = SELF_CHK;
            end
            SELF_CHK: begin
                if (wFaultNow) begin
                    rNextState = FAULT;  rNextCnt = 32'd0;
                end else if (iBusVolt >= BUS_OK_V && rHoldCnt >= SELF_HOLD) begin
                    rNextState = NORMAL; rNextCnt = 32'd0;
                end else if (iBusVolt <= BUS_LO_V) begin
                    rNextState = START;  rNextCnt = 32'd0;
                end else begin
                    rNextCnt = rHoldCnt + 32'd1;      // 停留累计
                end
            end
            START: begin
                if (wFaultNow) begin
                    rNextState = FAULT;   rNextCnt = 32'd0;
                end else if (iBusVolt >= BUS_OK_V && rHoldCnt >= START_HOLD) begin
                    rNextState = SELF_CHK; rNextCnt = 32'd0;
                end else begin
                    rNextCnt = rHoldCnt + 32'd1;      // 停留累计
                end
            end
            NORMAL: begin
                rNextCnt = 32'd0;
                if (wFaultNow)              rNextState = FAULT;
            end
            FAULT: begin
                rNextCnt = 32'd0;
                if (!wFaultNow)             rNextState = IDLE;
            end
            default: begin
                rNextState = IDLE;                 // 异常/未定义状态回 IDLE
                rNextCnt   = 32'd0;
            end
        endcase
    end

    // ================= 3) 输出逻辑（组合，随现态稳定） =================
    always @(*) begin
        oState   = rCurrentState;
        oHoldCnt = rHoldCnt;
    end
endmodule
