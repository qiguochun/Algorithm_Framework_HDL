//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   PvStateCh.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
/*Description       :   通道级FSM(NO_PV/STARTING/WORKING/FAULT)，工作态判据与Voc锁存
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

module PvStateCh #(
    parameter integer W              = `ALGO_W_VOLT,
    parameter integer CH_MIN_V       = 70,      // 通道最低工作电压 (C)
    parameter integer CH_VALID_TICKS = 25000    // STARTING→WORKING 停留 (5 s@5k)
) (
    input  wire             iSysClk,
    input  wire             iSysRst,       // 高有效复位
    input  wire             iTick,         // 5 kHz 节拍
    input  wire             iIsErr,        // 本通道故障
    input  wire [W-1:0]     iVInst,        // 瞬时电压
    input  wire [W-1:0]     iVAve,         // 平均电压(观测)
    output reg  [1:0]       oState,        // 0=NO_PV 1=STARTING 2=WORKING 3=FAULT
    output reg              oValid,        // 通道有效(可工作)
    output reg  [31:0]      oValidCnt,     // STARTING 停留拍(观测)
    output reg  [W-1:0]     oVocEst        // 进入 WORKING 时锁存的电压(≈Voc 起始)
);
    localparam [1:0] NO_PV    = 2'd0;
    localparam [1:0] STARTING = 2'd1;
    localparam [1:0] WORKING  = 2'd2;
    localparam [1:0] FAULT    = 2'd3;

    // ================= 1) 现态寄存器（含停留计数/Voc 锁存，iTick 节拍推进） =================
    reg [1:0]   rCurrentState;
    reg [31:0]  rValidCnt;
    reg [W-1:0] rVocEst;
    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            rCurrentState <= NO_PV;
            rValidCnt     <= 32'd0;
            rVocEst       <= {W{1'b0}};
        end else if (iTick) begin
            rCurrentState <= rNextState;
            rValidCnt     <= rNextCnt;
            rVocEst       <= rNextVoc;
        end
    end

    // ================= 2) 次态组合逻辑（完备 default，异常回 NO_PV） =================
    reg [1:0]   rNextState;
    reg [31:0]  rNextCnt;
    reg [W-1:0] rNextVoc;
    always @(*) begin
        rNextState = rCurrentState;             // 默认保持
        rNextCnt   = 32'd0;
        rNextVoc   = rVocEst;                   // 默认保持 Voc
        case (rCurrentState)
            NO_PV: begin
                if (iIsErr)
                    rNextState = FAULT;
                else if (iVInst >= CH_MIN_V[W-1:0])
                    rNextState = STARTING;
            end
            STARTING: begin
                if (iIsErr) begin
                    rNextState = FAULT;
                end else if (iVInst < CH_MIN_V[W-1:0]) begin
                    rNextState = NO_PV;
                end else if (rValidCnt >= CH_VALID_TICKS - 1) begin
                    rNextState = WORKING;       // 进入工作，锁定 Voc 起始
                    rNextVoc   = iVAve;
                end else begin
                    rNextState = STARTING;      // 停留累计
                    rNextCnt   = rValidCnt + 32'd1;
                end
            end
            WORKING: begin
                if (iIsErr)
                    rNextState = FAULT;
                else if (iVInst < CH_MIN_V[W-1:0])
                    rNextState = NO_PV;
            end
            FAULT: begin
                if (!iIsErr)
                    rNextState = (iVInst >= CH_MIN_V[W-1:0]) ? STARTING : NO_PV;
            end
            default: begin
                rNextState = NO_PV;             // 异常/未定义状态回 NO_PV
            end
        endcase
    end

    // ================= 3) 输出逻辑（组合，随现态稳定） =================
    always @(*) begin
        oState    = rCurrentState;
        oValid    = (rCurrentState == WORKING) ? 1'b1 : 1'b0;
        oValidCnt = rValidCnt;
        oVocEst   = rVocEst;
    end
endmodule
