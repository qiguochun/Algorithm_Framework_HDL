//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   PvLoop.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
/*Description       :   离散位置式PI电压/功率环，输出duty限幅[0,255]，抗饱和
//                       多拍流水改造: 采样节拍 iSampleEn 快照 Vmeas ->
//                       err(一拍) -> 比例/积分项+求和(一拍) ->
//                       抗饱和判断+积分/输出提交(一拍), oValid 于提交拍拉高。
//                       每步一寄存器、无单周期组合乘加；数值与原组合实现一致
//                       (求和用提交前的旧 rAcc, 提交拍才更新 rAcc)。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.2
//modifier          :
//Modify Date       :
/*Modify Record     :
//  Rev 0.2: 组合乘加链拆分 rEv 多拍流水, 新增 oValid。
*/
//------------------------------------------------------------------------------
`include "algo_types.vh"

module PvLoop #(
    parameter integer W          = `ALGO_W_VOLT,   // 反馈位宽 16
    parameter integer VREF       = 20000,          // 设定点
    parameter integer KP_SHIFT   = 9,              // Kp = 2^-KP_SHIFT
    parameter integer KI_SHIFT   = 10,             // Ki = 2^-KI_SHIFT
    parameter integer DUTY_W     = `ALGO_W_DUTY,   // 输出位宽 8
    parameter integer DUTY_RESET = 64,             // 复位占空比
    parameter integer DUTY_MAX   = 255
) (
    input  wire              iSysClk,
    input  wire              iSysRst,       // 高有效复位
    input  wire              iSampleEn,     // 控制周期 strobe
    input  wire [W-1:0]      iVMeas,        // 反馈（无量纲满量程域）
    output reg  [DUTY_W-1:0] oDuty,
    output reg               oValid         // 提交拍有效脉冲(采样后 LATENCY=3 拍)
);
    localparam signed [31:0] VREF_S = VREF;

    // ---- 多拍事件移位 rEv: [0]=算err, [1]=求和, [2]=提交 ----
    reg [2:0] rEv;
    reg       rBusy;                          // 流水忙(一次采样处理中)
    reg [W-1:0] rV;                           // 采样快照 Vmeas

    reg signed [31:0] rErr;                   // 误差(一拍)
    reg signed [15:0] rDutySum;               // 求和(一拍)

    reg signed [31:0] rAcc = 32'sd0;          // 积分累加器(提交拍更新)

    // ---------------- 事件移位 + 输入快照 ----------------
    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            rEv   <= 3'b000;
            rBusy <= 1'b0;
            rV    <= {W{1'b0}};
        end else begin
            rEv[2] <= rEv[1];
            rEv[1] <= rEv[0];
            rEv[0] <= 1'b0;
            if (iSampleEn && !rBusy) begin
                rEv[0] <= 1'b1;               // 下拍进入算 err 阶段
                rV     <= iVMeas;             // 采样快照
                rBusy  <= 1'b1;
            end
            if (rEv[2]) rBusy <= 1'b0;        // 提交完成后空闲
        end
    end

    // ---------------- 计算级 ----------------
    // 一拍 err
    wire signed [31:0] wErr = VREF_S - $signed({ {(32-W){1'b0}}, rV });
    // 求和(三拍提交前仍读旧 rAcc: rAcc 提交拍才更新)
    wire signed [15:0] wPterm16 = (rErr >>> KP_SHIFT);
    wire signed [15:0] wIpi16   = (rAcc >>> KI_SHIFT);
    // 抗饱和: 饱和且误差同向 -> 冻结
    wire wSatHold = ((rDutySum <= 16'sd0) && (rErr < 32'sd0)) ||
                    ((rDutySum >= DUTY_MAX[15:0]) && (rErr > 32'sd0));

    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            rErr     <= 32'sd0;
            rDutySum <= 16'sd0;
            rAcc     <= 32'sd0;
            oDuty    <= DUTY_RESET[DUTY_W-1:0];
            oValid   <= 1'b0;
        end else begin
            oValid <= 1'b0;
            if (rEv[0])
                rErr <= wErr;                         // 一拍: 误差寄存
            if (rEv[1])
                rDutySum <= wPterm16 + wIpi16;        // 求和一拍(读旧 rAcc)
            if (rEv[2]) begin
                if (!wSatHold)
                    rAcc <= rAcc + rErr;              // 抗饱和积分
                if (rDutySum <= 16'sd0)
                    oDuty <= {DUTY_W{1'b0}};
                else if (rDutySum >= DUTY_MAX[15:0])
                    oDuty <= DUTY_MAX[DUTY_W-1:0];
                else
                    oDuty <= rDutySum[DUTY_W-1:0];
                oValid <= 1'b1;                       // 输出提交有效
            end
        end
    end
endmodule
