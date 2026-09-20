//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   AlgoSqrt.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.06
/*Description       :   整数开方库（多拍逐位迭代，iStart/oDone 握手，可综合）
//                      输出 = floor(sqrt(iNum))。业务禁止组合开方，统一例化本库(§6.3)。
//  接口: iStart 脉冲启动 -> oBusy=1 期间逐拍迭代(OW 拍) -> oDone 拉一拍后采 oRoot。
//        busy 期间 iStart 被忽略，调用方须等 oDone 后再发新请求。
//  算法: 移位恢复开方（radix-2），中间量位宽 MW=max(IW,2*OW) 参数化。
//  断言: OW 须 >= ceil(IW/2)，否则仿真 $finish 报错（错误传参不静默）。
//  延迟: OW 拍。
//  修订: 2026-09-08 v0.2（参数化位宽/消移位越界/rCnt $clog2/加参数断言）
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.1
//modifier          :
//Modify Date       :
/*Modify Record     :
                        余数为-当前根平方;试减量: 假设下一位取1，平方需增加的数;
                        余数-试减量为1时，该位取1
*/
//------------------------------------------------------------------------------

`include "algo_types.vh"

module AlgoSqrt #(
    parameter integer IW = 64,     // 输入位宽（无符号）
    parameter integer OW = 32      // 输出位宽 = ceil(IW/2)，迭代拍数 = OW
) (
    input  wire             iSysClk,
    input  wire             iSysRst,
    input  wire             iStart,        // 启动脉冲（一拍）
    input  wire [IW-1:0]    iNum,          // 被开方数（无符号）
    output reg              oBusy,         // 忙标志
    output reg              oDone,         // 完成脉冲（一拍）
    output reg  [OW-1:0]    oRoot          // floor(sqrt(iNum))
);
    // 参数守卫：OW < ceil(IW/2) 时仿真直接失败
    initial begin
        if (OW * 2 < IW) begin
            $display("ERROR AlgoSqrt: OW=%0d < ceil(IW/2) for IW=%0d", OW, IW);
            $finish;
        end
    end

    localparam [1:0] S_IDLE = 2'd0, S_BUSY = 2'd1, S_DONE = 2'd2;

    // 中间量统一位宽：够存被开方量(IW) 与 试位 2^(2*(OW-1))（需 2*OW 位）
    localparam integer MW = (IW > 2*OW) ? IW : 2*OW;

    reg [1:0]   rState;
    reg [$clog2(OW+1)-1:0] rCnt;     // 迭代计数，从 OW 减到 1（共 OW 次迭代）
    reg [MW-1:0] rVal;               // 剩余被开方量
    reg [MW-1:0] rOne;               // 当前试位 2^(2k)
    reg [MW-1:0] rRes;               // 已得结果

    // 组合：res + one（复用，减少一次加法；关键路径 = 一次 MW 位加法 + 比较）
    wire [MW-1:0] wSum = rRes + rOne;

    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            rState <= S_IDLE; oBusy <= 1'b0; oDone <= 1'b0; oRoot <= {OW{1'b0}};
        end else begin
            oDone <= 1'b0;
            case (rState)
                S_IDLE: begin
                    oBusy <= 1'b0;
                    if (iStart) begin
                        rVal  <= iNum;                                   // IW -> MW 零扩展
                        rRes  <= {MW{1'b0}};
                        rOne  <= {1'b1, {(2*(OW-1)){1'b0}}};             // 2^(2*(OW-1))，无移位越界
                        rCnt  <= OW;
                        rState <= S_BUSY;
                        oBusy  <= 1'b1;
                    end
                end
                S_BUSY: begin
                    oBusy <= 1'b1;
                    // 每拍一步：比较 val >= res+one，减或右移，one 右移 2
                    if (rVal >= wSum) begin
                        rVal <= rVal - wSum;
                        rRes <= (rRes >> 1) + rOne;
                    end else begin
                        rRes <= rRes >> 1;
                    end
                    rOne <= rOne >> 2;
                    if (rCnt == 1) begin
                        rState <= S_DONE;
                        oBusy  <= 1'b0;
                    end else begin
                        rCnt <= rCnt - 1'b1;
                    end
                end
                S_DONE: begin
                    oBusy <= 1'b0;
                    oDone <= 1'b1;
                    oRoot <= rRes[OW-1:0];
                    rState <= S_IDLE;
                end
                default: rState <= S_IDLE;
            endcase
        end
    end
endmodule
