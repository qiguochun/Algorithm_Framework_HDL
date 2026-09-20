//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   AlgoDiv.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.06
/*Description       :   有符号恢复除法库（多拍迭代，start/done 握手，参照 ROCA SignedDivision）
//                      业务禁止组合除法，统一例化本库(§6.3)。
//  接口: iStart 脉冲启动 -> oBusy=1 期间逐拍迭代(W 拍) -> oDone 拉一拍后采 oQuot。
//        busy 期间 iStart 被忽略，调用方须等 oDone 后再发新请求。
//  算法: 恢复除法(restoring division)，每拍左移比较减；符号 = 被除数^除数(异或)，IDLE 取绝对。
//  除零: 分母为 0 时商输出 0(不挂起)。
//  溢出: |商|=2^(W-1) 且符号为正时(如 -2^(W-1)/-1)饱和到 2^(W-1)-1。
//  延迟: W 拍(可预测)。
//  修订: 2026-09-08 v0.3（修 BLKSEQ/删废变量/修溢出饱和/rCnt 位宽）
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.1
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------

`include "algo_types.vh"

module AlgoDiv #(
    parameter integer W = 32      // 被除数/除数/商统一位宽（迭代拍数 = W）
) (
    input  wire             iSysClk,
    input  wire             iSysRst,
    input  wire             iStart,        // 启动脉冲（一拍）
    input  wire [W-1:0]     iNum,          // 被除数（有符号，两补码）
    input  wire [W-1:0]     iDen,          // 除数（有符号，两补码）
    output reg              oBusy,         // 忙标志（BUSY 期间为 1）
    output reg              oDone,         // 完成脉冲（一拍）
    output reg  [W-1:0]     oQuot          // 商（有符号，向 0 截断）
);
    localparam [1:0] S_IDLE = 2'd0, S_BUSY = 2'd1, S_DONE = 2'd2;

    reg [1:0]   rState;
    reg [$clog2(W+1)-1:0] rCnt;     // 迭代计数，从 W 减到 1（共 W 次迭代）
    reg [W-1:0] rNum;               // 被除数绝对值（每次迭代左移一位）
    reg [W-1:0] rDen;               // 除数绝对值（全程不变）
    reg [W-1:0] rRem;               // 余数（每次迭代左移一位，最高位为被除数最高位）
    reg [W-1:0] rQuot;              // 商（未定符号）
    reg         rQuotNeg;           // 商符号（1=负）
    reg         rDenZero;           // 除零标志

    // 组合：rem 左移入 num 最高位（时序块内不阻塞赋值，避免 BLKSEQ）
    wire [W-1:0] wNextRem = {rRem[W-2:0], rNum[W-1]};

    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            rState <= S_IDLE; oBusy <= 1'b0; oDone <= 1'b0; oQuot <= {W{1'b0}};
        end else begin
            oDone <= 1'b0;
            case (rState)
                S_IDLE: begin
                    oBusy <= 1'b0;
                    if (iStart) begin
                        // 取绝对值（rNum/rDen 为无符号 W 位，最小负数 2^(W-1) 亦能表示，不溢出）
                        rNum     <= (iNum[W-1]) ? (~iNum + 1'b1) : iNum;
                        rDen     <= (iDen[W-1]) ? (~iDen + 1'b1) : iDen;
                        rQuotNeg <= iNum[W-1] ^ iDen[W-1];
                        rDenZero <= (iDen == {W{1'b0}});
                        rRem     <= {W{1'b0}};
                        rQuot    <= {W{1'b0}};
                        rCnt     <= W;   // integer 参数直接赋给 rCnt，自动截断到 $clog2(W+1) 位
                        rState   <= S_BUSY;
                        oBusy    <= 1'b1;
                    end
                end
                S_BUSY: begin
                    oBusy <= 1'b1;
                    // 每拍一步：rem 左移入 num 最高位，num 左移，比较减，quot 左移
                    if (!rDenZero && wNextRem >= rDen) begin
                        rRem  <= wNextRem - rDen;
                        rQuot <= {rQuot[W-2:0], 1'b1};
                    end else begin
                        rRem  <= wNextRem;
                        rQuot <= {rQuot[W-2:0], 1'b0};
                    end
                    rNum <= {rNum[W-2:0], 1'b0};
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
                    if (rQuotNeg)
                        oQuot <= ~rQuot + 1'b1;                    // 负商（|商|=2^(W-1) 时 ~+1 得最小负数，正确）
                    else if (rQuot[W-1])
                        oQuot <= {1'b0, {(W-1){1'b1}}};            // 正商溢出(如 -2^(W-1)/-1) 饱和到最大正数
                    else
                        oQuot <= rQuot;
                    rState <= S_IDLE;
                end
                default: rState <= S_IDLE;
            endcase
        end
    end
endmodule
