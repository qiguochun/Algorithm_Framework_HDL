//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   CompRateLimit.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.06
/*Description       :   限幅速率/斜率限制器（slew rate limiter）。
                        约束 |y[n]-y[n-1]|<=RATE（每采样拍最大允许变化量，参数化）：
                        若 x[n] 相对上一拍输出变化在 RATE 内则直通 y[n]=x[n]；
                        否则按符号以 RATE 追赶上/下界 y[n]=y[n-1]+-RATE。
                        纯加减/比较实现，无乘法除法；全精度 64bit 累加，输出饱和 S16。
                        阶跃输入的上升速率被钳在 RATE/拍；慢于 RATE/拍 的缓变信号直通。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.1
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
module CompRateLimit #(
    parameter integer W    = 16,     // 信号位宽(有符号)
    parameter integer RATE = 100     // 每采样拍最大变化量 |dy|（参数化）
) (
    input  wire                iSysClk,  // 时钟
    input  wire                iSysRst,  // 复位（高有效）
    input  wire                iEn,      // 采样节拍（每拍一采样）
    input  wire signed [W-1:0] iXIn,     // 输入采样 x[n]
    output reg  signed [W-1:0] oYOut     // 限速后输出 y[n]
);
    localparam signed [63:0] YMAX = (64'sd1 << (W-1)) - 1;  // 输出上限
    localparam signed [63:0] YMIN = -(64'sd1 << (W-1));     // 输出下限

    // ---- 内部状态 ----
    reg signed [63:0] rY;         // 上一拍输出（全精度，含饱和后的值）

    // ---- 组合临时量（阻塞算好，末尾一次非阻塞写） ----
    reg signed [63:0] t_d;        // x-y 差值
    reg signed [63:0] t_y;        // 限速后的输出(未饱和)
    reg signed [63:0] t_o;        // 饱和后输出

    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            rY    <= 64'sd0;
            oYOut <= {W{1'b0}};
        end else if (iEn) begin
            t_d = $signed(iXIn) - rY;
            if (t_d >  $signed(RATE))       t_y = rY + $signed(RATE);
            else if (t_d < -$signed(RATE))  t_y = rY - $signed(RATE);
            else                            t_y = $signed(iXIn);
            if (t_y > YMAX)      t_o = YMAX;
            else if (t_y < YMIN) t_o = YMIN;
            else                 t_o = t_y;
            rY    <= t_o;
            oYOut <= t_o[W-1:0];
        end
    end
endmodule
