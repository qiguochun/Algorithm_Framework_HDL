//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   tb_CompLmsAlpha.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
/*Description       :   CompLmsAlpha 单元 TB（自包含）
                        模型: iCt = alpha_true * iOwn (alpha_true=1.3, iOwn=50*sin, 32 点/周期)。
                        期望: oAlphaHat -> 1.3（alpha_q = round(1.3*2^16) = 85197，±3%）。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.1
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
`timescale 1ns/1ps

module TbCompLmsAlpha;
    localparam integer PER     = 32;     // 周期样本数
    localparam integer A_OWN   = 50;     // 参考幅度
    localparam real    ALPHA_T = 1.3;    // 真实极性系数
    localparam integer WARM    = 2500;   // 预热(收敛)样本
    localparam integer MEAS    = 1500;   // 测量样本
    localparam integer AT_Q    = 85197;  // round(1.3*65536)

    reg clk = 0;
    reg iSysRst = 1;
    reg en = 0;
    integer n = 0;
    reg signed [15:0] i_own;
    reg signed [15:0] i_ct;
    wire signed [31:0] alpha_hat;

    always @(posedge clk) n <= n + 1;

    real ph, ov;
    always @* begin
        ph = 2.0 * 3.141592653589793 * (n % PER) / PER;
        ov = $itor(A_OWN) * $sin(ph);
        i_own = (ov >= 0) ? $rtoi(ov + 0.5) : $rtoi(ov - 0.5);   // 四舍五入
        i_ct  = (ALPHA_T * ov >= 0) ? $rtoi(ALPHA_T * ov + 0.5)
                                    : $rtoi(ALPHA_T * ov - 0.5);
    end

    CompLmsAlpha U_Dut (
        .iSysClk(clk), .iSysRst(iSysRst), .iEn(en), .iCt(i_ct), .iOwn(i_own),
        .oAlphaHat(alpha_hat)
    );

    always #5 clk = ~clk;

    integer i;
    integer acc_a;
    real avg_a;
    integer fail = 0;

    initial begin
        iSysRst = 1; en = 1;
        repeat (3) @(posedge clk);
        iSysRst = 0;
        for (i = 0; i < WARM; i = i + 1) @(posedge clk);

        acc_a = 0;
        for (i = 0; i < MEAS; i = i + 1) begin
            @(posedge clk);
            acc_a = acc_a + alpha_hat;
        end
        avg_a = acc_a / MEAS;
        $display("LMSA  alpha_q_avg=%0.1f (expect %0d)", avg_a, AT_Q);
        if (avg_a > AT_Q * 0.97 && avg_a < AT_Q * 1.03)
            $display("PASS lms_alpha: alpha_hat converges to alpha_true (1.3)");
        else begin
            $display("FAIL lms_alpha: alpha out of range");
            fail = 1;
        end
        if (fail) $finish;
        $finish;
    end
endmodule
