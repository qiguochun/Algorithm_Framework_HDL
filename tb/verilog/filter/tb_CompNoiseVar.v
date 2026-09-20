//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   tb_CompNoiseVar.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
/*Description       :   CompNoiseVar 单元 TB（自包含，method=1 EWMA）
                        输入 = 直流 3000 + 均匀噪声 U(-300,+300)。
                        期望: var -> 30100(±8%)，std -> sqrt(30100)≈173。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.1
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
`timescale 1ns/1ps

module TbCompNoiseVar;
    localparam integer NOISE   = 300;     // 均匀噪声幅度
    localparam integer DC      = 3000;    // 直流
    localparam integer WARM    = 2500;    // 预热样本
    localparam integer MEAS    = 2500;    // 测量样本
    localparam integer VEXP    = 30100;   // 离散均匀方差理论值

    reg clk = 0;
    reg iSysRst = 1;
    reg en = 0;
    reg signed [15:0] x = 0;
    wire [39:0] o_var;
    wire [31:0] o_std;

    // 每个时钟半周期更新一次噪声采样（negedge，避免与 DUT 采样竞争）
    always @(negedge clk) begin
        x = DC + (($random & 32'h7fffffff) % (2 * NOISE + 1)) - NOISE;
    end

    CompNoiseVar U_Dut (
        .iSysClk(clk), .iSysRst(iSysRst), .iEn(en), .iXIn(x), .oVar(o_var), .oStd(o_std)
    );

    always #5 clk = ~clk;

    integer i;
    integer acc_v, acc_s;
    real avg_v, avg_s;
    integer fail = 0;

    initial begin
        iSysRst = 1; en = 1;
        repeat (3) @(posedge clk);
        iSysRst = 0;
        for (i = 0; i < WARM; i = i + 1) @(posedge clk);

        acc_v = 0; acc_s = 0;
        for (i = 0; i < MEAS; i = i + 1) begin
            @(posedge clk);
            acc_v = acc_v + o_var;
            acc_s = acc_s + o_std;
        end
        avg_v = acc_v / MEAS;
        avg_s = acc_s / MEAS;
        $display("NVAR  var_avg=%0.1f (expect %0d)  std_avg=%0.1f", avg_v, VEXP, avg_s);
        if (avg_v > VEXP * 0.92 && avg_v < VEXP * 1.08)
            $display("PASS noise_var: variance estimate ~ theoretical");
        else begin
            $display("FAIL noise_var: variance out of range");
            fail = 1;
        end
        if (avg_s > 160 && avg_s < 186) begin
            $display("PASS noise_var: std ~ sqrt(var)");
        end else begin
            $display("FAIL noise_var: std out of range");
            fail = 1;
        end
        if (fail) $finish;
        $finish;
    end
endmodule
