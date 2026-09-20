`timescale 1ns/1ps

module TbInv3phVloop;
    reg clk = 0, rst = 1, en = 0;
    always #5 clk = ~clk;
    reg signed [15:0] ud = 0, uq = 0, i2d = 0, i2q = 0, udr = 311, uqr = 0;
    wire signed [15:0] i1d, i1q;
    wire vld;
    integer fail = 0, k;
    integer sum_d;

    Inv3phVloop U_Dut (
        .iSysClk(clk), .iSysRst(rst), .iEn(en),
        .iUd(ud), .iUq(uq), .iI2d(i2d), .iI2q(i2q),
        .iUdRef(udr), .iUqRef(uqr),
        .oI1dRef(i1d), .oI1qRef(i1q), .oValid(vld)
    );

    task sample;
        begin
            en = 1; @(posedge clk); #1; en = 0;
            while (!vld) begin @(posedge clk); #1; end
        end
    endtask

    initial begin
        repeat (2) @(posedge clk); rst = 0; @(posedge clk);
        // 阶跃: Ud 从 0 追 Ud_ref=311，电流参考应向正方向爬升后稳定非零
        for (k = 0; k < 200; k = k + 1) begin
            ud = 0; uq = 0; i2d = 0; i2q = 0;
            sample;
        end
        if (i1d <= 0) begin
            $display("FAIL vloop no positive Idref=%0d", i1d);
            fail = fail + 1;
        end else $display("  ok step Id_ref=%0d Iq_ref=%0d", i1d, i1q);

        // 复位后从 Ud=Ud_ref 启动: 误差≈0，Id_ref 应贴近 I2d(=0)
        rst = 1; @(posedge clk); #1; rst = 0; @(posedge clk); #1;
        sum_d = 0;
        for (k = 0; k < 100; k = k + 1) begin
            ud = 311; uq = 0; i2d = 0; i2q = 0;
            sample;
            if (k >= 50) sum_d = sum_d + i1d;
        end
        sum_d = sum_d / 50;
        if (sum_d > 20 || sum_d < -20) begin
            $display("FAIL matched mean Id=%0d", sum_d);
            fail = fail + 1;
        end else $display("  ok matched mean Id_ref=%0d", sum_d);

        if (fail == 0) $display("PASS Inv3phVloop");
        else $display("FAIL Inv3phVloop: %0d", fail);
        $finish;
    end
endmodule
