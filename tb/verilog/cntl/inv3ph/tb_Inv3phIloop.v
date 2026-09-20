`timescale 1ns/1ps

module TbInv3phIloop;
    reg clk = 0, rst = 1, en = 0;
    always #5 clk = ~clk;
    reg signed [15:0] id = 0, iq = 0, ud = 0, uq = 0, idr = 10, iqr = 0;
    wire signed [15:0] ed, eq;
    wire vld;
    integer fail = 0, k;

    Inv3phIloop #(.SEQ(1)) U_Dut (
        .iSysClk(clk), .iSysRst(rst), .iEn(en),
        .iId(id), .iIq(iq), .iUd(ud), .iUq(uq),
        .iIdRef(idr), .iIqRef(iqr),
        .oEdRef(ed), .oEqRef(eq), .oValid(vld)
    );

    task sample;
        begin
            en = 1; @(posedge clk); #1; en = 0;
            while (!vld) begin @(posedge clk); #1; end
        end
    endtask

    initial begin
        repeat (2) @(posedge clk); rst = 0; @(posedge clk);
        for (k = 0; k < 100; k = k + 1) sample;
        if (ed <= 0) begin
            $display("FAIL Ed not positive Ed=%0d", ed);
            fail = fail + 1;
        end else $display("  ok track Id_ref Ed=%0d Eq=%0d", ed, eq);

        // 跟上参考后 Ed 应回落到接近 Ud(=0) + 解耦
        id = 10;
        for (k = 0; k < 200; k = k + 1) sample;
        if (ed > 80 || ed < -80) begin
            $display("FAIL settle Ed=%0d", ed);
            fail = fail + 1;
        end else $display("  ok settle Ed=%0d", ed);

        if (fail == 0) $display("PASS Inv3phIloop");
        else $display("FAIL Inv3phIloop: %0d", fail);
        $finish;
    end
endmodule
