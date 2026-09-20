`timescale 1ns/1ps
`include "inv3ph_cfg.vh"

module TbInv3phState;
    reg clk = 0, rst = 1, tick = 0;
    always #5 clk = ~clk;
    reg en_run = 0, en_grid = 0, en_grid_run = 0, is_err = 0;
    reg self_ok = 0, soft_done = 0, sync_ok = 0;
    wire [2:0] st;
    wire pwm, rly, grid_mode;
    integer fail = 0, i;

    Inv3phState #(.SELF_HOLD(3), .START_HOLD(5)) U_Dut (
        .iSysClk(clk), .iSysRst(rst), .iTick(tick),
        .iEnRun(en_run), .iEnGrid(en_grid), .iEnGridRun(en_grid_run),
        .iIsErr(is_err), .iSelfCheckPass(self_ok),
        .iSoftStartDone(soft_done), .iSyncOk(sync_ok),
        .oState(st), .oPwmEn(pwm), .oRlyEn(rly), .oGridMode(grid_mode)
    );

    task step;
        begin tick = 1; @(posedge clk); #1; tick = 0; @(posedge clk); #1; end
    endtask

    initial begin
        repeat (2) @(posedge clk); rst = 0; @(posedge clk);
        if (st !== `INV3PH_ST_IDLE) fail = fail + 1;

        en_run = 1; step;
        if (st !== `INV3PH_ST_SELFCHECK) begin $display("FAIL to self"); fail = fail + 1; end
        else $display("  ok IDLE->SELFCHECK");

        self_ok = 1;
        step; step; step; step;
        if (st !== `INV3PH_ST_STARTING) begin $display("FAIL to starting st=%0d", st); fail = fail + 1; end
        else $display("  ok SELFCHECK->STARTING");

        soft_done = 1; step;
        if (st !== `INV3PH_ST_DISCONNECTED) begin $display("FAIL to disc"); fail = fail + 1; end
        else $display("  ok STARTING->DISCONNECTED pwm=%0b", pwm);

        en_grid = 1; step;
        if (st !== `INV3PH_ST_CONNECTING) begin $display("FAIL to connecting"); fail = fail + 1; end
        else $display("  ok ->CONNECTING");

        en_grid_run = 1; sync_ok = 1; step;
        if (st !== `INV3PH_ST_CONNECTED || !rly || !grid_mode) begin
            $display("FAIL to connected"); fail = fail + 1;
        end else $display("  ok ->CONNECTED rly=%0b", rly);

        is_err = 1; step;
        if (st !== `INV3PH_ST_ERR) begin $display("FAIL to err"); fail = fail + 1; end
        else $display("  ok ->ERR");

        if (fail == 0) $display("PASS Inv3phState");
        else $display("FAIL Inv3phState: %0d", fail);
        $finish;
    end
endmodule
