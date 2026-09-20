//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   tb_PvObserver.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
/*Description       :   PvObserver(清洗版PV卡尔曼观测器)单元TB：CCM→1、
//                      DCM→0、近开路 Voc 下收敛
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.1
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
`timescale 1ns/1ps

module TbPvObserver;
    localparam integer W   = 16;
    localparam integer SC  = 1 << 24;          // Q0.24 满量程
    localparam integer DUTY  = 32768;          // D=0.5 (Q0.16)
    localparam integer BUSV  = 400;            // V
    localparam integer VCCM  = 200;            // (1-0.5)*400

    reg iSysClk = 0;
    reg iSysRst = 0;
    always #5 iSysClk = ~iSysClk;

    reg mppt_en = 0;
    reg [DW-1:0] duty_q = 0;  localparam integer DW = 16;
    reg [W-1:0]  v_meas = 0;
    reg [W-1:0]  i_meas = 0;
    reg [W-1:0]  bus_volt = BUSV;
    wire [W-1:0] o_v_ccm, o_v_dcm;
    wire [24:0]  o_alpha_hat, o_alpha_hat_smooth, o_alpha_hat_corrected;
    wire [W-1:0] o_voc_est;

    integer ticks;
    real ar, asm;
    integer fail = 0;

    PvObserver #(.W(W), .CW(W), .DW(DW), .VOC_INIT_V(1000)) U_Dut (
        .iSysClk(iSysClk), .iSysRst(iSysRst), .iMpptEn(mppt_en),
        .iDutyQ(duty_q), .iVMeas(v_meas), .iMeas(i_meas), .iBusVolt(bus_volt),
        .oVccm(o_v_ccm), .oVdcm(o_v_dcm),
        .oAlphaHat(o_alpha_hat), .oAlphaHatSmooth(o_alpha_hat_smooth),
        .oAlphaHatCorrected(o_alpha_hat_corrected),
        .oVocEst(o_voc_est));

    // 每调用执行一次 mppt_en 拍(状态更新一次)
    task tick;
        begin
            @(posedge iSysClk); mppt_en = 1;
            @(posedge iSysClk); mppt_en = 0;
        end
    endtask

    task run_ticks(input integer n);
        integer j;
        begin
            for (j = 0; j < n; j = j + 1) tick;
        end
    endtask

    task report_alpha(input integer phase);
        begin
            ar  = o_alpha_hat / 16777216.0;
            asm = o_alpha_hat_smooth / 16777216.0;
            $display("  phase%0d: alpha_hat=%.5f smooth=%.5f corrected=%.5f (v_ccm=%0d v_dcm=%0d)",
                     phase, ar, asm,
                     o_alpha_hat_corrected / 16777216.0, o_v_ccm, o_v_dcm);
        end
    endtask

    initial begin
        iSysRst = 1;
        repeat (4) @(posedge iSysClk);
        iSysRst = 0;
        duty_q = DUTY; bus_volt = BUSV;

        // ---------------- P1: CCM 平衡点 ----------------
        $display("[P1] CCM: bus=400 D=0.5 I=4000mA Vmeas=Vccm=200");
        i_meas = 4000; v_meas = VCCM;
        run_ticks(1500);
        report_alpha(1);
        if (o_v_ccm !== VCCM) begin
            $display("FAIL v_ccm=%0d want=%0d", o_v_ccm, VCCM);
            fail = fail + 1;
        end
        if (o_alpha_hat < (90 * SC) / 100) begin
            $display("FAIL P1 alpha_hat not converged high: %0d", o_alpha_hat);
            fail = fail + 1;
        end else
            $display("  P1 alpha_hat high ok (raw=%0d >0.9)", o_alpha_hat);

        // ---------------- P2: 轻载 DCM ----------------
        $display("[P2] DCM: I=200mA<Icrit, Vmeas=Vdcm");
        i_meas = 200;
        repeat (200) @(posedge iSysClk);    // 等 Vdcm 多拍除法结算(I=200 -> 42)再采
        tick;
        v_meas = o_v_dcm;                   // Vmeas = 模块自身 Vdcm -> z2=0
        run_ticks(1500);
        report_alpha(2);
        if (o_v_dcm >= o_v_ccm) begin
            $display("FAIL DCM 应 v_dcm<v_ccm: vdcm=%0d vccm=%0d", o_v_dcm, o_v_ccm);
            fail = fail + 1;
        end
        if (o_alpha_hat >= (10 * SC) / 100) begin
            $display("FAIL P2 alpha_hat not dropped: %0d", o_alpha_hat);
            fail = fail + 1;
        end else
            $display("  P2 alpha_hat low ok (raw=%0d <0.1)", o_alpha_hat);
        if (o_alpha_hat_smooth >= (15 * SC) / 100) begin
            $display("FAIL P2 smooth not dropped: %0d", o_alpha_hat_smooth);
            fail = fail + 1;
        end

        // ---------------- P3: 近开路 Voc 通路 ----------------
        $display("[P3] 近开路: D=2000/65536(<0.1) I=50mA Vmeas=300 -> Voc 下收敛");
        duty_q = 2000; i_meas = 50; v_meas = 300;
        run_ticks(2000);
        $display("  phase3: voc_est=%0d V (init 1000)", o_voc_est);
        if (o_voc_est > 450) begin
            $display("FAIL P3 Voc 未向下收敛: %0d", o_voc_est);
            fail = fail + 1;
        end else
            $display("  P3 Voc decreased ok (voc_est=%0d)", o_voc_est);

        // ---------------- 汇总 ----------------
        if (fail == 0) begin
            $display("PASS tb_PvObserver: CCM->1, DCM->0, Voc 收敛, 全部断言过");
        end else begin
            $display("FAIL tb_PvObserver: %0d 断言失败", fail);
        end
        $finish;
    end
endmodule
