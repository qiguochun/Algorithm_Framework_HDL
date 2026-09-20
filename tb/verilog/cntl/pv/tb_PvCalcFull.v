//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   tb_PvCalcFull.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
/*Description       :   PvCalcFull 单元 TB：均值窗/LPF收敛、软启斜坡、CCM/DCM
//                      duty 限幅与 MPPT 前瞻校验
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.1
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
`timescale 1ns/1ps
`include "algo_types.vh"

module TbPvCalcFull;
    // ---------------- 时钟/节拍 ----------------
    reg iSysClk = 0;
    reg iSysRst = 0;
    always #5 iSysClk = ~iSysClk;                       // 100MHz
    reg tick = 0;
    reg [3:0] dcnt = 0;                         // 每 10 clk 一拍(≈任意, 节拍语义由 tick 决定)
    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin dcnt <= 0; tick <= 0; end
        else if (dcnt == 4'd9) begin dcnt <= 0; tick <= 1; end
        else begin dcnt <= dcnt + 1; tick <= 0; end
    end

    reg [31:0] nticks = 0;
    reg [15:0] B1 = 0, B2 = 0;                  // 软启参考快照
    reg [15:0] VA_EARLY = 0;                    // 首个均值窗结束(520 拍)采样
    reg chk1_f = 0, chk2_f = 0;
    reg signed [15:0] mppt_dv = 0;

    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            nticks  <= 0; B1 <= 0; B2 <= 0; chk1_f <= 0; chk2_f <= 0; mppt_dv <= 0;
            VA_EARLY <= 0;
        end else if (tick) begin
            case (nticks)
                32'd100 :  B1 <= bus_volt_ref;                // 软启前 0V
                32'd300 :  B2 <= bus_volt_ref;                // 第200拍后 +1V
                32'd520 :  VA_EARLY <= v_avg;                 // AVG_N=500 窗后均值窗输出
                32'd40000: chk1_f <= 1'b1;                     // 主收敛校验点
                32'd40500: mppt_dv <= 16'd10;                  // MPPT 前瞻: d_vref=1.0V, 加 500 拍
                32'd41000: begin mppt_dv <= 0; chk2_f <= 1'b1; end // MPPT 校验点
            endcase
            nticks <= nticks + 1;
        end
    end

    // ---------------- 输入驱动 ----------------
    reg        en_run = 1;
    reg        soft_en = 1;
    reg [15:0] v_meas = 2500;                   // 250.0V
    reg signed [15:0] curr_inst = 200;          // 200 mA (轻载)
    reg [23:0] p_inst = 500;                    // 50.0W (0.1W)
    reg [23:0] pwr_limit = 60000;               // 6000W
    reg [15:0] bus_volt = 4000;                 // 400.0V
    reg [15:0] alpha_hat = 32768;               // Q0.16 = 0.5 (CCM/DCM 各半)

    PvCalcFull #(
        .AVG_N(500)
    ) U_Dut (
        .iSysClk(iSysClk), .iSysRst(iSysRst), .iTick(tick),
        .iEnRun(en_run), .iSoftEn(soft_en),
        .iVMeas(v_meas), .iCurrInst(curr_inst), .iPInst(p_inst),
        .iPwrLimit(pwr_limit), .iBusVolt(bus_volt),
        .iAlphaHat(alpha_hat), .iMpptDVref(mppt_dv),
        .oVRef(o_vref), .oDutyQ(duty_q), .oDCcm(d_ccm), .oDDcm(d_dcm),
        .oDutyOffset(duty_offset), .oBusVoltRef(bus_volt_ref),
        .oVAvg(v_avg), .oVFilter(v_filter), .oBusVoltFilter(bus_volt_filter),
        .oCurrFilter(curr_filter), .oPFilter(p_filter),
        .oCurrAve(curr_ave), .oPAve(p_ave)
    );
    wire [15:0] o_vref, duty_q, d_ccm, d_dcm, duty_offset, bus_volt_ref;
    wire [15:0] v_avg, v_filter, bus_volt_filter, curr_filter, p_filter;
    wire [15:0] curr_ave, p_ave;

    integer fail = 0;
    integer t;
    integer ccm_exp;

    initial begin
        $display("=== tb_PvCalcFull start ===");
        // reset（高有效）
        iSysRst = 1;
        repeat (3) @(posedge iSysClk);
        @(negedge iSysClk); iSysRst = 0;

        // ---- 软启斜坡校验 (第 200 拍 +1V) ----
        while (B1 == 0 && nticks < 150) @(posedge iSysClk);
        while (nticks < 320) @(posedge iSysClk);
        @(negedge iSysClk);
        $display("bus_volt_ref: B1(100拍)=%0d  B2(300拍)=%0d", B1, B2);
        if (B1 == 0 && B2 == 10)
            $display("PASS soft-start: +1.0V per 200 ticks");
        else begin
            $display("FAIL soft-start B1=%0d B2=%0d (expect 0 -> 10)", B1, B2);
            fail = fail + 1;
        end

        // ---- 均值窗: 首个 AVG_N(500)窗结束即输出≈输入 ----
        while (nticks < 560) @(posedge iSysClk);
        @(negedge iSysClk);
        if (VA_EARLY >= 2498 && VA_EARLY <= 2502)
            $display("PASS v_avg right after 1st window (500 ticks) = %0d ~ 2500", VA_EARLY);
        else begin
            $display("FAIL v_avg early=%0d (expect ~2500 after AVG_N ticks)", VA_EARLY);
            fail = fail + 1;
        end

        // ---- 主收敛校验 (长稳 40000 拍) ----
        while (!chk1_f) @(posedge iSysClk);
        repeat (4) @(posedge iSysClk);
        @(negedge iSysClk);

        // 1) 均值窗/LPF 收敛
        if (v_avg >= 2498 && v_avg <= 2502)
            $display("PASS v_avg=%0d ~ 2500 (0.1V)", v_avg);
        else begin
            $display("FAIL v_avg=%0d (expect ~2500)", v_avg); fail = fail + 1;
        end
        if (v_filter >= 2498 && v_filter <= 2502)
            $display("PASS v_filter=%0d ~ 2500", v_filter);
        else begin
            $display("FAIL v_filter=%0d (expect ~2500)", v_filter); fail = fail + 1;
        end
        if (bus_volt_filter >= 3997 && bus_volt_filter <= 4003)
            $display("PASS bus_volt_filter=%0d ~ 4000", bus_volt_filter);
        else begin
            $display("FAIL bus_volt_filter=%0d (expect ~4000)", bus_volt_filter); fail = fail + 1;
        end
        if (p_filter >= 490 && p_filter <= 510)
            $display("PASS p_filter=%0d ~ 500 (0.1W)", p_filter);
        else begin
            $display("FAIL p_filter=%0d (expect ~500)", p_filter); fail = fail + 1;
        end
        if (curr_filter >= 185 && curr_filter <= 205)
            $display("PASS curr_filter=%0d ~ 200 mA", curr_filter);
        else begin
            $display("FAIL curr_filter=%0d (expect ~200mA)", curr_filter); fail = fail + 1;
        end
        // 均值窗值(长稳后若干窗, 恒输入 -> 精确)
        if (p_ave >= 498 && p_ave <= 502)
            $display("PASS p_ave=%0d", p_ave);
        else begin
            $display("FAIL p_ave=%0d", p_ave); fail = fail + 1;
        end
        if (curr_ave >= 198 && curr_ave <= 202)
            $display("PASS curr_ave=%0d", curr_ave);
        else begin
            $display("FAIL curr_ave=%0d", curr_ave); fail = fail + 1;
        end

        // 2) o_vref 稳定 ~250V, D_ccm ~ 1-o_vref/bus
        if (o_vref >= 2498 && o_vref <= 2502)
            $display("PASS o_vref=%0d ~ 250.0V", o_vref);
        else begin
            $display("FAIL o_vref=%0d", o_vref); fail = fail + 1;
        end
        // 以输出量回算: 期望 Dccm = 65536*(1 - o_vref/bus_filter)
        ccm_exp = 65536 - ((65536 * o_vref) / bus_volt_filter);
        $display("d_ccm=%0d exp=%0d (1-o_vref/bus, tol 300)  d_dcm=%0d", d_ccm, ccm_exp, d_dcm);
        t = (d_ccm > ccm_exp) ? (d_ccm - ccm_exp) : (ccm_exp - d_ccm);
        if (t <= 300)
            $display("PASS D_ccm formula within tolerance");
        else begin
            $display("FAIL D_ccm mismatch"); fail = fail + 1;
        end
        if (d_dcm > 1000)
            $display("PASS D_dcm active (DCM branch), d_dcm=%0d", d_dcm);
        else begin
            $display("FAIL D_dcm not active (%0d)", d_dcm); fail = fail + 1;
        end

        // 3) duty 限幅范围
        $display("duty_q=%0d  [min=%0d, max=%0d]  d_ccm=%0d  offset=%0d",
                 duty_q, 16'd3277, 16'd62259, d_ccm, duty_offset);
        if (duty_q >= 16'd3277 && duty_q <= 16'd62259)
            $display("PASS duty in [minD,maxD]");
        else begin
            $display("FAIL duty out of range"); fail = fail + 1;
        end
        if (duty_q <= d_ccm + 1)
            $display("PASS duty <= D_ccm");
        else begin
            $display("FAIL duty > D_ccm"); fail = fail + 1;
        end

        // ---- MPPT 前瞻: 500 拍 * 0.002*1.0V = 1.0V 前移 ----
        while (!chk2_f) @(posedge iSysClk);
        repeat (4) @(posedge iSysClk);
        @(negedge iSysClk);
        $display("o_vref after MPPT fwd = %0d (expect ~2510)", o_vref);
        if (o_vref >= 2505 && o_vref <= 2515)
            $display("PASS MPPT forward feed ~+1.0V/500ticks");
        else begin
            $display("FAIL MPPT forward feed"); fail = fail + 1;
        end

        // ---- 汇总 ----
        if (fail == 0)
            $display("=== tb_PvCalcFull: PASS ===");
        else
            $display("=== tb_PvCalcFull: FAIL (%0d checks) ===", fail);
        $finish;
    end
endmodule
