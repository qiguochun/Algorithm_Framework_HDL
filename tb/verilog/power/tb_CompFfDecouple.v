//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   tb_CompFfDecouple.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.06
/*Description       :   CompFfDecouple 单元 TB: 纯组合。给定 id/iq/w/L(标幺),
//                       验证 vd_ff = -wL*iq, vq_ff = +wL*id 的数值与方向。
//                       用独立物理模型估幅值(容差 3 counts) + 整数镜像精确比对(容差 0)。
//                       例: w=1.0pu, L=0.2pu -> wL=0.2pu; id=0.8 -> vq≈0.16pu(≈5243 counts)。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.1
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
`timescale 1ns/1ps

module TbCompFfDecouple;
    reg  signed [15:0] iId = 0, iIq = 0, iOmega = 0, iL = 0;
    wire signed [15:0] oVdFf, oVqFf;
    integer fail = 0;

    CompFfDecouple U_Dut (
        .iId(iId), .iIq(iIq), .iOmega(iOmega), .iL(iL),
        .oVdFf(oVdFf), .oVqFf(oVqFf)
    );

    // 物理模型估计: w_pu = iOmega/256, L_pu=iL/32768, i_pu=i/32768, v_pu counts=*32768
    real    wl_real, vq_real, vd_real;
    integer i;
    integer wlx, vqx, vdx, e, g;
    real    errVq, errVd;

    task chk(input integer id, iq, om, ll);
        integer cap;
        begin
            iId = id; iIq = iq; iOmega = om; iL = ll;
            #2;                                        // 组合稳定
            wlx = (om*ll + 128) >>> 8;                 // wl Q15(与 RTL 同式)
            vqx = (wlx*id + 16384) >>> 15;
            vdx = (wlx*iq + 16384) >>> 15;
            // 整数镜像精确比对(经整数变量, 保证符号/位宽)
            e = 0;
            g = oVqFf; if (g !== vqx) begin $display("FAIL 镜像 vq: got %0d exp %0d", g, vqx); e = 1; end
            g = oVdFf; if (g !== (-vdx)) begin $display("FAIL 镜像 vd: got %0d exp %0d", g, -vdx); e = 1; end
            // 独立物理量模型估幅(容差 3 counts)
            wl_real = (om/256.0)*(ll/32768.0);
            vq_real = wl_real*(id/32768.0)*32768.0;
            vd_real = -(wl_real*(iq/32768.0)*32768.0);
            errVq = oVqFf - vq_real; if (errVq < 0.0) errVq = -errVq;
            errVd = oVdFf - vd_real; if (errVd < 0.0) errVd = -errVd;
            $display("  id=%0d iq=%0d w=%.2fpu L=%.3fpu -> vd=%0d(vd_r=%.1f) vq=%0d(vq_r=%.1f)",
                     id, iq, om/256.0, ll/32768.0, oVdFf, vd_real, oVqFf, vq_real);
            if (errVq > 3.0 || errVd > 3.0) begin
                $display("FAIL 物理量偏差 vq=%.1f vd=%.1f", errVq, errVd); e = 1;
            end
            // 方向: iq 越正 vd 越负; id 越正 vq 越正 (wl>0 时)
            if (wlx > 0) begin
                if ((id > 0) && (oVqFf <= 0)) begin $display("FAIL vq 方向"); e = 1; end
                if ((iq > 0) && (oVdFf >= 0)) begin $display("FAIL vd 方向"); e = 1; end
            end
            fail = fail + e;
        end
    endtask

    initial begin
        // 用例: 幅值与符号
        chk( 26214, -19661, 256,  6554);   // id=0.8, iq=-0.6, w=1.0, L=0.2
        chk(-13107,  16384, 128,  3277);   // id=-0.4,iq=0.5, w=0.5, L=0.1
        chk(  8192,   4096, 256, 16384);   // id=0.25,iq=0.125,w=1.0, L=0.5
        chk(     0,      0, 256,  6554);   // 零电流 -> 零输出
        chk( 16384, -16384,  64,  8192);   // 等幅异号电流, 小 w
        if (fail == 0)
            $display("PASS CompFfDecouple: vd_ff=-wL*iq, vq_ff=+wL*id 数值/方向正确");
        else
            $display("FAIL CompFfDecouple: %0d 例失败", fail);
        $finish;
    end
endmodule
