//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   PvCalcFull.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
//Description       :   完整PvCalcFull单模块：均值窗/LPF调理+软启参考+三环保护型P
//                      与CCM/DCM占空比融合+MPPT前瞻(5kHz,单模块仿真通路)
//                      除法统一例化多拍 AlgoDiv/AlgoSqrt (§6.3 ROCA start/done)
//                      控制链每除法一个独立 AlgoDiv/AlgoSqrt(独立握手,空闲自动重启
//                      连续跟踪,oDone打一拍后采商); 窗口均值专用除法(窗满事件触发)
//                      除法无关积分(LPF/窗口累加/软启/MPPT前馈入 rOvrefVu) 多拍流水:
//                      iTick 采样 -> 输入尺度乘法 V*500(一拍) -> LPF 差*alpha /
//                      duty偏置衰减 / CCM-DCM 权重混合 乘法(一拍, 各单寄存) ->
//                      LPF 移位加得新滤波态(一拍) -> 提交全部状态/输出(一拍), oValid。
//                      每个乘法结果单独寄存, 无单周期组合乘加; 输出注册于提交拍。
//                      LATENCY=3 (采样节拍后 oValid 拉高)。
//------------------------------------------------------------------------------
//Version           :   Rev 0.4
//modifier          :
//Modify Date       :
//Modify Record     :   2026-09-06 组合除法/开方改为多拍 AlgoDiv/AlgoSqrt 握手化
//                      Rev 0.4 除法无关乘加链拆 rEv 多拍流水, 新增 oValid
//------------------------------------------------------------------------------

`include "algo_types.vh"

module PvCalcFull #(
    // ---------------- 控制节拍 / 均值窗 ----------------
    parameter integer AVG_N          = 500,    // 均值窗长度 = FS_LOOP/FS_MPPT = 5000/10 = 500 拍
    parameter integer VREF_INIT_V100 = 2500,   // oVRef 初值 250.0V(0.1V/bit), 清洗版初始化点
    // ---------------- 定点/量纲 ----------------
    parameter integer V100_TO_VU     = 500,    // 0.1V -> 内部 VU (内部电压 1LSB=1/5000V=0.2mV)
    parameter integer PCF_X          = 10,     // 电流/功率滤波小数位: 内部 = 物理值 << PCF_X
    // ---------------- 软启参考斜坡 ----------------
    parameter integer SOFT_PERIOD    = 200,    // 软启步进间隔(拍): 每 SOFT_PERIOD 拍 +SOFT_STEP_V100
    parameter integer SOFT_STEP_V100 = 10,     // 每 200 拍 +1.0V(=10*0.1V), 即 1V/200 拍
    parameter integer BUSREF_MAX_V100= 6200,   // 软启上限 620V
    // ---------------- 三环(保护型 P-only, 输出 clamp [0,0.25V]) ----------------
    parameter integer CURR_MAX_MA    = 16000,  // 16A
    parameter integer BUS_MAX_V100   = 9000,   // 900V
    parameter integer VREF_MIN_V100  = 700,    // 70V
    parameter integer LOOP_CLAMP_VU  = 1250,   // 环输出上限 0.25V = 0.25*5000 VU
    // ---------------- LPF alpha (Q0.16, 由 2*pi 折算) ----------------
    parameter integer ALPHA_VP_Q16   = 823,    // 电压/功率 LPF
    parameter integer ALPHA_BUS_Q16  = 411,    // 母线 LPF
    parameter integer ALPHA_CURR_Q16 = 8,      // 电流 LPF
    parameter integer DBIAS_THD_V100 = 30,     // oDutyOffset 积分死区 3V(0.1V*30)
    parameter integer DBIAS_MAX_Q16  = 13107,  // oDutyOffset 上限 0.2 = 0.2*65536
    parameter integer LEAK_Q16       = 1,      // 偏置衰减 alpha
    // ---------------- 占空比限幅 (Q0.16) ----------------
    parameter integer DUTY_MIN_Q16   = 3277,
    parameter integer DUTY_MAX_Q16   = 62259,
    // ---------------- DCM / 硬件常数 ----------------
    parameter integer PV_K_VU_PER_MA = 300     // 2*L*fsw 折算 VU/ma
) (
    input  wire               iSysClk,
    input  wire               iSysRst,
    input  wire               iTick,            // 5kHz 节拍 strobe
    input  wire               iEnRun,
    input  wire               iSoftEn,

    // ---- 测量输入 ----
    input  wire [15:0]        iVMeas,
    input  wire signed [15:0] iCurrInst,
    input  wire [23:0]        iPInst,
    input  wire [23:0]        iPwrLimit,
    input  wire [15:0]        iBusVolt,
    // ---- 观测/MPPT 输入 ----
    input  wire [15:0]        iAlphaHat,
    input  wire signed [15:0] iMpptDVref,

    // ---- 输出 ----
    output reg  [15:0]        oVRef,
    output reg  [15:0]        oDutyQ,
    output reg  [15:0]        oDCcm,
    output reg  [15:0]        oDDcm,
    output reg signed [15:0]  oDutyOffset,
    output reg  [15:0]        oBusVoltRef,
    // ---- 可观测中间量 ----
    output reg  [15:0]        oVAvg,
    output reg  [15:0]        oVFilter,
    output reg  [15:0]        oBusVoltFilter,
    output reg signed [15:0]  oCurrFilter,
    output reg  [15:0]        oPFilter,
    output reg signed [15:0]  oCurrAve,
    output reg  [15:0]        oPAve,
    output reg                oValid            // 提交拍有效脉冲(采样后 LATENCY=3 拍)
);
    // =====================================================================
    // 状态寄存器 (每提交拍更新)
    // =====================================================================
    reg signed [31:0] rOvrefVu;          // oVRef 内部 VU
    reg signed [31:0] rVfVu;             // volt LPF (VU)
    reg signed [31:0] rBfVu;             // bus  LPF (VU)
    reg signed [31:0] rPfX;              // pwr  LPF (0.1W << PCF_X)
    reg signed [31:0] rCfX;              // curr LPF (mA  << PCF_X)
    reg signed [31:0] rDutyOff;          // oDutyOffset Q0.16
    reg signed [63:0] rVSum;             // volt 均值窗累加(0.1V)
    reg signed [63:0] rPSum;             // pwr  均值窗累加(0.1W)
    reg signed [63:0] rCSum;             // curr 均值窗累加(mA)
    reg [9:0]         rCnt;              // 均值窗样本计数 0..AVG_N-1
    reg [9:0]         rBusrefCnt;        // 软启步进计数

    // ---- 窗口均值快照/触发 (除法完成后更新输出) ----
    reg signed [63:0] sVsum, sPsum, sCsum;
    reg rAvgPend;
    reg [31:0] rq_vavg, rq_pavg, rq_cavg;
    // ---- 除法商缓存 (控制链, oDone 打一拍后采) ----
    reg [31:0] rq_lo_curr, rq_lo_pwr, rq_lo_bus;   // 三环正方向 P (VU)
    reg [31:0] rq_ccm;                              // CCM Q16 比
    reg [31:0] rq_sn, rq_sd;                        // DCM sqrt(num)/sqrt(den)
    reg [31:0] rq_ddcm;                             // DCM Q16 占空
    reg [31:0] rq_ov100;                            // oVRef VU -> V100
    reg [31:0] rq_errtrm;                           // duty 误差修正项
    reg [31:0] rq_ndel;                             // oDutyOffset 慢积分增量
    reg [31:0] rq_vf, rq_bf;                        // volt/bus LPF -> V100

    // ---- oDone 打拍 ----
    reg rDoLC, rDoLP, rDoLB, rDoCcm, rDoSN, rDoSD, rDoDcm, rDoOv, rDoEt, rDoNd, rDoVf, rDoBf, rDoAvg;

    // =====================================================================
    // 自适应缩放函数 (64bit 幅度 -> 使 num/den 右移后均 <2^31)
    // =====================================================================
    function [5:0] f_hiidx;
        input [63:0] v;
        integer i;
        reg   got;
        begin
            f_hiidx = 6'd0; got = 1'b0;
            for (i = 63; i >= 0; i = i - 1)
                if (!got && v[i]) begin f_hiidx = i; got = 1'b1; end
        end
    endfunction

    function [5:0] f_shift;
        input [63:0] num;
        input [63:0] den;
        reg [5:0] sn, sd;
        begin
            sn = f_hiidx(num);
            sd = f_hiidx(den);
            sn = (sn > 6'd30) ? (sn - 6'd30) : 6'd0;
            sd = (sd > 6'd30) ? (sd - 6'd30) : 6'd0;
            f_shift = (sn > sd) ? sn : sd;
        end
    endfunction

    // ---------------- 有符号 64 扩展线网 ----------------
    wire signed [63:0] v_meas_64 = $signed({48'b0, iVMeas});
    wire signed [63:0] curr_64   = $signed({ {48{iCurrInst[15]}}, iCurrInst });
    wire signed [63:0] p_inst_64 = $signed({40'b0, iPInst});
    wire signed [63:0] bus_64    = $signed({48'b0, iBusVolt});
    wire signed [63:0] ov_64     = $signed({ {32{rOvrefVu[31]}}, rOvrefVu });
    wire signed [63:0] vf_64     = $signed({ {32{rVfVu[31]}},   rVfVu });
    wire signed [63:0] bf_64     = $signed({ {32{rBfVu[31]}},   rBfVu });
    wire signed [63:0] pf_64     = $signed({ {32{rPfX[31]}},    rPfX });
    wire signed [63:0] cf_64     = $signed({ {32{rCfX[31]}},    rCfX });
    wire signed [63:0] doff_64   = $signed({ {32{rDutyOff[31]}},rDutyOff });
    wire signed [63:0] a_q16     = ALPHA_VP_Q16;
    wire signed [63:0] b_q16     = ALPHA_BUS_Q16;
    wire signed [63:0] c_q16     = ALPHA_CURR_Q16;
    wire signed [63:0] c_ave_64  = $signed({ {48{oCurrAve[15]}}, oCurrAve });
    wire signed [63:0] v_meas_vu = v_meas_64 * V100_TO_VU;
    wire signed [63:0] bus_vu    = bus_64 * V100_TO_VU;
    wire signed [63:0] p_tgt     = p_inst_64 << PCF_X;
    wire signed [63:0] c_tgt     = c_ave_64 << PCF_X;

    // =====================================================================
    // LPF (一阶 EMA) 仍供除法/开方分子组合引用 (w_*), 状态提交用流水新值
    // =====================================================================
    wire signed [63:0] w_vf = vf_64 + (((v_meas_vu - vf_64) * a_q16) >>> 16);
    wire signed [63:0] w_pf = pf_64 + (((p_tgt     - pf_64) * a_q16) >>> 16);
    wire signed [63:0] w_cf = cf_64 + (((c_tgt     - cf_64) * c_q16) >>> 16);
    wire signed [63:0] w_bf = bf_64 + (((bus_vu    - bf_64) * b_q16) >>> 16);

    // =====================================================================
    // 商缓存 → 组合派生 (提交拍读当前已提交状态/缓存)
    // =====================================================================
    wire [31:0] loV = (rq_lo_curr > 32'(LOOP_CLAMP_VU)) ? 32'(LOOP_CLAMP_VU) : rq_lo_curr;
    wire [31:0] loP = (rq_lo_pwr  > 32'(LOOP_CLAMP_VU)) ? 32'(LOOP_CLAMP_VU) : rq_lo_pwr;
    wire [31:0] loB = (rq_lo_bus  > 32'(LOOP_CLAMP_VU)) ? 32'(LOOP_CLAMP_VU) : rq_lo_bus;
    wire signed [63:0] ovn_64 = ov_64;                        // 当前已提交 oVRef(VU)

    // ---- CCM: q_ccm(Q16) = (ov<<16)/bf ; 使用 64bit 全值做分支, 缩放喂除法 ----
    wire signed [63:0] ccm_num = ovn_64 << 16;
    wire        ccm_den_ok = (rBfVu > 32'sd0);
    wire        ccm_ovf    = ccm_den_ok && (($unsigned(ccm_num) >> 32) >= $unsigned(bf_64));
    wire [63:0] ccm_dn_num = (ccm_den_ok && !ccm_ovf) ? $unsigned(ccm_num) : 64'd0;
    wire [63:0] ccm_dn_den = ccm_den_ok ? $unsigned(bf_64) : 64'd1;
    wire signed [63:0] ccm_rat = ccm_den_ok ?
                                 (ccm_ovf ? 64'h7FFF_FFFF_FFFF_FFFF : $signed({32'd0, rq_ccm})) : 64'sd0;
    wire signed [63:0] ccm_calc = 64'd65536 - ccm_rat;
    wire signed [31:0] ccm_cl   = (ccm_calc < 0) ? 32'sd0 :
                                  ((ccm_calc > 64'd65535) ? 32'd65535 : ccm_calc[31:0]);

    // ---- DCM: ddcm(Q16) = (sqrt(num_dcm)<<16)/sqrt(den_dcm) ----
    wire signed [63:0] cf_ma  = cf_64 >>> PCF_X;
    wire signed [63:0] cf_abs = (cf_ma < 0) ? -cf_ma : cf_ma;
    wire signed [63:0] pvk    = cf_abs * PV_K_VU_PER_MA;
    wire signed [63:0] bfo    = bf_64 - ovn_64;
    wire signed [63:0] bfo_abs= (bfo < 0) ? -bfo : bfo;
    wire signed [63:0] num_dcm= pvk * bfo_abs;
    wire signed [63:0] den_dcm= ovn_64 * bf_64;
    wire dcm_ratio_hi = (den_dcm > 64'sd0) && (num_dcm >= den_dcm);
    wire signed [31:0] ddcm_raw = dcm_ratio_hi ? 32'd65536 : $signed(rq_ddcm);
    wire signed [31:0] ddcm_cl  = (ddcm_raw > 64'd65535) ? 32'd65535 : ddcm_raw;

    // ---- v_err = oVRef - volt_inst (0.1V) ----
    wire signed [63:0] ov_v100_64 = $signed({32'd0, rq_ov100});
    wire signed [31:0] v_err = ov_v100_64[31:0] - v_meas_64[31:0];
    wire signed [63:0] v_err64 = $signed({ {32{v_err[31]}}, v_err });
    wire signed [63:0] v_err_abs = (v_err64 < 0) ? -v_err64 : v_err64;
    wire big_err = (v_err_abs > DBIAS_THD_V100);

    // ---- oDutyOffset 更新量 (leak 乘法已流水寄存, 此处理解为商/比较) ----
    wire signed [63:0] n_del  = (v_err64 < 0) ? -$signed({32'd0, rq_ndel})
                                              :  $signed({32'd0, rq_ndel});
    wire signed [63:0] off_u  = big_err ? (doff_64 - n_del) : (doff_64 - rLeak >>> 16);
    wire signed [63:0] off_ca = (off_u >  DBIAS_MAX_Q16) ? DBIAS_MAX_Q16 :
                                (off_u < -DBIAS_MAX_Q16) ? (-DBIAS_MAX_Q16) : off_u;
    wire signed [31:0] off_cl = off_ca[31:0];

    // ---- 融合占空比 (权重混合乘法已流水寄存 rBl1/rBl2) ----
    wire signed [63:0] alpha64    = $signed({48'b0, iAlphaHat}); // Q0.16 CCM 权重(无符号)
    wire signed [63:0] om_alpha64 = 64'd65536 - alpha64;
    wire signed [63:0] err_trm = (v_err64 < 0) ? -$signed({32'd0, rq_errtrm})
                                               :  $signed({32'd0, rq_errtrm});
    wire signed [63:0] blend     = (rBl1 + rBl2) >>> 16;
    wire signed [63:0] duty_t0    = blend + off_ca - err_trm;
    wire signed [63:0] duty_t1    = (duty_t0 > $signed({ {32{ccm_cl[31]}}, ccm_cl })) ?
                                    $signed({ {32{ccm_cl[31]}}, ccm_cl }) : duty_t0;
    wire signed [63:0] duty_t2    = (duty_t1 > DUTY_MAX_Q16) ? DUTY_MAX_Q16 : duty_t1;
    wire signed [63:0] duty_t3    = (duty_t2 < DUTY_MIN_Q16) ? DUTY_MIN_Q16 : duty_t2;
    wire [15:0]        nxt_duty   = (iEnRun) ? duty_t3[15:0] : 16'd0;

    // ---- oVRef 积分式 (三环只增 + MPPT前瞻), 限幅 [MIN,bus] ----
    wire signed [63:0] mppt_fwd = (iEnRun) ? $signed({ {48{iMpptDVref[15]}}, iMpptDVref }) : 64'sd0;
    wire signed [63:0] ov_accum = ov_64 + $signed({32'd0, loV}) +
                                  $signed({32'd0, loP}) + $signed({32'd0, loB}) + mppt_fwd;
    wire signed [63:0] ov_h = (ov_accum > rXB) ? rXB : ov_accum;         // bus_vu 用流水寄存
    wire signed [63:0] ov_l = (ov_h < (VREF_MIN_V100 * V100_TO_VU)) ?
                              (VREF_MIN_V100 * V100_TO_VU) : ov_h;
    wire signed [63:0] ov_nxt = (iEnRun) ? ov_l : ov_64;

    // ---- 软启参考斜坡 (提交拍, 纯加) ----
    wire        win_last = (rCnt == (AVG_N - 1));
    wire signed [63:0] nxt_v_sum = rVSum + v_meas_64;
    wire signed [63:0] nxt_p_sum = rPSum + p_inst_64;
    wire signed [63:0] nxt_c_sum = rCSum + curr_64;
    wire        ss_edge = (rBusrefCnt == (SOFT_PERIOD - 1));
    wire [9:0]  n_cnt_ss = (iSoftEn && ss_edge) ? 10'd0 :
                           (iSoftEn) ? (rBusrefCnt + 10'd1) : rBusrefCnt;
    wire [15:0] ss_inc  = oBusVoltRef + SOFT_STEP_V100;
    wire [15:0] n_ref_ss = (iSoftEn && ss_edge) ?
                           ((ss_inc >= BUSREF_MAX_V100) ? BUSREF_MAX_V100[15:0] : ss_inc)
                           : oBusVoltRef;

    // =====================================================================
    // 多拍流水寄存器: rEv 事件移位; 每 iTick 一拍尺度乘法 / 乘法 / 移位加 / 提交
    // =====================================================================
    reg [2:0] rEv;
    reg       rBusy;
    reg signed [63:0] rXV;      // V 输入 *V100 (尺度乘法一拍)
    reg signed [63:0] rXB;      // bus 输入*V100 (尺度乘法一拍)
    reg signed [63:0] rPv, rPb, rPc, rPp;   // LPF 差*alpha 乘积(一拍, 各单寄存)
    reg signed [63:0] rLeak;    // oDutyOffset 衰减乘积(一拍)
    reg signed [63:0] rBl1;     // alpha*Dccm (一拍)
    reg signed [63:0] rBl2;     // (1-alpha)*Ddcm (一拍)
    reg signed [63:0] rVfN, rBfN, rPfN, rCfN;   // 新 LPF 状态(移位加一拍)
    wire signed [63:0] wCfMaN = rCfN >>> PCF_X;  // curr 滤波输出 缩放(组合自流水态)
    wire signed [63:0] wPfMaN = rPfN >>> PCF_X;  // pwr  滤波输出 缩放

    // 事件移位 + 输入尺度乘法 (S0)
    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            rEv   <= 3'b000;
            rBusy <= 1'b0;
            rXV   <= 64'sd0;
            rXB   <= 64'sd0;
        end else begin
            rEv[2] <= rEv[1];
            rEv[1] <= rEv[0];
            rEv[0] <= 1'b0;
            if (iTick && !rBusy) begin
                rEv[0] <= 1'b1;
                rXV    <= v_meas_64 * V100_TO_VU;   // 一拍: V 尺度乘法
                rXB    <= bus_64    * V100_TO_VU;   // 一拍: bus 尺度乘法(并行)
                rBusy  <= 1'b1;
            end
            if (rEv[2]) rBusy <= 1'b0;              // 提交完成后空闲
        end
    end

    // 多拍计算 + 提交 (S1 乘法 / S2 移位加 / S3 提交)
    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            rPv <= 64'sd0; rPb <= 64'sd0; rPc <= 64'sd0; rPp <= 64'sd0;
            rLeak <= 64'sd0; rBl1 <= 64'sd0; rBl2 <= 64'sd0;
            rVfN <= 64'sd0; rBfN <= 64'sd0; rPfN <= 64'sd0; rCfN <= 64'sd0;
            rVfVu <= 32'sd0; rBfVu <= 32'sd0; rPfX <= 32'sd0; rCfX <= 32'sd0;
            rOvrefVu <= VREF_INIT_V100 * V100_TO_VU;
            rDutyOff <= 32'sd0;
            rVSum <= 64'sd0; rPSum <= 64'sd0; rCSum <= 64'sd0;
            rCnt <= 10'd0; rBusrefCnt <= 10'd0;
            oVRef <= VREF_INIT_V100[15:0];
            oDutyQ <= 16'd0; oDCcm <= 16'd0; oDDcm <= 16'd0;
            oDutyOffset <= 16'sd0; oBusVoltRef <= 16'd0;
            oVFilter <= 16'd0; oBusVoltFilter <= 16'd0;
            oCurrFilter <= 16'sd0; oPFilter <= 16'd0;
            oValid <= 1'b0;
        end else begin
            oValid <= 1'b0;
            // ---- S1: 各乘法并行一拍, 结果分别寄存 ----
            if (rEv[0]) begin
                rPv   <= (rXV - vf_64) * a_q16;          // volt LPF 乘积
                rPb   <= (rXB - bf_64) * b_q16;          // bus  LPF 乘积
                rPc   <= (c_tgt - cf_64) * c_q16;        // curr LPF 乘积
                rPp   <= (p_tgt - pf_64) * a_q16;        // pwr  LPF 乘积
                rLeak <= doff_64 * LEAK_Q16;             // 偏置衰减乘积
                rBl1  <= alpha64 * $signed({ {32{ccm_cl[31]}}, ccm_cl });   // CCM 权重
                rBl2  <= om_alpha64 * $signed({ {32{ddcm_cl[31]}}, ddcm_cl }); // DCM 权重
            end
            // ---- S2: 移位加得新 LPF 状态 ----
            if (rEv[1]) begin
                rVfN <= vf_64 + (rPv >>> 16);
                rBfN <= bf_64 + (rPb >>> 16);
                rPfN <= pf_64 + (rPp >>> 16);
                rCfN <= cf_64 + (rPc >>> 16);
            end
            // ---- S3: 提交全部状态/输出 (无乘法, 纯加/限幅/比较) ----
            // 均值快照/rAvgPend/oVAvg* 仅由下方 Avg FSM always 驱动，避免多驱动
            if (rEv[2]) begin
                if (win_last) begin
                    rCnt   <= 10'd0;
                    rVSum  <= 64'sd0;
                    rPSum  <= 64'sd0;
                    rCSum  <= 64'sd0;
                end else begin
                    rCnt   <= rCnt + 10'd1;
                    rVSum  <= nxt_v_sum[63:0];
                    rPSum  <= nxt_p_sum[63:0];
                    rCSum  <= nxt_c_sum[63:0];
                end
                // LPF / oVRef / 软启 / 偏置 (读流水新 LPF 态)
                rVfVu        <= rVfN[31:0];
                rPfX         <= rPfN[31:0];
                rCfX         <= rCfN[31:0];
                rBfVu        <= rBfN[31:0];
                rOvrefVu     <= ov_nxt[31:0];
                rDutyOff     <= off_cl;
                rBusrefCnt   <= n_cnt_ss;
                oBusVoltRef <= n_ref_ss;
                // 输出 (读最新商缓存; 均值窗由 AvgDiv 完成拍更新)
                oVRef          <= rq_ov100[15:0];
                oDutyQ         <= nxt_duty;
                oDCcm          <= ccm_cl[15:0];
                oDDcm          <= ddcm_cl[15:0];
                oDutyOffset    <= off_cl[15:0];
                oVFilter       <= rq_vf[15:0];
                oBusVoltFilter <= rq_bf[15:0];
                oCurrFilter    <= wCfMaN[15:0];
                oPFilter       <= wPfMaN[15:0];
                oValid         <= 1'b1;
            end
        end
    end

    // =====================================================================
    // 多拍除法例化 (控制链: 空闲自动重启连续跟踪)   ———— 见文件下方实例区
    // =====================================================================
    // (声明区: 忙/完成/商线网在下方实例处声明, 但商缓存采样统一在末尾 always)
    // ------- 均值窗专用除法(窗满触发) -------
    wire oBusyAvg, oDoneAvg; wire [31:0] oQuotAvg;

    // ------- 控制链除法/开方 -------
    wire oBusyLC,  oDoneLC;  wire [31:0] oQuotLC;
    wire oBusyLP,  oDoneLP;  wire [31:0] oQuotLP;
    wire oBusyLB,  oDoneLB;  wire [31:0] oQuotLB;
    wire oBusyCcm, oDoneCcm; wire [31:0] oQuotCcm;
    wire oBusySN,  oDoneSN;  wire [31:0] oQuotSN;
    wire oBusySD,  oDoneSD;  wire [31:0] oQuotSD;
    wire oBusyDcm, oDoneDcm; wire [31:0] oQuotDcm;
    wire oBusyOv,  oDoneOv;  wire [31:0] oQuotOv;
    wire oBusyEt,  oDoneEt;  wire [31:0] oQuotEt;
    wire oBusyNd,  oDoneNd;  wire [31:0] oQuotNd;
    wire oBusyVf,  oDoneVf;  wire [31:0] oQuotVf;
    wire oBusyBf,  oDoneBf;  wire [31:0] oQuotBf;

    // =====================================================================
    // 窗口均值除法触发: 窗满(提交拍)快照并置 pend, FSM 顺序除 V/P/C
    //   电流均值带符号: 取模后除再回置符号(对齐原截断商)
    // =====================================================================
    localparam [1:0] A_IDLE=2'd0, A_V=2'd1, A_P=2'd2, A_C=2'd3;
    reg [1:0] rAvgSt;
    reg       rAvgGo;                       // 一拍启动脉冲
    reg [31:0] avg_num;
    wire [31:0] avg_in_den = 32'(AVG_N);

    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            rAvgSt  <= A_IDLE;
            rAvgPend<= 1'b0;
            rAvgGo  <= 1'b0;
            sVsum <= 64'sd0; sPsum <= 64'sd0; sCsum <= 64'sd0;
            avg_num <= 32'd0;
            rq_vavg <= 32'd0; rq_pavg <= 32'd0; rq_cavg <= 32'd0;
            oVAvg <= 16'd0; oPAve <= 16'd0; oCurrAve <= 16'sd0;
        end else begin
            rAvgGo <= 1'b0;
            // 窗满且均值 FSM 空闲：本 always 独占快照/pend（busy 时丢窗，避免覆盖）
            if (rEv[2] && win_last && (rAvgSt == A_IDLE) && !rAvgPend) begin
                sVsum   <= nxt_v_sum;
                sPsum   <= nxt_p_sum;
                sCsum   <= nxt_c_sum;
                avg_num <= nxt_v_sum[31:0];
                rAvgPend<= 1'b1;
            end
            case (rAvgSt)
                A_IDLE: begin
                    if (rAvgPend) begin
                        rAvgPend<= 1'b0;
                        rAvgSt <= A_V;
                        rAvgGo <= 1'b1;      // avg_num 已为本窗 V 分子
                    end
                end
                A_V: if (oDoneAvg) begin
                        rq_vavg <= oQuotAvg;
                        avg_num <= sPsum[31:0];
                        rAvgSt  <= A_P;
                        rAvgGo  <= 1'b1;
                     end
                A_P: if (oDoneAvg) begin
                        rq_pavg <= oQuotAvg;
                        avg_num <= (sCsum < 64'sd0) ? $unsigned(-sCsum) : $unsigned(sCsum);
                        rAvgSt  <= A_C;
                        rAvgGo  <= 1'b1;
                     end
                A_C: if (oDoneAvg) begin
                        rq_cavg <= oQuotAvg;
                        oVAvg    <= rq_vavg[15:0];
                        oPAve    <= rq_pavg[15:0];
                        oCurrAve <= (sCsum < 64'sd0) ? (-$signed({32'd0, oQuotAvg}))
                                                     :  $signed({32'd0, oQuotAvg});
                        rAvgSt  <= A_IDLE;
                     end
                default: rAvgSt <= A_IDLE;
            endcase
        end
    end

    AlgoDiv #(.W(32)) U_AvgDiv (
        .iSysClk(iSysClk), .iSysRst(iSysRst),
        .iStart(rAvgGo),
        .iNum(avg_num), .iDen(avg_in_den),
        .oBusy(oBusyAvg), .oDone(oDoneAvg), .oQuot(oQuotAvg));

    // =====================================================================
    // 控制链除法/开方实例 (各除法独立, 空闲自动重启)
    // =====================================================================
    // ---- 三环正方向 P: 分子=(e>0)?e*K:0; 全部 <2^31 直接喂 ----
    wire [63:0] lo_num_curr = (e_curr > 64'sd0) ? $unsigned(lo_mul_curr) : 64'd0;
    wire signed [63:0] e_curr = curr_64 - CURR_MAX_MA;
    wire signed [63:0] pf_100 = pf_64 >>> PCF_X;
    wire signed [63:0] e_pwr  = pf_100 - $signed({40'b0, iPwrLimit});
    wire signed [63:0] e_bus  = rXB - (BUS_MAX_V100 * V100_TO_VU);
    wire signed [63:0] lo_mul_curr = e_curr * 64'sd1;
    wire signed [63:0] lo_mul_pwr  = e_pwr  * 64'sd5;
    wire signed [63:0] lo_mul_bus  = e_bus  * 64'sd1;
    wire [63:0] lo_num_pwr = (e_pwr  > 64'sd0) ? $unsigned(lo_mul_pwr) : 64'd0;
    wire [63:0] lo_num_bus = (e_bus  > 64'sd0) ? $unsigned(lo_mul_bus) : 64'd0;

    AlgoDiv #(.W(32)) U_LoCurr (
        .iSysClk(iSysClk), .iSysRst(iSysRst), .iStart(~oBusyLC),
        .iNum(lo_num_curr[31:0]), .iDen(32'd2),
        .oBusy(oBusyLC), .oDone(oDoneLC), .oQuot(oQuotLC));
    AlgoDiv #(.W(32)) U_LoPwr (
        .iSysClk(iSysClk), .iSysRst(iSysRst), .iStart(~oBusyLP),
        .iNum(lo_num_pwr[31:0]), .iDen(32'd2),
        .oBusy(oBusyLP), .oDone(oDoneLP), .oQuot(oQuotLP));
    AlgoDiv #(.W(32)) U_LoBus (
        .iSysClk(iSysClk), .iSysRst(iSysRst), .iStart(~oBusyLB),
        .iNum(lo_num_bus[31:0]), .iDen(32'd10000),
        .oBusy(oBusyLB), .oDone(oDoneLB), .oQuot(oQuotLB));

    // ---- CCM: floor( (ov<<16) / bf ) (Q16), 自适应缩放 ----
    wire [5:0] shCcm = f_shift(ccm_dn_num, ccm_dn_den);
    wire [31:0] ccm_num32 = (ccm_dn_num >> shCcm);
    wire [31:0] ccm_den32 = (ccm_dn_den >> shCcm);
    AlgoDiv #(.W(32)) U_CcmDiv (
        .iSysClk(iSysClk), .iSysRst(iSysRst), .iStart(~oBusyCcm),
        .iNum(ccm_num32), .iDen((ccm_den32 == 32'd0) ? 32'd1 : ccm_den32),
        .oBusy(oBusyCcm), .oDone(oDoneCcm), .oQuot(oQuotCcm));

    // ---- DCM: sqrt(num_dcm), sqrt(den_dcm) 独立开方 ----
    AlgoSqrt #(.IW(64), .OW(32)) U_SqrtNum (
        .iSysClk(iSysClk), .iSysRst(iSysRst), .iStart(~oBusySN),
        .iNum($unsigned(num_dcm)),
        .oBusy(oBusySN), .oDone(oDoneSN), .oRoot(oQuotSN));
    AlgoSqrt #(.IW(64), .OW(32)) U_SqrtDen (
        .iSysClk(iSysClk), .iSysRst(iSysRst), .iStart(~oBusySD),
        .iNum($unsigned(den_dcm)),
        .oBusy(oBusySD), .oDone(oDoneSD), .oRoot(oQuotSD));

    // ---- DCM 比: (sqrt(num)<<16)/sqrt(den)  (Q16), 自适应缩放 ----
    wire [63:0] dcm_ratio_num = ({32'd0, rq_sn}) << 16;   // <= 2^48
    wire [63:0] dcm_ratio_den = {32'd0, rq_sd};
    wire [5:0] shDcm = f_shift(dcm_ratio_num, dcm_ratio_den);
    wire [31:0] dcm_num32 = (dcm_ratio_num >> shDcm);
    wire [31:0] dcm_den32 = (dcm_ratio_den >> shDcm);
    AlgoDiv #(.W(32)) U_DcmDiv (
        .iSysClk(iSysClk), .iSysRst(iSysRst), .iStart(~oBusyDcm),
        .iNum(dcm_num32), .iDen((dcm_den32 == 32'd0) ? 32'd1 : dcm_den32),
        .oBusy(oBusyDcm), .oDone(oDoneDcm), .oQuot(oQuotDcm));

    // ---- oVRef VU -> V100 = rOvrefVu/500 ; 电流环分母等 (数值 <2^31 直接喂) ----
    wire [63:0] ov100_num = $unsigned(ovn_64);
    AlgoDiv #(.W(32)) U_Ov100Div (
        .iSysClk(iSysClk), .iSysRst(iSysRst), .iStart(~oBusyOv),
        .iNum(ov100_num[31:0]), .iDen(32'(V100_TO_VU)),
        .oBusy(oBusyOv), .oDone(oDoneOv), .oQuot(oQuotOv));

    // ---- duty 误差修正 / oDutyOffset 慢积分增量 (分子 |v_err*65536|) ----
    wire signed [63:0] et_num = v_err64 * 64'sd65536;
    wire signed [63:0] nd_num = v_err64 * 64'sd65536;
    wire [63:0] et_mag = (et_num < 64'sd0) ? $unsigned(-et_num) : $unsigned(et_num);
    wire [63:0] nd_mag = (nd_num < 64'sd0) ? $unsigned(-nd_num) : $unsigned(nd_num);
    AlgoDiv #(.W(32)) U_ErrTrDiv (
        .iSysClk(iSysClk), .iSysRst(iSysRst), .iStart(~oBusyEt),
        .iNum(et_mag[31:0]), .iDen(32'd1000),
        .oBusy(oBusyEt), .oDone(oDoneEt), .oQuot(oQuotEt));
    AlgoDiv #(.W(32)) U_NdelDiv (
        .iSysClk(iSysClk), .iSysRst(iSysRst), .iStart(~oBusyNd),
        .iNum(nd_mag[31:0]), .iDen(32'd1000000),
        .oBusy(oBusyNd), .oDone(oDoneNd), .oQuot(oQuotNd));

    // ---- LPF 输出 VU -> V100 (oVFilter/oBusVoltFilter) ----
    AlgoDiv #(.W(32)) U_VfV100 (
        .iSysClk(iSysClk), .iSysRst(iSysRst), .iStart(~oBusyVf),
        .iNum(w_vf[31:0]), .iDen(32'(V100_TO_VU)),
        .oBusy(oBusyVf), .oDone(oDoneVf), .oQuot(oQuotVf));
    AlgoDiv #(.W(32)) U_BfV100 (
        .iSysClk(iSysClk), .iSysRst(iSysRst), .iStart(~oBusyBf),
        .iNum(w_bf[31:0]), .iDen(32'(V100_TO_VU)),
        .oBusy(oBusyBf), .oDone(oDoneBf), .oQuot(oQuotBf));

    // =====================================================================
    // oDone 打一拍后采商 (控制链)
    // =====================================================================
    always @(posedge iSysClk or posedge iSysRst) begin
        if (iSysRst) begin
            rq_lo_curr <= 32'd0; rq_lo_pwr <= 32'd0; rq_lo_bus <= 32'd0;
            rq_ccm <= 32'd0; rq_sn <= 32'd0; rq_sd <= 32'd0; rq_ddcm <= 32'd0;
            rq_ov100 <= 32'd0; rq_errtrm <= 32'd0; rq_ndel <= 32'd0;
            rq_vf <= 32'd0; rq_bf <= 32'd0;
            rDoLC <= 1'b0; rDoLP <= 1'b0; rDoLB <= 1'b0; rDoCcm <= 1'b0;
            rDoSN <= 1'b0; rDoSD <= 1'b0; rDoDcm <= 1'b0; rDoOv <= 1'b0;
            rDoEt <= 1'b0; rDoNd <= 1'b0; rDoVf <= 1'b0; rDoBf <= 1'b0;
        end else begin
            rDoLC <= oDoneLC;  if (rDoLC) rq_lo_curr <= oQuotLC;
            rDoLP <= oDoneLP;  if (rDoLP) rq_lo_pwr  <= oQuotLP;
            rDoLB <= oDoneLB;  if (rDoLB) rq_lo_bus  <= oQuotLB;
            rDoCcm<= oDoneCcm; if (rDoCcm) rq_ccm    <= oQuotCcm;
            rDoSN <= oDoneSN;  if (rDoSN) rq_sn      <= oQuotSN;
            rDoSD <= oDoneSD;  if (rDoSD) rq_sd      <= oQuotSD;
            rDoDcm<= oDoneDcm; if (rDoDcm) rq_ddcm   <= oQuotDcm;
            rDoOv <= oDoneOv;  if (rDoOv) rq_ov100   <= oQuotOv;
            rDoEt <= oDoneEt;  if (rDoEt) rq_errtrm  <= oQuotEt;
            rDoNd <= oDoneNd;  if (rDoNd) rq_ndel    <= oQuotNd;
            rDoVf <= oDoneVf;  if (rDoVf) rq_vf      <= oQuotVf;
            rDoBf <= oDoneBf;  if (rDoBf) rq_bf      <= oQuotBf;
        end
    end
endmodule
