// ---------------------------------------------------------------------------
// pv_cfg.vh  ——  PV 控制参数/节拍 单一配置源（本库 + Algo_Demo_pv_HDL 共用）
// 数值与 Algorithm_Framework/_develop_proj_sim/pv_cfg.h 对齐（人工同步）
// ---------------------------------------------------------------------------
`ifndef PV_CFG_VH
`define PV_CFG_VH

// --- 时钟与节拍 ---
`define PV_CLK_HZ              100_000_000             //系统时钟100MHz
`define PV_LOOP_FS             5000                    // 控制环采样频率5000Hz
`define PV_MPPT_CNTL_FS        10                      // MPPT 频率10Hz
`define PV_LOOP_DIV            (`PV_CLK_HZ / `PV_LOOP_FS)        // 20000 @100MHz
`define PV_MPPT_CNT            (`PV_LOOP_FS / `PV_MPPT_CNTL_FS)  // MPPT 周期500
`define PV_AVG_SAMPLES         (`PV_MPPT_CNT)      // 均值窗 = MPPT 周期500个采样点

// --- 占空比限幅（Q0.8：0..255 ↔ 0..1） ---
`define PV_DUTY_MAX            8'd242             // ≈0.95
`define PV_DUTY_MIN            8'd13              // ≈0.05

// --- MPPT 参数（Phase 3 用） ---
`define PV_MPPT_D_VREF_MAX     20                 // V/s 参考斜率上限，电压参考变化率上限
`define PV_MPPT_D_VREF_K       40                 //MPPT调节Vref的增益/系数

// --- scaffold 测试参数（PvLoop 单测等用） ---
`define PV_SCAFFOLD_VREF       20000              // P 控制器参考电压（scaffold 域）

`endif
