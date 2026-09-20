// ---------------------------------------------------------------------------
// inv3ph_cfg.vh —— 三相逆变器 FPGA 配置（采样率、故障阈值、状态编码）
// 本库与上层 demo 共同 -I 引用。
// 修订: 2026-09-20 v0.1
// ---------------------------------------------------------------------------
`ifndef INV3PH_CFG_VH
`define INV3PH_CFG_VH

// --- 硬件 LC（定点换算见各模块 parameter 默认值）---
`define INV3PH_HW_C1_UF            8          // 8 uF
`define INV3PH_HW_L1_UH            900        // 900 uH
`define INV3PH_HW_L2_UH            30         // 30 uH

// --- 额定与节拍 ---
`define INV3PH_BUS_VOL_NORMAL      800        // V
`define INV3PH_VOL_PEAK_NORMAL     325        // V 相电压峰值（RMS≈230）
`define INV3PH_FREQ_NORMAL         50         // Hz
`define INV3PH_CNTL_FS             20000      // 控制环采样 Hz
`define INV3PH_SOFT_START_VPS      100        // 软启斜率 V/s

// --- 故障阈值（正常电压档，整数 V / A）---
`define INV3PH_ERR_BUS_VOL_LOW         640
`define INV3PH_ERR_BUS_VOL_OVER        950
`define INV3PH_ERR_BUS_PEAK_OVER       1050
`define INV3PH_ERR_INV_VOL_RMS_LOW     176
`define INV3PH_ERR_INV_VOL_INST_OVER   450
`define INV3PH_ERR_INV_VOL_RMS_OVER    264
`define INV3PH_ERR_INV_CUR_INST_OVER   42
`define INV3PH_ERR_INV_CUR_RMS_OVER    30
`define INV3PH_ERR_OVERLOAD_PHASE_1    6000
`define INV3PH_ERR_OVERLOAD_TOTAL_1    18000
`define INV3PH_ERR_OVERLOAD_PHASE_2    6400
`define INV3PH_ERR_OVERLOAD_TOTAL_2    19000
`define INV3PH_ERR_OVERLOAD_1_TIME     50
`define INV3PH_ERR_OVERLOAD_2_TIME     5

// --- 状态编码（与 EN_INV3PH_STATE 对齐）---
`define INV3PH_ST_IDLE            3'd0
`define INV3PH_ST_ERR             3'd1
`define INV3PH_ST_SELFCHECK       3'd2
`define INV3PH_ST_STARTING        3'd3
`define INV3PH_ST_DISCONNECTED    3'd4
`define INV3PH_ST_CONNECTING      3'd5
`define INV3PH_ST_CONNECTED       3'd6
`define INV3PH_ST_DISCONNECTING   3'd7

`endif
