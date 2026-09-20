# Algorithm_Framework_HDL

电力电子控制算法的 **可综合 HDL 库**（Verilog，命名对齐 `FPGA_standard` Rev 1.5：PascalCase 模块 + `i/o/r/w` 前缀 + `U_` 实例 + 高有效复位 `iSysRst`）。

对应 C 侧 `Algorithm_Framework/AlgoFw`（仅作算法规格参考，结构/公式对齐、定点化），并作为配置的**单一来源**：
- `rtl/include/pv_cfg.vh`、`rtl/include/algo_types.vh`、`rtl/include/algo_arith.vh` 供本库与仿真 demo（`Algo_Demo_pv_HDL`）共同 `-I` 引用。
- 实现规范 `docs/HDL_IMPL_CONVENTIONS.md`、命名契约 `docs/FPGA_STD_RENAME_SPEC.md`。

## 目录

```
Algorithm_Framework_HDL/
├── rtl/include/             pv_cfg.vh, algo_types.vh, algo_arith.vh
├── rtl/verilog/
│   ├── math/                AlgoSqrt / AlgoDiv / AlgoTrig(+mem)
│   ├── controller/          CompPi / CompPr / CompPir（双线性离散）
│   ├── filter/              标量滤波 + 巴特沃斯低通（见下表）
│   ├── transform/           Clark / Park / 正负序分离
│   ├── ortho/               SOGI / 直流提取 / PLL / FLL
│   ├── corr/                GCC / XCorr 互相关
│   ├── fft/                 CompFft（基-2 DIT）
│   ├── power/               前馈解耦 / 软启 / 死区
│   └── cntl/pv/             PV 控制链（见下表）
├── tb/verilog/              每模块单元 TB（按功能分目录）
├── reviews/                 代码审查报告
└── Makefile                 analyze / lint / unit
```

## 已实现模块（2026-09-06）

### cntl/pv（PV 控制链）
| 模块 | 说明 | 单测 |
|------|------|------|
| `PvLoop` | 位置式 PI（移位增益、抗饱和、限幅） | PASS |
| `PvCalc` | 一阶 EMA 低通 | PASS |
| `PvCalcFull` | 均值窗500+4路LPF+母线软启+三环+CCM/DCM占空比 | PASS |
| `PvMppt` | P&O hill-climb（10Hz，方向记忆+边界反转） | PASS |
| `PvFault` | 过流/短路/反接(锁存)+欠压/过压(回差) | PASS |
| `PvStateSys` | 系统 5 态 FSM（三段式） | PASS |
| `PvStateCh` | 通道 4 态 FSM + valid 计数（三段式） | PASS |
| `PvObserver` | 清洗版标量 KF（CCM/DCM 混合系数 + Voc） | PASS |

### controller（电力电子控制器，双线性离散）
| 模块 | 说明 | 单测 |
|------|------|------|
| `CompPi` | PI 控制器（双线性梯形积分 + 抗饱和） | PASS（阶跃无静差） |
| `CompPr` | PR 谐振控制器（Kp + 二阶谐振 IIR） | PASS（50Hz 跟踪误差 0.58%） |
| `CompPir` | PIR（PI + 谐振并联） | PASS（直流+基波跟踪） |
| `CompMultiPr` | 多谐振 PR（基波+5/7 次谐波并联） | PASS（双频误差趋小） |
| `CompRep` | 重复控制器 RC（z⁻ᴺ/(1-Q·z⁻ᴺ)） | PASS（周期误差学习） |

### fft（谐波分析）
| 模块 | 说明 | 单测 |
|------|------|------|
| `CompFft` | 基-2 DIT FFT（N=16/64，Q15，谱峰验证） | PASS |
| `CompGoertzel` | Goertzel 单频检测（|X[k]|≈A·N/2） | PASS |

### power（电源专用）
| 模块 | 说明 | 单测 |
|------|------|------|
| `CompFfDecouple` | 三相 dq 前馈解耦（vd_ff=-ωL·iq） | PASS |
| `CompSoftStart` | 软启动斜坡（吸附目标不过冲） | PASS |
| `CompDeadTime` | 死区补偿（按电流方向） | PASS |

### math（数学运算库，多拍 start/done 握手）
| 模块 | 说明 | 单测 |
|------|------|------|
| `AlgoSqrt` | 整数开方库（多拍逐位迭代） | PASS |
| `AlgoDiv` | 有符号恢复除法库（多拍迭代，除零 0） | PASS |
| `AlgoTrig`(+`.mem`) | 1024 点 sin/cos ROM | PASS |

### filter（滤波）
| 模块 | 说明 | 单测 |
|------|------|------|
| `CompLpf1`/`CompLpf2` | 一阶/二阶**巴特沃斯低通**（双线性） | PASS |
| `CompHpf2`/`CompBpf2`/`CompBsf2` | 二阶**巴特沃斯**高通/带通/带阻（双线性） | PASS |
| `CompHpFilter`/`CompDiff`/`CompNotch` | 一阶高通/微分/陷波 | PASS |
| `CompMovAvg`/`CompMedian`/`CompRateLimit` | 滑动平均/中值/限幅速率 | PASS |
| `CompRms`/`CompNoiseVar`/`CompKalman1d`/`CompLmsAlpha` | RMS/方差/卡尔曼/LMS | PASS |

### transform（坐标变换）
| 模块 | 说明 | 单测 |
|------|------|------|
| `CompClark`(+Inv)/`CompPark`(+Inv) | Clarke/Park 变换 | PASS |
| `CompAbc2pnz`/`Comp3phDecouple` | 正负序分离/三相解耦 | PASS |

### ortho（正交/锁相）
| 模块 | 说明 | 单测 |
|------|------|------|
| `CompSogi`/`CompDcExtract` | SOGI / 直流提取 | PASS |
| `CompPll`/`CompFll3ph` | 单相 PLL / 三相 SOGI-FLL | PASS |
| `CompDogi`/`CompSrfPll` | DSOGI-FLL / SRF-PLL（dq 锁相） | PASS |

### corr（互相关）
| 模块 | 说明 | 单测 |
|------|------|------|
| `CompGcc`/`CompXcorr` | 互相关方向/时延 | PASS |

自检：`make analyze && make lint && make unit`（48 个 TB 全 PASS，lint 0 error；warning 另见 Verilator 输出）。

## 审查报告

代码审查报告见 `reviews/`（如 `reviews/CODE_REVIEW_2026-09-19.md`）。

## 使用约定

- 头文件只放 `rtl/include/`；模块用 `` `include "pv_cfg.vh" `` / `"algo_types.vh"`。
- 可综合铁律：无 `real` 运算通路；**除法/开方统一例化 `AlgoDiv`/`AlgoSqrt` 数学库**（规范 §6.3，多拍 `iStart/oDone` 握手，参照 ROCA `SignedDivision`），业务模块禁止组合除法/自行摊开。
- 命名遵循 `FPGA_standard` + `AI_HDL_RTL_Development_Guideline`（详见 `docs/FPGA_STD_RENAME_SPEC.md`）。

## 文档

方案与工作流见 `Algo_Demo_pv_HDL`（闭环仿真仓）的 README 与
`/home/win/wsl-proj/Algo_HDL_Sim/docs/方案总览.md`（权威总览）、
`/home/win/wsl-proj/FPGA_standard/FPGA编程标准与规范.md`（编码规范原文）。
