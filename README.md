# Algorithm_Framework_HDL

面向 FPGA 的电力电子 / 电机控制 **可综合 Verilog 算法库**（命名对齐 `FPGA_standard` Rev 1.5：PascalCase 模块 + `i/o/r/w` 前缀 + `U_` 实例 + 高有效复位 `iSysRst`）。

本仓即算法规格与实现的**单一来源**：定点、流水、可综合 RTL；配置头统一放 `rtl/include/`。
- 共享头：`pv_cfg.vh` / `inv3ph_cfg.vh` / `algo_types.vh` / `algo_arith.vh` / `algo_filt_coef.vh`
- 实现规范 `docs/HDL_IMPL_CONVENTIONS.md`、命名契约 `docs/FPGA_STD_RENAME_SPEC.md`

## 目录

```
Algorithm_Framework_HDL/
├── rtl/include/             pv_cfg.vh, algo_types.vh, algo_arith.vh, algo_filt_coef.vh
├── rtl/verilog/
│   ├── math/                AlgoSqrt / AlgoDiv / AlgoTrig(+mem)
│   ├── controller/          CompPi / CompPr / CompPir（双线性离散）
│   ├── filter/              标量滤波 + 巴特沃斯（FS/FC 编译期算系数）
│   ├── transform/           Clark / Park / 正负序分离
│   ├── ortho/               SOGI / 直流提取 / PLL / FLL
│   ├── corr/                GCC / XCorr 互相关
│   ├── fft/                 CompFft（基-2 DIT）
│   ├── power/               前馈解耦 / 软启 / 死区
│   └── cntl/
│       ├── pv/              PV 控制链
│       └── inv3ph/          三相逆变器（故障/状态机/电压电流环/软启）
├── tb/verilog/              每模块单元 TB（按功能分目录）
├── docs/                    使用指南 / 实现规范 / 命名契约 / 逆变·LLC·DAB 分册
├── reviews/                 代码审查报告
└── Makefile                 analyze / lint / unit
```

**使用指南（例化、握手、注意事项）：** [`docs/USAGE_GUIDE.md`](docs/USAGE_GUIDE.md)

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

### cntl/inv3ph（三相逆变器 FPGA 控制链）
| 模块 | 说明 | 单测 |
|------|------|------|
| `Inv3phFault` | 快检：母线欠/过压、相电压/电流过限、短路锁存 | PASS |
| `Inv3phState` | 8 态 FSM（IDLE…CONNECTED…ERR）+ PWM/继电器/并网模式 | PASS |
| `Inv3phVloop` | dq 电压环（`CompPi`×2 + L2/C1 前馈） | PASS |
| `Inv3phIloop` | dq 电流环（`CompPi`×2 + `CompFfDecouple`，SEQ±1/0） | PASS |
| `Inv3phSoftStart` | 开环幅值软启（`CompSoftStart`） | PASS |

配置头：`rtl/include/inv3ph_cfg.vh`。控制方案对比见 [`docs/INV3PH_ALGO_COMPARE.md`](docs/INV3PH_ALGO_COMPARE.md)。

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

- **完整使用指南**：[`docs/USAGE_GUIDE.md`](docs/USAGE_GUIDE.md)（接口握手、滤波器编译期系数、选型表、踩坑清单）。
- 头文件只放 `rtl/include/`；模块用 `` `include "pv_cfg.vh" `` / `"inv3ph_cfg.vh"` / `"algo_types.vh"` / `"algo_filt_coef.vh"`。
- 可综合铁律：无 `real` 运算通路；**除法/开方统一例化 `AlgoDiv`/`AlgoSqrt` 数学库**（规范 §6.3，多拍 `iStart/oDone` 握手），业务模块禁止组合除法/自行摊开。
- 频率类滤波器：`FS`/`FC`/`F0` 等为 **parameter 编译期设定**，系数 elabor 自动算，**不允许在线改**。
- 命名遵循 `FPGA_standard`（详见 `docs/FPGA_STD_RENAME_SPEC.md`）。

## 文档

| 文档 | 内容 |
|------|------|
| [`docs/USAGE_GUIDE.md`](docs/USAGE_GUIDE.md) | **使用指南**（推荐先读） |
| [`docs/HDL_IMPL_CONVENTIONS.md`](docs/HDL_IMPL_CONVENTIONS.md) | 实现硬性规则 |
| [`docs/FPGA_STD_RENAME_SPEC.md`](docs/FPGA_STD_RENAME_SPEC.md) | 命名契约 |
| [`docs/INV3PH_ALGO_COMPARE.md`](docs/INV3PH_ALGO_COMPARE.md) | 三相 DQ vs 三独立单相控制（FPGA 选型） |
| [`docs/LLC_CTRL_FRAMEWORK.md`](docs/LLC_CTRL_FRAMEWORK.md) | LLC 难点、策略与算法框架（对齐逆变对比文档深度） |
| [`docs/DAB_CTRL_FRAMEWORK.md`](docs/DAB_CTRL_FRAMEWORK.md) | DAB 难点、策略与算法框架（V–i_L 双环） |

编码风格以 `docs/FPGA_STD_RENAME_SPEC.md` 为准；闭环联调可另建 FPGA 仿真/板级工程引用本库。
