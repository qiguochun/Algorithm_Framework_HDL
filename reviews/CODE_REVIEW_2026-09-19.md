# Algorithm_Framework_HDL 代码审查

- **日期**: 2026-09-19
- **结论**: Request changes
- **范围**: `rtl/` + `tb/` + `Makefile` + `docs/`
- **五轴**: 正确性 / 可读性 / 架构 / FPGA 安全 / 性能

## 摘要

| 项 | 数值 |
|----|------|
| RTL 模块文件 | 48 |
| 单元 TB | 47（README 写 63） |
| Lint | 0 error / 190 warning |
| 合入结论 | Request changes |

架构方向正确：高有效 `iSysRst`、多拍 `AlgoDiv`/`AlgoSqrt`、控制器拆拍流水。但 `PvCalcFull` / `PvObserver` / `CompXcorr` / `CompGcc` 存在同时钟双 `always` 驱动同一 `reg`（综合多驱动），`CompXcorr` 还把整窗互相关放在组合核里。当前 iverilog PASS 盖不住这些问题——Icarus 不报多驱动，真实 FPGA 综合会失败或时序炸掉。

---

## Critical

| ID | 模块 | 问题 | 影响 | 修复 |
|----|------|------|------|------|
| C1 | PvCalcFull | `rAvgPend` / `avg_num` / `sPsum` / `oVAvg` 等被两个时序 always 共同驱动 | FPGA 综合多驱动网；仿真调度未定义 | 均值状态只放一个 always，提交拍只发脉冲 |
| C2 | PvObserver | `rKfReq` 在 KF FSM 与 `iMpptEn` 流水块双写 | 同拍置位/清零会丢请求或粘死 | 单比特 pending：accept 置位、K_IDLE 消费 |
| C3 | CompXcorr / CompGcc | `oRPeak`/`oLagPeak` 与 `oRho`/`oDirection`/`oValid` 双 always 驱动 | 综合报错或复位竞态 | 合并为一个时序块，复位只清一次 |
| C4 | CompXcorr | `always @(*)` 双层循环：WIN=128、MAX_LAG=16 每拍上万次 16×16 乘加 | 无法在 100 MHz 收敛；DSP/LUT 爆炸 | 改成逐拍 MAC 状态机；窗长用 2 的幂与位掩码 |
| C5 | PvCalc | 异步复位未清 `rVFilt`；`oVFilt` 组合跟该寄存器 | 系统复位后 EMA 残留旧值 | 复位分支 `rVFilt <= 0`，并由同一 always 驱动输出 |

---

## Required

| ID | 位置 | 问题 | 修复 |
|----|------|------|------|
| R1 | PvMppt | 无 `tb_PvMppt`，README 却写「闭环 PASS」 | 补边界反向、首拍只采样、功率持平续向 |
| R2 | README / Makefile | 自称 63 PASS；仓库仅 47 个 TB；lint 0 error 但 190 warning | 按实际 TB 数更正；分类处理 PINCONNECTEMPTY/BLKSEQ |
| R3 | CompPll / CompFll3ph / CompXcorr | 空命名端口 `.oValid()` / `.oBusy()`（规范禁止） | 接到哑 wire，或省略未连接端口 |
| R4 | CompClark.v / CompPark.v | 正/反变换两个 module 同文件 | 拆成 CompInvClark.v / CompInvPark.v |
| R5 | PLL / FLL / DOGI / SRF-PLL | `FS`/`F_NOM` 不驱动 `WNOM_Q`、`C_PH`（写死 50 Hz / 5 kHz） | 用参数生成常量，或删掉无效 parameter |
| R6 | CompKalman1d | `wPN[63:0] < 0` 对无符号切片比较，协方差钳零恒不成立 | 改为 `if (wPN < 0) rP <= 0` |
| R7 | CompRep | `rBuf` 写回取 `wUNew[31:0]` 无饱和，超 ±2³¹ 绕回 | 写缓冲前 S32 饱和；TB 覆盖大误差 |
| R8 | CompSoftStart | `oValid` 跟 `wRun`，`oOut` 落后 `rAcc` 一拍；终值出现时 `oValid=0` | `oValid` 与输出寄存器对齐 |
| R9 | PV / 控制器 TB | 多数未接 `oValid`；PI 不测抗饱和；节拍远密于真实 `PV_LOOP_DIV` | 断言 LATENCY；限幅冻结；采样间隔 ≥ 除法延迟 |
| R10 | docs/*.md | 仍写 `component/`、扁平 `tb/verilog/tb_*.v`、snake_case 旧名 | 与现目录和 FPGA_standard 命名对齐 |

---

## 测试覆盖缺口

- `make unit` 每个 TB 都链入全部 RTL（无 `-s` 指定 top），能编过但 O(N²)、无法证明单模块隔离。
- 缺 `tb_PvMppt`。PvLoop / PvCalc / PvCalcFull / PvObserver 的 TB 未连接 `oValid`。
- AlgoDiv 未覆盖 `-2^(W-1)/-1` 饱和。控制器 TB 全是小信号跟踪，抗饱和可恒真/恒假仍 PASS。
- PLL/FLL 常量写死 50 Hz，TB 也只跑该点；改 `FS`/`F_NOM` 会静默算错。

## 做得好的部分

- 可综合路径无 `real`；组合除法/开方已收到 `AlgoDiv` / `AlgoSqrt` 多拍库。
- 复位极性已统一为 `posedge iSysClk or posedge iSysRst`；端口 `i/o`、实例 `U_` 大体符合 FPGA_standard。
- PI/PIR 有条件积分抗饱和；二阶 IIR 有 busy 门控；`pv_cfg.vh` 作节拍单一来源。
- `$readmemh("../rtl/verilog/math/algo_sin_lut.mem")` 相对 `build/` 路径正确（文档仍写旧 component/）。

## 架构与性能

### PvCalcFull 过大且职责混杂

569 行：均值窗、四路 EMA、三环、CCM/DCM 占空比、13 路除法/开方。提交拍 always 与均值 FSM 抢同一批寄存器，是 C1 的根因。建议拆成 AvgWindow + LoopDuty + 顶层接线。

### 参数名不驱动数值

CompSogi / CompPll / CompFll3ph / CompDogi 的 Q15 系数和 `WNOM_Q=20106`、`C_PH=2136` 都按 50 Hz / 5 kHz 抄死。`K_DAMP` / `K_SOGI` 是文档型死参数。改采样率必须手改常量表。

### 长组合路径

CompClark / CompPark / AlgoTrig ROM 为组合；CompGoertzel 单拍多 64-bit 乘；CompFfDecouple / CompMedian 无输出寄存器。嵌进 100 MHz 环时需自行加拍。

---

## 建议修复顺序

| 优先级 | 动作 | 验收 |
|--------|------|------|
| 1 | 消除全部同时钟双驱动（CalcFull / Observer / Xcorr / Gcc） | 综合 0 multi-driven；复位后信号确定 |
| 2 | CompXcorr 组合核改流水 MAC；接上 `.oBusy` | 默认 WIN=128 可综合，时序有 slack |
| 3 | PvCalc 复位清 `rVFilt`；Kalman 用有符号比较钳 P；Rep 写回饱和 | 复位/数值边界 TB PASS |
| 4 | 补 `tb_PvMppt`；TB 接 `oValid`；更正 README 47 测 | `make unit` 与文档一致 |
| 5 | 拆 Clark/Park；PLL 常量跟 FS；更新过时 docs | Verilator DECLFILENAME/PINCONNECTEMPTY = 0 |

---

## 验证依据

- 本仓库 `rtl/` / `tb/` / `docs/`
- Verilator `-Wall`：0 Error，190 Warning（UNUSEDSIGNAL 105、BLKSEQ 48、UNUSEDPARAM 26、PINCONNECTEMPTY 4）
- BLKSEQ 与项目规范「临时量阻塞、寄存器 NBA」部分一致，不单独作为阻断项
