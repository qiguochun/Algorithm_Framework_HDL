# Algorithm_Framework_HDL 使用指南

面向 FPGA 的电力电子 / 电机控制 **可综合 Verilog 算法库**：定点、流水、可综合 RTL。本文说明如何选型、例化、联调，以及必须遵守的注意事项。

相关文档：

- 模块清单与自检：`README.md`
- 实现硬性规则：`docs/HDL_IMPL_CONVENTIONS.md`
- 命名契约：`docs/FPGA_STD_RENAME_SPEC.md`
- 三相逆变算法对比：`docs/INV3PH_ALGO_COMPARE.md`
- 审查报告：`reviews/`

---

## 1. 工程定位与目录

| 路径                        | 用途                                                                               |
| --------------------------- | ---------------------------------------------------------------------------------- |
| `rtl/include/`            | 公共头：`pv_cfg.vh`、`inv3ph_cfg.vh`、`algo_types.vh`、`algo_arith.vh`、`algo_filt_coef.vh` |
| `rtl/verilog/math/`       | 开方 / 除法 / 三角函数 ROM                                                         |
| `rtl/verilog/filter/`     | 标量滤波、巴特沃斯、陷波、微分、RMS 等                                             |
| `rtl/verilog/controller/` | PI / PR / PIR / 多谐振 / 重复控制                                                  |
| `rtl/verilog/transform/`  | Clarke / Park / 正负序                                                             |
| `rtl/verilog/ortho/`      | SOGI / PLL / FLL / 直流提取                                                        |
| `rtl/verilog/power/`      | 前馈解耦 / 软启 / 死区                                                             |
| `rtl/verilog/fft/`        | FFT / Goertzel                                                                     |
| `rtl/verilog/corr/`       | 互相关                                                                             |
| `rtl/verilog/cntl/pv/`    | PV 控制链                                                                          |
| `rtl/verilog/cntl/inv3ph/`| 三相逆变器（故障 / FSM / V·I 环 / 软启）                                           |
| `tb/verilog/`             | 每模块单元测试（按功能子目录）                                                     |
| `Makefile`                | `analyze` / `lint` / `unit`                                                  |

本库是**组件库**，不是完整顶层 SoC。顶层采样分频、PWM、ADC 接口等由上层工程（如 `Algo_Demo_pv_HDL`）提供。

---

## 2. 通用接口约定（先读这段）

几乎所有业务模块遵循同一套握手习惯。

### 2.1 端口命名

| 信号                       | 含义                                                             |
| -------------------------- | ---------------------------------------------------------------- |
| `iSysClk`                | 系统时钟                                                         |
| `iSysRst`                | **高有效**同步/异步复位（按模块文件头为准）                |
| `iEn`                    | **采样节拍**：单拍脉冲 = 一个新样本                        |
| `iXIn` / `iErrIn` / … | 数据输入（多为有符号 S16）                                       |
| `oYOut` / `oOut` / …  | 数据输出                                                         |
| `oValid`                 | 本帧结果有效：`iEn` 之后再过 `LATENCY` 拍拉高 **一拍** |

实例名建议：`U_Lpf`、`U_Pi`（`U_` 前缀）。

### 2.2 多拍流水与采样间隔（最重要）

多数 IIR / 控制器已拆成多拍流水，避免单周期组合乘加链。

**约束：相邻两次 `iEn` 的间隔必须 ≥ `LATENCY` 个时钟周期。**

否则上一帧还在飞、下一帧又采样，数值会错、状态会乱。

典型做法：控制环采样率 `FS`（如 5 kHz），系统时钟 100 MHz，用分频器每 `CLK/FS` 拍打一次 `iEn`。`LATENCY`（数拍～十几拍）远小于采样周期，天然满足。

```
系统时钟:  |‾|_|‾|_|‾|_|‾|_|‾|_|‾|_|‾|_ ...
iEn:       ^                 ^                 ^   （采样周期 >> LATENCY）
oValid:         ^                 ^                 ^
               ←── LATENCY ──→
```

### 2.3 复位与初态

- 复位后输出一般为 0，滤波器/控制器内部状态清零。
- 上电后建议先喂若干 `iEn` 做预热，再采稳态指标（TB 里的 `WARM` 即此意）。

### 2.4 定点

- 信号默认多为 **S16**（有符号 16 bit）。
- 系数常用 Q14 / Q15 / Q20，见各模块文件头注释。
- 以可综合稳定性与定点数值约定为准；仿真 golden 用本仓 TB / 定点模型。

---

## 3. 如何引用本库

### 3.1 仿真 / 综合 include 路径

```bash
# iverilog
iverilog -g2012 -I rtl/include -o build/xxx.vvp \
  rtl/verilog/filter/CompLpf1.v \
  tb/verilog/filter/tb_CompLpf1.v

# Verilator lint
unset VERILATOR_ROOT
verilator --lint-only -Wall -Wno-fatal \
  -Wno-TIMESCALEMOD -Wno-WIDTHTRUNC -Wno-WIDTHEXPAND \
  +incdir+rtl/include rtl/verilog/.../*.v
```

工程内一键：

```bash
make analyze   # 全库语法
make lint      # Verilator
make unit      # 全部单元 TB
```

### 3.2 例化模板

```verilog
CompLpf1 #(
    .W(16),
    .FS(5000),   // 采样率 Hz（须与 iEn 节拍一致）
    .FC(50)      // 截止频率 Hz
) U_Lpf (
    .iSysClk(iSysClk),
    .iSysRst(iSysRst),
    .iEn    (iSampleEn),   // 单拍脉冲
    .iXIn   (wMeas),
    .oYOut  (wFilt),
    .oValid (wFiltVld)
);
```

依赖数学库的模块（如 `CompRms`、`CompPark`、`CompNoiseVar`）必须同时编译进工程：

- `rtl/verilog/math/AlgoSqrt.v`
- `rtl/verilog/math/AlgoDiv.v`
- `rtl/verilog/math/AlgoTrig.v`
- `rtl/verilog/math/algo_sin_lut.mem`（`$readmemh` 路径按仿真 cwd 配置，常见从 `build/` 跑时用 `../rtl/verilog/math/algo_sin_lut.mem`）

---

## 4. 按功能选型

### 4.1 数学库 `math/`

| 模块         | 用途                   | 握手                                      |
| ------------ | ---------------------- | ----------------------------------------- |
| `AlgoSqrt` | `floor(sqrt(x))`     | `iStart` → 忙 → `oDone` + `oRoot` |
| `AlgoDiv`  | 有符号除法（除零出 0） | 同上                                      |
| `AlgoTrig` | 1024 点 sin/cos LUT    | 组合查表（角度输入）                      |

**注意：**

- 业务模块禁止自己写组合除法/开方摊开；统一例化本库。
- `AlgoSqrt` 要求 `OW >= ceil(IW/2)`，否则仿真直接报错退出。
- busy 期间不要再发 `iStart`。

### 4.2 滤波器 `filter/`

#### 策略（强制）

**频率类滤波特性只在编译期用 `parameter` 设定，不允许运行时端口改 `Fs`/`Fc`/系数。**

综合后系数为常量乘法。改截止频率 / 采样率 = 改 parameter 后重新综合。

系数由 `rtl/include/algo_filt_coef.vh` 在 elabor 期按双线性公式整数定点自动计算。

| 模块                            | 主要 parameter                             | 说明                                      |
| ------------------------------- | ------------------------------------------ | ----------------------------------------- |
| `CompLpf1` / `CompLpf2`     | `FS`, `FC`                             | 一阶/二阶巴特沃斯低通                     |
| `CompHpf2`                    | `FS`, `FC`                             | 二阶巴特沃斯高通                          |
| `CompBpf2` / `CompBsf2`     | `FS`, `F0`, `QREF`                   | 带通/带阻；`QREF`=Q×1000（0.707→707） |
| `CompHpFilter`                | `FS`, `FC`                             | 一阶高通                                  |
| `CompNotch`                   | `FS`, `FC`, `C1_MILLI`, `C2_MILLI` | II 型陷波；`C*_MILLI`=小数×1000        |
| `CompDiff`                    | `FS`, `FC`, `N`                      | 带前置 LPF 的微分                         |
| `CompNoiseVar`                | `FS`, `FC`                             | EWMA 方差；`alpha≈2π·fc/fs`          |
| `CompMovAvg` / `CompMedian` | `N`                                      | 窗长；MovAvg 的`N` 须为 2 的幂          |
| `CompRateLimit`               | `RATE`                                   | 每采样最大 `                              |
| `CompRms`                     | `N`                                      | 窗长；内部用 AlgoDiv/AlgoSqrt             |
| `CompKalman1d`                | `QRAW`, `RRAW`                         | 过程/量测噪声（编译期）                   |
| `CompLmsAlpha`                | `MUQ`, `AFRAC`                         | 步长固定；`oAlphaHat` 会随数据自适应    |

例：

```verilog
CompBpf2 #(.FS(10000), .F0(50), .QREF(707)) U_Bpf (...);
CompNotch #(.FS(5000), .FC(100), .C1_MILLI(500), .C2_MILLI(0)) U_N (...);
```

**不要**再传已删除的 `COEF_B0` / `P_A1` 等外部系数参数。

#### 频率与采样率匹配

- `FS` 必须等于实际 `iEn` 对应的采样频率，否则截止频率名义对、实际错。
- 双线性变换在 `fc` 接近 Nyquist（`fs/2`）时误差变大；尽量 `fc << fs/2`。
- 当前实现**不做** `tan` 频率预畸变（与库内既有 Python 黄金公式一致）。

### 4.3 控制器 `controller/`

| 模块            | 用途        | 系数形态                            |
| --------------- | ----------- | ----------------------------------- |
| `CompPi`      | PI + 抗饱和 | `P_KP`、`P_KIT2`(=Ki·Ts/2) Q14 |
| `CompPr`      | PR          | 谐振 IIR 系数 parameter             |
| `CompPir`     | PI + 谐振   | 同上                                |
| `CompMultiPr` | 基波+5/7 次 | 多组谐振系数                        |
| `CompRep`     | 重复控制    | 周期`N`、`Q` 等                 |

控制器系数目前多为**手工 Q 定点 parameter**（尚未统一成 `FS/Kp/Ki` 自动换算）。改 `Fs` 时须同步改 `P_KIT2` 等与 `Ts` 相关的量。

### 4.4 坐标变换 `transform/`

| 模块                             | 说明                             |
| -------------------------------- | -------------------------------- |
| `CompClark` / `CompInvClark` | αβ ↔ abc                      |
| `CompPark` / `CompInvPark`   | dq ↔ αβ（内部用`AlgoTrig`） |
| `CompAbc2pnz`                  | 正负序分离                       |
| `Comp3phDecouple`              | 三相解耦                         |

Park 类需要角度输入；确认角度格式与 `AlgoTrig` 一致（LUT 相位）。

### 4.5 正交 / 锁相 `ortho/`

| 模块                          | 说明                                                |
| ----------------------------- | --------------------------------------------------- |
| `CompSogi`                  | SOGI；系数目前为 Q15 常量表（默认约 50 Hz / 5 kHz） |
| `CompDcExtract`             | 直流提取；`MODE=0` SOGI，`MODE=1` 内联 Notch    |
| `CompPll` / `CompSrfPll`  | 单相 / SRF-PLL；有`FS`、`F_NOM` 等              |
| `CompFll3ph` / `CompDogi` | 三相 FLL / DSOGI-FLL                                |

**注意：** `CompSogi`、`CompDcExtract` 的 Notch 路径仍有部分**写死系数**（按默认 50/100 Hz、5 kHz）。改电网频率或采样率时，要核对文件头与常量表，不能假设“只改 `FS` 即可”。后续宜对齐 `algo_filt_coef.vh` 策略。

### 4.6 电源专用 `power/`

| 模块               | 说明               |
| ------------------ | ------------------ |
| `CompFfDecouple` | dq 前馈解耦（ωL） |
| `CompSoftStart`  | 软启斜坡           |
| `CompDeadTime`   | 按电流方向死区补偿 |

### 4.7 FFT / 相关 `fft/` `corr/`

| 模块                        | 说明                                             |
| --------------------------- | ------------------------------------------------ |
| `CompFft`                 | 基-2 DIT，`N` 为 2 的幂                        |
| `CompGoertzel`            | 单频；`COEF_Q14 = round(2·cos(2πk/N)·2^14)` |
| `CompGcc` / `CompXcorr` | 互相关；`ALPHAQ` 等为 EWMA 系数                |

### 4.8 PV 控制链 `cntl/pv/`

面向光伏 Boost 等场景的专用链：`PvLoop` / `PvCalc` / `PvCalcFull` / `PvMppt` / `PvFault` / `PvStateSys` / `PvStateCh` / `PvObserver`。

节拍与限幅等共享配置见 `rtl/include/pv_cfg.vh`（如 `PV_LOOP_FS=5000`）。改采样率时优先改该头文件，并检查各模块 parameter 是否仍匹配。

### 4.9 三相逆变器 `cntl/inv3ph/`

FPGA 三相逆变控制链（故障 / 状态机 / 电压·电流环 / 软启）：

| 模块 | 说明 |
|------|------|
| `inv3ph_cfg.vh` | FS=20kHz、故障阈值、状态编码 |
| `Inv3phFault` | 母线/相电压电流快检，故障锁存 |
| `Inv3phState` | 8 态 FSM + `oPwmEn`/`oRlyEn`/`oGridMode` |
| `Inv3phVloop` | 例化 `CompPi`×2 + L2/C1 前馈拼装 |
| `Inv3phIloop` | 例化 `CompPi`×2 + `CompFfDecouple`；`SEQ=+1/-1/0` |
| `Inv3phSoftStart` | 例化 `CompSoftStart`（`P_STEP`/`P_TARGET`） |

例化示意（离网电压环 + 正序电流环）：

```verilog
Inv3phVloop U_Vp (...);                      // 正序电压环
Inv3phVloop U_Vn (...);                      // 负序电压环（Ud_ref/Uq_ref=0）
Inv3phIloop #(.SEQ(1))  U_Ip (...);          // 正序
Inv3phIloop #(.SEQ(-1)) U_In (...);          // 负序
Inv3phIloop #(.SEQ(0))  U_Iz (...);          // 零序
```

**待补充的专用封装（可用现有组件拼装）：**
- 正负序解耦前端：例化 `Comp3phDecouple` / `CompAbc2pnz` + `CompPark`
- 占空比：逆 Park（`CompInvPark`）后按分裂母线做 `AlgoDiv`
- 并网 P/Q→dq 电流给定、慢检过载延时：可后续加 `Inv3phGridIref` / 扩展 Fault
- 算法方案对比文档：[`docs/INV3PH_ALGO_COMPARE.md`](INV3PH_ALGO_COMPARE.md)

---

## 5. 注意事项（踩坑清单）

1. **`iEn` 间隔 ≥ `LATENCY`**顶层采样使能不要每个时钟都拉高。
2. **`FS` 与 `iEn` 必须一致**滤波器 / 部分 ortho 模块的 Hz 参数只描述“设计意图”；实际频率由 `iEn` 决定。
3. **滤波器系数不可在线改**没有 `iFc` 端口。要扫频或变带宽，只能多例化几组或重新综合。
4. **除法 / 开方只用 math 库**综合友好、时序可控；不要在 always 里写 `/` 做定点除。
5. **复位高有效 `iSysRst`**与部分 IP 的低有效复位相反，连线时注意极性。
6. **ROM 路径**`AlgoTrig` 的 `$readmemh` 相对仿真工作目录。推荐在 `build/` 下 `vvp`，路径写成 `../rtl/verilog/math/algo_sin_lut.mem`。
7. **注释里禁止特殊 Unicode 箭头**部分工具对 `→` 等字符敏感；文件头用 `->`。
8. **语言子集**Verilog-2001/2012；禁止依赖 `logic`、SV interface 等（见实现规范）。
9. **内联重复实现**如 `CompDcExtract` MODE=1 内联了 Notch 系数，与 `CompNotch` 默认值一致，但**不会**随 `CompNotch` 的 `FS`/`FC` 参数自动变。改频率时要两边一起查。
10. **控制器 / SOGI 系数尚未全面“FS 自动换算”**滤波器已统一；PI/PR/SOGI 等仍可能是手工 Q 表。改 `Fs` 前读文件头公式，必要时重算 parameter。
11. **位宽与饱和**内部常有 48/64/96 bit 中间量，输出再饱和到 S16。输入勿长期顶满量程，以免中间溢出观感异常。
12. **单元测试是行为契约**
    改接口或公式后，至少跑对应 `tb/verilog/.../tb_CompXxx.v`，确认打印 `PASS`。

---

## 6. 推荐联调流程

```
1. 确定系统时钟、控制采样率 FS、iEn 分频
2. 选型模块，按默认 parameter 例化
3. 用 TB 或简易激励看 oValid 时序与阶跃/正弦响应
4. 再改 FS/FC/增益等到目标工况
5. make lint + 相关 unit
6. 接入顶层（ADC → 滤波 → 控制 → PWM）
```

单模块仿真示例：

```bash
mkdir -p build
iverilog -g2012 -I rtl/include -o build/tb_CompLpf1.vvp \
  tb/verilog/filter/tb_CompLpf1.v rtl/verilog/filter/CompLpf1.v
cd build && vvp tb_CompLpf1.vvp
```

---

## 7. 补充说明与已知差距

### 7.1 已统一 / 未统一

| 类别                                                      | 编译期 Fs/Fc 自动算系数 | 说明                        |
| --------------------------------------------------------- | ----------------------- | --------------------------- |
| 巴特沃斯 LPF/HPF/BPF/BSF、HpFilter、Notch、Diff、NoiseVar | 是                      | `algo_filt_coef.vh`       |
| MovAvg / Median / RateLimit / Rms / Kalman / LMS          | 参数即特性              | 无模拟截止频率或不适用      |
| CompPi / CompPr / …                                      | 否                      | 手工 Q 系数                 |
| CompSogi                                                  | 否                      | Q15 常量表                  |
| CompDcExtract Notch 路径                                  | 否                      | 内联写死默认 100 Hz / 5 kHz |

### 7.2 工程定位

- 本仓是 FPGA 算法库本体：规格、定点约定、可综合 RTL 与 TB 均以本仓为准。
- 配置改动优先改 `rtl/include/*.vh`，再核对相关模块 parameter。

### 7.3 建议后续补强（非阻塞）

- `CompDcExtract` Notch 改为 `FS`/`FC` + `fn_notch_*`。
- `CompSogi` / PLL 系系数改为由 `FS`/`F0`/`K` 编译期生成。
- `CompPi` 等增加 `FS`/`KP`/`KI` → `P_KIT2` 自动换算（仍保持编译期）。
- README 模块表注明“系数编译期自动 / 手工 Q 表”。

### 7.4 编码规范

命名与复位约定见 `docs/FPGA_STD_RENAME_SPEC.md`；实现硬规则见 `docs/HDL_IMPL_CONVENTIONS.md`。

---

## 8. 快速对照：常见需求 → 模块

| 需求        | 建议模块                                            |
| ----------- | --------------------------------------------------- |
| 去噪、缓变  | `CompLpf1` / `CompLpf2` / `CompMovAvg`        |
| 去直流      | `CompHpFilter` / `CompHpf2` / `CompDcExtract` |
| 提取某频带  | `CompBpf2`                                        |
| 消工频纹波  | `CompNotch` / `CompBsf2`                        |
| 斜率限制    | `CompRateLimit`                                   |
| 电流/电压环 | `CompPi` / `CompPr` / `CompPir`               |
| 谐波抑制    | `CompMultiPr` / `CompRep`                       |
| 锁相        | `CompPll` / `CompSrfPll` / `CompDogi`         |
| 三相逆变控制 | `Inv3phState` + `Inv3phVloop`/`Inv3phIloop` + 解耦/Park |
| 正交生成    | `CompSogi`                                        |
| dq 解耦     | `CompFfDecouple` + Park/Clark                     |
| 单频检测    | `CompGoertzel`                                    |
| 窗 RMS      | `CompRms`                                         |

---

修订：2026-09-20。接口变更以各模块文件头与 `algo_filt_coef.vh` 为准；本文与实现冲突时以 RTL 为准。
