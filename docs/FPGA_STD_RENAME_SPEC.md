# FPGA_standard Rev 1.5 命名对齐契约（全面重命名依据）

本文件是「全面重命名对齐 `FPGA_standard/FPGA编程标准与规范.md`」的**唯一冻结依据**。所有并行执行者必须严格按此表改写，禁止自行发明名字。改完自测：`iverilog PASS + verilator lint 0 error`。

## 1. 模块名映射（snake_case → PascalCase）

### Algorithm_Framework_HDL / cntl/pv
| 旧 | 新 | 旧 | 新 |
|----|----|----|----|
| pv_loop | PvLoop | pv_calc | PvCalc |
| pv_calc_full | PvCalcFull | pv_mppt | PvMppt |
| pv_fault | PvFault | pv_state_sys | PvStateSys |
| pv_state_ch | PvStateCh | pv_observer | PvObserver |

### Algorithm_Framework_HDL / component
| 旧 | 新 | 旧 | 新 |
|----|----|----|----|
| algo_trig | AlgoTrig | comp_hp_filter | CompHpFilter |
| comp_diff | CompDiff | comp_noise_var | CompNoiseVar |
| comp_lms_alpha | CompLmsAlpha | comp_kalman1d | CompKalman1d |
| comp_rms | CompRms | comp_clark | CompClark |
| comp_inv_clark | CompInvClark | comp_park | CompPark |
| comp_inv_park | CompInvPark | comp_sogi | CompSogi |
| comp_notch | CompNotch | comp_dc_extract | CompDcExtract |
| comp_abc2pnz | CompAbc2pnz | comp_3ph_decouple | Comp3phDecouple |
| comp_pll | CompPll | comp_fll_3ph | CompFll3ph |
| comp_gcc | CompGcc | comp_xcorr | CompXcorr |

### Algo_Demo_pv_HDL / rtl
| 旧 | 新 | 旧 | 新 |
|----|----|----|----|
| clk_sample_div | ClkSampleDiv | ctrl | Ctrl |
| plant | Plant | pv_dc_ctrl | PvDcCtrl |
| pv_top | PvTop | boost_pv_plant | BoostPvPlant |

文件命名：`PvLoop.v`（与 module 名一致）。

## 2. 端口前缀规则

- 时钟 `clk` → `iSysClk`（输入）
- 复位 `rst_n` → `iSysRst`（**高有效**，极性翻转，见 §5）
- 其余端口：输入加 `i`，输出加 `o`，后接 PascalCase（原 snake 名逐段大写）。
  例：`sample_en`→`iSampleEn`，`v_meas`→`iVMeas`，`duty`→`oDuty`，`v_ref`→`oVRef`，`state`→`oState`，`is_err`(输入)→`iIsErr` / (输出)→`oIsErr`，`o_valid`→`oValid`（去下划线），`sin_o`→`oSin`，`y_out`→`oYOut`，`x_in`→`iXIn`，`alpha_hat`(输入)→`iAlphaHat`/(输出)→`oAlphaHat`。
- 位宽/方向/signed 声明不变，仅改名。

## 3. 实例名

`u_xxx` → `U_Xxx`（PascalCase，`U_` 前缀）。例：`u_ema_v`→`U_EmaV`，`u_ctrl`→`U_Ctrl`。

## 4. 标准文件头（§4.1，每个 RTL + TB 顶部替换旧头注释）

```verilog
//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   <NewModuleName>.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
/*Description       :   <一句话功能，中文>
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.1
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
```

> 保留 `Moudle` 历史拼写。demo 仓 Project Name 填 `Algo_Demo_pv_HDL`。

## 5. 复位极性翻转（最高风险，逐处检查）

- `always @(posedge clk or negedge rst_n)` → `always @(posedge iSysClk or posedge iSysRst)`
- `if (!rst_n)` → `if (iSysRst)`（复位分支体不变，仍是赋初值）
- `else` 分支不变。
- TB 内：`rst_n = 0;` → `iSysRst = 1;`（进入复位）；`rst_n = 1;` → `iSysRst = 0;`（释放复位）。`initial` 里的复位段、`repeat(20) @(posedge clk); rst_n=1;` 等一并翻转。TB 信号声明 `reg rst_n = 0;` → `reg iSysRst = 0;`（TB 起始释放态，随后拉高复位再释放）。

## 6. 内部信号前缀（尽力）

- 时序寄存器 `reg` → `r` 前缀 PascalCase（`rAcc`、`rDutyI`…）。
- 组合连线 `wire` → `w` 前缀 PascalCase（`wErr`…）。
- `localparam` 保持全大写。若某模块内部信号过多，至少把对外语义寄存器/主状态信号加前缀，不强制全量。

## 7. 验证命令（每模块必须跑）

```bash
cd /home/win/wsl-proj/Algorithm_Framework_HDL
iverilog -g2012 -I rtl/include -o build/t_<x>.vvp <新模块文件...> tb/verilog/tb_<New>.v
cd build && vvp t_<x>.vvp          # 应 PASS
# lint（0 error）
unset VERILATOR_ROOT; verilator --lint-only -Wall -Wno-fatal -Wno-TIMESCALEMOD -Wno-WIDTHTRUNC -Wno-WIDTHEXPAND +incdir+../rtl/include <新模块文件...>
```

TB 文件也要一并改名对齐（`tb_<old>.v` → `tb_<New>.v`，内部 module 名 `tb_pv_loop`→`TbPvLoop`，例化/信号同步改）。

## 8. 禁止事项

- 不得改动本文件、Makefile、README、docs 其它文件、include 下三个 .vh、`algo_sin_lut.mem`。
- 只改你负责列表内的 `.v`（含对应 `.v` 文件名与 TB）。
- 跨域例化依赖模块时，模块名/端口名按**本契约**书写，不要读旧文件抄旧名。
- 完成后回报：每模块一行「新模块名: PASS/lint 0 error + 一句改动要点」。
