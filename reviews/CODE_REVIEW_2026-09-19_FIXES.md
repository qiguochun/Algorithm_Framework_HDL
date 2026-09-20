# Algorithm_Framework_HDL 代码审查 — 修复跟踪

- **原报告**: `reviews/CODE_REVIEW_2026-09-19.md`
- **修复日期**: 2026-09-19
- **状态**: Critical / Required 项已按报告落地（见下）

## 已修复

| ID | 项 | 处理 |
|----|----|------|
| C1 | PvCalcFull 多驱动 | 均值快照/`rAvgPend`/`oVAvg*` 仅由 Avg FSM always 驱动；busy 时丢窗 |
| C2 | PvObserver `rKfReq` | 改为 accept 侧一拍 `rKfKick`，KF FSM 只读 |
| C3 | CompXcorr/CompGcc 多驱动 | 峰值/相关输出单 always 驱动 |
| C4 | CompXcorr 组合核 | 改为逐拍 MAC + sqrt/div；显式 signed 乘；TB 用 WIN=128 并等待完成 |
| C5 | PvCalc `rVFilt` | 复位清零；`oVFilt` 同 always 寄存 |
| R1 | tb_PvMppt | 已新增并 PASS |
| R2 | README 测数 | 改为 48 PASS；补充 `power/`、`reviews/` |
| R3 | 空端口 | CompPll/CompFll3ph/CompXcorr/CompDcExtract 接哑线 |
| R4 | Clark/Park 双模块 | 拆出 `CompInvClark.v` / `CompInvPark.v` |
| R5 | FS/F_NOM | PLL/FLL/DOGI/SRF-PLL 的 WNOM_Q/C_PH 由参数推导 |
| R6 | CompKalman1d | `if (wPN < 0)` 有符号比较 |
| R7 | CompRep | 写回前 S32 饱和 |
| R8 | CompSoftStart | `oValid` 与 `oOut` 对齐（`rOutEn` 延迟） |
| — | AlgoSqrt | 参数断言 `OW*2 < IW` 时 `$finish` |

## 未在本轮强制改（可选/文档债）

| 项 | 说明 |
|----|------|
| R9 全量 TB 接 oValid / PI 抗饱和 | 部分仍弱；建议后续补 |
| R10 docs/component 路径 | 规范文档仍有旧路径，可另开清理 |
| CompFft BLKSEQ | 规范允许临时量阻塞；未改功能 |

## 验证

```bash
make analyze && make lint && make unit
```
