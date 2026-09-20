# Algorithm_Framework_HDL 实现规范（模块并行开发用）

供实现 Algorithm_Framework（C）各算法 HDL 版本时遵循。**结构/公式对齐 C，定点化与量程自选合理 Q 并注释**；不追求 float 逐点一致。

## 硬性规则（否则回归/lint 失败）
1. **每模块一个文件**，放 `rtl/verilog/...`（pv 相关在 `cntl/pv/`，通用组件在 `component/`）；单元 TB 放 `tb/verilog/tb_<module>.v`。
2. **头 4 行文件头注释**：名称/功能/定点说明/修订（可中文，禁止非 ASCII 箭头 → 等字符以免 iVerilog 解析问题，用 `->` 代替）。
3. 纯 Verilog-2001/-2012；**禁止** `logic`、`string`、SystemVerilog 类型化 task 参数、SV 接口；task 参数仅 `input integer/reg/wire`。
4. **禁止 BLKANDNBLK**：时序 always 内先用阻塞临时量算好，末尾对信号**只做一次非阻塞 `<=`**；`is_*` 聚合尽量组合 `assign`。
5. **禁止空命名端口连接**（`.sig()`）。
6. lint：`unset VERILATOR_ROOT; verilator --lint-only -Wall -Wno-fatal -Wno-TIMESCALEMOD -Wno-WIDTHTRUNC -Wno-WIDTHEXPAND +incdir+rtl/include <模块>...`。改完必须 **0 error**。
7. 组合除法/开方允许出现在**仿真对象/算法模块**，标注“非综合，后续 IP 化”；`$readmemh` 的 ROM 用相对 `build/` 的路径 `../rtl/verilog/component/xxx.mem`。
8. 位宽/宏只从 `rtl/include/pv_cfg.vh`、`algo_types.vh` 或模块 parameter 取；**禁止魔数**（常量要命名注释）。

## 验证方式（你自己必须跑）
```bash
cd /home/win/wsl-proj/Algorithm_Framework_HDL
iverilog -g2012 -I rtl/include -o build/t_<x>.vvp <模块源...> tb/verilog/tb_<x>.v
cd build && vvp t_<x>.vvp      # 应打印 PASS
```
涉及 algo_trig/ROM 时源里含 `component/algo_trig.v`。**不要**改动/新增 Makefile、README、docs/interface 之外的任何既有文件；**只新增**你负责列表里的模块与 TB。完成后回报：每模块一行 “模块名: PASS/FAIL + 你验证的关键行为”。

## 已有的既有模块（勿改，可例化）
`pv_calc`(EMA) / `pv_loop`(PI) / `pv_mppt` / `pv_fault` / `pv_state_sys` / `pv_state_ch` / `component/algo_trig.v`(+`algo_sin_lut.mem`)。
