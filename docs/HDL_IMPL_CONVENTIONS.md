# Algorithm_Framework_HDL 实现规范（模块并行开发用）

本规范约束本仓 **FPGA 可综合 Verilog 算法库**的实现方式：定点、多拍流水、可综合；量程与 Q 格式在模块头注释中写明。

## 硬性规则（否则回归/lint 失败）
1. **每模块一个文件**，放 `rtl/verilog/<功能域>/`；单元 TB 放 `tb/verilog/<同域>/tb_<Module>.v`。
2. **文件头注释**：名称/功能/定点说明/修订（可中文；禁止非 ASCII 箭头等易导致 iVerilog 解析问题的字符，用 `->` 代替）。
3. 纯 Verilog-2001/-2012；**禁止** `logic`、`string`、SystemVerilog 类型化 task 参数、SV 接口；task 参数仅 `input integer/reg/wire`。
4. **禁止 BLKANDNBLK**：时序 always 内先用阻塞临时量算好，末尾对信号**只做一次非阻塞 `<=`**；`is_*` 聚合尽量组合 `assign`。
5. **禁止空命名端口连接**（`.sig()`）。
6. lint：`unset VERILATOR_ROOT; verilator --lint-only -Wall -Wno-fatal -Wno-TIMESCALEMOD -Wno-WIDTHTRUNC -Wno-WIDTHEXPAND +incdir+rtl/include <模块>...`。改完必须 **0 error**。
7. **禁止组合除法/开方**：统一例化 `math/AlgoDiv.v`、`math/AlgoSqrt.v`（多拍 `iStart/oDone`）；`$readmemh` ROM 路径相对仿真 `build/`。
8. 位宽/宏只从 `rtl/include/*.vh` 或模块 parameter 取；**禁止魔数**（常量要命名注释）。
9. 描述与文档只写 FPGA/RTL 语义，**不引用**外部软件工程或其它产品线名称。

## 验证方式
```bash
cd /home/win/wsl-proj/Algorithm_Framework_HDL
iverilog -g2012 -I rtl/include -o build/t_<x>.vvp <模块源...> tb/verilog/.../tb_<x>.v
cd build && vvp t_<x>.vvp      # 应打印 PASS
```
或 `make analyze && make lint && make unit`。

## 可复用基础设施
- 数学：`AlgoDiv` / `AlgoSqrt` / `AlgoTrig`(+`.mem`)
- 控制器：`CompPi` / `CompPr` / `CompPir` / …
- 配置：`pv_cfg.vh`、`inv3ph_cfg.vh`、`algo_filt_coef.vh`
