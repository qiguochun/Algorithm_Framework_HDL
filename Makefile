# Algorithm_Framework_HDL — 库构建（分析/lint/单元仿真）
# 用法: make analyze   # 全库 RTL 语法编译（iverilog）
#       make lint      # Verilator 全库 lint
#       make unit      # 运行 tb/verilog/*.v 单元仿真（随模块补充）
BUILD_DIR = build
INC       = -I rtl/include        # iverilog include
VINC      = +incdir+rtl/include   # verilator include
IVERILOG  = iverilog -g2012 $(INC)

# 库源码：随 rtl/verilog/** 增长而追加（也可用 find）
SRC       = $(shell find rtl -name '*.v' 2>/dev/null)

BLUE  = \033[1;34m
GREEN = \033[1;32m
YELLOW= \033[1;33m
RED   = \033[1;31m
NC    = \033[0m

.PHONY: analyze lint unit clean

analyze:
	@mkdir -p $(BUILD_DIR)
	@printf "$(YELLOW)[LIB] Analyze $(SRC)$(NC)\n"
	@if [ -z "$(SRC)" ]; then \
	  echo "（暂无 RTL，跳过）"; \
	else \
	  $(IVERILOG) -o $(BUILD_DIR)/lib.vvp $(SRC) || exit 1; \
	fi
	@printf "$(GREEN)[OK] analyze$(NC)\n"

lint:
	@printf "$(BLUE)[VERILATOR] Lint lib RTL...$(NC)\n"
	@if [ -z "$(SRC)" ]; then \
	  echo "（暂无 RTL，跳过）"; \
	else \
	  unset VERILATOR_ROOT; verilator --lint-only -Wall -Wno-fatal \
	    -Wno-TIMESCALEMOD -Wno-WIDTHTRUNC -Wno-WIDTHEXPAND $(VINC) $(SRC); \
	fi
	@printf "$(GREEN)[OK] lint$(NC)\n"

unit:
	@mkdir -p $(BUILD_DIR)
	@printf "$(BLUE)[UNIT] run tb/verilog unit tests...$(NC)\n"
	@for tb in $$(find tb/verilog -name '*.v' | sort); do \
	  echo "-- $$tb"; \
	  $(IVERILOG) -o $(BUILD_DIR)/u_tb.vvp $$tb $(SRC) && \
	  (cd $(BUILD_DIR) && vvp u_tb.vvp) || exit 1; \
	done
	@printf "$(GREEN)[OK] unit$(NC)\n"

clean:
	rm -rf $(BUILD_DIR)
