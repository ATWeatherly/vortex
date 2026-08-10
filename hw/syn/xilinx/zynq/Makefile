ROOT_DIR := $(realpath ../../../..)
include $(ROOT_DIR)/config.mk

# Target board: arty-z7-10 (XC7Z010) or zybo-z7-20 (XC7Z020). The board name
# selects the Digilent board part (PS7 DDR/MIO preset) in project.tcl and the
# default device below.
BOARD ?= arty-z7-10
ifeq ($(BOARD),zybo-z7-20)
DEVICE ?= xc7z020clg400-1
else
DEVICE ?= xc7z010clg400-1
endif

# PL clock (FCLK_CLK0) in MHz
CLK_MHZ ?= 50

# Digilent board files (PS7 DDR3/MIO preset), from
# https://github.com/Digilent/vivado-boards (the new/board_files subdir).
BOARD_REPO ?= $(TOOLDIR)/digilent-vivado-boards/new/board_files

MAX_JOBS ?= 8
OPT_LEVEL ?= 3

VIVADO := $(XILINX_VIVADO)/bin/vivado

SRC_DIR := $(VORTEX_HOME)/hw/syn/xilinx/zynq

RTL_DIR := $(VORTEX_HOME)/hw/rtl
SCRIPT_DIR := $(VORTEX_HOME)/hw/scripts

NCPUS := $(shell lscpu | grep "^Core(s) per socket:" | awk '{print $$4}')
JOBS ?= $(shell echo $$(( $(NCPUS) > $(MAX_JOBS) ? $(MAX_JOBS) : $(NCPUS) )))

# Frozen XC7Z010 configuration (fits with ~5k LUT margin; see README.md).
# The relocated VX_MEM_* map (kernel @0x10000000, IO @0x1FF00000, stacks
# below 0x1FFF0000) is baked into <build>/hw/VX_types.vh at configure time —
# see README.md "Memory map" for the regeneration command.
CONFIGS ?= -DVX_CFG_EXT_F_DISABLE \
	-DVX_CFG_NUM_WARPS=2 -DVX_CFG_NUM_THREADS=2 \
	-DVX_CFG_ICACHE_SIZE=4096 -DVX_CFG_ICACHE_NUM_WAYS=1 \
	-DVX_CFG_DCACHE_SIZE=4096 -DVX_CFG_DCACHE_NUM_WAYS=1 \
	-DVX_CFG_PLATFORM_MEMORY_NUM_BANKS=1 \
	-DVX_CFG_MEM_BLOCK_SIZE=8

CONFIGS += -DSYNTHESIS -DVIVADO -DNDEBUG

XCONFIGS := $(shell python3 $(ROOT_DIR)/ci/gen_config.py --config=$(VORTEX_HOME)/VX_config.toml --cflags='$(CONFIGS) -DVX_CFG_XLEN=$(XLEN)')

RTL_INCLUDE = -I$(ROOT_DIR)/sw -I$(ROOT_DIR)/hw -I$(RTL_DIR) -I$(RTL_DIR)/libs -I$(RTL_DIR)/interfaces -I$(RTL_DIR)/core -I$(RTL_DIR)/mem -I$(RTL_DIR)/cache -I$(RTL_DIR)/fpu
RTL_INCLUDE += -I$(SRC_DIR)

# CP=1 builds the Command Processor integration (VX_afu_wrap top) instead of
# the minimal shim. Same BD topology plus the CP's host-memory AXI master.
CP ?= 0
ifeq ($(CP),1)
VX_TOP := Vortex_z7_cp_top
RTL_INCLUDE += -I$(RTL_DIR)/afu/xrt -I$(RTL_DIR)/cp
else
VX_TOP := Vortex_z7_top
endif

# Xilinx floating_point IP generation when the config has an FPU
ifeq (,$(filter -DVX_CFG_EXT_F_DISABLE,$(CONFIGS)))
FPU_IP_ENV = FPU_IP=project_1/ip
else
FPU_IP_ENV =
endif

CFLAGS += -DVX_CFG_XLEN=$(XLEN) -DVX_CFG_XLEN_$(XLEN)
CFLAGS += $(CONFIGS)
CFLAGS += $(RTL_INCLUDE)

all: build

gen-sources: project_1/sources.txt
project_1/sources.txt:
	mkdir -p project_1
	$(SCRIPT_DIR)/gen_sources.sh $(CFLAGS) -T$(VX_TOP) -P -Cproject_1/src -Oproject_1/sources.txt

pre_opt_hook.tcl: $(SRC_DIR)/pre_opt_hook.tcl
	cp $< $@

build: done.bit
done.bit: project_1/sources.txt pre_opt_hook.tcl
	MAX_JOBS=$(JOBS) OPT_LEVEL=$(OPT_LEVEL) AREA_OPT=$(AREA_OPT) CLK_MHZ=$(CLK_MHZ) BOARD=$(BOARD) BOARD_REPO=$(BOARD_REPO) VX_TOP=$(VX_TOP) $(FPU_IP_ENV) TOOL_DIR=$(SCRIPT_DIR) \
	  $(VIVADO) -mode batch -source $(SRC_DIR)/project.tcl -tclargs $(DEVICE) project_1/sources.txt
	echo done > done.bit

open:
	TOOL_DIR=$(SCRIPT_DIR) $(VIVADO) project_1/project_1.xpr &

clean:
	rm -rf project_1 .Xil *.log *.jou *.rpt *.bit *.xsa done.bit pre_opt_hook.tcl

.PHONY: all gen-sources build open clean
