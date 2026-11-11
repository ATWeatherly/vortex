ROOT_DIR := $(realpath ../../..)
include $(ROOT_DIR)/config.mk

# Geometry of the block under test (8x16 for a fast run, 8x64 = the 512-MAC block)
ROWS     ?= 8
COLS     ?= 16
OUT_COLS ?= 2

PROJECT := tcu_mac_block

RTL_DIR := $(VORTEX_HOME)/hw/rtl
DPI_DIR := $(VORTEX_HOME)/hw/dpi

SRC_DIR := $(VORTEX_HOME)/hw/unittest/$(PROJECT)

CXXFLAGS := -I$(SRC_DIR) -I$(VORTEX_HOME)/hw/unittest/common -I$(SW_COMMON_DIR)
CXXFLAGS += -DROWS=$(ROWS) -DCOLS=$(COLS) -DOUT_COLS=$(OUT_COLS)

SRCS += $(SRC_DIR)/main.cpp

DBG_TRACE_FLAGS :=

RTL_PKGS :=

RTL_INCLUDE := -I$(RTL_DIR)/tcu/mac

TOP := VX_tcu_mac_block

PARAMS := -GROWS=$(ROWS) -GCOLS=$(COLS) -GOUT_COLS=$(OUT_COLS)

OPTS ?=

include ../common.mk
