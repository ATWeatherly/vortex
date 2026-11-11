# Platform specific configurations
# Add your platform specific configurations here

CONFIGS += -DPLATFORM_MEMORY_DATA_WIDTH=512

ifeq ($(DEV_ARCH), zynquplus)
# zynquplus
CONFIGS += -DVX_CFG_PLATFORM_MEMORY_NUM_BANKS=1 -DVX_CFG_PLATFORM_MEMORY_ADDR_WIDTH=32
else ifeq ($(DEV_ARCH), versal)
# versal
CONFIGS += -DVX_CFG_PLATFORM_MEMORY_NUM_BANKS=1 -DVX_CFG_PLATFORM_MEMORY_ADDR_WIDTH=32
ifneq ($(findstring xilinx_vck5000,$(XSA)),)
	CONFIGS += -DPLATFORM_MEMORY_OFFSET=40'hC000000000
endif
else
# alveo
# The Command Processor's host-memory master (m_axi_host) reaches host DRAM
# through the platform slave-bridge / Host Memory Access aperture. All Alveo
# XDMA shells expose this as the HOST[0] connectivity tag.
VPP_FLAGS += --connectivity.sp vortex_afu_1.m_axi_host:HOST[0]
ifneq ($(findstring xilinx_u55c,$(XSA)),)
  # 16 GB of HBM2 with 32 channels (512 MB per channel)
  CONFIGS += -DVX_CFG_PLATFORM_MEMORY_NUM_BANKS=32 -DVX_CFG_PLATFORM_MEMORY_ADDR_WIDTH=34
  CONFIGS += -DPLATFORM_MERGED_MEMORY_INTERFACE
  VPP_FLAGS += --connectivity.sp vortex_afu_1.m_axi_mem_0:HBM[0:31]
  #VPP_FLAGS += $(foreach i,$(shell seq 0 31), --connectivity.sp vortex_afu_1.m_axi_mem_$(i):HBM[$(i)])
else ifneq ($(findstring xilinx_u50,$(XSA)),)
  # 8 GB of HBM2 with 32 channels (256 MB per channel)
  CONFIGS += -DVX_CFG_PLATFORM_MEMORY_NUM_BANKS=32 -DVX_CFG_PLATFORM_MEMORY_ADDR_WIDTH=33
  CONFIGS += -DPLATFORM_MERGED_MEMORY_INTERFACE
  VPP_FLAGS += --connectivity.sp vortex_afu_1.m_axi_mem_0:HBM[0:31]
  # Tile stream unit (balanced-design spec phase 1): TSU_MASTERS=<n> (32, 16 or 8) adds n read-only
  # masters m_axi_str_<i>. An sp link is the master's WHOLE address space: a master linked to HBM[k] reads
  # garbage outside PC k (u50wgts_2c200, 2026-11-11: TSU path 528k/1M values wrong; the old note "the HBM
  # switch reaches the rest of its group" was false). Two link modes:
  #   TSU_LINK=full (default): every master linked to HBM[0:31], so any buffer address is correct (phase 0:
  #     4 masters on HBM[0:31] through the switch ran at the one-PC rate; hmss cost is per port).
  #   TSU_LINK=group: master i linked to its PC group HBM[i*g : i*g+g-1], g = 32/n (phase 0 tgen16.cfg:
  #     16 x 2 PCs = 190.8 GB/s). Correct ONLY for PC-striped data in which every burst routed to master i
  #     lies in group i; it adds -DVX_CFG_TSU_LINK_GROUP, the RTL assert that checks exactly that. Socket 1's
  #     masters serve PCs 16-31: the host programs full-width (VX_CFG_TSU_ADDR_WIDTH = 33) descriptor bases,
  #     kernel pointers stay 32-bit. Production: TSU_MASTERS=8 TSU_LINK=group (1 TSU worker per socket, any
  #     PC of the socket's 16 reachable). TSU_MASTERS=16 needs TSU_LINK=full until landings are steered to the
  #     worker that owns the tile's PC (2 workers per socket; v1 steers by sequencer block).
  # Either way VX_CFG_TSU_SEL_LSB = 28 + log2(g) routes a burst to master (PC group mod 4) of its worker, so
  # a master never alternates between PC groups. The masters are split over sockets and TSU workers,
  # so n must be a multiple of sockets x VX_CFG_TSU_MASTERS_PER_WORKER (VX_afu_wrap asserts it).
  TSU_LINK ?= full
  ifneq ($(TSU_MASTERS),)
    CONFIGS += -DVX_CFG_TSU_ENABLE -DVX_CFG_STREAM_NUM_MASTERS=$(TSU_MASTERS)
    ifeq ($(TSU_LINK),group)
      CONFIGS += -DVX_CFG_TSU_LINK_GROUP
      VPP_FLAGS += $(shell n=$(TSU_MASTERS); i=0; while [ $$i -lt $$n ]; do echo "--connectivity.sp vortex_afu_1.m_axi_str_$$i:HBM[$$((i*32/n)):$$((i*32/n + 32/n - 1))]"; i=$$((i+1)); done)
    else
      VPP_FLAGS += $(shell n=$(TSU_MASTERS); i=0; while [ $$i -lt $$n ]; do echo "--connectivity.sp vortex_afu_1.m_axi_str_$$i:HBM[0:31]"; i=$$((i+1)); done)
    endif
  endif
else ifneq ($(findstring xilinx_u280,$(XSA)),)
  # 8 GB of HBM2 with 32 channels (256 MB per channel)
  CONFIGS += -DVX_CFG_PLATFORM_MEMORY_NUM_BANKS=32 -DVX_CFG_PLATFORM_MEMORY_ADDR_WIDTH=33
  VPP_FLAGS += --connectivity.sp vortex_afu_1.m_axi_mem_0:HBM[0:31]
else ifneq ($(findstring xilinx_u250,$(XSA)),)
  # 16 GB of DDR4 (single channel, bank 0). Multi-bank requires per-bank XRT
  # VA offsets that aren't known at synthesis time.
  CONFIGS += -DVX_CFG_PLATFORM_MEMORY_NUM_BANKS=1 -DVX_CFG_PLATFORM_MEMORY_ADDR_WIDTH=34
  VPP_FLAGS += --connectivity.sp vortex_afu_1.m_axi_mem_0:DDR[0]
  CONFIGS += -DPLATFORM_MEMORY_OFFSET=40\'h4000000000
else ifneq ($(findstring xilinx_u200,$(XSA)),)
  # 64 GB of DDR4 with 4 channels (16 GB per channel)
  CONFIGS += -DVX_CFG_PLATFORM_MEMORY_NUM_BANKS=4 -DVX_CFG_PLATFORM_MEMORY_ADDR_WIDTH=36
else
  CONFIGS += -DVX_CFG_PLATFORM_MEMORY_NUM_BANKS=1 -DVX_CFG_PLATFORM_MEMORY_ADDR_WIDTH=32
endif
endif
