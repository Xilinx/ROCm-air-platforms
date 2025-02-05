# Copyright (C) 2022, Xilinx Inc.
# Copyright (C) 2022-2025, Advanced Micro Devices, Inc.
# SPDX-License-Identifier: MIT

TOP = $(shell git rev-parse --show-toplevel)
PLATFORM ?= xilinx_vck5000_air
BASE_DIR = $(TOP)/platform/vck5000_bp
XSA_DIR  = $(BASE_DIR)/vivado
PLATFORM_DIR = $(BASE_DIR)/aie_platform

.phony: all

BP_FW ?= bp

all: $(PLATFORM)

$(PLATFORM): xsa platform
	@echo 'Platform build complete'

blackparrot_ip:
	cp $(TOP)/platform/ip/black-parrot/blackparrot_ip.tar.gz vivado/
	cd vivado && tar xzf blackparrot_ip.tar.gz
	cp $(TOP)/firmware/bp/exe/$(BP_FW)_vivado.mem vivado/main.mem
	cp $(TOP)/firmware/bp/exe/$(BP_FW).mem vivado/bp.mem

blackparrot_ip_clean:
	rm -rf vivado/blackparrot_ip*
	rm -f vivado/*.mem

xsa: blackparrot_ip
	$(MAKE) -C vivado PLATFORM=$(PLATFORM)

xsa_clean: blackparrot_ip_clean
	$(MAKE) -C vivado clean PLATFORM=$(PLATFORM)
	$(MAKE) -C vivado ultraclean PLATFORM=$(PLATFORM)

platform: vivado/xilinx_vck5000_air.xsa
	$(MAKE) -C aie_platform all

platform_clean:
	$(MAKE) -C aie_platform clean
	$(MAKE) -C aie_platform ultraclean

ARCHIVE ?= $(PLATFORM).tar.gz
IMPL_BASE_DIR = $(PLATFORM_DIR)/_x/link/vivado/vpl
IMPL_LOG = $(IMPL_BASE_DIR)/vivado.log
IMPL = $(shell grep -E "First Timing Closed" $(IMPL_LOG) | grep -oE "impl_.*")
IMPL_DIR = $(IMPL_BASE_DIR)/prj/prj.runs/$(IMPL)
IMPL_RPT = $(notdir $(shell find $(IMPL_DIR) -name "*.rpt"))
IMPL_FW_DIR = $(PLATFORM_DIR)/firmware
BP_FW_MEM = $(notdir $(shell find $(XSA_DIR) -maxdepth 1 -type f -name "*.mem"))

archive:
	tar czvf $(ARCHIVE) \
	-C $(XSA_DIR) $(PLATFORM).xsa blackparrot_ip.tar.gz $(BP_FW_MEM) \
	-C $(PLATFORM_DIR) final_vck5000.pdi \
	-C $(IMPL_DIR) xilinx_qdma_pcie_ep.pdi xilinx_qdma_pcie_ep.ltx $(IMPL_RPT) \
	-C $(IMPL_FW_DIR) airrt_cpp.elf

clean_archive:
	rm -f $(ARCHIVE)

clean: xsa_clean platform_clean clean_archive
	${RM} -r platform_repo

