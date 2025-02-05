# Copyright(C) 2022-2025 Advanced Micro Devices, Inc. All Rights Reserved.
# SPDX-License-Identifier: MIT
#

PLATFORMS_TOP        := $(shell git rev-parse --show-toplevel)
export IP_BASE_DIR   := $(abspath $(PLATFORMS_TOP)/platform/ip)

include $(IP_BASE_DIR)/Makefile.common

# Export variables required by the vivado tcl script
export PART          ?= xcvc1902-vsva2197-2MP-e-S
export IP_NAME        = blackparrot
export TOP_MODULE     = blackparrot
export PROJECT_NAME   = $(IP_NAME)_ip_proj
export PROJECT_DIR    = $(BLACKPARROT_IP_DIR)/$(PROJECT_NAME)
export PROJECT_IP_DIR = $(BLACKPARROT_IP_DIR)/$(IP_NAME)_ip

export BASE_FLIST     = $(BLACKPARROT_IP_DIR)/base.flist
export IP_BASE_FLIST  = $(BLACKPARROT_IP_DIR)/ip_base.flist
export FLIST          = $(BLACKPARROT_IP_DIR)/ip.flist
export LINT_FLIST     = $(BLACKPARROT_IP_DIR)/lint.flist

export BASEJUMP_STL_DIR      = $(BLACKPARROT_DIR)/external/basejump_stl
export BP_FE_DIR             = $(BLACKPARROT_DIR)/bp_fe
export BP_ME_DIR             = $(BLACKPARROT_DIR)/bp_me
export BP_BE_DIR             = $(BLACKPARROT_DIR)/bp_be
export BP_TOP_DIR            = $(BLACKPARROT_DIR)/bp_top
export BP_COMMON_DIR         = $(BLACKPARROT_DIR)/bp_common
export HARDFLOAT_DIR         = $(BLACKPARROT_DIR)/external/HardFloat
export IP_SRC_DIR            = $(BLACKPARROT_IP_DIR)/src
export TCL_DIR               = $(BLACKPARROT_IP_DIR)/tcl

BP_FLIST                     = $(BP_TOP_DIR)/syn/flist.vcs
LINT_TOP                    ?= blackparrot_wrapper

PATCH_DIR                    = $(BLACKPARROT_IP_DIR)/patch
BLACKPARROT_PATCH            = $(PATCH_DIR)/0001-air-config.patch

STATUS_LOG                   = $(BLACKPARROT_IP_DIR)/status.log
STATUS_DIFF                  = $(BLACKPARROT_IP_DIR)/status.diff
STATUS_BP_DIFF               = $(BLACKPARROT_IP_DIR)/status_bp.diff
STATUS_BP_SUB_DIFF           = $(BLACKPARROT_IP_DIR)/status_bp_sub.diff

checkout:
	git submodule update --init --checkout -- $(BLACKPARROT_DIR)
	cd $(BLACKPARROT_DIR) && git submodule update --init
	git submodule update --init --checkout -- $(BLACKPARROT_SUB_DIR)
	git submodule update --init --checkout -- $(BLACKPARROT_SDK_DIR)

deinit:
	git submodule deinit -- $(BLACKPARROT_DIR)
	git submodule deinit -- $(BLACKPARROT_SUB_DIR)
	git submodule deinit -- $(BLACKPARROT_SDK_DIR)

patch_bp:
	cd $(BLACKPARROT_DIR) && git apply $(BLACKPARROT_PATCH)

bleach_patch:
	cd $(BLACKPARROT_DIR) && git restore .

repo_status: clean_status
	echo "mlir-air status:" >> $(STATUS_LOG)
	git rev-parse HEAD >> $(STATUS_LOG)
	git status -s >> $(STATUS_LOG)
	git diff HEAD >> $(STATUS_DIFF)
	echo "black-parrot status:" >> $(STATUS_LOG)
	cd $(BLACKPARROT_DIR) && git rev-parse HEAD >> $(STATUS_LOG)
	cd $(BLACKPARROT_DIR) && git status -s >> $(STATUS_LOG)
	cd $(BLACKPARROT_DIR) && git diff HEAD >> $(STATUS_BP_DIFF)
	echo "black-parrot-subsystems status:" >> $(STATUS_LOG)
	cd $(BLACKPARROT_SUB_DIR) && git rev-parse HEAD >> $(STATUS_LOG)
	cd $(BLACKPARROT_SUB_DIR) && git status -s >> $(STATUS_LOG)
	cd $(BLACKPARROT_SUB_DIR) && git diff HEAD >> $(STATUS_BP_SUB_DIFF)

base_flist: $(IP_BASE_FLIST)
$(IP_BASE_FLIST): $(BP_FLIST) $(BASE_FLIST)
	echo "+incdir+$(IP_SRC_DIR)" > $@
	# substitute paths in BP flist
	cat $^ | envsubst >> $@
	# Replace hardened memory which is incorrectly inferred on some Xilinx FPGAs
	sed -i "/bsg_mem_1rw_sync_mask_write_bit.sv/d" $@
	sed -i "/bsg_mem_1rw_sync_mask_write_bit_synth.sv/d" $@
	sed -i "/bsg_mem_1rw_sync_mask_write_byte.sv/d" $@
	sed -i "/bsg_mem_1rw_sync_mask_write_byte_synth.sv/d" $@
	echo "$(BASEJUMP_STL_DIR)/bsg_mem/bsg_mem_1rw_sync_mask_write_bit_from_1r1w.sv" >> $@
	echo "$(BASEJUMP_STL_DIR)/hard/ultrascale_plus/bsg_mem/bsg_mem_1rw_sync_mask_write_bit.sv" >> $@
	echo "$(BASEJUMP_STL_DIR)/hard/ultrascale_plus/bsg_mem/bsg_mem_1rw_sync_mask_write_byte.sv" >> $@

# lint with Verilator
lint_flist: $(LINT_FLIST)
$(LINT_FLIST): $(IP_BASE_FLIST)
	cp $< $@

.NOTPARALLEL: lint
lint: lint_flist
	$(VERILATOR) --lint-only --Wall --Wno-unoptflat -Wno-timescalemod --top-module $(LINT_TOP) \
		-f $(LINT_FLIST) config.vlt 2>&1 | tee -i lint.log

clean_lint:
	rm -f $(IP_BASE_FLIST)
	rm -f $(LINT_FLIST)
	rm -f lint.log

ip_flist: $(FLIST)
$(FLIST): $(IP_BASE_FLIST)
	cp $< $@
	# Harden synchronizers and clock muxes
	#sed -i "s#.*bsg_launch_sync_sync.sv#$(BASEJUMP_STL_DIR)/hard/ultrascale_plus/bsg_async/bsg_launch_sync_sync.sv#g" $@
	#sed -i "s#.*bsg_mux.sv#$(BASEJUMP_STL_DIR)/hard/ultrascale_plus/bsg_misc/bsg_mux.sv#g" $@

.NOTPARALLEL: build_ip
build_ip: clean_ip clean_project repo_status ip_flist
	vivado -source build-ip.tcl -mode batch

pack_ip:
	cp $(STATUS_LOG) $(STATUS_DIFF) $(STATUS_BP_DIFF) $(STATUS_BP_SUB_DIFF) $(IP_NAME)_ip/
	tar -czvf $(IP_NAME)_ip.tar.gz $(IP_NAME)_ip

clean_ip: clean_flist
	rm -f $(IP_BASE_FLIST)
	rm -rf $(PROJECT_IP_DIR)
	rm -f $(PROJECT_IP_DIR).tar.gz

clean_project:
	rm -rf $(PROJECT_DIR)

clean_flist:
	rm -f $(FLIST)

clean_status:
	rm -f $(STATUS_LOG)
	rm -f $(STATUS_DIFF)
	rm -f $(STATUS_BP_DIFF)
	rm -f $(STATUS_BP_SUB_DIFF)

clean: clean_ip clean_project clean_lint clean_flist clean_status
	rm -f *.log
	rm -f *.jou

sdk:
	make -C $(BLACKPARROT_SDK_DIR) -j1 checkout
	make -C $(BLACKPARROT_SDK_DIR) -j1 apply_patches
	make -C $(BLACKPARROT_SDK_DIR) -j sdk_lite
	make -C $(BLACKPARROT_SDK_DIR) -j1 build.perch WITH_MARCH=rv64g

bleach_sdk:
	make -C $(BLACKPARROT_SDK_DIR) bleach_all

.NOTPARALLEL: bleach_all
bleach_all: clean bleach_sdk bleach_patch deinit

