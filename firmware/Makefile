# Copyright (C) 2022, Xilinx Inc.
# Copyright (C) 2022, Advanced Micro Devices, Inc.
# SPDX-License-Identifier: MIT

BUILD_DIR := build
APP_NAME := acdc_agent

all: ${BUILD_DIR}/${APP_NAME}/acdc_agent.elf fixup

${BUILD_DIR}/${APP_NAME}/acdc_agent.elf : ../platform/vck5000/vivado/xilinx_vck5000_air.xsa
	rm -rf build && xsct build_arm.tcl

fixup: ${BUILD_DIR}/${APP_NAME}/src/lscript.ld
	echo -e "\nModifying $< to increase the stack and heap size\n"; \
	sed -i 's/_SIZE : 0x2000/_SIZE : 0x100000/g' $<; \
	cd ${BUILD_DIR}/${APP_NAME}/Debug && make acdc_agent.elf

clean:
	rm -rf build
