# Copyright (C) 2022, Xilinx Inc.
# Copyright (C) 2022, Advanced Micro Devices, Inc.
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
# OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.

VIVADO=$(XILINX_VIVADO)/bin/vivado


#------------------------------------------------------------------------------


ROOT_DIR=$(realpath $(dir $(lastword $(MAKEFILE_LIST))))


.PHONY: all pfm ultraclean clean sd_card ${XCLBIN} package

# Target: pfm
# Generate the Vitis platform with XSCT
# Input: vck5000_custom.xsa in ../step1_vivado/build/vivado directory
pfm:
	xsct xsct_create_pfm.tcl xilinx_vck5000_air ../vivado && \
	echo "Platform created:" && \
	find . -name "*.xpfm"


#------------------------------------------------------------------------------


TARGET   = hw
MODE	 = linux
PLATFORM = ./platform_repo/xilinx_vck5000_air/export/xilinx_vck5000_air/xilinx_vck5000_air.xpfm

XCLBIN   = vck5000_aie_base_graph_${TARGET}.xsa
XSA	 = vck5000_aie_base_graph_${TARGET}.xsa
XPFM	 = vck5000_xpfm

GRAPH    = aie/graph.cpp
LIBADF  = libadf.a
AIE_CMPL_CMD = aiecompiler -platform=${PLATFORM} -include="./aie" -workdir=./Work ${GRAPH}
AIE_SIM_CMD = aiesimulator --pkg-dir=./Work --dump-vcd foo
EMU_CMD = ./launch_hw_emu.sh

# Putting where we put the PL source code
PL_SRC_REPO  = ./pl
COUNTER_KERNEL_TOP = counter_hls
COUNTER_KERNEL_SRC = $(PL_SRC_REPO)/counter/$(COUNTER_KERNEL_TOP).cpp
COUNTER_KERNEL_XO  = $(COUNTER_KERNEL_TOP).$(TARGET)
COUNTER_KERNEL_VPP_FLAGS = --hls.clock $(VPP_CLOCK_FREQ):$(COUNTER_KERNEL_TOP)

##########################################################################################################################################################
### DO NOT MODIFY BELOW THIS LINE UNLESS NECESSARY
################################################################################################################################################


VCC      = v++
VPP_SPEC =system.cfg
VPP_FLAGS=--save-temps --verbose --config ${VPP_SPEC}
VPP_STRATEGY=--vivado.impl.strategies "Performance_ExplorePostRoutePhysOpt,Performance_NetDelay_high,Performance_WLBlockPlacementFanoutOpt"
LDCLFLAGS=

.PHONY: clean

###
# Guarding Checks. Do not modify.
###
check_defined = \
	$(strip $(foreach 1,$1, \
		$(call __check_defined,$1,$(strip $(value 2)))))

__check_defined = \
	$(if $(value $1),, \
		$(error Undefined $1$(if $2, ($2))))

guard-PLATFORM_REPO_PATHS:
	$(call check_defined, PLATFORM_REPO_PATHS, Set your where you downloaded xilinx_vck190_es1_base_202110_1)

guard-CXX:
	$(call check_defined, CXX, Run: xilinx-versal-common-v2021.1/environment-setup-cortexa72-cortexa53-xilinx-linux)

###

all: pfm ${LIBADF} ${XCLBIN} package_${TARGET}

aie: guard-PLATFORM_REPO_PATHS ${LIBADF}
${LIBADF}: aie/*
	${AIE_CMPL_CMD}

aiesim: ${LIBADF}
	${AIE_SIM_CMD}

xclbin: guard-PLATFORM_REPO_PATHS ${XCLBIN}

${COUNTER_KERNEL_XO}.xo:
	v++ --target $(TARGET) $(COUNTER_KERNEL_VPP_FLAGS) \
		$(VPP_FLAGS) -c -k $(COUNTER_KERNEL_TOP) \
		--platform ${PLATFORM} \
		$(COUNTER_KERNEL_SRC) -o $@


${XCLBIN}: $(COUNTER_KERNEL_XO).xo ${LIBADF} ${VPP_SPEC} 
	${VCC} -g -l --platform ${PLATFORM} ${COUNTER_KERNEL_XO}.xo ${LIBADF} -t ${TARGET} ${VPP_STRATEGY} ${VPP_FLAGS} -o $@ --to_step vpl.update_bd
	vivado -mode batch -source fixup_pl_design.tcl
	${VCC} -g -l --platform ${PLATFORM} ${COUNTER_KERNEL_XO}.xo ${LIBADF} -t ${TARGET} ${VPP_STRATEGY} ${VPP_FLAGS} -o $@ --from_step vpl.generate_target
  

package_${TARGET}: ${LIBADF} ${XCLBIN}
	bootgen -arch versal -image vck5000_bootgen.bif -w on -o final_vck5000.pdi -log error

pdi/microblaze_reset.cdo: pdi/microblaze_reset.txt
	cdoutil -output-binary-be -output-file pdi/microblaze_reset.cdo pdi/microblaze_reset.txt

program_vck5000: final_vck5000.pdi
	$(VIVADO) -mode batch -source program_vck5000.tcl

clean:
	$(RM) -r vitis_* xrc.log ./sd_dir/boot.scr rootfs ./sd_card/* *.ltx
	$(RM) -rf build sd_card.img BOOT.BIN boot_image.bif aie.merged.cdo.bin a.xclbin *.pdi
	$(RM) -rf _x v++_* ${XOS} ${OS} ${LIBADF} *.o.* *.o *.xpe *.xo.* \
	       vck5000_aie_base*.xclbin* *.xsa *.log *.package_summary *.jou xnwOut Work Map_Report.csv \
	       ilpProblem* sol.db drivers .Xil *.txt *.link_summary

ultraclean:
	$(RM) -r vitis_* xrc.log build platform_repo ./sd_dir/boot.scr output rootfs sw_comp

