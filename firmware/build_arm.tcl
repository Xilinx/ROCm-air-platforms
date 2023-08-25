# Copyright (C) 2022, Xilinx Inc.
# Copyright (C) 2022, Advanced Micro Devices, Inc.
# SPDX-License-Identifier: MIT

setws build

set script_directory [file dirname [file normalize [info script]]]

set root_directory [file normalize [file join $script_directory ".."]]

platform create -name "arm" -hw [file join ${root_directory} "platform/vck5000/vivado/xilinx_vck5000_air.xsa"]

domain create -name airrt_arm -proc psv_cortexa72_0 -os standalone

platform generate

app create -lang c++ -name acdc_agent -platform arm -domain airrt_arm -template "Empty Application (C++)"
#app config -name acdc_agent build-config Release
#app config -name acdc_agent compiler-misc {-O1}

importsources -name acdc_agent -soft-link -path [file join $root_directory "firmware/amd_hsa.h"]
importsources -name acdc_agent -soft-link -path [file join $root_directory "firmware/hsa_ext_air.h"]
importsources -name acdc_agent -soft-link -path [file join $root_directory "firmware/debug.h"]
importsources -name acdc_agent -soft-link -path [file join $root_directory "firmware/hsa.cpp"]
importsources -name acdc_agent -soft-link -path [file join $root_directory "firmware/hsa.h"]
importsources -name acdc_agent -soft-link -path [file join $root_directory "firmware/hsa_csr.cpp"]
importsources -name acdc_agent -soft-link -path [file join $root_directory "firmware/hsa_csr.h"]
importsources -name acdc_agent -soft-link -path [file join $root_directory "firmware/main.cpp"]
importsources -name acdc_agent -soft-link -path [file join $root_directory "firmware/memory.cpp"]
importsources -name acdc_agent -soft-link -path [file join $root_directory "firmware/memory.h"]
importsources -name acdc_agent -soft-link -path [file join $root_directory "firmware/platform.cpp"]
importsources -name acdc_agent -soft-link -path [file join $root_directory "firmware/platform.h"]
importsources -name acdc_agent -soft-link -path [file join $root_directory "firmware/cdma.cpp"]
importsources -name acdc_agent -soft-link -path [file join $root_directory "firmware/cdma.h"]
importsources -name acdc_agent -soft-link -path [file join $root_directory "firmware/shell.cpp"]
importsources -name acdc_agent -soft-link -path [file join $root_directory "firmware/shell.h"]
# app config -name acdc_agent include-path [file join $root_directory "runtime_lib/airhost/include"]

# define ARM_CONTROLLER
configapp -app acdc_agent define-compiler-symbols ARM_CONTROLLER

app build -name acdc_agent
