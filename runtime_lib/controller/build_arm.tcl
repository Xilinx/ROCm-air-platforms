# Copyright (C) 2022, Xilinx Inc.
# Copyright (C) 2022, Advanced Micro Devices, Inc.
# SPDX-License-Identifier: MIT

setws build

set script_directory [file dirname [file normalize [info script]]]

set root_directory [file normalize [file join $script_directory "../.."]]

platform create -name "arm" -hw [file join ${root_directory} "platforms/xilinx_vck5000_air/vivado/xilinx_vck5000_air.xsa"]

domain create -name airrt_arm -proc psv_cortexa72_0 -os standalone

platform generate

app create -lang c++ -name acdc_agent -platform arm -domain airrt_arm -template "Empty Application (C++)"
#app config -name acdc_agent build-config Release
#app config -name acdc_agent compiler-misc {-O1}

importsources -name acdc_agent -soft-link -path [file join $root_directory "runtime_lib/controller/amd_hsa.h"]
importsources -name acdc_agent -soft-link -path [file join $root_directory "runtime_lib/controller/debug.h"]
importsources -name acdc_agent -soft-link -path [file join $root_directory "runtime_lib/controller/hsa.cpp"]
importsources -name acdc_agent -soft-link -path [file join $root_directory "runtime_lib/controller/hsa.h"]
importsources -name acdc_agent -soft-link -path [file join $root_directory "runtime_lib/controller/hsa_csr.cpp"]
importsources -name acdc_agent -soft-link -path [file join $root_directory "runtime_lib/controller/hsa_csr.h"]
importsources -name acdc_agent -soft-link -path [file join $root_directory "runtime_lib/controller/main.cpp"]
importsources -name acdc_agent -soft-link -path [file join $root_directory "runtime_lib/controller/memory.cpp"]
importsources -name acdc_agent -soft-link -path [file join $root_directory "runtime_lib/controller/memory.h"]
importsources -name acdc_agent -soft-link -path [file join $root_directory "runtime_lib/controller/platform.cpp"]
importsources -name acdc_agent -soft-link -path [file join $root_directory "runtime_lib/controller/platform.h"]
importsources -name acdc_agent -soft-link -path [file join $root_directory "runtime_lib/controller/arm_bp_intf.cpp"]
importsources -name acdc_agent -soft-link -path [file join $root_directory "runtime_lib/controller/arm_bp_intf.h"]
importsources -name acdc_agent -soft-link -path [file join $root_directory "runtime_lib/controller/shell.cpp"]
importsources -name acdc_agent -soft-link -path [file join $root_directory "runtime_lib/controller/shell.h"]
app config -name acdc_agent include-path [file join $root_directory "runtime_lib/airhost/include"]
app config -name acdc_agent include-path [file join $root_directory "runtime_lib/controller/bp/include/"]

app build -name acdc_agent
