# Copyright (C) 2024, Advanced Micro Devices, Inc.
# SPDX-License-Identifier: MIT

# Making sure we have an argument
if { $argc != 1 } {
  puts "Firmware reset script not provided path to firmware binary"
  return
} 

# Process to reload the firmware on the device
connect
ta 6
rst -processor
dow [lindex $argv 0 ]
con

