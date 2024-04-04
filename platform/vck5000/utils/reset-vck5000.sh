# Copyright (C) 2024, Advanced Micro Devices, Inc.
# SPDX-License-Identifier: MIT

#!/usr/bin/env bash

SCRIPT_DIR=$(pwd)/$(dirname $BASH_SOURCE)
DRIVER_DIR=${SCRIPT_DIR}/../../../driver/

echo "Resetting VCK5000"

echo "Reloading firmware..."
xsdb ${SCRIPT_DIR}/firmware-reset.tcl ${SCRIPT_DIR}/../aie_platform/firmware/airrt_cpp.elf
echo "Reloading firmware complete"

if [ ! -f ${DRIVER_DIR}/amdair.ko ]; then
  echo "No .ko file. Building driver..."
  make -C ${DRIVER_DIR}
  echo "Building driver complete"
fi

echo "Reloading driver..."
sudo rmmod -s amdair
sudo insmod ${DRIVER_DIR}/amdair.ko
echo "Reloading driver complete"

# Making it so we can access the sysfs without sudo
sudo chmod 777 /sys/class/amdair/amdair/00/address
sudo chmod 777 /sys/class/amdair/amdair/00/value
