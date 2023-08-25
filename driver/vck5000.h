// Copyright (C) 2023, Advanced Micro Devices, Inc.
// SPDX-License-Identifier: MIT

#ifndef VCK5000_H_
#define VCK5000_H_

/**
 * Range of AIE CSR in the VCK5000
 */
#define VCK5000_AIE_MEM_RANGE 0x20000000

/**
 * vck5000_dev_init - Initialize a VCK5000 device.
 */
void vck5000_dev_init(struct amdair_device *air_dev);

#endif /* VCK5000_H_ */
