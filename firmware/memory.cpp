//===- memory.cpp -----------------------------------------------*- C++ -*-===//
//
// Copyright (C) 2023, Advanced Micro Devices, Inc.
// SPDX-License-Identifier: MIT
//
//===----------------------------------------------------------------------===//

#include "hsa_csr.h"

uint64_t translate_virt_to_phys(uint64_t va) {
  return va - hsa_csr->queue_dram_cpu_va[1] + DRAM_BASE;
}
