// SPDX-License-Identifier: MIT
// Copyright (c) 2023, Advanced Micro Devices, Inc. All rights reserved.

#include "hsa.h"

#include "amd_hsa.h"
#include "memory.h"

void HSA_API hsa_signal_subtract_scacq_screl(hsa_signal_t signal,
                                             hsa_signal_value_t value) {
  uint64_t phys_addr(translate_virt_to_phys(signal.handle));
  amd_signal_t *amd_signal(reinterpret_cast<amd_signal_t*>(phys_addr));
  amd_signal->value = amd_signal->value - value;
}
