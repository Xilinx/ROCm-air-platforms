// SPDX-License-Identifier: MIT
// Copyright (c) 2023, Advanced Micro Devices, Inc. All rights reserved.

#include "hsa.h"

#include "amd_hsa.h"
#include "memory.h"

void HSA_API hsa_signal_subtract_scacq_screl(hsa_signal_t signal,
                                             hsa_signal_value_t value) {

  uint64_t phys_addr(translate_virt_to_phys(signal.handle));
  amd_signal_t *amd_signal(reinterpret_cast<amd_signal_t *>(phys_addr));
  amd_signal->value = amd_signal->value - value;
}

hsa_signal_value_t HSA_API hsa_signal_wait_scacquire(
    hsa_signal_t signal, hsa_signal_condition_t condition,
    hsa_signal_value_t compare_value, uint64_t timeout_hint,
    hsa_wait_state_t wait_state_hint) {

  // Signal handle of 0 means we do not need to wait on it
  if (signal.handle == 0)
    return 0;

  hsa_signal_value_t ret = 0;
  uint64_t timeout = timeout_hint;

  // Spinning until the signal is complete
  do {

    // Obtaining the signal value pointed to by the signal handle
    uint64_t signal_handle_pa = translate_virt_to_phys(signal.handle);
    amd_signal_t *amd_signal(
        reinterpret_cast<amd_signal_t *>(signal_handle_pa));
    ret = amd_signal->value;

    // Testing the signal value
    if (condition == HSA_SIGNAL_CONDITION_EQ && ret == compare_value ||
        condition == HSA_SIGNAL_CONDITION_NE && ret != compare_value ||
        condition == HSA_SIGNAL_CONDITION_LT && ret < compare_value ||
        condition == HSA_SIGNAL_CONDITION_GTE && ret >= compare_value)
      return compare_value;

  } while (timeout--);

  return ret;
}
