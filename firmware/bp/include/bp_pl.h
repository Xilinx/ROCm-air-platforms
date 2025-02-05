// Copyright (C) 2022, Advanced Micro Devices, Inc.
// SPDX-License-Identifier: MIT

#ifndef BP_PL_H
#define BP_PL_H

#include "bp.h"

inline uint64_t bp_get_global_id() {
  uint64_t* p = (uint64_t*)(BP_CFG_OFFSET + BP_CFG_GLOBAL_ID_OFFSET);
  return *p;
}

inline uint64_t bp_get_did() {
  uint64_t* p = (uint64_t*)(BP_CFG_OFFSET + BP_CFG_DID_OFFSET);
  return *p;
}

inline uint64_t bp_get_mimpid() {
  uint64_t mimpid;
  __asm__ volatile("csrr %0, mimpid": "=r"(mimpid): :);
  return mimpid;
}

inline uint64_t bp_get_mcycle() {
  uint64_t mcycle;
  __asm__ volatile("csrr %0, mcycle": "=r"(mcycle): :);
  return mcycle;
}

#endif // BP_PL_H
