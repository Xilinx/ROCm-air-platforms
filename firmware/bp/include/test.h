// Copyright (C) 2022, Advanced Micro Devices, Inc.
// SPDX-License-Identifier: MIT

#ifndef BP_TEST_H
#define BP_TEST_H

#include "debug.h"

inline void zero_memory(volatile uint64_t *memory, uint64_t dwords) {
  for (uint64_t i = 0; i < dwords; i++) {
    memory[i] = 0;
  }
}

inline void report(uint64_t start, uint64_t end, uint64_t ops, const char* label) {
  uint64_t cycles = end - start;
  double d_ops = (double)ops;
  double d_cycles = ((double)end - (double)start) / d_ops;
  uint64_t whole, frac;
  whole = d_cycles;
  frac = (d_cycles - whole) * 1000;
  xil_printf("\n\r%s\n\r", label);
  xil_printf("  %lu ops took %lu cyles (%lu.%lu cycles per op)\n\r\n\r", ops, cycles, whole, frac);
}

#endif // BP_TEST_H
