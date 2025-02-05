//===- arm_intf.h -----------------------------------------------*- C++ -*-===//
//
// Copyright (C) 2025, Advanced Micro Devices, Inc.
// SPDX-License-Identifier: MIT
//
//===----------------------------------------------------------------------===//

#ifndef __ARM_INTF_H_
#define __ARM_INTF_H_

#include "unistd.h"
#include <cstdint>

// Templated Read/Write methods
// base_addr is the virtual address returned by an mmap call (e.g., from bp_map_cfg or bp_map_dram)
// offset is a byte offset into the mapped address range (e.g., BP_CFG_FREEZE_OFFSET)
// T is the data type to read/write (e.g., uint32_t or uint64_t)
// Each function first computes the proper virtual address and casts it to the appropriate type
// Then, the read or write is performed
template <class T>
void mmio_write(void* base_addr, off_t offset, T value) {
  T* addr = reinterpret_cast<T*>(reinterpret_cast<uintptr_t>(base_addr) + offset);
  *addr = value;
}

template <class T>
T mmio_read(void* base_addr, off_t offset) {
  T* addr = reinterpret_cast<T*>(reinterpret_cast<uintptr_t>(base_addr) + offset);
  return *addr;
}

// byte reversal - works for integer data types
template <class T>
T reverse_bytes(T bytes) {
  int s = sizeof(T);
  T result = 0;
  // shift input right to select correct byte
  // shift selected byte left to place into result
  // start with LSB of bytes (placed into MSB of result)
  for (int i = 0; i < s; i++) {
    result |= (((bytes >> (i<<3)) & 0xff) << ((s-i-1)<<3));
  }
  return result;
}

#endif // __ARM_INTF_H_
