// SPDX-License-Identifier: MIT
// Copyright (c) 2023, Advanced Micro Devices, Inc. All rights reserved.

#ifndef MEMORY_H_
#define MEMORY_H_

inline constexpr uint64_t BRAM_BASE(0x20100000000);
inline constexpr uint64_t DRAM_BASE(0x800000000);
inline constexpr uint64_t PAGE_SIZE(0x1000);

uint64_t translate_virt_to_phys(uint64_t va);

#endif // MEMORY_H_
