// SPDX-License-Identifier: MIT
// Copyright (c) 2023, Advanced Micro Devices, Inc. All rights reserved.

#ifndef HSA_CSR_H_
#define HSA_CSR_H_

#include <cstdint>

#include "amd_hsa.h"
#include "memory.h"

inline constexpr int NUM_AQL_QUEUES(7);
inline constexpr int AQL_QUEUE_NUM_ENTRIES(64);
inline constexpr int AQL_QUEUE_BYTES_PER_ENTRY(64);
inline constexpr int NUM_HEAP_PAGES(7);

/// @struct HsaControlStatusRegs
///
/// @brief Structure holding information about the agent's queues,
/// doorbells, and signals. This struct occupies the first page
/// of BRAM and acts as CSRs that can be accessed via MMIO by the
/// driver.
struct __attribute__((aligned(PAGE_SIZE))) HsaControlStatusRegs {
  int version;
  int num_aql_queues;
  /// AQL queue size in number of entries. Each entry is 64B.
  int aql_queue_size;
  /// AQL queue size in bytes.
  int aql_queue_size_bytes;

  int queue_desc_offset;
  int queue_desc_size;

  int queue_buf_offset;
  int queue_buf_size;

  int heap_offset;
  int heap_size;

  int doorbell_offset;
  int doorbell_size;

  /// Associate a process' CPU virtual address for its on-device DRAM heap with
  /// the queues it owns. This allows the packet processors to easily translate
  /// from virtual addresses in the packets to physical addresses.
  uint64_t queue_dram_cpu_va[NUM_AQL_QUEUES];

  /// Below here are FW-private fields that are not meant to be used by the
  /// driver.
  uint64_t queue_dram_cpu_pa[NUM_AQL_QUEUES];
  uint64_t queue_desc_base;
  uint64_t queue_buf_base;
  uint64_t heap_base;
  uint64_t doorbell_base;

  uint64_t global_barrier;

  /// The HSA AQL queue descriptors. These function as the hardware queues.
  amd_queue_t *amd_aql_queues[NUM_AQL_QUEUES];
  /// Address of queue's doorbells. We give one page worth of doorbells to
  /// each queue.
  uint64_t doorbells[NUM_AQL_QUEUES];
  /// Addresses of queue circular buffer space. One page of space per queue.
  uint64_t queue_bufs[NUM_AQL_QUEUES];
  /// Addresses of heap pages.
  uint64_t heap[NUM_HEAP_PAGES];

  uint32_t dna_reg[4];
};

static_assert(sizeof(HsaControlStatusRegs) == PAGE_SIZE,
              "HsaControlStatusRegs must be aligned to a single page.");
static_assert(NUM_AQL_QUEUES * sizeof(amd_queue_t) <= PAGE_SIZE,
	      "Only one page worth of queue descriptors is supported.");

extern HsaControlStatusRegs *hsa_csr;

void hsa_csr_init();
void hsa_csr_print();

#endif // HSA_CSR_H_
