// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2023, Advanced Micro Devices, Inc.

#ifndef AMDAIR_MEM_MANAGER_H_
#define AMDAIR_MEM_MANAGER_H_

#include <linux/interval_tree.h>

struct amdair_buf_object;
struct amdair_device;

/* Give a process 8MB of on-device DRAM heap space. */
#define DRAM_HEAP_SIZE_PER_PROCESS (8 * 1024 * 1024)

/**
 * struct amdair_mem_manager - Memory allocator for on-device memory
 *                             (i.e., BRAM/DRAM).
 *
 * @dev: Device that owns this memory manager.
 *
 * @dram_heap_free_list: List of free heap space that is available for new
 *                       processes.
 *
 * @dram_heap_allocated_list: List of heap space that is allocated to processes.
 *
 * @dram_size: Size in bytes of the on-device DRAM aperture.
 */
struct amdair_mem_manager {
	struct amdair_device *dev;
	struct rb_root_cached dram_heap_free;
	struct rb_root_cached dram_heap_allocated;
	resource_size_t dram_size;
};

/**
 * amdair_mman_init - Initialize a devices memory manager.
 *
 * @air_dev: The devices whose memory manager is being initialized.
 *
 * For now the memory manager divides the DRAM space up into evenly sized
 * chunks which are given out to processes to use as on-device heaps which
 * can then be managed by the user-space runtime (ROCr).
 */
int amdair_mman_init(struct amdair_device *air_dev);
int amdair_mman_alloc_dram(struct amdair_mem_manager *air_mman,
			   struct amdair_buf_object **air_bo, uint64_t size);
int amdair_mman_free_dram(struct amdair_mem_manager *air_mman,
			  struct amdair_buf_object *air_bo);
void amdair_mman_free_resources(struct amdair_mem_manager *air_mman);

#endif /* AMDAIR_MEM_MANAGER_H_ */
