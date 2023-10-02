// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2023, Advanced Micro Devices, Inc.

#include "amdair_mem_manager.h"

#include <linux/pci.h>

#include "amdair_device.h"
#include "amdair_object.h"

int amdair_mman_init(struct amdair_device *air_dev)
{
	struct amdair_buf_object *air_bo = NULL;
	int num_dram_heaps = 0;
	int i = 0;
	int ret = 0;

	if (!air_dev)
		return -EINVAL;

	air_dev->mman.dev = air_dev;

	num_dram_heaps = air_dev->dram_size / DRAM_HEAP_SIZE_PER_PROCESS;
	air_dev->mman.dram_size = air_dev->dram_size;

	for (i = 0; i < num_dram_heaps; ++i) {
		air_bo = kzalloc(sizeof(struct amdair_buf_object), GFP_KERNEL);

		if (!air_bo) {
			ret = -ENOMEM;
			/* Clean up if we did allocate some BOs. */
			goto err_bo_alloc;
		}

		air_bo->dev = air_dev;
		air_bo->it_node.start =
			air_dev->dram_base + i * DRAM_HEAP_SIZE_PER_PROCESS;
		air_bo->it_node.last =
			air_bo->it_node.start + DRAM_HEAP_SIZE_PER_PROCESS - 1;
		air_bo->heap_type = BO_HEAP_TYPE_DRAM;

		interval_tree_insert(&air_bo->it_node,
				     &air_dev->mman.dram_heap_free);
	}

	return 0;

err_bo_alloc:
	amdair_mman_free_resources(&air_dev->mman);
	return ret;
}

int amdair_mman_alloc_dram(struct amdair_mem_manager *air_mman,
			   struct amdair_buf_object **air_bo, uint64_t size)
{
	struct interval_tree_node *it_node = NULL;
	struct amdair_device *air_dev = air_mman->dev;

	if (!air_dev)
		return -EINVAL;

	it_node = interval_tree_iter_first(
		&air_dev->mman.dram_heap_free, air_dev->dram_base,
		air_dev->dram_base + air_dev->dram_size);

	if (!it_node || (it_node->last - it_node->start + 1) < size)
		return -ENOMEM;

	*air_bo = container_of(it_node, struct amdair_buf_object, it_node);
	if (!air_bo)
		return -ENOMEM;

	interval_tree_remove(it_node, &air_dev->mman.dram_heap_free);
	interval_tree_insert(it_node, &air_dev->mman.dram_heap_allocated);

	return 0;
}

int amdair_mman_free_dram(struct amdair_mem_manager *air_mman,
			  struct amdair_buf_object *air_bo)
{
	interval_tree_remove(&air_bo->it_node, &air_mman->dram_heap_allocated);
	interval_tree_insert(&air_bo->it_node, &air_mman->dram_heap_free);
	return 0;
}

void amdair_mman_free_resources(struct amdair_mem_manager *air_mman)
{
	struct amdair_device *air_dev = air_mman->dev;
	struct amdair_buf_object *air_bo = NULL;
	struct interval_tree_node *it_node = NULL;

	it_node = interval_tree_iter_first(
		&air_dev->mman.dram_heap_free, air_dev->dram_base,
		air_dev->dram_base + air_dev->dram_size);
	while (it_node) {
		air_bo = container_of(it_node, struct amdair_buf_object,
				      it_node);
		kfree(air_bo);
		interval_tree_remove(it_node, &air_dev->mman.dram_heap_free);
		it_node = interval_tree_iter_first(
			&air_dev->mman.dram_heap_free, air_dev->dram_base,
			air_dev->dram_base + air_dev->dram_size);
	}

	it_node = interval_tree_iter_first(
		&air_dev->mman.dram_heap_allocated, air_dev->dram_base,
		air_dev->dram_base + air_dev->dram_size);
	while (it_node) {
		air_bo = container_of(it_node, struct amdair_buf_object,
				      it_node);
		kfree(air_bo);
		interval_tree_remove(it_node,
				     &air_dev->mman.dram_heap_allocated);
		it_node = interval_tree_iter_first(
			&air_dev->mman.dram_heap_allocated, air_dev->dram_base,
			air_dev->dram_base + air_dev->dram_size);
	}
}
