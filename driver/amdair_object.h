// Copyright (C) 2023, Advanced Micro Devices, Inc.
// SPDX-License-Identifier: MIT

#ifndef AMDAIR_OBJECT_H_
#define AMDAIR_OBJECT_H_

#include <linux/interval_tree.h>

#include "amdair_device.h"

#define MAX_BUF_OBJ_ID 0xFFFF

enum bo_heap_type {
	BO_HEAP_TYPE_BRAM,
	BO_HEAP_TYPE_DRAM
};

/**
 * struct amdair_buf_object - Buffer object for tracking heap allocations to
 *                            on-device memory.
 *
 * @air_dev: The device that owns this allocation.
 *
 * @it_node: Interval tree node for tracking the buffer object's address range.
 */
struct amdair_buf_object {
	struct amdair_device *dev;
	struct interval_tree_node it_node;
	enum bo_heap_type heap_type;
};

#endif /* AMDAIR_OBJECT_H_ */
