// Copyright (C) 2023, Advanced Micro Devices, Inc.
// SPDX-License-Identifier: MIT

#ifndef AMDAIR_OBJECT_H_
#define AMDAIR_OBJECT_H_

#include <linux/types.h>
#include <linux/list.h>

enum {
	AMDAIR_OBJECT_TYPE_MEM_REGION,
	AMDAIR_OBJECT_TYPE_QUEUE,
};

struct amdair_object {
	struct list_head list;
	uint64_t handle;
	struct vck5000_device *dev; /* device that owns the memory */
	pid_t owner;
	uint32_t id;
	uint8_t type;
	uint8_t range; /* which address range the base address belongs in */
	uint64_t base; /* physical address or pcie address */
	uint64_t size;
};

void amdair_add_object(struct amdair_object *obj);
struct amdair_object *amdair_find_object_by_handle(uint64_t handle);
void amdair_remove_object(uint64_t handle);
void amdair_destroy_object_list(void);

#endif /* AMDAIR_OBJECT_H_ */
