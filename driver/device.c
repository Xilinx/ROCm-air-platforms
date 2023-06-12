// Copyright (C) 2023, Advanced Micro Devices, Inc.
// SPDX-License-Identifier: MIT

#include <linux/module.h>
#include "device.h"

/* Offsets into 'info page' in BRAM */
#define REG_HERD_CONTROLLER_COUNT 0x208
#define HERD_CONTROLLER_BASE_ADDR(_base, _x)                                   \
	((((uint64_t)ioread32(_base + (_x * sizeof(uint64_t) + 4))) << 32) |   \
	 ioread32(_base + (_x * sizeof(uint64_t) + 0)))

LIST_HEAD(device_list);

void add_device(struct amdair_device *dev)
{
	list_add_tail(&dev->list, &device_list);
}

struct amdair_device *get_device_by_id(uint32_t device_id)
{
	struct amdair_device *dev;
	list_for_each_entry (dev, &device_list, list) {
		if (dev->device_id == device_id)
			return dev;
	}

	return NULL;
}

/*
	How many controllers are available in this device
*/
uint32_t get_controller_count(struct amdair_device *dev)
{
	return ioread32(dev->bram_bar + REG_HERD_CONTROLLER_COUNT);
}

/*
	Read the address that the controller is polling on
*/
uint64_t get_controller_base_address(struct amdair_device *dev,
				     uint32_t ctrlr_idx)
{
	return HERD_CONTROLLER_BASE_ADDR(dev->bram_bar, ctrlr_idx);
}

/*
	Find a controller belonging to the specified device that does not have its
	queue allocated yet. Return the total number of controllers if there are
	none free.
*/
uint32_t find_free_controller(struct amdair_device *dev)
{
	uint32_t idx;

	for (idx = 0; idx < dev->controller_count; idx++) {
		if (!(dev->queue_used & (1ULL << idx))) {
			printk("Controller %u has a free queue\n", idx);
			return idx;
		}
	}

	return dev->controller_count;
}

/*
	When controllers can handle more than a single queue, these functions
	will be removed
*/
void mark_controller_busy(struct amdair_device *dev, uint32_t ctrlr_idx,
			  pid_t pid)
{
	if (dev->queue_used & (1ULL << ctrlr_idx)) {
		printk("Controller %u is already busy!\n", ctrlr_idx);
	}

	dev->queue_used |= (1ULL << ctrlr_idx);
	dev->queue_owner[ctrlr_idx] = pid;
}

void mark_controller_free(struct amdair_device *dev, uint32_t ctrlr_idx)
{
	if (!(dev->queue_used & (1ULL << ctrlr_idx))) {
		printk("Controller %u is not busy!\n", ctrlr_idx);
	}

	dev->queue_used &= ~(1ULL << ctrlr_idx);
	dev->queue_owner[ctrlr_idx] = 0;
}

