// Copyright (C) 2023, Advanced Micro Devices, Inc.
// SPDX-License-Identifier: MIT

#include <linux/module.h>
#include <linux/pci.h>

#include "amdair_device.h"

/* Offsets into 'info page' in BRAM */
#define REG_HERD_CONTROLLER_COUNT 0x208
#define HERD_CONTROLLER_BASE_ADDR(_base, _x)                                   \
	((((uint64_t)ioread32(_base + (_x * sizeof(uint64_t) + 4))) << 32) |   \
	 ioread32(_base + (_x * sizeof(uint64_t) + 0)))

LIST_HEAD(device_list);

int amdair_device_init(struct amdair_device *air_dev)
{
	struct pci_dev *pdev;

	if (!air_dev)
		return -EINVAL;

	pdev = air_dev->pdev;
	air_dev->dram_bar = pcim_iomap_table(pdev)[DRAM_BAR_INDEX];
	air_dev->aie_bar = pcim_iomap_table(pdev)[AIE_BAR_INDEX];
	air_dev->bram_bar = pcim_iomap_table(pdev)[BRAM_BAR_INDEX];
	air_dev->dram_bar_len = pci_resource_len(pdev, DRAM_BAR_INDEX);
	air_dev->aie_bar_len = pci_resource_len(pdev, AIE_BAR_INDEX);
	air_dev->bram_bar_len = pci_resource_len(pdev, BRAM_BAR_INDEX);
	dev_info(&pdev->dev, "bar 0: 0x%lx (0x%llx)",
		 (unsigned long)air_dev->dram_bar, air_dev->dram_bar_len);
	dev_info(&pdev->dev, "bar 2: 0x%lx (0x%llx)",
		 (unsigned long)air_dev->aie_bar, air_dev->aie_bar_len);
	dev_info(&pdev->dev, "bar 4: 0x%lx (0x%llx)",
		 (unsigned long)air_dev->bram_bar, air_dev->bram_bar_len);

	air_dev->mem_addr = 0xbadbeef;
	air_dev->queue_used = 0;
	air_dev->controller_count = get_controller_count(air_dev);

	/* Take queue 0 for exclusive use by the driver */
	mark_controller_busy(air_dev, 0, 0);

	return 0;
}

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

