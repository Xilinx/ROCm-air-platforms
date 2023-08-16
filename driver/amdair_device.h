// Copyright (C) 2023, Advanced Micro Devices, Inc.
// SPDX-License-Identifier: GPL-2.0

#ifndef AMDAIR_DEVICE_H_
#define AMDAIR_DEVICE_H_

#include <linux/kobject.h>

#include "amdair_device_type.h"
#include "amdair_doorbell.h"
#include "amdair_mem_manager.h"
#include "amdair_queue.h"

/* The indices in config space (64-bit BARs) */
#define DRAM_BAR_INDEX 0
#define AIE_BAR_INDEX 2
#define BRAM_BAR_INDEX 4

#define MAX_AIE_INSTANCE 1

/**
 * struct amdair_aie_info - Structure holding all instances of AIE devices.
 */
struct amdair_aie_info {
	struct amdair_device *aie_instance[MAX_AIE_INSTANCE];
	int num_aie_devs;
};

extern struct amdair_aie_info aie_info;

/**
 * struct amdair_device_asic_funcs - Functions that initialize device
 * information for specific chips.
 *
 * @init_queues: Initialize the device's queue information.
 *
 * @init_doorbells: Intialize the devices doorbell information.
 *
 * @set_device_heap: Set the heap address for a given queue.
 *
 * @send_admin_queue_cmd: Send a privileged command to the command processor.
 */
struct amdair_device_asic_funcs {
	void (*init_queues)(struct amdair_device *air_dev);
	void (*init_doorbells)(struct amdair_device *air_dev);
	void (*set_device_heap)(struct amdair_device *air_dev,
				int queue_reg_off,
				uint64_t vaddr);
	void (*send_admin_queue_cmd_and_wait)(struct amdair_device *air_dev,
					      uint16_t cmd_type, uint64_t *arg,
					      int num_args);
};

/**
 * struct amdair_device - Holding all information for each AIE device in the
 * system.
 *
 * @dev_asic_funcs: Device-specific functions for initializing device state.
 *
 * @mman: Manages the device's on-chip memory.
 *
 * @queue_mgr: Manages the device's queues.
 *
 * @doorbell: Information about the device's doorbell aperture.
 *
 * @dev_type: Device type that specifies which chip this device corresponds to.
 *
 * @device_id: Unique device ID number.
 *
 * @dram_base: Base address of the DRAM BAR.
 *
 * @dram_size: Size of the DRAM BAR.
 *
 * @bram_base: Base address of the BRAM BAR.
 *
 * @bram_size: Size of the BRAM BAR.
 *
 * This represents a single Versal ACAP card. This does not use the Linux
 * kernel 'device' infrastructure because we want to have only a single
 * character device interface (/dev/amdair) regardless of how many physical
 * devices are attached. This follows the AMDKFD driver design.
*/
struct amdair_device {
	struct pci_dev *pdev;
	struct kobject kobj_aie;

	struct amdair_device_asic_funcs *dev_asic_funcs;
	struct amdair_mem_manager mman;
	struct amdair_queue_manager queue_mgr;
	struct amdair_doorbell doorbell;
	enum amdair_device_type dev_type;

	uint32_t device_id;

	resource_size_t dram_base;
	resource_size_t dram_size;
	resource_size_t bram_base;
	resource_size_t bram_size;

	void __iomem *aie_bar;
	uint64_t aie_bar_len;
	void __iomem *bram_bar;

	/* AIE memory can be accessed indirectly through sysfs.
		It is a two-step protocol:
		(1) write the memory address to:
		/sys/class/amdair/amdair/<id>/address
		(2) Read (or write) the value from:
		/sys/class/amdair/amdair/<id>/value
	*/
	uint64_t mem_addr; /* address for indirect memory access */
};

int amdair_device_init(struct amdair_device *air_dev,
		       enum amdair_device_type dev_type);
int amdair_register_aie_instance(struct amdair_device *air_dev);
void amdair_device_free_resources(struct amdair_device *air_dev);
struct amdair_device *amdair_device_get_by_id(uint32_t device_id);

#endif /* AMDAIR_DEVICE_H_ */
