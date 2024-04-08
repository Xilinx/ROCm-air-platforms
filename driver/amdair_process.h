// SPDX-License-Identifier: MIT
// Copyright (C) 2023, Advanced Micro Devices, Inc.

#ifndef AMDAIR_PROCESS_H_
#define AMDAIR_PROCESS_H_

#include <linux/idr.h>
#include <linux/interval_tree.h>

#include "amdair_device.h"

struct amdair_buf_object;

/**
 * One page of doorbells per processes. Each doorbell is 64b as only HSA large
 * mode will supported going forwarded.
 */
#define DOORBELLS_PER_PROCESS ((PAGE_SIZE) / sizeof(uint64_t))

/**
 * struct amdair_process_device - Per process and per device information.
 *
 * @dev: A device this process is using.
 *
 * @process: The process that owns process-specific information associated with
 *	     the device.
 *
 * @db_page_id: The process' index into the device's doorbell aperture.
 *              Specifies which doorbell page is allocated to the process.
 *
 * @num_dbs: Number of doorbells the process has allocated on the device.
 * 
 * @queue_id : Array containing all of the queue IDs on a particular device 
 *             opened by a process 
 *
 * @doorbell_id_map: Free list of doorbells. 0 means free 1 means in use or
 *                   unavailable.
 *
 * @dram_heap: On-chip DRAM heap allocations for this process. Maintained as a
 *             red-black tree.
 *
 * @alloc_idr: On-chip buffer object unique handle allocations.
 */
struct amdair_process_device {
	struct amdair_device *dev;
	struct amdair_process *process;

	uint32_t db_page_id;
	int num_dbs;

	uint32_t queue_id[MAX_HW_QUEUES];
	DECLARE_BITMAP(doorbell_id_map, DOORBELLS_PER_PROCESS);

	struct rb_root_cached dram_heap;
	struct idr alloc_idr;
};

int amdair_process_device_create_bo_handle(struct amdair_process_device *air_pd,
					   uint32_t type_flags, uint64_t base,
					   uint64_t size, int *handle);
int amdair_process_device_destroy_bo_handle(struct amdair_process_device *air_pd,
					    int handle);
struct amdair_buf_object* amdair_process_device_find_bo(
	struct amdair_process_device *air_pd, int handle);

/**
 * struct amdair_process - Per process information.
 *
 * @proc_dev: Structure that holds information that is device-specific for
 *	      this process.
 *
 * @num_proc_devs: Number of process device structs.
 */
struct amdair_process {
	struct amdair_process_device proc_devs[MAX_AIE_INSTANCE];
	int num_proc_devs;
};

int amdair_process_create(const struct task_struct *process,
			  struct amdair_process **air_process);
/**
 * amdair_process_release_resources - Release resources allocated to the
 *                                    process, such as its doorbell page. This
 *                                    is meant to be called before the process
 *                                    is destroyed.
 *
 * @air_process: Process whose resources are being freed.
 */
int amdair_process_release_resources(struct amdair_process *air_process);

int amdair_process_create_process_device(struct amdair_process *air_process);
int amdair_process_get_process_device(struct amdair_process *air_process,
				      uint32_t dev_id,
				      struct amdair_process_device **air_pd);


int amdair_process_assign_queue(struct amdair_process *air_process, 
           uint32_t dev_id, uint32_t *queue_id);
int amdair_process_assign_doorbell(struct amdair_process *air_process,
				   uint32_t dev_id, uint32_t *db_id);
int amdair_process_assign_queue(struct amdair_process *air_process,
				   uint32_t dev_id, uint32_t *queue_id);
int amdair_process_doorbell_release(struct amdair_process *air_process,
				    uint32_t dev_id, uint32_t db_id);
#endif /* AMDAIR_PROCESS_H_ */
