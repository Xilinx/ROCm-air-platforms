// SPDX-License-Identifier: MIT
// Copyright (C) 2023, Advanced Micro Devices, Inc.

#ifndef AMDAIR_PROCESS_H_
#define AMDAIR_PROCESS_H_

#include "amdair_device.h"

/**
 * One page of doorbells per processes. Each doorbell is 64b as only HSA large
 * mode will supported going forwarded.
 */
#define DOORBELLS_PER_PROCESS ((PAGE_SIZE) / sizeof(uint64_t))

/**
 * struct amdair_process_device: Per process and per device information.
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
 * @doorbell_id_map: Free list of doorbells. 0 means free 1 means in use or
 *                   unavailable.
 */
struct amdair_process_device {
	struct amdair_device *dev;
	struct amdair_process *process;
	uint32_t db_page_id;
	int num_dbs;
	DECLARE_BITMAP(doorbell_id_map, DOORBELLS_PER_PROCESS);
};

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

int amdair_process_create_process_device(struct amdair_process *air_process);
int amdair_process_get_process_device(struct amdair_process *air_process,
				      uint32_t dev_id,
				      struct amdair_process_device **air_proc_dev);

int amdair_process_assign_doorbell(struct amdair_process *air_process,
				   uint32_t dev_id, uint32_t *db_id);

#endif /* AMDAIR_PROCESS_H_ */
