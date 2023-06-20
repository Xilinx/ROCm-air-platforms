// SPDX-License-Identifier: MIT
// Copyright (C) 2023, Advanced Micro Devices, Inc.

#ifndef AMDAIR_PROCESS_H_
#define AMDAIR_PROCESS_H_

#include "amdair_device.h"

/**
 * struct amdair_process_device: Per process and per device information.
 * @dev: A device this process is using.
 * @process: The process that owns process-specific information associated with
 *	     the device.
 * @doorbell_index: The process' index into the device's doorbell aperture.
 *                  Specifies which doorbell page is allocated to the process.
 */
struct amdair_process_device {
	struct amdair_device *dev;
	struct amdair_process *process;
	int doorbell_idx;
};

/**
 * struct amdair_process - Per process information.
 * @proc_dev: Structure that holds information that is device-specific for
 *	      this process.
 */
struct amdair_process {
	struct amdair_process_device proc_dev[MAX_AIE_INSTANCE];
	int num_proc_devs;
};

int amdair_process_create(const struct task_struct *process,
			  struct amdair_process *air_process);

int amdair_process_create_process_device(struct amdair_process *air_process);


#endif /* AMDAIR_PROCESS_H_ */
