// SPDX-License-Identifier: MIT
// Copyright (C) 2023, Advanced Micro Devices, Inc.

#include <linux/device.h>
#include <linux/pci.h>
#include <linux/sched.h>
#include <linux/slab.h>

#include "amdair_device.h"
#include "amdair_process.h"

int amdair_process_create(const struct task_struct *process,
			  struct amdair_process *air_process)
{
	int ret = 0;

	air_process = kzalloc(sizeof(struct amdair_process), GFP_KERNEL);
	if (!air_process) {
		ret = -ENOMEM;
		goto err_alloc_process;
	}

	air_process->num_proc_devs = 0;

	ret = amdair_process_create_process_device(air_process);

	if (ret)
		goto err_alloc_process_device;

	return 0;

err_alloc_process_device:
	kfree(air_process);
err_alloc_process:
	return ret;
}

int amdair_process_create_process_device(struct amdair_process *air_process)
{
	struct amdair_device *air_dev = NULL;
	int i = 0;
	int ret = 0;

	if (!air_process)
		return -EINVAL;

	for (i = 0; i < aie_info.num_aie_devs; ++i) {
		air_dev = aie_info.aie_instance[i];
		if (!air_dev) {
			ret = -ENODEV;
			goto err_no_dev;
		}
		air_process->proc_dev[i].dev = air_dev;
		air_process->proc_dev[i].doorbell_idx = -1;
		air_process->num_proc_devs++;
	}

	return 0;

err_no_dev:
	return ret;
}
