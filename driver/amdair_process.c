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
	struct amdair_process_device *air_proc_dev = NULL;
	int i = 0;
	int ret = 0;

	if (!air_process)
		return -EINVAL;

	air_proc_dev = kzalloc(sizeof(struct amdair_process_device),
			       GFP_KERNEL);

	if (!air_proc_dev)
		return -ENOMEM;
	
	for (i = 0; i < aie_info.num_aies; ++i) {
		air_dev = aie_info.aie_instance[i];
		if (!air_dev) {
			ret = -ENODEV;
			goto err_no_dev;
		}
	}

	return 0;

err_no_dev:
	kfree(air_proc_dev);
	return ret;
}
