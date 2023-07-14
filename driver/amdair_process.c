// SPDX-License-Identifier: MIT
// Copyright (C) 2023, Advanced Micro Devices, Inc.

#include <linux/device.h>
#include <linux/pci.h>
#include <linux/sched.h>
#include <linux/slab.h>

#include "amdair_device.h"
#include "amdair_doorbell.h"
#include "amdair_process.h"

int amdair_process_create(const struct task_struct *process,
			  struct amdair_process **air_process)
{
	int ret = 0;

	*air_process = kzalloc(sizeof(struct amdair_process), GFP_KERNEL);
	if (!*air_process) {
		ret = -ENOMEM;
		goto err_alloc_process;
	}

	(*air_process)->num_proc_devs = 0;

	ret = amdair_process_create_process_device(*air_process);

	if (ret)
		goto err_alloc_process_device;

	return 0;

err_alloc_process_device:
	kfree(*air_process);
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
		air_process->proc_devs[i].dev = air_dev;
		air_process->proc_devs[i].process = air_process;
		air_process->proc_devs[i].db_page_id = DOORBELL_INVALID_ID;
		air_process->proc_devs[i].num_dbs = 0;
		bitmap_clear(air_process->proc_devs[i].doorbell_id_map, 0,
			     DOORBELLS_PER_PROCESS);
		air_process->num_proc_devs++;
	}

	return 0;

err_no_dev:
	return ret;
}

int amdair_process_get_process_device(struct amdair_process *air_process,
				      uint32_t dev_id,
				      struct amdair_process_device **air_proc_dev)
{
	if (dev_id >= air_process->num_proc_devs)
		return -ENODEV;
	*air_proc_dev = &air_process->proc_devs[dev_id];
	return 0;
}

int amdair_process_assign_doorbell(struct amdair_process *air_process,
				   uint32_t dev_id, uint32_t *db_id)
{
	struct amdair_process_device *air_proc_dev = NULL;
	struct amdair_device *air_dev = NULL;
	int ret = 0;

	ret = amdair_process_get_process_device(air_process, dev_id,
						&air_proc_dev);

	if (ret)
		goto err_no_dev;

	air_dev = air_proc_dev->dev;

	if (air_proc_dev->db_page_id == DOORBELL_INVALID_ID) {
		air_proc_dev->db_page_id = amdair_doorbell_find_free(air_dev);
		if (air_proc_dev->db_page_id == DOORBELL_INVALID_ID)
			return -ENOSPC;
		air_proc_dev->num_dbs = DOORBELLS_PER_PROCESS;
	}

	*db_id = find_first_zero_bit(air_proc_dev->doorbell_id_map,
				     air_proc_dev->num_dbs);

	if (*db_id == air_proc_dev->num_dbs) {
		ret = -ENOSPC;
		goto err_invalid_db_id;
	}

	return 0;

err_invalid_db_id:
	amdair_doorbell_release(air_dev, air_proc_dev->db_page_id);
err_no_dev:
	return ret;
}
