// SPDX-License-Identifier: MIT
// Copyright (C) 2023, Advanced Micro Devices, Inc.

#include <linux/device.h>
#include <linux/pci.h>

#include "amdair_device.h"
#include "amdair_doorbell.h"
#include "amdair_ioctl.h"
#include "amdair_mem_manager.h"
#include "amdair_object.h"
#include "amdair_process.h"

int amdair_process_device_create_bo_handle(struct amdair_process_device *air_pd,
					   uint32_t type_flags, uint64_t base,
					   uint64_t size, int *handle)
{
	struct amdair_device *air_dev = air_pd->dev;
	struct amdair_buf_object *air_bo = NULL;
	int ret = 0;

	if (!handle)
		return -EINVAL;

	if (!air_dev)
		return -ENODEV;

	if (type_flags & AMDAIR_IOC_ALLOC_MEM_HEAP_TYPE_DRAM)
		ret = amdair_mman_alloc_dram(&air_dev->mman, &air_bo, size);

	if (ret || !air_bo)
		return ret;

	if (type_flags & AMDAIR_IOC_ALLOC_MEM_HEAP_TYPE_DRAM) {
		air_bo->heap_type = BO_HEAP_TYPE_DRAM;
		interval_tree_insert(&air_bo->it_node, &air_pd->dram_heap);
	}

	*handle = idr_alloc(&air_pd->alloc_idr, air_bo, 0, MAX_BUF_OBJ_ID + 1,
			    GFP_KERNEL);
	if (*handle < 0)
		return *handle;

	return 0;
}

int amdair_process_device_destroy_bo_handle(
	struct amdair_process_device *air_pd, int handle)
{
	struct amdair_buf_object *air_bo = NULL;

	if (handle < 0 || !air_pd)
		return -EINVAL;

	air_bo = amdair_process_device_find_bo(air_pd, handle);

	if (air_bo) {
		idr_remove(&air_pd->alloc_idr, handle);
		interval_tree_remove(&air_bo->it_node, &air_pd->dram_heap);
		amdair_mman_free_dram(&air_pd->dev->mman, air_bo);
	} else {
		return -EINVAL;
	}

	return 0;
}

struct amdair_buf_object *
amdair_process_device_find_bo(struct amdair_process_device *air_pd, int handle)
{
	if (handle < 0)
		return NULL;

	return idr_find(&air_pd->alloc_idr, handle);
}

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

int amdair_process_release_resources(struct amdair_process *air_process)
{
	int i = 0;

	for (i = 0; i < air_process->num_proc_devs; ++i) {
		amdair_doorbell_release(air_process->proc_devs[i].dev,
					air_process->proc_devs[i].db_page_id);
		idr_destroy(&air_process->proc_devs[i].alloc_idr);
	}

	return 0;
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

		idr_init(&air_process->proc_devs[i].alloc_idr);
	}

	return 0;

err_no_dev:
	return ret;
}

int amdair_process_get_process_device(struct amdair_process *air_process,
				      uint32_t dev_id,
				      struct amdair_process_device **air_pd)
{
	if (dev_id >= air_process->num_proc_devs)
		return -ENODEV;
	*air_pd = &air_process->proc_devs[dev_id];
	return 0;
}

int amdair_process_assign_doorbell(struct amdair_process *air_process,
				   uint32_t dev_id, uint32_t *db_id)
{
	struct amdair_process_device *air_pd = NULL;
	struct amdair_device *air_dev = NULL;
	int ret = 0;

	ret = amdair_process_get_process_device(air_process, dev_id, &air_pd);

	if (ret)
		goto err_no_dev;

	air_dev = air_pd->dev;

	if (air_pd->db_page_id == DOORBELL_INVALID_ID) {
		air_pd->db_page_id = amdair_doorbell_find_free(air_dev);
		if (air_pd->db_page_id == DOORBELL_INVALID_ID)
			return -ENOSPC;
		air_pd->num_dbs = DOORBELLS_PER_PROCESS;
	}

	dev_info(&air_dev->pdev->dev, "Assigning doorbell page %u",
		 air_pd->db_page_id);
	*db_id = find_first_zero_bit(air_pd->doorbell_id_map, air_pd->num_dbs);

	if (*db_id == air_pd->num_dbs) {
		ret = -ENOSPC;
		goto err_invalid_db_id;
	}
	set_bit(*db_id, air_pd->doorbell_id_map);

	return 0;

err_invalid_db_id:
	amdair_doorbell_release(air_dev, air_pd->db_page_id);
err_no_dev:
	return ret;
}

int amdair_process_doorbell_release(struct amdair_process *air_process,
				    uint32_t dev_id, uint32_t db_id)
{
	struct amdair_process_device *air_pd = NULL;
	int ret =
		amdair_process_get_process_device(air_process, dev_id, &air_pd);
	if (ret)
		return ret;
	clear_bit(db_id, air_pd->doorbell_id_map);
	return 0;
}
