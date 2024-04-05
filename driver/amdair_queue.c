// Copyright (C) 2023, Advanced Micro Devices, Inc.
// SPDX-License-Identifier: MIT

#include "amdair_queue.h"
#include "amdair_device.h"
#include "amdair_process.h"

int amdair_queue_find_free(struct amdair_device *air_dev)
{
	int queue_id = find_first_zero_bit(air_dev->queue_mgr.hw_queue_map,
					   air_dev->queue_mgr.num_hw_queues);
	if (queue_id == air_dev->queue_mgr.num_hw_queues)
		return QUEUE_INVALID_ID;
	set_bit(queue_id, air_dev->queue_mgr.hw_queue_map);
	return queue_id;
}

int amdair_queue_release(struct amdair_process *air_process, uint32_t queue_id)
{
	int i = 0, q = 0;
	struct amdair_device *air_dev = NULL;

	// Searching for the amdair_process_device that is using this
	// queue so we can mark that it will be removed. While we are 
	// at it can get a pointer to the device as well
	for (i = 0; i < air_process->num_proc_devs; i++) {
		for (q = 0; q < MAX_HW_QUEUES; q++) {
			if(queue_id == air_process->proc_devs[i].queue_id[q]) {
				air_process->proc_devs[i].queue_id[q] = QUEUE_INVALID_ID;
				air_dev = air_process->proc_devs[i].dev;
				break;
			}
		}
	}

	if(air_dev == NULL) {
		return -EINVAL;
	}

	if (queue_id >= air_dev->queue_mgr.num_hw_queues) {
		return -EINVAL;
	}


	clear_bit(queue_id, air_dev->queue_mgr.hw_queue_map);

	return 0;
}
