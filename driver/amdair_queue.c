#include "amdair_queue.h"
#include "amdair_device.h"

int amdair_queue_find_free(struct amdair_device *air_dev)
{
	int queue_id = find_first_zero_bit(air_dev->queue_mgr.hw_queue_map,
					   air_dev->queue_mgr.num_hw_queues);
	if (queue_id == air_dev->queue_mgr.num_hw_queues)
		return QUEUE_INVALID_ID;
	set_bit(queue_id, air_dev->queue_mgr.hw_queue_map);
	return queue_id;
}

int amdair_queue_release(struct amdair_device *air_dev, uint32_t queue_id)
{
	if (queue_id >= air_dev->queue_mgr.num_hw_queues) {
		return -EINVAL;
	}

	clear_bit(queue_id, air_dev->queue_mgr.hw_queue_map);

	return 0;
}
