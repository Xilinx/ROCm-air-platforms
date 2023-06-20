#ifndef AMDAIR_QUEUE_H_
#define AMDAIR_QUEUE_H_

#include <linux/bitmap.h>

#define MAX_HW_QUEUES		16
#define QUEUE_INVALID_ID	-1

struct amdair_device;

/**
 * struct amdair_queue_manager - Manages a device's HW queue resources.
 *
 * @queue_base: Base address of the device's queue descriptors.
 *
 * @queue_size: Size of queue descriptor aperture (page-aligned).
 *
 * @queue_buf_base: Base address of the device's queue buffer aperture, which
 *                  holds AQL packets.
 *
 * @queue_buf_size: Size of queue buffer aperture (page-aligned).
 *
 * @kernel_id: Index into the queue descriptor aperture reserved for the kernel.
 *
 * @num_hw_queues: Number of HW queues pages in the queue descriptor aperture.
 *
 * @hw_queue_map: Free list of queue descriptor pages. 0 means free 1 means
 *                used/unavailable.
 */
struct amdair_queue_manager {
	resource_size_t queue_base;
	resource_size_t queue_size;
	resource_size_t queue_buf_base;
	resource_size_t queue_buf_size;
	int kernel_id;
	int num_hw_queues;
	DECLARE_BITMAP(hw_queue_map, MAX_HW_QUEUES);
};

int amdair_queue_find_free(struct amdair_device *air_dev);

#endif /* AMDAIR_QUEUE_H_ */
