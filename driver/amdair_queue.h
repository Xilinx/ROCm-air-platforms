// SPDX-License-Identifier: MIT
// Copyright (C) 2023, Advanced Micro Devices, Inc.

#ifndef AMDAIR_QUEUE_H_
#define AMDAIR_QUEUE_H_

#include <linux/bitmap.h>

#define MAX_HW_QUEUES 16
#define QUEUE_INVALID_ID 0xFFFFFFFFU

struct amdair_device;

/**
 * struct amdair_admin_queue - Command queue managed by the kernel to send
 *                             privileged commands to the device's command
 *                             processor.
 *
 * @descriptor: Queue descriptor that holds the queue's read and write pointers.
 *
 * @ring_buf: Base address of the circular buffer that holds the command
 *            packets.
 *
 * @db: Queue's doorbell.
 */
struct amdair_admin_queue {
	void __iomem *descriptor;
	void __iomem *ring_buf;
	void __iomem *db;
};

/**
 * struct amdair_queue_manager - Manages a device's HW queue resources.
 *
 * @admin_queue: Administrator queue used by the kernel driver to send
 *               privileged commands to the device's command processor.
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
 * @queue_num_entries: Size of the queue in terms of the number of entries.
 *
 * @hw_queue_map: Free list of queue descriptor pages. 0 means free 1 means
 *                used/unavailable.
 */
struct amdair_queue_manager {
	struct amdair_admin_queue admin_queue;
	resource_size_t queue_base;
	resource_size_t queue_size;
	resource_size_t queue_buf_base;
	resource_size_t queue_buf_size;
	uint32_t kernel_id;
	int num_hw_queues;
	int queue_num_entries;
	DECLARE_BITMAP(hw_queue_map, MAX_HW_QUEUES);
};

int amdair_queue_find_free(struct amdair_device *air_dev);
int amdair_queue_release(struct amdair_device *air_dev, uint32_t queue_id);

#endif /* AMDAIR_QUEUE_H_ */
