// Copyright (C) 2023, Advanced Micro Devices, Inc.
// SPDX-License-Identifier: MIT

#include <linux/io-64-nonatomic-hi-lo.h>
#include <linux/pci.h>

#include "amdair_admin_aql_queue.h"
#include "amdair_device.h"
#include "amdair_queue.h"
#include "vck5000.h"
#include "vck5000_regs.h"

static uint64_t vck5000_get_aie_mem_range(void) {
  return VCK5000_AIE_MEM_RANGE;
}

static uint32_t vck5000_bram_reg_read32(struct amdair_device *air_dev,
				       int reg_off)
{
	return ioread32(air_dev->bram_bar + reg_off);
}

static void vck5000_bram_reg_write64(struct amdair_device *air_dev, int reg_off,
				     uint64_t val)
{
	iowrite64(val, air_dev->bram_bar + reg_off);
}

static void vck5000_init_queues(struct amdair_device *air_dev)
{
	int num_aql_queues
		= vck5000_bram_reg_read32(air_dev, NUM_QUEUES_REG);
	int queue_num_entries
		= vck5000_bram_reg_read32(air_dev, QUEUE_NUM_ENTRIES_REG);
	resource_size_t queue_off
		= vck5000_bram_reg_read32(air_dev, QUEUE_DESC_OFFSET_REG);
	resource_size_t queue_size
		= vck5000_bram_reg_read32(air_dev, QUEUE_DESC_SIZE_REG);
	resource_size_t queue_buf_off
		= vck5000_bram_reg_read32(air_dev, QUEUE_BUF_OFFSET_REG);
	resource_size_t queue_buf_size
		= vck5000_bram_reg_read32(air_dev, QUEUE_BUF_SIZE_REG);

	air_dev->queue_mgr.num_hw_queues = num_aql_queues;
	air_dev->queue_mgr.queue_num_entries = queue_num_entries;
	air_dev->queue_mgr.queue_base = air_dev->bram_base + queue_off;
	air_dev->queue_mgr.queue_size = queue_size;
	air_dev->queue_mgr.queue_buf_base = air_dev->bram_base + queue_buf_off;
	air_dev->queue_mgr.queue_buf_size = queue_buf_size;
	air_dev->queue_mgr.kernel_id = 0;
	bitmap_fill(air_dev->queue_mgr.hw_queue_map, MAX_HW_QUEUES);
	bitmap_clear(air_dev->queue_mgr.hw_queue_map, 0, num_aql_queues);

	/* Reserve kernel's queue up front. */
	set_bit(air_dev->queue_mgr.kernel_id, air_dev->queue_mgr.hw_queue_map);
	air_dev->queue_mgr.admin_queue.descriptor
		= air_dev->bram_bar + queue_off;
	air_dev->queue_mgr.admin_queue.ring_buf
		= air_dev->bram_bar + queue_buf_off;

	dev_info(&air_dev->pdev->dev, "%s: Initializing queues, queue map %lx "
		 "queue size %llx, queue buf size %llx", __func__,
		 *air_dev->queue_mgr.hw_queue_map, queue_size, queue_buf_size);
}

static void vck5000_init_doorbells(struct amdair_device *air_dev)
{
	resource_size_t doorbell_off
		= vck5000_bram_reg_read32(air_dev, DOORBELL_OFFSET_REG);
	resource_size_t doorbell_size
		= vck5000_bram_reg_read32(air_dev, DOORBELL_SIZE_REG);

	air_dev->doorbell.num_db_pages = doorbell_size / PAGE_SIZE;
	air_dev->doorbell.base = air_dev->bram_base + doorbell_off;
	air_dev->doorbell.size = doorbell_size;
	air_dev->doorbell.kernel_page_id = 0;
	bitmap_fill(air_dev->doorbell.page_id_map, MAX_HW_DOORBELL_PAGES);
	bitmap_clear(air_dev->doorbell.page_id_map, 0,
		     air_dev->doorbell.num_db_pages);

	/* Reserve kernel queue's doorbell up front. */
	set_bit(air_dev->doorbell.kernel_page_id,
		air_dev->doorbell.page_id_map);
	air_dev->queue_mgr.admin_queue.db = air_dev->bram_bar + doorbell_off;

	dev_info(&air_dev->pdev->dev,
		 "%s: Initializing doorbells, size %llx, "
		 "doorbell map %lx, num db pages %d", __func__,
		 air_dev->doorbell.size, *air_dev->doorbell.page_id_map,
		 air_dev->doorbell.num_db_pages);
}

static void vck5000_set_device_heap(struct amdair_device *air_dev,
				    int queue_reg_off, uint64_t vaddr)
{
	vck5000_bram_reg_write64(air_dev,
				 DRAM_HEAP_VADDR_REG + queue_reg_off
				 * sizeof(uint64_t), vaddr);
}

static void vck5000_send_admin_queue_cmd_and_wait(struct amdair_device *air_dev,
						  uint16_t cmd_type,
						  uint64_t *arg, int num_args)
{
	uint64_t wr_idx = ioread64(air_dev->queue_mgr.admin_queue.descriptor
				   + ADMIN_QUEUE_WR_ID_OFFSET);
	int pkt_id = wr_idx % air_dev->queue_mgr.queue_num_entries;
	uint64_t completion_signal_val = 1;
	int i = 0;
 	uint64_t timeout_count = 0;

	/* Advance the write index to reserve space in the buffer. */
	iowrite64(wr_idx + 1, air_dev->queue_mgr.admin_queue.descriptor
		  + ADMIN_QUEUE_WR_ID_OFFSET);
	/* Populate the packet fields. */
	iowrite16(AQL_PKT_TYPE_AGENT, air_dev->queue_mgr.admin_queue.ring_buf
		  + pkt_id * AQL_PKT_SIZE + AQL_PKT_HEADER_OFFSET);
	iowrite16(cmd_type, air_dev->queue_mgr.admin_queue.ring_buf
		  + pkt_id * AQL_PKT_SIZE + AQL_PKT_TYPE_OFFSET);
	/* Set the arguments. */
	for (i = 0; i < num_args; ++i) {
		iowrite64(arg[i], air_dev->queue_mgr.admin_queue.ring_buf
			  + pkt_id * AQL_PKT_SIZE + AQL_PKT_ARG_OFFSET
			  + i * sizeof(uint64_t));
	}

	iowrite64(completion_signal_val, air_dev->queue_mgr.admin_queue.ring_buf
		  + pkt_id * AQL_PKT_SIZE + AQL_PKT_COMPLETION_SIGNAL_OFFSET);

	wmb();

	/* Ring the doorbell to let the CP know there is work in the queue. */
	iowrite64(wr_idx, air_dev->queue_mgr.admin_queue.db);

	/* Wait for the CP to mark the packet as completed. */
	completion_signal_val
		= ioread64(air_dev->queue_mgr.admin_queue.ring_buf
			   + pkt_id * AQL_PKT_SIZE
			   + AQL_PKT_COMPLETION_SIGNAL_OFFSET);

	while (completion_signal_val) {
		completion_signal_val
			= ioread64(air_dev->queue_mgr.admin_queue.ring_buf
				   + pkt_id * AQL_PKT_SIZE
			   	   + AQL_PKT_COMPLETION_SIGNAL_OFFSET);

    		if(timeout_count >= KERNEL_CMD_TIMEOUT_VAL) {
    			dev_warn(&air_dev->pdev->dev, "Timed out on kernel command. Firmware most likely needs to be reset.");
    			break;
    		}
    		timeout_count++;
	}

	if (cmd_type == AQL_AIR_PKT_TYPE_READ_AIE_REG32) {
		arg[num_args] = ioread32(air_dev->queue_mgr.admin_queue.ring_buf
					 + pkt_id * AQL_PKT_SIZE
					 + AQL_PKT_ARG_OFFSET + num_args
					 * sizeof(uint64_t));
	}
}

static struct amdair_device_asic_funcs vck5000_asic_funcs = {
	.init_queues = &vck5000_init_queues,
	.init_doorbells = &vck5000_init_doorbells,
	.set_device_heap = &vck5000_set_device_heap,
  .send_admin_queue_cmd_and_wait = &vck5000_send_admin_queue_cmd_and_wait,
  .get_aie_mem_range =  &vck5000_get_aie_mem_range
};

void vck5000_dev_init(struct amdair_device *air_dev)
{
	air_dev->dev_asic_funcs = &vck5000_asic_funcs;
}
