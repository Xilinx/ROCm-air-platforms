#include <linux/pci.h>

#include "amdair_device.h"
#include "amdair_queue.h"
#include "vck5000_regs.h"


static int32_t vck5000_bram_reg_read32(struct amdair_device *air_dev,
				       int reg_off)
{
	return ioread32(air_dev->bram_bar + reg_off);
}

static void vck5000_init_queues(struct amdair_device *air_dev)
{
	int num_aql_queues = vck5000_bram_reg_read32(air_dev, NUM_QUEUES_REG);
	resource_size_t queue_off
		= vck5000_bram_reg_read32(air_dev, QUEUE_DESC_OFFSET_REG);
	resource_size_t queue_size
		= vck5000_bram_reg_read32(air_dev, QUEUE_DESC_SIZE_REG);
	resource_size_t queue_buf_off
		= vck5000_bram_reg_read32(air_dev, QUEUE_BUF_OFFSET_REG);
	resource_size_t queue_buf_size
		= vck5000_bram_reg_read32(air_dev, QUEUE_BUF_SIZE_REG);

	air_dev->queue_mgr.num_hw_queues = num_aql_queues;
	air_dev->queue_mgr.queue_base = air_dev->bram_base + queue_off;
	air_dev->queue_mgr.queue_size = queue_size;
	air_dev->queue_mgr.queue_buf_base = air_dev->bram_base + queue_buf_off;
	air_dev->queue_mgr.queue_buf_size = queue_buf_size;
	air_dev->queue_mgr.kernel_id = 0;
	bitmap_fill(air_dev->queue_mgr.hw_queue_map, MAX_HW_QUEUES);
	bitmap_clear(air_dev->queue_mgr.hw_queue_map, 0, num_aql_queues);

	/* Reserve kernel's queue up front. */
	set_bit(air_dev->queue_mgr.kernel_id, air_dev->queue_mgr.hw_queue_map);

	dev_info(&air_dev->pdev->dev, "VCK5000: Initializing queues, "
		 "queue map %lx, queue addr %llx, queue buf addr, %llx",
		 *air_dev->queue_mgr.hw_queue_map,
		 air_dev->queue_mgr.queue_base,
		 air_dev->queue_mgr.queue_buf_base);
	dev_info(&air_dev->pdev->dev, "VCK5000: queue off %llx, queue buf off "
		 "%llx, queue size %llx, queue buf size %llx", queue_off,
		 queue_buf_off, queue_size, queue_buf_size);
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
	air_dev->doorbell.kernel_id = 0;
	bitmap_fill(air_dev->doorbell.id_map, MAX_HW_DOORBELL_PAGES);
	bitmap_clear(air_dev->doorbell.id_map, 0,
		     air_dev->doorbell.num_db_pages);

	/* Reserve kernel queue's doorbell up front. */
	set_bit(air_dev->doorbell.kernel_id, air_dev->doorbell.id_map);

	dev_info(&air_dev->pdev->dev,
		 "VCK5000: Initializing doorbells, addr %llx, size %llx, "
		 "doorbell map %lx, num db pages %d", air_dev->doorbell.base,
		 air_dev->doorbell.size, *air_dev->doorbell.id_map,
		 air_dev->doorbell.num_db_pages);
}

static struct amdair_device_init_funcs vck5000_dev_init_funcs = {
	.init_queues = &vck5000_init_queues,
	.init_doorbells = &vck5000_init_doorbells
};

void vck5000_dev_init(struct amdair_device *air_dev)
{
	air_dev->dev_init_funcs = &vck5000_dev_init_funcs;
}
