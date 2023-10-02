#ifndef AMDAIR_DOORBELL_H_
#define AMDAIR_DOORBELL_H_

#include <linux/bitmap.h>

struct amdair_device;

#define MAX_HW_DOORBELL_PAGES 16
#define DOORBELL_INVALID_ID 0xFFFFFFFFU

/**
 * struct amdair_doorbell - Holds information about a device's doorbell
 * aperture.
 *
 * @base: Base address of the doorbell aperture.
 *
 * @size: Size of the doorbell aperture (page-aligned).
 *
 * @kernel_page_id: Index into the doorbell aperture reserved for the kernel.
 *
 * @num_db_pages: Number of pages in the doorbell aperture. Currently there is
 *                one doorbell page per HW queue.
 *
 * @page_id_map: Free list of doorbell pages. 0 means free 1 means in use or
 *               unavailable.
 */
struct amdair_doorbell {
	resource_size_t base;
	resource_size_t size;
	uint32_t kernel_page_id;
	int num_db_pages;
	DECLARE_BITMAP(page_id_map, MAX_HW_DOORBELL_PAGES);
};

/**
 * amdair_doorbell_find_free - Find a free doorbell page.
 *
 * @air_dev: Device on which to find a free doorbell page.
 */
uint32_t amdair_doorbell_find_free(struct amdair_device *air_dev);

/**
 * amdair_doorbell_release - Release doorbell page.
 *
 * @db_page_id: ID of doorbell page to release.
 */
void amdair_doorbell_release(struct amdair_device *air_dev,
			     uint32_t db_page_id);

#endif /* AMDAIR_DOORBELL_H_ */
