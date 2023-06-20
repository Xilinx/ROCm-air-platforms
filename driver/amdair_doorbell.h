#ifndef AMDAIR_DOORBELL_H_
#define AMDAIR_DOORBELL_H_

#include <linux/bitmap.h>

struct amdair_device;

#define MAX_HW_DOORBELL_PAGES	16
#define DOORBELL_INVALID_ID	-1

/**
 * struct amdair_doorbell - Holds information about a device's doorbell
 * aperture.
 *
 * @base: Base address of the doorbell aperture.
 *
 * @size: Size of the doorbell aperture (page-aligned).
 *
 * @kernel_id: Index into the doorbell aperture reserved for the kernel.
 *
 * @num_db_pages: Number of pages in the doorbell aperture. Currently there is
 *                one doorbell page per HW queue.
 *
 * @id_map: Free list of doorbell pages. 0 means free 1 means used/unavailable.
 */
struct amdair_doorbell {
	resource_size_t base;
	resource_size_t size;
	int kernel_id;
	int num_db_pages;
	DECLARE_BITMAP(id_map, MAX_HW_DOORBELL_PAGES);
};

/**
 * amdair_doorbell_find_free - Find a free doorbell page.
 *
 * @air_dev: Device on which to find a free doorbell page.
 */
int amdair_doorbell_find_free(struct amdair_device *air_dev);

#endif /* AMDAIR_DOORBELL_H_ */
