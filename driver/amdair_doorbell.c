#include "amdair_doorbell.h"

#include "amdair_device.h"

uint32_t amdair_doorbell_find_free(struct amdair_device *air_dev)
{
	int db_page_id = find_first_zero_bit(air_dev->doorbell.page_id_map,
					     air_dev->doorbell.num_db_pages);
	if (db_page_id == air_dev->doorbell.num_db_pages)
		return DOORBELL_INVALID_ID;
	set_bit(db_page_id, air_dev->doorbell.page_id_map);
	return db_page_id;
}

void amdair_doorbell_release(struct amdair_device *air_dev, uint32_t db_page_id)
{
	clear_bit(db_page_id, air_dev->doorbell.page_id_map);
}
