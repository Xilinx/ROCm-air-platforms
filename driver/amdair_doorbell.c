#include "amdair_doorbell.h"

#include "amdair_device.h"

int amdair_doorbell_find_free(struct amdair_device *air_dev)
{
	int db_id = find_first_zero_bit(air_dev->doorbell.id_map,
					air_dev->doorbell.num_db_pages);
	if (db_id == air_dev->doorbell.num_db_pages)
		return DOORBELL_INVALID_ID;
	set_bit(db_id, air_dev->doorbell.id_map);
	return db_id;
}
