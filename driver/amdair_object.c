// Copyright (C) 2023, Advanced Micro Devices, Inc.
// SPDX-License-Identifier: MIT

#include <linux/list.h>
#include <linux/slab.h>

#include "amdair_object.h"

LIST_HEAD(object_list);

void amdair_add_object(struct amdair_object *obj)
{
	list_add_tail(&obj->list, &object_list);
}

struct amdair_object *amdair_find_object_by_handle(uint64_t handle)
{
	struct amdair_object *obj;
	list_for_each_entry (obj, &object_list, list) {
		if (obj->handle == handle)
			return obj;
	}

	return NULL;
}

/*
	Find an object by its handle, remove it from the list and free it
*/
void amdair_remove_object(uint64_t handle)
{
	struct amdair_object *obj, *tmp;
	list_for_each_entry_safe(obj, tmp, &object_list, list) {
		if (obj->handle == handle) {
			list_del(&obj->list);
			kfree(obj);
		}
	}
}

/*
	Remove all objects from the list and destroy them
*/
void amdair_destroy_object_list(void)
{
	struct amdair_object *obj, *tmp;
	list_for_each_entry_safe(obj, tmp, &object_list, list) {
		list_del(&obj->list);
		kfree(obj);
	}
}
