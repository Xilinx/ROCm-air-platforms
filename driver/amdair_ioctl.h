// Copyright (C) 2023, Advanced Micro Devices, Inc.
// SPDX-License-Identifier: MIT

#ifndef AMDAIR_HEADER_H_
#define AMDAIR_HEADER_H_

#define AMDAIR_IOCTL_MAJOR_VERSION 1
#define AMDAIR_IOCTL_MINOR_VERSION 0

struct amdair_get_version_args {
	uint32_t major_version; /* from driver */
	uint32_t minor_version; /* from driver */
};

enum amdair_mem_range {
	AMDAIR_MEM_RANGE_LOCAL,
	AMDAIR_MEM_RANGE_BRAM,
	AMDAIR_MEM_RANGE_DRAM,
	AMDAIR_MEM_RANGE_AIE,
};

enum amdair_queue_type {
	AMDAIR_QUEUE_DEVICE, /* queue is in device memory */
};

struct amdair_destroy_object_args {
	uint64_t handle; /* in: identifer of object to be destroyed */
};

struct amdair_create_queue_args {
	uint64_t ring_base_address; /* in: virtual address of ring entries */
	uint64_t doorbell_offset; /* out */
	uint64_t handle; /* out: object identifer for mapping, unmapping, etc. */

	uint32_t ring_size_bytes; /* in: ring buffer size in bytes*/
	uint32_t device_id; /* in: which device/card consumes queue entries */
	uint32_t queue_type; /* in: see amdair_queue_type */
	uint32_t queue_id; /* out: globally unique queue id */
};

struct amdair_create_mr_args {
	uint64_t handle; /* out: object identifer for mapping, unmapping, etc. */
	uint32_t region; /* in: see amdair_mem_range */
	uint32_t device_id; /* in: which device/card owns this mem region */
	uint64_t start; /* in: start offset in bytes */
	uint64_t size; /* in: length of range in bytes */
};

#define AMDAIR_COMMAND_START 1
#define AMDAIR_COMMAND_END 4

#define AMDAIR_IOCTL_BASE 'Y'
#define AMDAIR_IO(nr) _IO(AMDAIR_IOCTL_BASE, nr)
#define AMDAIR_IOR(nr, type) _IOR(AMDAIR_IOCTL_BASE, nr, type)
#define AMDAIR_IOW(nr, type) _IOW(AMDAIR_IOCTL_BASE, nr, type)
#define AMDAIR_IOWR(nr, type) _IOWR(AMDAIR_IOCTL_BASE, nr, type)

#define AMDAIR_IOC_GET_VERSION AMDAIR_IOR(0x01, struct amdair_get_version_args)

#define AMDAIR_IOC_DESTROY_OBJECT                                              \
	AMDAIR_IOWR(0x02, struct amdair_destroy_object_args)

#define AMDAIR_IOC_CREATE_QUEUE                                                \
	AMDAIR_IOWR(0x03, struct amdair_create_queue_args)

#define AMDAIR_IOC_CREATE_MEM_REGION                                           \
	AMDAIR_IOWR(0x04, struct amdair_create_mr_args)

#endif /* AMDAIR_HEADER_H_ */
