// Copyright (C) 2023, Advanced Micro Devices, Inc.
// SPDX-License-Identifier: MIT

#include <linux/compat.h>
#include <linux/device.h>
#include <linux/dma-buf.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/fdtable.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/pci.h>
#include <linux/processor.h>
#include <linux/ptrace.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/stddef.h>
#include <linux/time.h>
#include <linux/uaccess.h>
#include <linux/version.h>

#include "amdair_admin_aql_queue.h"
#include "amdair_chardev.h"
#include "amdair_device.h"
#include "amdair_doorbell.h"
#include "amdair_ioctl.h"
#include "amdair_object.h"
#include "amdair_process.h"
#include "amdair_queue.h"

/**
 * Define an entry in the ioctl table. The entry's index equals the ioctl
 * number.
 */
#define AMDAIR_IOCTL_DEF(ioctl, _func, _flags)	\
	[_IOC_NR(ioctl)] = { .cmd = ioctl,	\
			     .func = _func,	\
			     .flags = _flags,	\
			     .cmd_drv = 0,	\
			     .name = #ioctl }

/**
 * Use upper bits of mmap offset to store AMDAIR driver-specific information.
 * BITS[63:62] - Encode mmap type.
 * BITS[61:54] - Encode Versal device ID.
 * BITS[53:38] - Encode the buffer object (BO) handle.
 * BITS[37:35] - Encode the queue ID.
 */
#define AMDAIR_MMAP_TYPE_SHIFT 62ULL
#define AMDAIR_MMAP_TYPE_MASK 0x3ULL
#define AMDAIR_MMAP_TYPE_DOORBELL 0x0ULL
#define AMDAIR_MMAP_TYPE_QUEUE 0x1ULL
#define AMDAIR_MMAP_TYPE_QUEUE_BUF 0x2ULL
#define AMDAIR_MMAP_TYPE_BO 0x3ULL
#define AMDAIR_MMAP_GET_TYPE(offset_) ((offset_ >> (AMDAIR_MMAP_TYPE_SHIFT)) \
	& AMDAIR_MMAP_TYPE_MASK)

#define AMDAIR_MMAP_DEV_ID_SHIFT 54ULL
#define AMDAIR_MMAP_DEV_ID_WIDTH 8ULL
#define AMDAIR_MMAP_DEV_ID_MASK ((1ULL << (AMDAIR_MMAP_DEV_ID_WIDTH)) - 1ULL)
#define AMDAIR_MMAP_GET_DEV_ID(offset_) \
	((offset_ >> (AMDAIR_MMAP_DEV_ID_SHIFT)) & (AMDAIR_MMAP_DEV_ID_MASK))

#define AMDAIR_MMAP_BO_HANDLE_SHIFT 38ULL
#define AMDAIR_MMAP_BO_HANDLE_WIDTH 16ULL
#define AMDAIR_MMAP_BO_HANDLE_MASK ((1ULL << (AMDAIR_MMAP_BO_HANDLE_WIDTH)) - 1ULL)
#define AMDAIR_MMAP_BO_GET_HANDLE(offset_) \
	((offset_ >> (AMDAIR_MMAP_BO_HANDLE_SHIFT)) \
		& (AMDAIR_MMAP_BO_HANDLE_MASK))

#define AMDAIR_MMAP_QUEUE_ID_SHIFT 35ULL
#define AMDAIR_MMAP_QUEUE_ID_WIDTH 3ULL
#define AMDAIR_MMAP_QUEUE_ID_MASK ((1ULL << (AMDAIR_MMAP_QUEUE_ID_WIDTH)) - 1ULL)
#define AMDAIR_MMAP_GET_QUEUE_ID(offset_) \
	((offset_ >> (AMDAIR_MMAP_QUEUE_ID_SHIFT)) \
		& (AMDAIR_MMAP_QUEUE_ID_MASK))

#define AMDAIR_MMAP_CREATE_OFFSET(type_, dev_id_, bo_handle_, queue_id_) \
	(((type_ & (AMDAIR_MMAP_TYPE_MASK)) << (AMDAIR_MMAP_TYPE_SHIFT)) \
	| ((dev_id_ & (AMDAIR_MMAP_DEV_ID_MASK)) \
		<< (AMDAIR_MMAP_DEV_ID_SHIFT)) \
	| ((bo_handle_ & (AMDAIR_MMAP_BO_HANDLE_MASK)) \
		<< (AMDAIR_MMAP_BO_HANDLE_SHIFT)) \
	| ((queue_id_ & (AMDAIR_MMAP_QUEUE_ID_MASK)) \
		<< (AMDAIR_MMAP_QUEUE_ID_SHIFT)))

enum aie_address_validation {
	AIE_ADDR_OK,
	AIE_ADDR_ALIGNMENT,
	AIE_ADDR_RANGE,
};

typedef int amdair_ioctl_t(struct file *filp, void *data,
			   struct amdair_process *air_process);

struct amdair_ioctl_desc {
	unsigned int cmd;
	int flags;
	amdair_ioctl_t *func;
	unsigned int cmd_drv;
	const char *name;
};

struct amdair_attribute {
	struct attribute attr;
	ssize_t (*show)(struct kobject *kobj, struct attribute *attr,
			char *buf);
	ssize_t (*store)(struct kobject *kobj, struct attribute *attr,
			 const char *buf, size_t count);
};

/* Some forward declarations */
static ssize_t aie_show(struct kobject *kobj, struct attribute *attr,
			char *buf);
static ssize_t aie_store(struct kobject *kobj, struct attribute *attr,
			 const char *buf, size_t count);
static ssize_t address_show(struct kobject *kobj, struct attribute *attr,
			    char *buf);
static ssize_t address_store(struct kobject *kobj, struct attribute *attr,
			     const char *buf, size_t count);
static ssize_t value_show(struct kobject *kobj, struct attribute *attr,
			  char *buf);
static ssize_t value_store(struct kobject *kobj, struct attribute *attr,
			   const char *buf, size_t count);

static long amdair_ioctl(struct file *, unsigned int, unsigned long);
static int amdair_open(struct inode *, struct file *);
static int amdair_release(struct inode *, struct file *);
static int amdair_mmap(struct file *, struct vm_area_struct *);

static int amdair_ioctl_get_version(struct file *filp, void *data,
				    struct amdair_process *air_process);
static int amdair_ioctl_create_queue(struct file *filp, void *data,
				     struct amdair_process *air_process);
static int amdair_ioctl_destroy_queue(struct file *filp, void *data,
				      struct amdair_process *air_process);
static int amdair_ioctl_alloc_device_memory(struct file *filp, void *data,
					    struct amdair_process *air_process);
static int amdair_ioctl_free_device_memory(struct file *filp, void *data,
					   struct amdair_process *air_process);

/* define sysfs attributes */
struct amdair_attribute aie_attr_address = __ATTR_RW(address);
struct amdair_attribute aie_attr_value = __ATTR_RW(value);

static const struct sysfs_ops aie_sysfs_ops = {
	.show = aie_show,
	.store = aie_store,
};

static struct kobj_type aie_sysfs_type = {
	.sysfs_ops = &aie_sysfs_ops,
};

struct attribute *aie_sysfs_attrs[] = { &aie_attr_address.attr,
					&aie_attr_value.attr, NULL };

ATTRIBUTE_GROUPS(aie_sysfs);

static const struct file_operations amdair_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = amdair_ioctl,
	.compat_ioctl = compat_ptr_ioctl,
	.open = amdair_open,
	.release = amdair_release,
	.mmap = amdair_mmap,
};

extern bool enable_aie;
static int chardev_major = -1;
static struct class *amdair_class;
static struct device *amdair_chardev;

static const struct amdair_ioctl_desc amdair_ioctl_table[] = {
	AMDAIR_IOCTL_DEF(AMDAIR_IOC_GET_VERSION, amdair_ioctl_get_version, 0),
	AMDAIR_IOCTL_DEF(AMDAIR_IOC_CREATE_QUEUE, amdair_ioctl_create_queue, 0),
	AMDAIR_IOCTL_DEF(AMDAIR_IOC_DESTROY_QUEUE,
			 amdair_ioctl_destroy_queue, 0),
	AMDAIR_IOCTL_DEF(AMDAIR_IOC_ALLOC_DEVICE_MEMORY,
			 amdair_ioctl_alloc_device_memory, 0),
	AMDAIR_IOCTL_DEF(AMDAIR_IOC_FREE_DEVICE_MEMORY,
			 amdair_ioctl_free_device_memory, 0),
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,2,0)
static char *amdair_devnode(const struct device *dev, umode_t *mode)
#else
static char *amdair_devnode(struct device *dev, umode_t *mode)
#endif
{
	if (mode)
		*mode = 0666;
	return NULL;
}

int amdair_chardev_init(struct pci_dev *pdev)
{
	int ret = 0;

	ret = register_chrdev(0, amdair_dev_name(), &amdair_fops);
	if (ret < 0)
		goto err_register;

	chardev_major = ret;

	amdair_class = class_create(THIS_MODULE, amdair_dev_name());
	ret = PTR_ERR(amdair_class);
	if (IS_ERR(amdair_class))
		goto err_class;

	amdair_class->devnode = amdair_devnode;
	amdair_chardev =
		device_create(amdair_class, &pdev->dev,
			      MKDEV(chardev_major, 0), pdev, "amdair");

	ret = PTR_ERR(amdair_chardev);
	if (IS_ERR(amdair_chardev))
		goto err_device;

	return 0;

err_device:
	class_destroy(amdair_class);

err_class:
	unregister_chrdev(chardev_major, amdair_dev_name());

err_register:
	return -ENODEV;
}

void amdair_chardev_exit(void)
{
	device_destroy(amdair_class, MKDEV(chardev_major, 0));
	class_destroy(amdair_class);
	unregister_chrdev(chardev_major, amdair_dev_name());

	amdair_chardev = NULL;
}

static long amdair_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct amdair_process *air_process = filp->private_data;
	uint32_t amdkfd_size;
	uint32_t usize, asize;
	char stack_kdata[128];
	char *kdata = NULL;
	amdair_ioctl_t *func;
	const struct amdair_ioctl_desc *ioctl = NULL;
	unsigned int nr = _IOC_NR(cmd);
	int ret;

	if (!air_process) {
		return -EINVAL;
	}

	if ((nr < AMDAIR_COMMAND_START) || (nr > AMDAIR_COMMAND_END)) {
		dev_err(amdair_chardev, "%s invalid %u", __func__, nr);
		return 0;
	}

	ioctl = &amdair_ioctl_table[nr];

	amdkfd_size = _IOC_SIZE(ioctl->cmd);
	usize = asize = _IOC_SIZE(cmd);
	if (amdkfd_size > asize)
		asize = amdkfd_size;

	cmd = ioctl->cmd;
	func = ioctl->func;
	if (cmd & (IOC_IN | IOC_OUT)) {
		if (asize <= sizeof(stack_kdata)) {
			kdata = stack_kdata;
		} else {
			kdata = kmalloc(asize, GFP_KERNEL);
			if (!kdata) {
				return -ENOMEM;
			}
		}
		if (asize > usize)
			memset(kdata + usize, 0, asize - usize);
	}

	if (cmd & IOC_IN) {
		if (copy_from_user(kdata, (void __user *)arg, usize) != 0) {
			if (kdata != stack_kdata)
				kfree(kdata);
			dev_warn(amdair_chardev, "Missing data in ioctl %u",
				 nr);
			return -EFAULT;
		}
	} else if (cmd & IOC_OUT) {
		memset(kdata, 0, usize);
	}
	ret = func(filp, kdata, air_process);

	/* copy any results back to userspace */
	if (cmd & IOC_OUT)
		if (copy_to_user((void __user *)arg, kdata, usize) != 0)
			ret = -EFAULT;

	if (kdata != stack_kdata)
		kfree(kdata);
	return ret;
}

static int amdair_open(struct inode *node, struct file *filp)
{
	struct amdair_process *air_process = NULL;
	int ret = 0;

	dev_info(amdair_chardev, "%s", __func__);

	ret = amdair_process_create(current, &air_process);

	if (ret)
		goto err_process_create;

	filp->private_data = air_process;

err_process_create:
	return ret;
}

/*
	Called when userspace closes the handle to the driver
	Release all queues and clean up
*/
static int amdair_release(struct inode *node, struct file *filp)
{
	struct amdair_process *air_process = filp->private_data;

	dev_info(amdair_chardev, "%s", __func__);

	if (air_process) {
		amdair_process_release_resources(air_process);
		kfree(air_process);
	}

	return 0;
}

/*
	The offset passed to mmap is the handle for a driver object.
	The driver must have previously created the object, such as a queue or
	signal, and returned the handle to the caller.
*/
static int amdair_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct amdair_process *air_process = filp->private_data;
	struct amdair_process_device *air_pd = NULL;
	struct amdair_buf_object *air_bo = NULL;
	size_t size = vma->vm_end - vma->vm_start;
	uint64_t start = 0;
	uint64_t end = 0;
	uint64_t mmap_offset = vma->vm_pgoff << PAGE_SHIFT;
	uint32_t dev_id = AMDAIR_MMAP_GET_DEV_ID(mmap_offset);
	uint32_t queue_id = AMDAIR_MMAP_GET_QUEUE_ID(mmap_offset);
	int handle = AMDAIR_MMAP_BO_GET_HANDLE(mmap_offset);
	int ret = 0;

	ret = amdair_process_get_process_device(air_process, dev_id,
						&air_pd);
	if (ret)
		return ret;

        dev_info(&air_pd->dev->pdev->dev, "%s: offset %llx", __func__,
	         mmap_offset);

	switch (AMDAIR_MMAP_GET_TYPE(mmap_offset)) {
	case AMDAIR_MMAP_TYPE_DOORBELL:
		start = air_pd->dev->doorbell.base
			+ air_pd->db_page_id * PAGE_SIZE;
		end = start + size;
		break;
	case AMDAIR_MMAP_TYPE_QUEUE:
		start = air_pd->dev->queue_mgr.queue_base
			+ queue_id * PAGE_SIZE;
		end = start + size;
		break;
	case AMDAIR_MMAP_TYPE_QUEUE_BUF:
		start = air_pd->dev->queue_mgr.queue_buf_base
			+ queue_id * PAGE_SIZE;
		end = start + size;
		break;
	case AMDAIR_MMAP_TYPE_BO:
		air_bo = amdair_process_device_find_bo(air_pd, handle);

		if (!air_bo) {
			dev_err(&air_pd->dev->pdev->dev,
			        "%s: No object with handle %d", __func__,
				handle);
			return -EINVAL;
		}

		dev_info(&air_pd->dev->pdev->dev,
			 "%s: Mapping BO with handle %d", __func__, handle);

		switch (air_bo->heap_type) {
		case BO_HEAP_TYPE_BRAM:
			dev_warn(&air_pd->dev->pdev->dev,
				 "%s: BRAM heap not currently supported",
				 __func__);
			return -EINVAL;
		case BO_HEAP_TYPE_DRAM:
			start = air_bo->it_node.start;
			end = start + size;
			break;
		default:
			dev_err(&air_pd->dev->pdev->dev,
				"%s: Invalid BO heap type", __func__);
			return -EINVAL;
		}
		break;
	default:
		dev_err(&air_pd->dev->pdev->dev,
			"%s: Unrecognized MMAP offset %llx", __func__,
			mmap_offset);
		return -EFAULT;
	}

	
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	/* io_remap_pfn_range will mark the range VM_IO. */
	ret = io_remap_pfn_range(vma, vma->vm_start, start >> PAGE_SHIFT, size,
				 vma->vm_page_prot);
	if (ret)
		return -EAGAIN;

	return 0;
}

static int amdair_ioctl_get_version(struct file *filp, void *data,
				    struct amdair_process *air_process)
{
	struct amdair_get_version_args *args = data;

	dev_warn(amdair_chardev, "%s %u.%u", __func__,
		 AMDAIR_IOCTL_MAJOR_VERSION, AMDAIR_IOCTL_MINOR_VERSION);
	args->major_version = AMDAIR_IOCTL_MAJOR_VERSION;
	args->minor_version = AMDAIR_IOCTL_MINOR_VERSION;

	return 0;
}

/**
 * amdair_ioctl_create_queue - Allocate a HW queue to a user-space process.
 *
 * @air_process: Process requesting a queue.
 *
 * Allocate a queue of a specified device to the calling process. It will
 * allocate a HW queue and return an offset that can be used to map the memory
 * in a later call.
 */
static int amdair_ioctl_create_queue(struct file *filp, void *data,
				     struct amdair_process *air_process)
{
	struct amdair_create_queue_args *args = data;
	struct amdair_process_device *air_pd = NULL;
	struct amdair_device *air_dev = NULL;
	uint32_t queue_id = 0;
	uint32_t db_id = 0;
	int q = 0;
	int ret = 0;

	if (!air_process)
		return -EINVAL;

	if (args->ring_size_bytes & (args->ring_size_bytes - 1)) {
		dev_err(amdair_chardev, "Ring size %u is not a power of 2",
			args->ring_size_bytes);
		return -EINVAL;
	}

	switch (args->queue_type) {
	/**
	 * For now the queue descriptors and buffers are stored in BRAM and
	 * the CP FW can only access them via physical addresses on the device.
	 * Because of this we tightly couple the HW queue with a dedicated
	 * queue buffer in BRAM and do not allow the user process to provide
	 * the queue buffer. In the future when it is possible to access shared
	 * virtual memory from the device we can support user-provided and
	 * and dynamically allocated queue buffers.
	 */
	case AMDAIR_QUEUE_DEVICE:
		air_dev = amdair_device_get_by_id(args->device_id);
		if (!air_dev || air_dev->device_id != args->device_id)
			return -ENODEV;

		queue_id = amdair_queue_find_free(air_dev);	
		if (queue_id == QUEUE_INVALID_ID)
			return -ENOSPC;
		
		amdair_process_get_process_device(air_process, args->device_id,
						&air_pd);

		// Finding first available slot in datastructure to track
		// queue IDs used by this process device
		for(q = 0; q < MAX_HW_QUEUES; q++) {
			if(air_pd->queue_id[q] == QUEUE_INVALID_ID) {
				air_pd->queue_id[q] = queue_id;
				break;
			}
		}

		ret = amdair_process_assign_doorbell(air_process,
						     args->device_id, &db_id);
		if (ret)
			goto err_invalid_db;

		args->queue_id = queue_id;
		args->doorbell_id = db_id;
		air_dev->dev_asic_funcs->set_device_heap(air_dev, queue_id,
							 args->dram_heap_vaddr);

		args->doorbell_offset
			= AMDAIR_MMAP_CREATE_OFFSET(AMDAIR_MMAP_TYPE_DOORBELL,
						    args->device_id, 0, 0);
		args->queue_offset
			= AMDAIR_MMAP_CREATE_OFFSET(AMDAIR_MMAP_TYPE_QUEUE,
						    args->device_id, 0,
						    queue_id);
		args->queue_buf_offset
			= AMDAIR_MMAP_CREATE_OFFSET(AMDAIR_MMAP_TYPE_QUEUE_BUF,
						    args->device_id, 0,
						    queue_id);

		dev_info(amdair_chardev, "doorbell offset %llx, "
			 "queue offset %llx, queue_id %x, db_id %x, dev id %d "
			 "DRAM heap CPU VA %llx",
			 args->doorbell_offset, args->queue_offset, queue_id,
			 db_id, args->device_id, args->dram_heap_vaddr);
		break;
	default:
		dev_err(amdair_chardev, "Queue type %u not supported",
			args->queue_type);
		return -EINVAL;
	}

	return 0;

err_invalid_db:
	return ret;
}

/**
 * amdair_ioctl_destroy_queue - Destroy a previously allocated HW queue.
 *
 * @air_process: Process destroying the queue.
 *
 * Destroy a queue of a specified device. It will release a HW queue so that it
 * can be reused by a future process.
 */
static int amdair_ioctl_destroy_queue(struct file *filp, void *data,
				      struct amdair_process *air_process)
{
	struct amdair_destroy_queue_args *args = data;
	struct amdair_device *air_dev
		= amdair_device_get_by_id(args->device_id);
	int ret = 0;

	if (!air_dev || air_dev->device_id != args->device_id)
		return -ENODEV;

	dev_info(&air_dev->pdev->dev, "Destroying queue ID %u", args->queue_id);
	ret = amdair_queue_release(air_process, args->queue_id);
	ret = amdair_process_doorbell_release(air_process, args->device_id,
					      args->doorbell_id);

	return ret;
}

static int amdair_ioctl_alloc_device_memory(struct file *filp, void *data,
					    struct amdair_process *air_process)
{
	struct amdair_alloc_device_memory_args *args = data;
	struct amdair_process_device *air_pd = NULL;
	resource_size_t base = 0;
	int handle = 0;
	int ret = 0;

	if (!(args->flags & AMDAIR_IOC_ALLOC_MEM_HEAP_TYPE_DRAM))
		return -EINVAL;

	ret = amdair_process_get_process_device(air_process, args->device_id,
						&air_pd);

	if (ret)
		return ret;
	
	base = air_pd->dev->dram_base;
	ret = amdair_process_device_create_bo_handle(air_pd, args->flags, base,
						     args->size, &handle);
	if (ret)
		return ret;
	
	args->handle = handle;
	args->mmap_offset = AMDAIR_MMAP_CREATE_OFFSET(AMDAIR_MMAP_TYPE_BO,
						      args->device_id, handle,
						      0);
	dev_info(&air_pd->dev->pdev->dev, "%s: Created buffer object, "
		 "handle %d, mmap offset %llx, size %llx", __func__, handle,
		 args->mmap_offset, args->size);

	return 0;
}

static int amdair_ioctl_free_device_memory(struct file *filp, void *data,
					   struct amdair_process *air_process)
{
	struct amdair_free_device_memory_args *args = data;
	struct amdair_process_device *air_pd = NULL;
	int ret = 0;

	if (args->handle < 0)
		return -EINVAL;

	ret = amdair_process_get_process_device(air_process, args->device_id,
						&air_pd);
	if (ret)
		return ret;

	amdair_process_device_destroy_bo_handle(air_pd, args->handle);

	return 0;
}

static int validate_aie_address(uint64_t offset, struct amdair_device *dev)
{
	/* alignment */
	if (offset & 0x3) {
		printk("%s: 0x%llx not aligned to 4 bytes\n", __func__, offset);
		return AIE_ADDR_ALIGNMENT;
	}

	/* range within the specified BAR */
	if (offset >= dev->dev_asic_funcs->get_aie_mem_range()) { 
		printk("%s: invalid offset 0x%llx (max 0x%llx)\n", __func__,
		       offset, dev->dev_asic_funcs->get_aie_mem_range());
		return AIE_ADDR_RANGE;
	}

	return AIE_ADDR_OK;
}

static ssize_t address_show(struct kobject *kobj, struct attribute *attr,
			    char *buf)
{
	struct amdair_device *drv_priv =
		container_of(kobj, struct amdair_device, kobj_aie);

	snprintf(buf, PAGE_SIZE, "0x%llx\n", drv_priv->mem_addr);
	return strlen(buf) + 1;
}

static ssize_t address_store(struct kobject *kobj, struct attribute *attr,
			     const char *buf, size_t count)
{
	unsigned long address;
	struct amdair_device *drv_priv =
		container_of(kobj, struct amdair_device, kobj_aie);

	kstrtoul(buf, 0, &address);
	drv_priv->mem_addr = address;
	return count;
}

static ssize_t value_show(struct kobject *kobj, struct attribute *attr,
			  char *buf)
{
	struct amdair_device *air_dev = container_of(kobj, struct amdair_device,
						     kobj_aie);
	uint64_t arg[4] = { air_dev->mem_addr, 0, 0, 0 };
	uint16_t pkt_func = AQL_AIR_PKT_TYPE_READ_AIE_REG32;

	if (validate_aie_address(arg[0], air_dev) != AIE_ADDR_OK) {
		snprintf(buf, PAGE_SIZE, "0xffffffff\n");
		return strlen(buf) + 1;
	}

	air_dev->dev_asic_funcs->send_admin_queue_cmd_and_wait(air_dev,
							       pkt_func, arg,
							       2);

	snprintf(buf, PAGE_SIZE, "0x%x\n", (uint32_t)(arg[2] & 0xffffffffULL));
	return strlen(buf) + 1;
}

static ssize_t value_store(struct kobject *kobj, struct attribute *attr,
			   const char *buf, size_t count)
{
	struct amdair_device *air_dev = container_of(kobj, struct amdair_device,
						     kobj_aie);
	uint64_t arg[4] = { air_dev->mem_addr, 0, 0, 0 };
	uint16_t pkt_func = AQL_AIR_PKT_TYPE_WRITE_AIE_REG32;

	if (validate_aie_address(arg[0], air_dev) == AIE_ADDR_OK) {
		kstrtouint(buf, 0, (uint32_t*)(&arg[1]));
		air_dev->dev_asic_funcs->send_admin_queue_cmd_and_wait(air_dev,
								       pkt_func,
								       arg, 2);
	}

	return count;
}

static ssize_t aie_show(struct kobject *kobj, struct attribute *attr, char *buf)
{
	struct amdair_attribute *air_attr =
		container_of(attr, struct amdair_attribute, attr);

	if (!air_attr->show) {
		printk("Missing show method for %s\r\n", attr->name);
		return 0;
	}

	return air_attr->show(kobj, attr, buf);
}

static ssize_t aie_store(struct kobject *kobj, struct attribute *attr,
			 const char *buf, size_t count)
{
	struct amdair_attribute *air_attr =
		container_of(attr, struct amdair_attribute, attr);

	if (!air_attr->store) {
		printk("Missing store method for %s\r\n", attr->name);
		return 0;
	}

	return air_attr->store(kobj, attr, buf, count);
}

int create_aie_mem_sysfs(struct amdair_device *priv, uint32_t index)
{
	int err;

	err = kobject_init_and_add(&priv->kobj_aie, &aie_sysfs_type,
				   &amdair_chardev->kobj, "%02u", index);
	if (err) {
		dev_err(amdair_chardev, "Error creating sysfs device");
		kobject_put(&priv->kobj_aie);
		return -1;
	}

	sysfs_create_groups(&priv->kobj_aie, aie_sysfs_groups);

	return 0;
}
