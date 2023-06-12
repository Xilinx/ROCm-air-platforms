// Copyright (C) 2023, Advanced Micro Devices, Inc.
// SPDX-License-Identifier: MIT

#include <stddef.h>
#include <linux/device.h>
#include <linux/export.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/compat.h>
#include <linux/time.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/ptrace.h>
#include <linux/dma-buf.h>
#include <linux/fdtable.h>
#include <linux/processor.h>
#include <linux/pci.h>
#include "chardev.h"
#include "amdair_ioctl.h"
#include "object.h"

/*
	Define an entry in the ioctl table
	The entry's index equals the ioctl number
*/
#define AMDAIR_IOCTL_DEF(ioctl, _func, _flags)                                 \
	[_IOC_NR(ioctl)] = { .cmd = ioctl,                                     \
			     .func = _func,                                    \
			     .flags = _flags,                                  \
			     .cmd_drv = 0,                                     \
			     .name = #ioctl }

/*
The object handle is a simple capability that the kernel grants to a user.
The handle is opaque to the user but is made up from deterministic parts:
[39:36] device id
[35:20] object id
[19:12] type
[11:0] empty (PAGE_SHIFT)
*/
#define HANDLE_TYPE_SHIFT 0
#define HANDLE_TYPE_MASK 0xFF
#define HANDLE_OBJECT_SHIFT 8
#define HANDLE_OBJECT_MASK 0xFFFF
#define HANDLE_DEVID_SHIFT 24
#define HANDLE_DEVID_MASK 0xFF

#define AMDAIR_CREATE_HANDLE(_dev_id, _obj_id, _type)                          \
	(((((uint64_t)_dev_id & HANDLE_DEVID_MASK) << HANDLE_DEVID_SHIFT) |    \
	  (((uint64_t)_obj_id & HANDLE_OBJECT_MASK) << HANDLE_OBJECT_SHIFT) |  \
	  ((uint64_t)_type & HANDLE_TYPE_MASK))                                \
	 << PAGE_SHIFT)
#define AMDAIR_HANDLE_TYPE(_handle)                                            \
	((_handle >> (HANDLE_TYPE_SHIFT + PAGE_SHIFT)) & HANDLE_TYPE_MASK)
#define AMDAIR_HANDLE_OBJ_ID(_handle)                                          \
	((_handle >> (HANDLE_OBJECT_SHIFT + PAGE_SHIFT)) & HANDLE_OBJECT_MASK)

/*
	Physical address of the BRAM
	This is only needed because the device's info page uses physical addresses
	Once that is fixed, this can be removed
*/
#define BRAM_PA 0x20100000000ULL

enum aie_address_validation {
	AIE_ADDR_OK,
	AIE_ADDR_ALIGNMENT,
	AIE_ADDR_RANGE,
};

typedef int amdair_ioctl_t(struct file *filep, void *data);

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

static int amdair_ioctl_get_version(struct file *filep, void *data);
static int amdair_ioctl_destroy_object(struct file *filep, void *data);
static int amdair_ioctl_create_queue(struct file *filep, void *data);
static int amdair_ioctl_create_mem_region(struct file *filep, void *data);

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
	AMDAIR_IOCTL_DEF(AMDAIR_IOC_DESTROY_OBJECT, amdair_ioctl_destroy_object,
			 0),
	AMDAIR_IOCTL_DEF(AMDAIR_IOC_CREATE_QUEUE, amdair_ioctl_create_queue, 0),
	AMDAIR_IOCTL_DEF(AMDAIR_IOC_CREATE_MEM_REGION,
			 amdair_ioctl_create_mem_region, 0),
};

static char *amdair_devnode(struct device *dev, umode_t *mode)
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

/*
	Allocate a queue in device memory

	This allocates some BRAM or DRAM and returns the base address to the caller
	Each controller can have only one queue at the moment. Find a controller
	that is free, and return its handle.

	@device_id ID of the card/device that will use this queue
	@owner ID of the process that created the queue
	@depth number of entries in the queue
*/
static struct amdair_object *alloc_device_queue(uint32_t device_id, pid_t owner,
						size_t ring_size_bytes)
{
	uint32_t ctrlr_idx;
	struct amdair_object *queue;
	struct amdair_device *dev = get_device_by_id(device_id);

	if (!dev) {
		printk("Can't find device id %u\n", device_id);
		return NULL;
	}

	ctrlr_idx = find_free_controller(dev);
	if (ctrlr_idx == get_controller_count(dev)) {
		printk("All controllers are busy\n");
		return NULL;
	}

	mark_controller_busy(dev, ctrlr_idx, owner);

	queue = kzalloc(sizeof(*queue), GFP_KERNEL);
	if (!queue) {
		printk("Error allocating queue object");
		mark_controller_free(dev, ctrlr_idx);
		return NULL;
	}

	/* queue id is global (i.e. across all controllers) and there is only one
		queue per controller at this time, so queue id == controller id
	*/
	queue->id = ctrlr_idx;
	queue->handle = AMDAIR_CREATE_HANDLE(device_id, ctrlr_idx,
					     AMDAIR_OBJECT_TYPE_QUEUE);
	queue->dev = dev;
	queue->owner = owner;
	queue->type = AMDAIR_OBJECT_TYPE_QUEUE;
	queue->range =
		AMDAIR_MEM_RANGE_BRAM; /* The base address is currently hard-coded to the BRAM range */
	queue->base = get_controller_base_address(dev, ctrlr_idx) - BRAM_PA;
	queue->size = ring_size_bytes;

	/* Add it to the list of managed objects */
	amdair_add_object(queue);

	printk("Allocated queue %u with handle 0x%llx\n", queue->id,
	       queue->handle);
	return queue;
}

static void free_device_queue(struct amdair_object *queue)
{
	mark_controller_free(queue->dev, queue->id);
	amdair_remove_object(queue->handle);
}

static long amdair_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
	uint32_t amdkfd_size;
	uint32_t usize, asize;
	char stack_kdata[128];
	char *kdata = NULL;
	amdair_ioctl_t *func;
	const struct amdair_ioctl_desc *ioctl = NULL;
	unsigned int nr = _IOC_NR(cmd);
	int ret;

	if ((nr < AMDAIR_COMMAND_START) || (nr > AMDAIR_COMMAND_END)) {
		dev_warn(amdair_chardev, "%s invalid %u", __func__, nr);
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
	ret = func(filep, kdata);

	/* copy any results back to userspace */
	if (cmd & IOC_OUT)
		if (copy_to_user((void __user *)arg, kdata, usize) != 0)
			ret = -EFAULT;

	if (kdata != stack_kdata)
		kfree(kdata);
	return ret;
}

static int amdair_open(struct inode *node, struct file *f)
{
	dev_warn(amdair_chardev, "%s", __func__);
	return 0;
}

/*
	Called when userspace closes the handle to the driver
	Release all queues and clean up
*/
static int amdair_release(struct inode *node, struct file *f)
{
	dev_warn(amdair_chardev, "%s", __func__);
	return 0;
}

/*
	The offset passed to mmap is the handle for a driver object.
	The driver must have previously created the object, such as a queue or
	signal, and returned the handle to the caller.
*/
static int amdair_mmap(struct file *f, struct vm_area_struct *vma)
{
	unsigned long pgoff = 0;
	unsigned long start, end;
	size_t size = vma->vm_end - vma->vm_start;
	uint64_t handle = (loff_t)vma->vm_pgoff << PAGE_SHIFT;
	struct pci_dev *pdev;

	struct amdair_object *obj = amdair_find_object_by_handle(handle);
	if (!obj) {
		printk("Can't find object with handle 0x%llx\n", handle);
		return -1;
	}

	if (obj->owner != current->pid) {
		printk("PID %u is trying to steal object 0x%llx!\n",
		       current->pid, obj->handle);
		return -1;
	}

	dev_warn(amdair_chardev,
		 "%s start 0x%lx end 0x%lx range %u offset 0x%llx", __func__,
		 vma->vm_start, vma->vm_end, obj->range, obj->base);

	pdev = obj->dev->pdev;

	switch (obj->range) {
	case AMDAIR_MEM_RANGE_AIE:
		if (!enable_aie) {
			dev_warn(amdair_chardev,
				 "mapping AIE BAR is not enabled");
			return -EOPNOTSUPP;
		}
		start = pci_resource_start(pdev, AIE_BAR_INDEX) + obj->base;
		end = pci_resource_end(pdev, AIE_BAR_INDEX);
		dev_warn(amdair_chardev, "mapping 0x%lx AIE at 0x%lx to 0x%lx",
			 size, start, vma->vm_start);
		break;

	case AMDAIR_MEM_RANGE_DRAM:
		start = pci_resource_start(pdev, DRAM_BAR_INDEX) + obj->base;
		end = pci_resource_end(pdev, DRAM_BAR_INDEX);
		dev_warn(amdair_chardev,
			 "mapping 0x%lx DRAM at 0x%lx to 0x%lx", size, start,
			 vma->vm_start);
		break;

	case AMDAIR_MEM_RANGE_BRAM:
		start = pci_resource_start(pdev, BRAM_BAR_INDEX) + obj->base;
		end = pci_resource_end(pdev, BRAM_BAR_INDEX);
		dev_warn(amdair_chardev,
			 "mapping 0x%lx BRAM at 0x%lx to 0x%lx", size, start,
			 vma->vm_start);
		break;

	default:
		dev_warn(amdair_chardev, "Unrecognized mmap range %u",
			 obj->range);
		return -EOPNOTSUPP;
	}

	if ((start + obj->size) >= end) {
		dev_err(amdair_chardev,
			"size 0x%lx starting at 0x%lx exceeds BAR", size,
			start);
		return -EINVAL;
	}

	pgoff = (start >> PAGE_SHIFT);

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	/* Remap-pfn-range will mark the range VM_IO */
	if (remap_pfn_range(vma, vma->vm_start, pgoff, size,
			    vma->vm_page_prot)) {
		return -EAGAIN;
	}

	return 0;
}

static int amdair_ioctl_get_version(struct file *filep, void *data)
{
	struct amdair_get_version_args *args = data;

	dev_warn(amdair_chardev, "%s %u.%u", __func__,
		 AMDAIR_IOCTL_MAJOR_VERSION, AMDAIR_IOCTL_MINOR_VERSION);
	args->major_version = AMDAIR_IOCTL_MAJOR_VERSION;
	args->minor_version = AMDAIR_IOCTL_MINOR_VERSION;

	return 0;
}

/*
	Destroy a previously created object

	TODO: Probably a good idea to quiesce and drain queues before destroying
*/
static int amdair_ioctl_destroy_object(struct file *filep, void *data)
{
	struct amdair_object *queue;
	struct amdair_destroy_object_args *args =
		(struct amdair_destroy_object_args *)data;

	dev_warn(amdair_chardev, "%s %llu from pid %u", __func__, args->handle,
		 current->pid);

	queue = amdair_find_object_by_handle(args->handle);
	if (!queue) {
		dev_warn(amdair_chardev,
			 "Could not find queue with handle %llu", args->handle);
		return -EINVAL;
	}

	free_device_queue(queue);

	return 0;
}

/*
	Allocate a queue of a specified device to the pid of the caller.
	It will create a new queue and return a handle that can be used
	to map the memory in a later call.
*/
static int amdair_ioctl_create_queue(struct file *filep, void *data)
{
	struct amdair_object *queue;
	struct amdair_create_queue_args *args =
		(struct amdair_create_queue_args *)data;

	dev_warn(amdair_chardev, "%s from pid %u requesting queue of size %d", __func__, current->pid,
		 args->ring_size_bytes);

	if (args->ring_size_bytes & (args->ring_size_bytes - 1)) {
		dev_warn(amdair_chardev, "Ring size %u is not a power of 2",
			 args->ring_size_bytes);
		return -EINVAL;
	}

	switch (args->queue_type) {
	case AMDAIR_QUEUE_DEVICE:
		queue = alloc_device_queue(args->device_id, current->pid,
					   args->ring_size_bytes);
		if (!queue) {
			dev_err(amdair_chardev,
				"Error allocating device queue type=%u devid=%u pid=%u",
				args->queue_type, args->device_id,
				current->pid);
			return -ENOMEM;
		}
		args->doorbell_offset = 0xbad2;
		args->ring_base_address = queue->base;
		args->queue_id = queue->id;
		args->handle = queue->handle;
		break;

	default:
		dev_err(amdair_chardev, "Queue type %u not supported",
			args->queue_type);
		return -EINVAL;
	}

	return 0;
}

static int amdair_ioctl_create_mem_region(struct file *filep, void *data)
{
	struct amdair_device *dev;
	struct amdair_object *obj;
	struct amdair_create_mr_args *args =
		(struct amdair_create_mr_args *)data;

	dev_warn(amdair_chardev, "%s from pid %u", __func__, current->pid);

	dev = get_device_by_id(args->device_id);
	if (!dev) {
		printk("Can't find device id %u\n", args->device_id);
		return -EINVAL;
	}

	if (args->region < AMDAIR_MEM_RANGE_BRAM ||
	    args->region > AMDAIR_MEM_RANGE_AIE) {
		printk("Invalid memory region %u\n", args->region);
		return -EINVAL;
	}

	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (!obj) {
		printk("Error allocating mem region object");
		return -ENOMEM;
	}

	obj->dev = dev;
	obj->owner = current->pid;
	obj->range = args->region;
	obj->base = args->start;
	obj->size = args->size;

	/*
		create a unique handle per device & region. Note: two processes can
		register the same region for the same device but only the first will
		be able to map it
	*/
	obj->handle = AMDAIR_CREATE_HANDLE(args->device_id, args->region,
					   AMDAIR_OBJECT_TYPE_MEM_REGION);

	/* Add it to the list of managed objects */
	amdair_add_object(obj);

	args->handle = obj->handle;

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
	if (offset >= dev->aie_bar_len) {
		printk("%s: invalid offset 0x%llx (max 0x%llx)\n", __func__,
		       offset, dev->aie_bar_len);
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
	uint32_t value;
	struct amdair_device *drv_priv =
		container_of(kobj, struct amdair_device, kobj_aie);
	uint64_t offset = drv_priv->mem_addr;

	if (validate_aie_address(offset, drv_priv)) {
		snprintf(buf, PAGE_SIZE, "0xffffffff\n");
		return strlen(buf) + 1;
	}

	value = ioread32(drv_priv->aie_bar + offset);

	snprintf(buf, PAGE_SIZE, "0x%x\n", value);
	return strlen(buf) + 1;
}

static ssize_t value_store(struct kobject *kobj, struct attribute *attr,
			   const char *buf, size_t count)
{
	uint32_t value;
	struct amdair_device *drv_priv =
		container_of(kobj, struct amdair_device, kobj_aie);
	uint64_t offset = drv_priv->mem_addr;

	if (validate_aie_address(offset, drv_priv) == AIE_ADDR_OK) {
		kstrtouint(buf, 0, &value);
		iowrite32(value, drv_priv->aie_bar + offset);
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
