// Copyright (C) 2023, Advanced Micro Devices, Inc.
// SPDX-License-Identifier: MIT

#include <linux/init.h>
#include <linux/module.h>
#include <linux/pci.h>

#include "amdair_chardev.h"
#include "amdair_device.h"
#include "amdair_object.h"

static const char air_dev_name[] = "amdair";
bool enable_aie;

static int amdair_pci_probe(struct pci_dev *pdev, const struct pci_device_id *ent);
static void amdair_pci_remove(struct pci_dev *pdev);

static struct pci_device_id amdair_pci_id_table[] = {
	{ PCI_DEVICE(0x10EE, 0xB034) },
	{ 0, }
};

MODULE_DEVICE_TABLE(pci, amdair_pci_id_table);

/* Driver registration structure */
static struct pci_driver amdair_pci_driver = {
	.name = air_dev_name,
	.id_table = amdair_pci_id_table,
	.probe = amdair_pci_probe,
	.remove = amdair_pci_remove
};

/*
	Register the driver with the PCI subsystem
*/
static int __init amdair_init(void)
{
	if (enable_aie)
		printk("%s: AIE bar access enabled\n", air_dev_name);

	//init_device_list();

	return pci_register_driver(&amdair_pci_driver);
}

static void __exit amdair_exit(void)
{
	/* Unregister */
	pci_unregister_driver(&amdair_pci_driver);
}

static int amdair_pci_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	struct amdair_device *air_dev;
	int ret;
	int bar_mask;
	uint32_t idx;

	/* Allocate memory for the device private data */
	air_dev = kzalloc(sizeof(struct amdair_device), GFP_KERNEL);
	if (!air_dev) {
		dev_err(&pdev->dev, "Error allocating AIR device");
		ret = -ENOMEM;
	}

	/* Enable device memory */
	ret = pcim_enable_device(pdev);
	if (ret) {
		dev_err(&pdev->dev, "Can't enable device memory");
		goto err_free;
	}

	air_dev->pdev = pdev;

	/* Find all memory BARs. We are expecting 3 64-bit BARs */
	bar_mask = pci_select_bars(pdev, IORESOURCE_MEM);
	if (bar_mask != 0x15) {
		dev_err(&pdev->dev,
			"These are not the bars we're looking for: 0x%x",
			bar_mask);
		ret = -ENOMEM;
		goto err_pci;
	}

	ret = pcim_iomap_regions_request_all(pdev, bar_mask, air_dev_name);
	if (ret) {
		dev_err(&pdev->dev, "Can't get memory region for bars");
		goto err_pci;
	}

	amdair_device_init(air_dev);

	/* Set driver private data */
	pci_set_drvdata(pdev, air_dev);

	/* Set up chardev interface */
	ret = amdair_chardev_init(pdev);
	if (ret) {
		dev_err(&pdev->dev, "Error creating char device");
		ret = -EINVAL;
		goto err_pci;
	}

	/* Query number of herd controllers */
	if (air_dev->controller_count > MAX_HERD_CONTROLLERS) {
		dev_err(&pdev->dev,
			"Number of controllers: %u exceeds maximum expected %u",
			air_dev->controller_count, MAX_HERD_CONTROLLERS);
		ret = -EINVAL;
		goto err_pci;
	}

	/* Each herd controller has a private memory region */
	for (idx = 0; idx < air_dev->controller_count; idx++) {
		dev_info(&pdev->dev, "Controller %u base address: 0x%llx", idx,
			 get_controller_base_address(air_dev, idx));
	}

	/* Create sysfs files for accessing AIE memory region */
	create_aie_mem_sysfs(air_dev, air_dev->device_id);

	return 0;

err_pci:
	pci_disable_device(pdev);
err_free:
	kfree(air_dev);

	return ret;
}

/* Clean up */
static void amdair_pci_remove(struct pci_dev *pdev)
{
	struct amdair_device *air_dev = pci_get_drvdata(pdev);

	amdair_chardev_exit();

	if (air_dev) {
		kobject_put(&air_dev->kobj_aie);
		kfree(air_dev);
	}

	dev_warn(&pdev->dev, "removed");
}

const char *amdair_dev_name(void)
{
	return air_dev_name;
}

module_init(amdair_init);
module_exit(amdair_exit);

module_param(enable_aie, bool, 0644);
MODULE_PARM_DESC(enable_aie, "Enable debug access to AIE BAR");

MODULE_LICENSE("Dual MIT/GPL");
MODULE_AUTHOR("Joel Nider <joel.nider@amd.com>");
MODULE_DESCRIPTION("AMD AIR driver");
MODULE_VERSION("1.0");
