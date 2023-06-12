// Copyright (C) 2023, Advanced Micro Devices, Inc.
// SPDX-License-Identifier: MIT

#include <linux/init.h>
#include <linux/module.h>
#include <linux/pci.h>
#include "chardev.h"
#include "device.h"
#include "object.h"

static const char air_dev_name[] = "amdair";
static int dev_idx; /* linear index of managed devices */
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
	int bar_mask;
	int err;
	struct amdair_device *dev_priv;
	uint32_t idx;

	/* Enable device memory */
	err = pcim_enable_device(pdev);
	if (err) {
		dev_err(&pdev->dev, "Can't enable device memory");
		return err;
	}

	/* Allocate memory for the device private data */
	dev_priv = kzalloc(sizeof(struct amdair_device), GFP_KERNEL);
	if (!dev_priv) {
		dev_err(&pdev->dev, "Error allocating private data");
		pci_disable_device(pdev);
		return -ENOMEM;
	}

	/* Find all memory BARs. We are expecting 3 64-bit BARs */
	bar_mask = pci_select_bars(pdev, IORESOURCE_MEM);
	if (bar_mask != 0x15) {
		dev_err(&pdev->dev,
			"These are not the bars we're looking for: 0x%x",
			bar_mask);
		pci_disable_device(pdev);
		return -ENOMEM;
	}

	err = pcim_iomap_regions_request_all(pdev, bar_mask, air_dev_name);
	if (err) {
		dev_err(&pdev->dev, "Can't get memory region for bars");
		// pci_disable_device(pdev);
		return err;
	}
	dev_priv->dram_bar = pcim_iomap_table(pdev)[DRAM_BAR_INDEX];
	dev_priv->aie_bar = pcim_iomap_table(pdev)[AIE_BAR_INDEX];
	dev_priv->bram_bar = pcim_iomap_table(pdev)[BRAM_BAR_INDEX];
	dev_priv->dram_bar_len = pci_resource_len(pdev, DRAM_BAR_INDEX);
	dev_priv->aie_bar_len = pci_resource_len(pdev, AIE_BAR_INDEX);
	dev_priv->bram_bar_len = pci_resource_len(pdev, BRAM_BAR_INDEX);
	dev_warn(&pdev->dev, "bar 0: 0x%lx (0x%llx)",
		 (unsigned long)dev_priv->dram_bar, dev_priv->dram_bar_len);
	dev_warn(&pdev->dev, "bar 2: 0x%lx (0x%llx)",
		 (unsigned long)dev_priv->aie_bar, dev_priv->aie_bar_len);
	dev_warn(&pdev->dev, "bar 4: 0x%lx (0x%llx)",
		 (unsigned long)dev_priv->bram_bar, dev_priv->bram_bar_len);

	dev_priv->mem_addr = 0xbadbeef;
	dev_priv->queue_used = 0;
	dev_priv->pdev = pdev;
	dev_priv->controller_count = get_controller_count(dev_priv);

	/* Set driver private data */
	pci_set_drvdata(pdev, dev_priv);

	/* Request interrupt and set up handler */

	/* set up chardev interface */
	err = amdair_chardev_init(pdev);
	if (err) {
		dev_err(&pdev->dev, "Error creating char device");
		return -EINVAL;
	}

	/* Query number of herd controllers */
	if (dev_priv->controller_count > MAX_HERD_CONTROLLERS) {
		dev_err(&pdev->dev,
			"Number of controllers: %u exceeds maximum expected %u",
			dev_priv->controller_count, MAX_HERD_CONTROLLERS);
		return -EINVAL;
	}

	/* Each herd controller has a private memory region */
	for (idx = 0; idx < dev_priv->controller_count; idx++) {
		dev_warn(&pdev->dev, "Controller %u base address: 0x%llx", idx,
			 get_controller_base_address(dev_priv, idx));
	}

	/* take queue 0 for exclusive use by driver */
	mark_controller_busy(dev_priv, 0, 0);

	dev_priv->device_id = dev_idx;
	add_device(dev_priv);

	/* Create sysfs files for accessing AIE memory region */
	create_aie_mem_sysfs(dev_priv, dev_idx);
	dev_idx++;

	return 0;
}

/* Clean up */
static void amdair_pci_remove(struct pci_dev *pdev)
{
	struct amdair_device *dev_priv = pci_get_drvdata(pdev);

	amdair_chardev_exit();

	if (dev_priv) {
		list_del(&dev_priv->list);
		kobject_put(&dev_priv->kobj_aie);
		kfree(dev_priv);
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
