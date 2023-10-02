// Copyright (C) 2023, Advanced Micro Devices, Inc.
// SPDX-License-Identifier: MIT

#include <linux/module.h>
#include <linux/pci.h>

#include "amdair_device.h"

#include "amdair_object.h"
#include "vck5000.h"

struct amdair_aie_info aie_info = { .num_aie_devs = 0 };

int amdair_device_init(struct amdair_device *air_dev,
		       enum amdair_device_type dev_type)
{
	int ret = 0;
	struct pci_dev *pdev = air_dev->pdev;

	air_dev->dev_type = dev_type;
	switch (air_dev->dev_type) {
	case AMDAIR_DEV_VCK5000:
		dev_info(&pdev->dev, "VCK5000 device found");
		vck5000_dev_init(air_dev);
		break;
	default:
		dev_err(&pdev->dev, "Unsupported or unrecognized device");
		ret = -ENODEV;
		goto err_dev;
	}

	air_dev->dram_base = pci_resource_start(pdev, DRAM_BAR_INDEX);
	air_dev->dram_size = pci_resource_len(pdev, DRAM_BAR_INDEX);
	dev_info(&pdev->dev, "DRAM BAR 0 0x%lx (0x%llx)",
		 (unsigned long)air_dev->dram_base, air_dev->dram_size);

	air_dev->bram_base = pci_resource_start(pdev, BRAM_BAR_INDEX);
	air_dev->bram_size = pci_resource_len(pdev, BRAM_BAR_INDEX);
	air_dev->bram_bar = pcim_iomap_table(pdev)[BRAM_BAR_INDEX];
	dev_info(&pdev->dev, "BRAM BAR 2 0x%lx (0x%llx)",
		 (unsigned long)air_dev->bram_bar, air_dev->bram_size);

	air_dev->dev_asic_funcs->init_queues(air_dev);
	air_dev->dev_asic_funcs->init_doorbells(air_dev);

	ret = amdair_mman_init(air_dev);
	if (ret)
		goto err_mem_mgr;

	ret = amdair_register_aie_instance(air_dev);
	if (ret)
		goto err_dev;

	return 0;

err_mem_mgr:
err_dev:
	return ret;
}

int amdair_register_aie_instance(struct amdair_device *air_dev)
{
	if (!air_dev || aie_info.num_aie_devs >= MAX_AIE_INSTANCE)
		return -EINVAL;
	dev_info(&air_dev->pdev->dev, "Adding AIE %d\n", aie_info.num_aie_devs);
	aie_info.aie_instance[aie_info.num_aie_devs] = air_dev;
	air_dev->device_id = aie_info.num_aie_devs;
	aie_info.num_aie_devs++;
	return 0;
}

void amdair_device_free_resources(struct amdair_device *air_dev)
{
	kobject_put(&air_dev->kobj_aie);
	amdair_mman_free_resources(&air_dev->mman);
}

struct amdair_device *amdair_device_get_by_id(uint32_t dev_id)
{
	if (dev_id >= aie_info.num_aie_devs || dev_id >= MAX_AIE_INSTANCE)
		return NULL;
	return aie_info.aie_instance[dev_id];
}
