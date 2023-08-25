// Copyright (C) 2023, Advanced Micro Devices, Inc.
// SPDX-License-Identifier: MIT

#ifndef AMDAIR_CHARDEV_H_
#define AMDAIR_CHARDEV_H_

#include "amdair_device.h"

int amdair_chardev_init(struct pci_dev *pdev);
void amdair_chardev_exit(void);
int create_aie_mem_sysfs(struct amdair_device *priv, uint32_t index);

const char *amdair_dev_name(void);

#endif /* AMDAIR_CHARDEV_H_ */
