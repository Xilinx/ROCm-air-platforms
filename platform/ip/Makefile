# Copyright(C) 2024-2025 Advanced Micro Devices, Inc. All Rights Reserved.
# SPDX-License-Identifier: MIT
#

include Makefile.common

blackparrot_build_ip:
	$(MAKE) -C $(BLACKPARROT_IP_DIR) build_ip
blackparrot_pack_ip:
	$(MAKE) -C $(BLACKPARROT_IP_DIR) pack_ip
blackparrot_clean_ip:
	$(MAKE) -C $(BLACKPARROT_IP_DIR) clean_ip

all_ip:
	$(MAKE) clean_ip
	$(MAKE) build_ip
	$(MAKE) pack_ip

build_ip: blackparrot_build_ip
pack_ip: blackparrot_pack_ip
clean_ip: blackparrot_clean_ip

