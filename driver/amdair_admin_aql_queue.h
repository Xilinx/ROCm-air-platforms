// SPDX-License-Identifier: MIT
// Copyright (C) 2023, Advanced Micro Devices, Inc.

#ifndef AMDAIR_ADMIN_AQL_QUEUE_H_
#define AMDAIR_ADMIN_AQL_QUEUE_H_

/**
 * AQL packet types and AIR function types.
 */
#define AQL_PKT_TYPE_AGENT			0x4
#define AQL_AIR_PKT_TYPE_READ_AIE_REG32		0x51
#define AQL_AIR_PKT_TYPE_WRITE_AIE_REG32	0x52

/**
 * AQL agent dispatch packet offsets. The admin queue only supports agent
 * dispatch packets.
 */
#define AQL_PKT_HEADER_OFFSET			0
#define AQL_PKT_TYPE_OFFSET			2
#define AQL_PKT_RETURN_ADDR_OFFSET		8
#define AQL_PKT_ARG_OFFSET			16
#define AQL_PKT_ARG0_OFFSET			(AQL_PKT_ARG_OFFSET)
#define AQL_PKT_ARG1_OFFSET			((AQL_PKT_ARG0_OFFSET) + 8)
#define AQL_PKT_ARG2_OFFSET			((AQL_PKT_ARG1_OFFSET) + 8)
#define AQL_PKT_ARG3_OFFSET			((AQL_PKT_ARG2_OFFSET) + 8)
#define AQL_PKT_COMPLETION_SIGNAL_OFFSET	56

#define AQL_PKT_SIZE				64

/**
 * Offsets for the admin queue descriptor.
 */
#define ADMIN_QUEUE_RD_ID_OFFSET		128
#define ADMIN_QUEUE_WR_ID_OFFSET		56

#endif /* AMDAIR_ADMIN_AQL_QUEUE_H_ */
