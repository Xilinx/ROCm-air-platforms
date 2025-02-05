//===- bp.cpp ---------------------------------------------------*- C++ -*-===//
//
// Copyright (C) 2020-2022, Xilinx Inc.
// Copyright (C) 2022-2025, Advanced Micro Devices, Inc.
// SPDX-License-Identifier: MIT
//
//===----------------------------------------------------------------------===//

// Includes

#include "unistd.h"
#include <cstdint>
#include <cstring>
#include <limits>

// Includes

#include "airbin.h"
#include "amd_hsa.h"
#include "bp_pl.h"
#include "bp_utils.h"
#include "debug.h"
#include "hsa_csr.h"
#include "hsa_ext_air.h"
#include "memory.h"
#include "xparameters.h"

extern "C" {
#include "xmutex.h"
}

// Defines and Globals
#define AIE_BASE 0x020000000000ULL
#define AIE_CSR_SIZE 0x000100000000ULL

#define HIGH_ADDR(addr) ((addr & 0xffffffff00000000ULL) >> 32)
#define LOW_ADDR(addr) (addr & 0x00000000ffffffffULL)

#define ALIGN(_x, _size) (((_x) + (_size-1)) & ~(_size-1))

// there are 4 channels per DMA, 2 in each direction
#define NUM_DMA_CH 4U

// shimDMA directions and channels
// directions
#define SHIM_DMA_S2MM 0U
#define SHIM_DMA_MM2S 1U
// channels, there are 4 total with 2 in each direction
#define SHIM_DMA_CHNUM(_dir, _chan) (((_dir) * 2U) + (_chan))
#define SHIM_DMA_CHNUM_S2MM0 (SHIM_DMA_CHNUM((SHIM_DMA_S2MM),(0U)))
#define SHIM_DMA_CHNUM_S2MM1 (SHIM_DMA_CHNUM((SHIM_DMA_S2MM),(1U)))
#define SHIM_DMA_CHNUM_MM2S0 (SHIM_DMA_CHNUM((SHIM_DMA_MM2S),(0U)))
#define SHIM_DMA_CHNUM_MM2S1 (SHIM_DMA_CHNUM((SHIM_DMA_MM2S),(1U)))
#define SHIM_DMA_NUM_BDS 16U
// statically allocate BD per channel
#define SHIM_DMA_NUM_CH_BDS ((SHIM_DMA_NUM_BDS)/(NUM_DMA_CH))
// there are 4 BDs per DMA channel, allocated statically
#define NUM_DMA_CH_BDS 4U
// base BD given a channel and direction
// S2MM0 = 0, S2MM1 = 4, MM2S0 = 8, MM2S1 = 12
#define SHIM_DMA_BASE_BD(_dir, _chan) (((_dir) * 8U) + ((_chan) * NUM_DMA_CH_BDS))

// DMAs in the array
#define NUM_SHIM_DMAS 16U
// columns with NOC tiles in AM015 not listed here are not connected
// in the VCK5000 device
uint8_t shim_dma_cols[NUM_SHIM_DMAS] = {2,  3,  6,  7,  10, 11, 18, 19,
                                        26, 27, 34, 35, 42, 43, 46, 47};
// Timeout handling
#define INVLD_COUNT_TIMEOUT 100

/*
 * Tile address format:
 * --------------------------------------------
 * |                7 bits  5 bits   18 bits  |
 * --------------------------------------------
 * | Array offset | Column | Row | Tile addr  |
 * --------------------------------------------
 */
#define AIE_TILE_WIDTH 18
#define AIE_ROW_WIDTH 5
#define AIE_COLUMN_WIDTH 7

#define AIE_ROW_SHIFT (AIE_TILE_WIDTH)
#define AIE_COLUMN_SHIFT (AIE_TILE_WIDTH + AIE_ROW_WIDTH)
#define AIE_ARRAY_SHIFT (AIE_TILE_WIDTH + AIE_ROW_WIDTH + AIE_COLUMN_WIDTH)
#define AIE_TILE_MASK ((1 << AIE_TILE_WIDTH) - 1)
#define AIE_ROW_MASK ((1 << AIE_ROW_WIDTH) - 1)
#define AIE_COLUMN_MASK ((1 << AIE_COLUMN_WIDTH) - 1)

#define GET_COLUMN(_addr) (((_addr) >> AIE_COLUMN_SHIFT) & AIE_COLUMN_MASK)
#define GET_ROW(_addr) (((_addr) >> AIE_ROW_SHIFT) & AIE_ROW_MASK)
#define GET_TILE(_addr) ((_addr)&AIE_TILE_MASK)

// AIE (ME) registers
#define REG_AIE_DMA_BD_ADDR_A(_idx) (0x1D000 + (0x20 * _idx))
#define REG_AIE_DMA_BD_ADDR_B(_idx) (0x1D004 + (0x20 * _idx))
#define AIE_DMA_BD_ADDR_LOCK (0xFUL << 22)
#define AIE_DMA_BD_ADDR_ENA_REL (1UL << 21)
#define AIE_DMA_BD_ADDR_REL_VAL (1UL << 20)
#define AIE_DMA_BD_ADDR_USE_REL_VAL (1UL << 19)
#define AIE_DMA_BD_ADDR_ENA_ACQ (1UL << 18)
#define AIE_DMA_BD_ADDR_ACQ_VAL (1UL << 17)
#define AIE_DMA_BD_ADDR_USE_ACQ_VAL (1UL << 16)
#define AIE_DMA_BD_ADDR_BASE (0x1FFFUL << 0)

#define REG_AIE_DMA_BD_2D_X(_idx) (0x1D008 + (0x20 * _idx))
#define REG_AIE_DMA_BD_2D_Y(_idx) (0x1D00C + (0x20 * _idx))
#define REG_AIE_DMA_BD_PKT(_idx) (0x1D010 + (0x20 * _idx))
#define AIE_DMA_BD_PKT_TYPE (0x3UL << 12)
#define AIE_DMA_BD_PKT_ID (0x1FUL << 0)

#define REG_AIE_DMA_BD_IS(_idx) (0x1D014 + (0x20 * _idx))
#define REG_AIE_DMA_BD_CTL(_idx) (0x1D018 + (0x20 * _idx))
#define AIE_DMA_BD_CTL_VALID (1UL << 31)
#define AIE_DMA_BD_CTL_ENA_AB (1UL << 30)
#define AIE_DMA_BD_CTL_ENA_FIFO (3UL << 28)
#define AIE_DMA_BD_CTL_ENA_PKT (1UL << 27)
#define AIE_DMA_BD_CTL_ENA_ILV (1UL << 26)
#define AIE_DMA_BD_CTL_ILV_CNT (0xFFUL << 18)
#define AIE_DMA_BD_CTL_USE_NEXT (1UL << 17)
#define AIE_DMA_BD_CTL_NEXT (0xFUL << 13)
#define AIE_DMA_BD_CTL_LEN (0x1FFFUL << 0)

#define REG_AIE_LOCK_RELEASE_0(_idx) (0x1E020 + (0x80 * _idx))
#define REG_AIE_CORE_CTL 0x00032000
#define REG_AIE_CORE_STATUS 0x00032004

// NoC (shim) registers
#define REG_SHIM_DMA_BD_ADDR(_idx) (0x1D000 + (0x14 * _idx))
#define REG_SHIM_DMA_BD_BUF_LEN(_idx) (0x1D004 + (0x14 * _idx))
#define REG_SHIM_DMA_BD_CTRL(_idx) (0x1D008 + (0x14 * _idx))
#define SHIM_DMA_BD_CTRL_VALID (1 << 0)
#define REG_SHIM_DMA_BD_AXI_CFG(_idx) (0x1D00C + (0x14 * _idx))
#define REG_SHIM_DMA_BD_PKT(_idx) (0x1D010 + (0x14 * _idx))

#define REG_SHIM_DMA_S2MM_CTRL(_chan) (0x1D140 + (0x8 * _chan))
#define REG_SHIM_DMA_S2MM_START_QUEUE(_chan) (0x1D144 + (0x8 * _chan))
#define REG_SHIM_DMA_MMS2_CTRL(_chan) (0x1D150 + (0x8 * _chan))
#define REG_SHIM_DMA_MMS2_START_QUEUE(_chan) (0x1D154 + (0x8 * _chan))

#define REG_SHIM_DMA_S2MM_STATUS (0x1D160)
#define REG_SHIM_DMA_MM2S_STATUS (0x1D164)

#define SHIM_DMA_QUEUE_OVERFLOW_1_SHIFT 29
#define SHIM_DMA_QUEUE_OVERFLOW_0_SHIFT 28
#define SHIM_DMA_QUEUE_OVERFLOW_WIDTH 1
#define SHIM_DMA_QUEUE_OVERFLOW_MASK 1U
#define SHIM_DMA_CURR_BD_1_SHIFT 20
#define SHIM_DMA_CURR_BD_0_SHIFT 16
#define SHIM_DMA_CURR_BD_WIDTH 4
#define SHIM_DMA_CURR_BD_MASK ((1 << SHIM_DMA_CURR_BD_WIDTH) - 1)
#define SHIM_DMA_QUEUE_SIZE_1_SHIFT 9
#define SHIM_DMA_QUEUE_SIZE_0_SHIFT 6
#define SHIM_DMA_QUEUE_SIZE_WIDTH 3
#define SHIM_DMA_QUEUE_SIZE_MASK ((1 << SHIM_DMA_QUEUE_SIZE_WIDTH) - 1)
#define SHIM_DMA_STALLED_1_SHIFT 5
#define SHIM_DMA_STALLED_0_SHIFT 4
#define SHIM_DMA_STALLED_WIDTH 1
#define SHIM_DMA_STALLED_MASK 1U
#define SHIM_DMA_STATUS_1_SHIFT 2
#define SHIM_DMA_STATUS_0_SHIFT 0
#define SHIM_DMA_STATUS_WIDTH 2
#define SHIM_DMA_STATUS_MASK ((1 << SHIM_DMA_STATUS_WIDTH) - 1)

#define REG_AIE_COL_RESET 0x00036048
#define REG_SHIM_RESET_ENA 0x0003604C

#define REG_AIE_CORE_CTL_RESET (1U << 1)
#define REG_AIE_CORE_CTL_ENABLE (1U << 0)

struct HerdConfig {
  uint32_t row_start;
  uint32_t num_rows;
  uint32_t col_start;
  uint32_t num_cols;
};

struct HerdConfig HerdCfgInst;

XMutex xmutex;
XMutex* xmutex_ptr = &xmutex;
XMutex_Config* xmutex_cfg;

// nd memcpy management data structures
typedef struct staged_nd_memcpy_s {
  uint32_t valid;
  hsa_agent_dispatch_packet_t *pkt;
  uint64_t paddr[3];
  uint32_t index[3];
} staged_nd_memcpy_t; // about 48B therefore @ 64 slots ~3kB

// GLOBAL storage for 'in progress' ND memcpy work
// Note: this is only visible to a single CP as the CPs do not share
//       globally defined variables in the firmware code
// one slot per DMA for each (direction, channel) tuple
// four slots per shim DMA
staged_nd_memcpy_t staged_nd_slot[NUM_SHIM_DMAS * NUM_DMA_CH];

//// GLOBAL for round-robin bd allocations
// next_bd tracks the BD ID in range [0,3] for every channel of every shim DMA
// this BD ID must be offset by the base slot to get the real BD to be used for
// the given channel
uint32_t next_bd[NUM_SHIM_DMAS * NUM_DMA_CH] = {0};

// Functions

inline uint64_t mymod(uint64_t a) {
  uint64_t result = a;
  while (result >= AQL_QUEUE_NUM_ENTRIES) {
    result -= AQL_QUEUE_NUM_ENTRIES;
  }
  return result;
}

bool packet_get_active(hsa_agent_dispatch_packet_t *pkt) {
  return pkt->reserved2 & 0x1;
}

void packet_set_active(hsa_agent_dispatch_packet_t *pkt, bool b) {
  pkt->reserved2 = (pkt->reserved2 & ~0x1) | b;
}

void lock_uart(uint32_t id) {
  XMutex_Lock(xmutex_ptr, XPAR_MUTEX_0_UART_LOCK, id);
}

void unlock_uart(uint32_t id) {
  XMutex_Unlock(xmutex_ptr, XPAR_MUTEX_0_UART_LOCK, id);
}

/*
  read 32 bit value from specified address
*/
static inline uint32_t in32(uint64_t Addr) {
  return *((volatile uint32_t*)(Addr));
}

/*
  write 32 bit value to specified address
*/
static inline void out32(uint64_t Addr, uint32_t Value) {
  *((volatile uint32_t*)(Addr)) = Value;
}

/*
 * Calculate the address of an AIE tile
 */
inline uint64_t getTileAddr(uint16_t ColIdx, uint16_t RowIdx) {
  uint64_t my_ta = (uint64_t)(AIE_BASE | (ColIdx << AIE_COLUMN_SHIFT) |
                    (RowIdx << AIE_ROW_SHIFT));
  return my_ta;
}

/*
 * Check if specific shimDMA at given tile is idle
 * shimDMA is specified by a direction (S2MM or MM2S) and channel (0 or 1)
 * returns true when wait completes, false on timeout
 */
bool shim_dma_wait_idle(uint64_t TileAddr, int direction, int channel, uint32_t ctrl_id) {
  uint32_t status_register_offset;
  uint32_t status_mask_shift;
  uint32_t start_queue_size_mask_shift;
  if (channel == 0) {
    status_mask_shift = SHIM_DMA_STATUS_0_SHIFT;
    start_queue_size_mask_shift = SHIM_DMA_QUEUE_SIZE_0_SHIFT;
  } else {
    status_mask_shift = SHIM_DMA_STATUS_1_SHIFT;
    start_queue_size_mask_shift = SHIM_DMA_QUEUE_SIZE_1_SHIFT;
  }

  if (direction == SHIM_DMA_S2MM) {
    status_register_offset = REG_SHIM_DMA_S2MM_STATUS;
  } else {
    status_register_offset = REG_SHIM_DMA_MM2S_STATUS;
  }

  // Will timeout if shim is busy
  uint32_t timeout_count = 0;
  uint32_t timeout_val = 10000;
  uint32_t status = 0x3; // reserved status
  uint32_t status_reg = in32(TileAddr + status_register_offset);
  uint32_t queue_size = (status_reg >> start_queue_size_mask_shift) & SHIM_DMA_QUEUE_SIZE_MASK;

  while (queue_size) {
    // fetch status and check
    status_reg = in32(TileAddr + status_register_offset);
    status = (status_reg >> status_mask_shift) & SHIM_DMA_STATUS_MASK;
    queue_size = (status_reg >> start_queue_size_mask_shift) & SHIM_DMA_QUEUE_SIZE_MASK;

    if (timeout_count >= timeout_val) {
      lock_uart(ctrl_id);
      air_printf("[WARNING] shim_dma_wait_idle timed out\r\n");
      unlock_uart(ctrl_id);
      return false;
    }
    timeout_count++;
  }

  return true;
}

/*
 * Query specific shimDMA start queue size
 * shimDMA is specified by a direction (S2MM or MM2S) and channel (0 or 1)
 */
inline uint32_t shim_dma_get_outstanding(uint64_t TileAddr, int direction,
                                       int channel) {
  uint32_t status_register_offset;
  uint32_t start_queue_size_mask_shift;
  if (channel == 0) {
    start_queue_size_mask_shift = SHIM_DMA_QUEUE_SIZE_0_SHIFT;
  } else {
    start_queue_size_mask_shift = SHIM_DMA_QUEUE_SIZE_1_SHIFT;
  }
  if (direction == SHIM_DMA_S2MM) {
    status_register_offset = REG_SHIM_DMA_S2MM_STATUS;
  } else {
    status_register_offset = REG_SHIM_DMA_MM2S_STATUS;
  }
  uint32_t status_reg = in32(TileAddr + status_register_offset);
  uint32_t outstanding = (status_reg >> start_queue_size_mask_shift) & SHIM_DMA_QUEUE_SIZE_MASK;
  return outstanding;
}

/*
 * returns true if the number of outstanding BDs (BDs in start queue)
 */
inline bool shim_dma_can_push_bd(uint64_t TileAddr, int direction, int channel) {
  // TODO: does BD ID being dequeued from start queue for the channel imply that
  // it is safe to overwrite the underlying BD?
  return (shim_dma_get_outstanding(TileAddr, direction, channel) < (NUM_DMA_CH_BDS-1));
}

/*
 * Query shimDMA status
 * shimDMA is specified by a direction (S2MM or MM2S) and channel (0 or 1)
 */
inline uint32_t shim_dma_get_status(uint64_t TileAddr, int direction, int channel) {
  uint32_t status_register_offset;
  uint32_t status_mask_shift;
  if (channel == 0) {
    status_mask_shift = SHIM_DMA_STATUS_0_SHIFT;
  } else {
    status_mask_shift = SHIM_DMA_STATUS_1_SHIFT;
  }
  if (direction == SHIM_DMA_S2MM) {
    status_register_offset = REG_SHIM_DMA_S2MM_STATUS;
  } else {
    status_register_offset = REG_SHIM_DMA_MM2S_STATUS;
  }
  uint32_t status_reg = in32(TileAddr + status_register_offset);
  uint32_t status = (status_reg >> status_mask_shift) & SHIM_DMA_STATUS_MASK;
  return status;
}

/*
 * Push a BD to a shimDMA
 * shimDMA is specified by a direction (S2MM or MM2S) and channel (0 or 1)
 */
void shim_dma_push_bd(uint64_t TileAddr, uint32_t slot, int direction, int channel,
                      uint16_t col, uint64_t addr, uint32_t len, uint32_t ctrl_id) {

  uint32_t status_mask_shift;
  if (channel == 0) {
    status_mask_shift = SHIM_DMA_STATUS_0_SHIFT;
  } else {
    status_mask_shift = SHIM_DMA_STATUS_1_SHIFT;
  }

  uint32_t status_register_offset;
  uint32_t control_register_offset;
  uint32_t start_queue_register_offset;
  if (direction == SHIM_DMA_S2MM) {
    status_register_offset = REG_SHIM_DMA_S2MM_STATUS;
    control_register_offset = REG_SHIM_DMA_S2MM_CTRL(channel);
    start_queue_register_offset = REG_SHIM_DMA_S2MM_START_QUEUE(channel);
    lock_uart(ctrl_id);
    air_printf("\n\r  S2MM Shim DMA %d start channel %d\n\r", col, channel);
    unlock_uart(ctrl_id);
  } else {
    status_register_offset = REG_SHIM_DMA_MM2S_STATUS;
    control_register_offset = REG_SHIM_DMA_MMS2_CTRL(channel);
    start_queue_register_offset = REG_SHIM_DMA_MMS2_START_QUEUE(channel);
    lock_uart(ctrl_id);
    air_printf("\n\r  MM2S Shim DMA %d start channel %d\n\r", col, channel);
    unlock_uart(ctrl_id);
  }

  uint32_t outstanding = shim_dma_get_outstanding(TileAddr, direction, channel);
  lock_uart(ctrl_id);
  air_printf("Outstanding pre : %d\n\r", outstanding);
  unlock_uart(ctrl_id);

  // lookup the next BD ID to use for this channel and offset
  // by the channel's base BD ID
  uint32_t bd = next_bd[slot] + SHIM_DMA_BASE_BD(direction, channel);
  // advance the BD tracking
  next_bd[slot] = (next_bd[slot] == (NUM_DMA_CH_BDS-1)) ? 0 : next_bd[slot] + 1;
  lock_uart(ctrl_id);
  air_printf("Selected bd %u for slot %u\n\r", bd, slot);
  unlock_uart(ctrl_id);

  // Push the BD

  // Mark the BD as invalid
  out32(TileAddr + REG_SHIM_DMA_BD_CTRL(bd), 0);

  // Set the registers directly ...
  out32(TileAddr + REG_SHIM_DMA_BD_ADDR(bd), LOW_ADDR(addr));

  // change length in bytes to 32 bit words
  out32(TileAddr + REG_SHIM_DMA_BD_BUF_LEN(bd), len >> 2);

  uint32_t control = (HIGH_ADDR(addr) << 16) | SHIM_DMA_BD_CTRL_VALID;
  out32(TileAddr + REG_SHIM_DMA_BD_CTRL(bd), control);
  out32(TileAddr + REG_SHIM_DMA_BD_AXI_CFG(bd),
        0x410); // Burst len [10:9] = 2 (16)
                // QoS [8:5] = 0 (best effort)
                // Secure bit [4] = 1 (set)

  out32(TileAddr + REG_SHIM_DMA_BD_PKT(bd), 0);

  // Check if the channel is running or not
  uint32_t status_reg = in32(TileAddr + status_register_offset);
  uint32_t precheck_status = (status_reg >> status_mask_shift) & SHIM_DMA_STATUS_MASK;

  if (precheck_status == 0b00) {
    // Stream traffic can run, we can issue AXI-MM, and the channel is enabled
    lock_uart(ctrl_id);
    air_printf("Enabling shim DMA [%u] channel %u\r\n", col, channel);
    unlock_uart(ctrl_id);
    out32(TileAddr + control_register_offset, 0x1);
  }

  lock_uart(ctrl_id);
  air_printf("Pushing bd %u into 0x%lx\r\n", bd,
             TileAddr + start_queue_register_offset);
  unlock_uart(ctrl_id);

  // push
  out32(TileAddr + start_queue_register_offset, bd);

  // debug print
  outstanding = shim_dma_get_outstanding(TileAddr, direction, channel);
  lock_uart(ctrl_id);
  air_printf("Outstanding post: %d\n\r", outstanding);
  unlock_uart(ctrl_id);
}

void complete_agent_dispatch_packet(hsa_agent_dispatch_packet_t *pkt) {
  // completion phase
  packet_set_active(pkt, false);
  pkt->header = HSA_PACKET_TYPE_INVALID;
  pkt->type = AIR_PKT_TYPE_INVALID;
  hsa_signal_subtract_scacq_screl(pkt->completion_signal, 1);
}

void complete_barrier_packet(void *pkt) {
  hsa_barrier_and_packet_t *p = (hsa_barrier_and_packet_t *)(pkt);
  // completion phase
  p->header = HSA_PACKET_TYPE_INVALID;
  hsa_signal_subtract_scacq_screl(p->completion_signal, 1);
}

void handle_packet_get_capabilities(hsa_agent_dispatch_packet_t *pkt, uint32_t ctrl_id) {
  // packet is in active phase
  packet_set_active(pkt, true);
  uint64_t *addr = (uint64_t *)(pkt->return_address);

  lock_uart(ctrl_id);
  air_printf("Writing to 0x%llx\n\r", (uint64_t)addr);
  unlock_uart(ctrl_id);
  // We now write a capabilities structure to the address we were just passed
  // We've already done this once - should we just cache the results?
  int user1 = 1;
  int user2 = 0;

  addr[0] = (uint64_t)ctrl_id;      // region id
  addr[1] = (uint64_t)user1;        // num regions
  addr[2] = (uint64_t)(user2 >> 8); // region controller firmware version
  addr[3] = 16L;                    // cores per region
  addr[4] = 32768L;                 // Total L1 data memory per core
  addr[5] = 8L;                     // Number of L1 data memory banks
  addr[6] = 16384L;                 // L1 program memory per core
  addr[7] = 0L;                     // L2 data memory per region
}

void handle_packet_get_info(hsa_agent_dispatch_packet_t *pkt, uint32_t ctrl_id) {
  // packet is in active phase
  packet_set_active(pkt, true);
  uint64_t attribute = (pkt->arg[0]);
  uint64_t *addr =
      (uint64_t *)(&pkt->return_address); // FIXME when we can use a VA

  int user1 = 1;
  int user2 = 0;
  char name[] = "ACDC";
  char vend[] = "AMD";

  // TODO change this to use pkt->return_address
  switch (attribute) {
  case AIR_AGENT_INFO_NAME:
    strcpy((char *)addr, name);
    break;
  case AIR_AGENT_INFO_VENDOR_NAME:
    strcpy((char *)addr, vend);
    break;
  case AIR_AGENT_INFO_CONTROLLER_ID:
    *addr = (uint64_t)ctrl_id; // region id
    break;
  case AIR_AGENT_INFO_FIRMWARE_VER:
    *addr = (uint64_t)(user2 >> 8); // region controller firmware version
    break;
  case AIR_AGENT_INFO_NUM_REGIONS:
    *addr = (uint64_t)user1; // num regions
    break;
  case AIR_AGENT_INFO_HERD_SIZE: // cores per region
    *addr = HerdCfgInst.num_cols * HerdCfgInst.num_rows;
    break;
  case AIR_AGENT_INFO_HERD_ROWS:
    *addr = HerdCfgInst.num_rows; // rows of cores
    break;
  case AIR_AGENT_INFO_HERD_COLS:
    *addr = HerdCfgInst.num_cols; // cols of cores
    break;
  case AIR_AGENT_INFO_TILE_DATA_MEM_SIZE:
    *addr = 32768L; // total L1 data memory per core
    break;
  case AIR_AGENT_INFO_TILE_PROG_MEM_SIZE:
    *addr = 16384L; // L1 program memory per core
    break;
  case AIR_AGENT_INFO_L2_MEM_SIZE: // L2 memory per region (cols * 256k)
    *addr = 262144L * HerdCfgInst.num_cols;
    break;
  default:
    *addr = 0;
    break;
  }
}

void handle_packet_hello(hsa_agent_dispatch_packet_t *pkt, uint32_t ctrl_id) {
  packet_set_active(pkt, true);

  uint64_t say_what = pkt->arg[0];
  lock_uart(ctrl_id);
  xil_printf("CTRL %d : HELLO %08X\n\r", ctrl_id, (uint32_t)say_what);
  unlock_uart(ctrl_id);
}

/*
 * check if slot is in use
 * Only one packet can occupy a slot at a time
 */
bool slot_filled(uint32_t slot) {
  return (bool)(staged_nd_slot[slot].valid);
}

/*
 * get the packet attached to a slot
 */
hsa_agent_dispatch_packet_t* get_slot_packet(uint32_t slot) {
  return staged_nd_slot[slot].pkt;
}

/*
 * lookup slot index based on column, direction, and channel
 */
uint32_t get_slot(uint32_t col, uint32_t space, uint32_t direction, uint32_t channel) {
  if (space == 2) {
    for (uint32_t i = 0; i < NUM_SHIM_DMAS; i++) {
      if (col == shim_dma_cols[i]) {
        return (i * 4) + SHIM_DMA_CHNUM(direction, channel);
      }
    }
    return (uint32_t)(-1);
  }
  return (uint32_t)(-1);
}

void nd_dma_put_checkpoint(hsa_agent_dispatch_packet_t **pkt, uint32_t slot,
                           uint32_t idx_4d, uint32_t idx_3d, uint32_t idx_2d,
                           uint64_t pad_3d, uint64_t pad_2d, uint64_t pad_1d) {
  staged_nd_slot[slot].pkt = *pkt;
  staged_nd_slot[slot].paddr[0] = pad_1d;
  staged_nd_slot[slot].paddr[1] = pad_2d;
  staged_nd_slot[slot].paddr[2] = pad_3d;
  staged_nd_slot[slot].index[0] = idx_2d;
  staged_nd_slot[slot].index[1] = idx_3d;
  staged_nd_slot[slot].index[2] = idx_4d;
}

void nd_dma_get_checkpoint(hsa_agent_dispatch_packet_t **pkt, uint32_t slot,
                           uint32_t &idx_4d, uint32_t &idx_3d, uint32_t &idx_2d,
                           uint64_t &pad_3d, uint64_t &pad_2d,
                           uint64_t &pad_1d) {
  *pkt = staged_nd_slot[slot].pkt;
  pad_1d = staged_nd_slot[slot].paddr[0];
  pad_2d = staged_nd_slot[slot].paddr[1];
  pad_3d = staged_nd_slot[slot].paddr[2];
  idx_2d = staged_nd_slot[slot].index[0];
  idx_3d = staged_nd_slot[slot].index[1];
  idx_4d = staged_nd_slot[slot].index[2];
}

/*
 * do the nd_memcpy packet
 * this may require multiple BDs to complete
 * returns true when all BDs have been completed
 * returns false otherwise, including if the packet needs to be processed
 *  further to send additional BDs
 */
bool do_packet_nd_memcpy(uint32_t slot, uint32_t ctrl_id) {

  // lookup the staged packet / nd_memcpy
  hsa_agent_dispatch_packet_t *a_pkt;
  uint64_t paddr_3d;
  uint64_t paddr_2d;
  uint64_t paddr_1d;
  uint32_t index_4d;
  uint32_t index_3d;
  uint32_t index_2d;
  nd_dma_get_checkpoint(&a_pkt, slot, index_4d, index_3d, index_2d, paddr_3d,
                        paddr_2d, paddr_1d);

  uint16_t channel = (a_pkt->arg[0] >> 24) & 0x00ff;
  uint16_t col = (a_pkt->arg[0] >> 32) & 0x00ff;
  uint16_t direction = (a_pkt->arg[0] >> 60) & 0x000f;
  uint32_t length_1d = (a_pkt->arg[2] >> 0) & 0xffffffff;
  uint32_t length_2d = (a_pkt->arg[2] >> 32) & 0x0000ffff;
  uint32_t stride_2d = (a_pkt->arg[2] >> 48) & 0x0000ffff;
  uint32_t length_3d = (a_pkt->arg[3] >> 0) & 0x0000ffff;
  uint32_t stride_3d = (a_pkt->arg[3] >> 16) & 0x0000ffff;
  uint32_t length_4d = (a_pkt->arg[3] >> 32) & 0x0000ffff;
  uint32_t stride_4d = (a_pkt->arg[3] >> 48) & 0x0000ffff;
  uint32_t outstanding = 0;

  lock_uart(ctrl_id);
  air_printf(
      "%s: col=%u dir=%u chan=%u paddr=0x%llx 4d stride=%u length=%u\r\n",
      __func__, col, direction, channel, paddr_1d, stride_4d, length_4d);
  air_printf(
      "  3d stride=%u length=%u, 2d stride=%u length=%u, 1d length=%u\r\n",
      stride_3d, length_3d, stride_2d, length_2d, length_1d);
  unlock_uart(ctrl_id);

  for (; index_4d < length_4d; index_4d++) {
    for (; index_3d < length_3d; index_3d++) {
      for (; index_2d < length_2d; index_2d++) {
        bool can_push_bd = shim_dma_can_push_bd(getTileAddr(col, 0), direction, channel);
        if (can_push_bd) {
          lock_uart(ctrl_id);
          air_printf("\n\rND start shim DMA %u %u [%u][%u][%u] paddr=0x%llx\r\n",
                     direction, channel, index_4d, index_3d, index_2d, paddr_1d);
          unlock_uart(ctrl_id);
          shim_dma_push_bd(getTileAddr(col, 0), slot, direction, channel, col,
                           paddr_1d, length_1d, ctrl_id);
        } else {
          nd_dma_put_checkpoint(&a_pkt, slot, index_4d, index_3d, index_2d,
                                paddr_3d, paddr_2d, paddr_1d);
          lock_uart(ctrl_id);
          air_printf("\n\rND CHECKPOINT shim DMA %u %u [%u][%u][%u] paddr=0x%llx\r\n",
                     direction, channel, index_4d, index_3d, index_2d, paddr_1d);
          unlock_uart(ctrl_id);
          return false;
        }
        paddr_1d += stride_2d;
      }
      index_2d = 0;
      paddr_2d += stride_3d;
      if (index_3d + 1 < length_3d)
        paddr_1d = paddr_2d;
      else
        paddr_1d = paddr_3d + stride_4d;
    }
    index_3d = 0;
    paddr_3d += stride_4d;
    paddr_2d = paddr_3d;
  }

  // only reaches here if all BDs have been pushed
  return true;

}

bool wait_packet_nd_memcpy(uint32_t slot, uint32_t ctrl_id) {
  // lookup the staged packet / nd_memcpy
  hsa_agent_dispatch_packet_t *a_pkt;
  uint64_t paddr_3d;
  uint64_t paddr_2d;
  uint64_t paddr_1d;
  uint32_t index_4d;
  uint32_t index_3d;
  uint32_t index_2d;
  nd_dma_get_checkpoint(&a_pkt, slot, index_4d, index_3d, index_2d, paddr_3d,
                        paddr_2d, paddr_1d);

  uint16_t channel = (a_pkt->arg[0] >> 24) & 0x00ff;
  uint16_t col = (a_pkt->arg[0] >> 32) & 0x00ff;
  uint16_t direction = (a_pkt->arg[0] >> 60) & 0x000f;

  // all BDs pushed
  // do a check for idle, but don't block
  bool wait_idle_ret =
      shim_dma_wait_idle(getTileAddr(col, 0), direction, channel, ctrl_id);

  // timeout is okay, leave packet staged and BD in progress, then check later
  // false return indicates timeout
  if (!wait_idle_ret) {
    lock_uart(ctrl_id);
    air_printf("WARN: wait_packet_nd_memcpy() timed out on shim_dma_wait()\n\r");
    unlock_uart(ctrl_id);
  }

  return wait_idle_ret;
}

bool do_ubench(uint32_t length_1d, uint32_t ctrl_id) {

  volatile uint32_t c = length_1d;
  if (length_1d) {
    while (c > 0) { c--; }
  }
  return c == 0;
}

/*
 * stage a nd_memcpy packet for processing
 * this makes the packet active and copies it into the provided slot
 */
void stage_packet_nd_memcpy(hsa_agent_dispatch_packet_t *pkt, uint32_t slot,
                            uint32_t ctrl_id) {
  lock_uart(ctrl_id);
  air_printf("stage_packet_nd_memcpy slot %d\n\r", slot);
  unlock_uart(ctrl_id);
  // make active
  packet_set_active(pkt, true);

  uint64_t paddr = translate_virt_to_phys(pkt->arg[1]);
  lock_uart(ctrl_id);
  air_printf("ND_MEMCPY: physical address 0x%lx\r\n", paddr);
  unlock_uart(ctrl_id);

  // copy to slot
  nd_dma_put_checkpoint(&pkt, slot, 0, 0, 0, paddr, paddr, paddr);
  staged_nd_slot[slot].valid = 1;
}

/*
 * unstage a packet
 * this does not inactivate the packet
 */
void unstage_packet_nd_memcpy(uint32_t slot, uint32_t ctrl_id) {
  staged_nd_slot[slot].valid = 0;
  staged_nd_slot[slot].pkt = nullptr;
  lock_uart(ctrl_id);
  air_printf("unstage_packet_nd_memcpy slot %d\n\r", slot);
  unlock_uart(ctrl_id);
}

/*
 * Attempt to complete the given packet, which is assumed to be at rd_id of the queue.
 * If the packet is not successfully completed (ran out of BDs, timeout on shimDMA wait),
 * additional packets may be staged and BDs pushed, but completion will not be attempted.
 * Staging terminates when any packet other than an ND_MEMCPY is encountered.
 *
 * At entry, all packets before the current one are complete, but the current
 * packet may already be staged.
 */
int handle_packet_nd_memcpy(amd_queue_t *amd_queue, hsa_agent_dispatch_packet_t *pkt,
                            uint64_t local_read_index, uint32_t ctrl_id, int queue_id) {
  int packets_processed = 0;

  // check memory space
  uint16_t memory_space = (pkt->arg[0] >> 16) & 0x00ff;
  if (memory_space != 2) {
    lock_uart(ctrl_id);
    air_printf("WARN: BAD nd_memcpy packet for memspace %u\n\r", memory_space);
    unlock_uart(ctrl_id);
    complete_agent_dispatch_packet(pkt);
    packets_processed++;
    return packets_processed;
  }

  // check slot
  uint16_t channel = (pkt->arg[0] >> 24) & 0x00ff;
  uint16_t direction = (pkt->arg[0] >> 60) & 0x000f;
  uint16_t col = (pkt->arg[0] >> 32) & 0x00ff;
  uint32_t slot = get_slot(col, memory_space, direction, channel);
  if (slot == (uint32_t)(-1)) {
    lock_uart(ctrl_id);
    air_printf("WARN: BAD nd_memcpy packet slot for col %u\n\r", col);
    unlock_uart(ctrl_id);
    complete_agent_dispatch_packet(pkt);
    packets_processed++;
    return packets_processed;
  }

  // check if slot has a staged packet (if so, it should be this packet)
  // if no packet staged, stage this one (also makes it active)
  if (!slot_filled(slot)) {
    stage_packet_nd_memcpy(pkt, slot, ctrl_id);
  }

  // attempt to push all of the BDs for the staged packet
  bool bds_pushed = do_packet_nd_memcpy(slot, ctrl_id);

  // if all BDs pushed, attempt to complete the packet
  if (bds_pushed) {
    bool shim_dma_idle = wait_packet_nd_memcpy(slot, ctrl_id);
    if (shim_dma_idle) {
      complete_agent_dispatch_packet(pkt);
      // clear the staged slot
      unstage_packet_nd_memcpy(slot, ctrl_id);
      // count the packet as fully processed
      packets_processed++;
      return packets_processed;
    }
    // else shim DMA still working, attempt to start additional packets
  }
  // else could not push all BDs yet, attempt to start additional packets

  // packet staging loop
  // reached only if initial packet did not fully complete
  // this loop does not complete packets, it only stages and pushes as many BDs as possible
  // from the next packets in the queue, stoping as soon as a packet other than ND_MEMCPY
  // is reached or if the needed slot is already occupied
  bool staging_done = false;
  while (!staging_done) {
    // move to next packet
    local_read_index++;
    // get the next packet
    hsa_agent_dispatch_packet_t *pkt_buf(
        reinterpret_cast<hsa_agent_dispatch_packet_t*>(hsa_csr->queue_bufs[queue_id]));
    pkt = &pkt_buf[local_read_index % amd_queue->hsa_queue.size];

    // verify current packet is an agent dispatch packet and op is ND_MEMCPY
    auto type = pkt->header & 0xffU;
    // get the dispatch operation
    auto op = pkt->type & 0xffff;
    if (type != HSA_PACKET_TYPE_AGENT_DISPATCH || op != AIR_PKT_TYPE_ND_MEMCPY) {
      staging_done = true;
      break;
    }

    // check memory space
    uint16_t memory_space = (pkt->arg[0] >> 16) & 0x00ff;
    if (memory_space != 2) {
      lock_uart(ctrl_id);
      air_printf("WARN: BAD nd_memcpy packet for memspace %u\n\r", memory_space);
      unlock_uart(ctrl_id);
      staging_done = true;
      break;
    }

    // lookup slot
    uint16_t channel = (pkt->arg[0] >> 24) & 0x00ff;
    uint16_t direction = (pkt->arg[0] >> 60) & 0x000f;
    uint16_t col = (pkt->arg[0] >> 32) & 0x00ff;
    uint32_t slot = get_slot(col, memory_space, direction, channel);
    if (slot == (uint32_t)(-1)) {
      lock_uart(ctrl_id);
      air_printf("WARN: BAD nd_memcpy packet slot for col %u\n\r", col);
      unlock_uart(ctrl_id);
      staging_done = true;
      break;
    }

    // check if slot has a staged packet, only proceed if no packet staged
    staging_done = slot_filled(slot);
    // if no packet staged, stage this one
    if (!staging_done) {
      // stage packet, mark active
      stage_packet_nd_memcpy(pkt, slot, ctrl_id);
      // push BDs, but don't check for completion
      do_packet_nd_memcpy(slot, ctrl_id);
    }
  }

  return packets_processed;
}

/*
 * process agent dispatch packet
 */
void handle_agent_dispatch_packet(amd_queue_t *amd_queue, uint32_t ctrl_id, int queue_id) {
  // get the packet
  volatile uint64_t *rd_id(&amd_queue->read_dispatch_id);
  uint64_t local_read_index = amd_queue->read_dispatch_id;
  hsa_agent_dispatch_packet_t *pkt_buf(
      reinterpret_cast<hsa_agent_dispatch_packet_t*>(hsa_csr->queue_bufs[queue_id]));
  hsa_agent_dispatch_packet_t *pkt(
      &pkt_buf[local_read_index % amd_queue->hsa_queue.size]);

  bool done = false;
  int packets_processed = 0;
  do {
    // verify current packet is an agent dispatch packet
    auto type = pkt->header & 0xffU;
    if (type != HSA_PACKET_TYPE_AGENT_DISPATCH) {
      done = true;
      break;
    }

    // get the dispatch operation
    auto op = pkt->type & 0xffff;
    // air_printf("Op is %04X\n\r",op);

    uint32_t length_1d = (pkt->arg[0] >> 0) & 0xffffffff;

    switch (op) {
    case AIR_PKT_TYPE_INVALID:
    default:
      complete_agent_dispatch_packet(pkt);
      packets_processed++;
      break;
    case AIR_PKT_TYPE_HELLO:
      handle_packet_hello(pkt, ctrl_id);
      complete_agent_dispatch_packet(pkt);
      packets_processed++;
      break;
    case AIR_PKT_TYPE_GET_CAPABILITIES:
      handle_packet_get_capabilities(pkt, ctrl_id);
      complete_agent_dispatch_packet(pkt);
      packets_processed++;
      break;
    case AIR_PKT_TYPE_GET_INFO:
      handle_packet_get_info(pkt, ctrl_id);
      complete_agent_dispatch_packet(pkt);
      packets_processed++;
      break;
    case AIR_PKT_TYPE_UBENCH:
      if (do_ubench(length_1d, ctrl_id)) {
        lock_uart(ctrl_id);
        air_printf("CTRL[%u]: ubench finished\n\r", ctrl_id);
        unlock_uart(ctrl_id);
      }
      complete_agent_dispatch_packet(pkt);
      packets_processed++;
      break;
    case AIR_PKT_TYPE_ND_MEMCPY:
      int nd_memcpy_processed = handle_packet_nd_memcpy(amd_queue, pkt, local_read_index,
                                                        ctrl_id, queue_id);
      if (nd_memcpy_processed) {
        packets_processed += nd_memcpy_processed;
      } else {
        done = true;
      }
      break; // case AIR_PKT_TYPE_ND_MEMCPY
    } // switch

    // advance local read pointer, get next packet
    local_read_index++;
    pkt = &pkt_buf[local_read_index % amd_queue->hsa_queue.size];

  } while (!done); // packet processing loop

  lock_uart(ctrl_id);
  air_printf("Completing: %d packets processed.\n\r", packets_processed);
  unlock_uart(ctrl_id);
  *rd_id += packets_processed;
}

void handle_barrier_and_packet(amd_queue_t *amd_queue, uint32_t ctrl_id, int queue_id) {

  volatile uint64_t *rd_id(&amd_queue->read_dispatch_id);
  uint64_t local_read_index = amd_queue->read_dispatch_id;
  hsa_barrier_and_packet_t *pkt_buf(
      reinterpret_cast<hsa_barrier_and_packet_t*>(hsa_csr->queue_bufs[queue_id]));
  hsa_barrier_and_packet_t *pkt(
      &pkt_buf[local_read_index % amd_queue->hsa_queue.size]);

  // TODO complete functionality with VAs
  hsa_signal_t s0 = pkt->dep_signal[0];
  hsa_signal_t s1 = pkt->dep_signal[1];
  hsa_signal_t s2 = pkt->dep_signal[2];
  hsa_signal_t s3 = pkt->dep_signal[3];
  hsa_signal_t s4 = pkt->dep_signal[4];

  while ( hsa_signal_wait_scacquire(s0, HSA_SIGNAL_CONDITION_EQ, 0, 0x80000, HSA_WAIT_STATE_ACTIVE) != 0 ||
          hsa_signal_wait_scacquire(s1, HSA_SIGNAL_CONDITION_EQ, 0, 0x80000, HSA_WAIT_STATE_ACTIVE) != 0 ||
          hsa_signal_wait_scacquire(s2, HSA_SIGNAL_CONDITION_EQ, 0, 0x80000, HSA_WAIT_STATE_ACTIVE) != 0 ||
          hsa_signal_wait_scacquire(s3, HSA_SIGNAL_CONDITION_EQ, 0, 0x80000, HSA_WAIT_STATE_ACTIVE) != 0 ||
          hsa_signal_wait_scacquire(s4, HSA_SIGNAL_CONDITION_EQ, 0, 0x80000, HSA_WAIT_STATE_ACTIVE) != 0) {
    lock_uart(ctrl_id);
    air_printf("CTRL %d : barrier AND packet completion signal timeout!\n\r",
               ctrl_id);
    for (int i = 0; i < 5; i++)
      air_printf("CTRL %d : dep_signal[%d] = %d\n\r", ctrl_id, i,
                 pkt->dep_signal[i]);
    unlock_uart(ctrl_id);
  }

  complete_barrier_packet(pkt);
  *rd_id += 1;

}

void handle_barrier_or_packet(amd_queue_t *amd_queue, uint32_t ctrl_id, int queue_id) {

  volatile uint64_t *rd_id(&amd_queue->read_dispatch_id);
  uint64_t local_read_index = amd_queue->read_dispatch_id;
  hsa_barrier_or_packet_t *pkt_buf(
      reinterpret_cast<hsa_barrier_or_packet_t*>(hsa_csr->queue_bufs[queue_id]));
  hsa_barrier_or_packet_t *pkt(
      &pkt_buf[local_read_index % amd_queue->hsa_queue.size]);

  // TODO complete functionality with VAs
  hsa_signal_t s0 = pkt->dep_signal[0];
  hsa_signal_t s1 = pkt->dep_signal[1];
  hsa_signal_t s2 = pkt->dep_signal[2];
  hsa_signal_t s3 = pkt->dep_signal[3];
  hsa_signal_t s4 = pkt->dep_signal[4];

  while ( hsa_signal_wait_scacquire(s0, HSA_SIGNAL_CONDITION_EQ, 0, 0x80000, HSA_WAIT_STATE_ACTIVE) != 0 &&
          hsa_signal_wait_scacquire(s1, HSA_SIGNAL_CONDITION_EQ, 0, 0x80000, HSA_WAIT_STATE_ACTIVE) != 0 &&
          hsa_signal_wait_scacquire(s2, HSA_SIGNAL_CONDITION_EQ, 0, 0x80000, HSA_WAIT_STATE_ACTIVE) != 0 &&
          hsa_signal_wait_scacquire(s3, HSA_SIGNAL_CONDITION_EQ, 0, 0x80000, HSA_WAIT_STATE_ACTIVE) != 0 &&
          hsa_signal_wait_scacquire(s4, HSA_SIGNAL_CONDITION_EQ, 0, 0x80000, HSA_WAIT_STATE_ACTIVE) != 0) {

    lock_uart(ctrl_id);
    air_printf("CTRL %d : barrier OR packet completion signal timeout!\n\r",
               ctrl_id);
    for (int i = 0; i < 5; i++)
      air_printf("CTRL %d : dep_signal[%d] = %d\n\r", ctrl_id, i,
                 pkt->dep_signal[i]);
    unlock_uart(ctrl_id);
  }

  complete_barrier_packet(pkt);
  *rd_id += 1;
}

int main() {

  int ctrl_id = -1;
  int maj = 0, min = 0, ver = 0;

  // init pointer to HSA CSR block
  hsa_csr_init_ro();

  uint64_t did = bp_get_did();
  ctrl_id = (int)did + 1;
  int mimpid = bp_get_mimpid();
  maj = (mimpid >> 24) & 0xff;
  min = (mimpid >> 16) & 0xff;
  ver = (mimpid >> 8) & 0xff;

  // initialize private view of the Mutex block
  xmutex_cfg = XMutex_LookupConfig(XPAR_MUTEX_0_DEVICE_ID);
  XMutex_CfgInitialize(xmutex_ptr, xmutex_cfg, xmutex_cfg->BaseAddress, ctrl_id);

  // wait for ARM to release cores for execution
  volatile uint64_t* barrier = reinterpret_cast<volatile uint64_t*>(&hsa_csr->global_barrier);
  while (*barrier == 0) { }

  // one admin queue, one queue per CP
  int num_ctrl = hsa_csr->num_aql_queues - 1;

  lock_uart(ctrl_id);
  xil_printf("RISCV %d of %d firmware %d.%d.%d created on %s at %s GMT\n\r",
             ctrl_id, num_ctrl, maj, min, ver, __DATE__,
             __TIME__);
  xil_printf("(c) Copyright 2020-2023 AMD, Inc. All rights reserved.\n\r");
  unlock_uart(ctrl_id);

  int hqd_id(ctrl_id);
  // every core gets one user queue allocated by the runtime
  amd_queue_t *amd_queue(hsa_csr->amd_aql_queues[hqd_id]);
  volatile uint64_t *doorbell(reinterpret_cast<uint64_t*>(
      hsa_csr->doorbells[hqd_id]));
  volatile uint64_t *rd_id(&amd_queue->read_dispatch_id);
  volatile uint64_t *wr_id(&amd_queue->write_dispatch_id);
  hsa_agent_dispatch_packet_t *queue_buf(
      reinterpret_cast<hsa_agent_dispatch_packet_t*>(
          hsa_csr->queue_bufs[hqd_id]));
  hsa_agent_dispatch_packet_t *aql_pkt(nullptr);

  *doorbell = std::numeric_limits<uint64_t>::max();
  *rd_id = 0;
  *wr_id = 0;
  lock_uart(ctrl_id);
  xil_printf("CTRL %d initialized queue at 0x%p\n\r", ctrl_id, amd_queue);
  unlock_uart(ctrl_id);

  uint64_t heartbeat = 1;
  uint64_t heartbeat_threshold = 10000000;

  while (true) {
    if (heartbeat % heartbeat_threshold == 0) {
      uint64_t mcycle = bp_get_mcycle();
      lock_uart(ctrl_id);
      xil_printf("RISCV %d of %d alive at cycle %llu\n\r", ctrl_id, num_ctrl, mcycle);
      xil_printf("RISCV %d queue at 0x%p, doorbell at 0x%p\n\r", ctrl_id, amd_queue, doorbell);
      unlock_uart(ctrl_id);
    }
    heartbeat++;

    if (*doorbell + 1 > *rd_id) {
      aql_pkt = &queue_buf[*rd_id % amd_queue->hsa_queue.size];
      uint32_t type(static_cast<uint32_t>(aql_pkt->header) & 0xffU);
      uint32_t func(static_cast<uint32_t>(aql_pkt->type) & 0xffffU);

      lock_uart(ctrl_id);
      air_printf("Doorbell rung %llu\n\r", *doorbell);
      air_printf("Packet type %u, func type %u, pkt data %llx\n\r", type, func,
                 aql_pkt->arg[0]);
      air_printf("queue heap addr %llx\n\r", hsa_csr->queue_dram_cpu_va[hqd_id]);
      unlock_uart(ctrl_id);

      uint32_t invalid_count = 0;
      while (type == HSA_PACKET_TYPE_INVALID) {
          aql_pkt = &queue_buf[*rd_id % amd_queue->hsa_queue.size];
          type = static_cast<uint32_t>(aql_pkt->header) & 0xffU;
          func = static_cast<uint32_t>(aql_pkt->type) & 0xffffU;

          // TODO: Come back to this for the multi-producer queue as we can hit this
          invalid_count++;
          if(invalid_count > INVLD_COUNT_TIMEOUT) {
            lock_uart(ctrl_id);
            xil_printf("[WARNING] We are stuck in an invalid packet and timed out. Breaking\r\n");
            xil_printf("\theader: 0x%x\r\n", aql_pkt->header);
            xil_printf("\ttype: 0x%x\r\n", type);
            xil_printf("\tfunc: 0x%x\r\n", func);
            xil_printf("\trd_id: 0x%x\r\n", *rd_id);
            xil_printf("\tdoorbell: 0x%x\r\n", *doorbell);
            unlock_uart(ctrl_id);
            break;
          }
      }

      switch (type) {
        case HSA_PACKET_TYPE_AGENT_DISPATCH:
          lock_uart(ctrl_id);
          air_printf("Dispatching agent dispatch packet\n\r");
          unlock_uart(ctrl_id);
          // this call may return without fully processing the packet, but
          // it will leave the packet in the queue and not advance the read
          // index if the packet must be retried or is in flight
          handle_agent_dispatch_packet(amd_queue, ctrl_id, hqd_id);
          lock_uart(ctrl_id);
          air_printf("Agent dispatch packet completed, back in main loop\n\r");
          unlock_uart(ctrl_id);
          break;
        case HSA_PACKET_TYPE_BARRIER_AND:
          lock_uart(ctrl_id);
          air_printf("Executing barrier and packet\r\n");
          unlock_uart(ctrl_id);
          handle_barrier_and_packet(amd_queue, ctrl_id, hqd_id);
          break;
        case HSA_PACKET_TYPE_BARRIER_OR:
          lock_uart(ctrl_id);
          air_printf("Executing barrier or packet\r\n");
          unlock_uart(ctrl_id);
          handle_barrier_or_packet(amd_queue, ctrl_id, hqd_id);
          break;
        // We are already handling the invalid packet above
        case HSA_PACKET_TYPE_INVALID:
          break;
        default:
          lock_uart(ctrl_id);
          air_printf("Unsupported packet type, skipping\n\r");
          unlock_uart(ctrl_id);
          aql_pkt->header = HSA_PACKET_TYPE_INVALID;
          ++(*rd_id);
          break;
      } // switch (type)
    } // doorbell rung
  } // while (true)

  return 0;
}
