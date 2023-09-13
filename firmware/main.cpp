//===- main.cpp -------------------------------------------------*- C++ -*-===//
//
// Copyright (C) 2020-2022, Xilinx Inc.
// Copyright (C) 2022, Advanced Micro Devices, Inc.
// SPDX-License-Identifier: MIT
//
//===----------------------------------------------------------------------===//


#include "unistd.h"
#include <cstdint>
#include <cstring>
#include <limits>

#include "amd_hsa.h"
#include "debug.h"
#include "hsa_csr.h"
#include "hsa_ext_air.h"
#include "memory.h"

extern "C" {

// Right now, only ARM can control ERNICs
#ifdef ARM_CONTROLLER
#include "pcie-ernic-defines.h"
#endif

#if defined(ARM_CONTROLLER)
#include "xaiengine.h"
#include "xil_cache.h"
#else
#include "pvr.h"
#endif

}

#include "airbin.h"
#include "cdma.h"
#include "platform.h"

#include "shell.h"

#define XAIE_NUM_ROWS 8
#define XAIE_NUM_COLS 50

#ifdef ARM_CONTROLLER
#define XAIE_ADDR_ARRAY_OFF 0
#else
#define XAIE_ADDR_ARRAY_OFF 0x800ULL
#endif // ARM_CONTROLLER

#define NUM_SHIM_DMA_S2MM_CHANNELS 2
#define NUM_SHIM_DMA_MM2S_CHANNELS 2
#define XAIEDMA_SHIM_CHNUM_S2MM0 0U
#define XAIEDMA_SHIM_CHNUM_S2MM1 1U
#define XAIEDMA_SHIM_CHNUM_MM2S0 2U
#define XAIEDMA_SHIM_CHNUM_MM2S1 3U

#define HIGH_ADDR(addr) ((addr & 0xffffffff00000000ULL) >> 32)
#define LOW_ADDR(addr) (addr & 0x00000000ffffffffULL)

#define ALIGN(_x, _size) (((_x) + (_size-1)) & ~(_size-1))

#define LOGICAL_HERD_DMAS 16

// direction
#define SHIM_DMA_S2MM 0
#define SHIM_DMA_MM2S 1

#define NUM_SHIM_DMAS 16
#define NUM_COL_DMAS 4

#define INVLD_COUNT_TIMEOUT 100

uint8_t shim_dma_cols[NUM_SHIM_DMAS] = {2,  3,  6,  7,  10, 11, 18, 19,
                                        26, 27, 34, 35, 42, 43, 46, 47};
uint8_t col_dma_cols[NUM_COL_DMAS] = {7, 8, 9, 10};

#define NUM_DMAS (NUM_SHIM_DMAS + NUM_COL_DMAS)

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

#define SHIM_DMA_NUM_BDS 16

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
#define REG_SHIM_DMA_CTRL(_chan) (0x1D140 + (0x8 * _chan))
#define REG_SHIM_DMA_START_QUEUE(_chan) (0x1D144 + (0x8 * _chan))

#define REG_SHIM_DMA_S2MM_STATUS (0x1D160)
#define SHIM_DMA_CURR_BD_SHIFT 16
#define SHIM_DMA_CURR_BD_WIDTH 4
#define SHIM_DMA_CURR_BD_MASK ((1 << SHIM_DMA_CURR_BD_WIDTH) - 1)
#define SHIM_DMA_QUEUE_SIZE_SHIFT 6
#define SHIM_DMA_QUEUE_SIZE_WIDTH 3
#define SHIM_DMA_QUEUE_SIZE_MASK ((1 << SHIM_DMA_QUEUE_SIZE_WIDTH) - 1)
#define SHIM_DMA_STATUS_SHIFT 0
#define SHIM_DMA_STATUS_WIDTH 2
#define SHIM_DMA_STATUS_MASK ((1 << SHIM_DMA_STATUS_WIDTH) - 1)
#define SHIM_DMA_STALLED_SHIFT 4
#define SHIM_DMA_STALLED_WIDTH 1
#define SHIM_DMA_STALLED_MASK 1
#define GET_SHIM_DMA(_field, _reg, _ch)                                        \
  ((_reg) >>                                                                   \
       (SHIM_DMA_##_field##_SHIFT + (SHIM_DMA_##_field##_WIDTH * (_ch))) &     \
   SHIM_DMA_##_field##_MASK)

#define REG_SHIM_DMA_MM2S_STATUS (0x1D164)

#define REG_AIE_COL_RESET 0x00036048
#define REG_SHIM_RESET_ENA 0x0003604C

#define REG_AIE_CORE_CTL_RESET (1U << 1)
#define REG_AIE_CORE_CTL_ENABLE (1U << 0)

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

constexpr uint32_t NUM_BD = 16;

#ifdef ARM_CONTROLLER
// The NPI registers we use to reset the array
constexpr auto NPI_MASK_REG = 0x0;
constexpr auto NPI_VAL_REG = 0x4;
constexpr auto NPI_LOCK_REG = 0xC;
#endif // ARM_CONTROLLER

struct HerdConfig {
  uint32_t row_start;
  uint32_t num_rows;
  uint32_t col_start;
  uint32_t num_cols;
};

HerdConfig HerdCfgInst;

#ifdef ARM_CONTROLLER
aie_libxaie_ctx_t *_xaie;

/*
  read 32 bit value from specified address
*/
static inline uint32_t in32(uint64_t Addr) {
  uint32_t Value = IO_READ32(Addr);
  return Value;
}

/*
  write 32 bit value to specified address
*/
static inline void out32(uint64_t Addr, uint32_t Value) {
  IO_WRITE32(Addr, Value);
}

uint32_t maskpoll32(uint64_t Addr, uint32_t Mask, uint32_t Value, uint32_t TimeOut) {
  uint32_t Ret = 1;

  uint32_t Count = 10 + TimeOut;

  while (Count > 0U) {
    if ((in32(Addr) & Mask) == Value) {
      Ret = 0;
      break;
    }
    Count--;
  }

  return Ret;
}

/*
  Calculate the address of an AIE tile
*/
uint64_t getTileAddr(uint16_t ColIdx, uint16_t RowIdx) {
#ifdef ARM_CONTROLLER
  uint64_t my_ta = (uint64_t)(AIE_BASE | (ColIdx << AIE_COLUMN_SHIFT) |
                    (RowIdx << AIE_ROW_SHIFT));
  return my_ta;
#else
  uint64_t TileAddr = 0;
  uint64_t ArrOffset = XAIE_ADDR_ARRAY_OFF;

#ifdef XAIE_BASE_ARRAY_ADDR_OFFSET
  ArrOffset = XAIE_BASE_ARRAY_ADDR_OFFSET;
#endif

  /*
   * Tile address format:
   * --------------------------------------------
   * |                7 bits  5 bits   18 bits  |
   * --------------------------------------------
   * | Array offset | Column | Row | Tile addr  |
   * --------------------------------------------
   */
  TileAddr = (uint64_t)((ArrOffset << XAIEGBL_TILE_ADDR_ARR_SHIFT) |
                   (ColIdx << XAIEGBL_TILE_ADDR_COL_SHIFT) |
                   (RowIdx << XAIEGBL_TILE_ADDR_ROW_SHIFT));

  return TileAddr;
#endif
}

static const char *decode_dma_state(uint32_t state) {
  switch (state) {
  case 0:
    return "idle";
  case 1:
    return "starting";
  case 2:
    return "running";
  }
  return "unknown";
}

void mlir_aie_init_libxaie(aie_libxaie_ctx_t *ctx) {
  if (!ctx)
    return;

  ctx->AieConfigPtr.AieGen = XAIE_DEV_GEN_AIE;
  ctx->AieConfigPtr.BaseAddr = AIE_BASE;
  ctx->AieConfigPtr.ColShift = AIE_COLUMN_SHIFT;
  ctx->AieConfigPtr.RowShift = AIE_ROW_SHIFT;
  ctx->AieConfigPtr.NumRows = XAIE_NUM_ROWS + 1;
  ctx->AieConfigPtr.NumCols = XAIE_NUM_COLS;
  ctx->AieConfigPtr.ShimRowNum = 0;      // XAIE_SHIM_ROW;
  ctx->AieConfigPtr.MemTileRowStart = 0; // XAIE_RES_TILE_ROW_START;
  ctx->AieConfigPtr.MemTileNumRows = 0;  // XAIE_RES_TILE_NUM_ROWS;
  ctx->AieConfigPtr.AieTileRowStart = 1; // XAIE_AIE_TILE_ROW_START;
  ctx->AieConfigPtr.AieTileNumRows = XAIE_NUM_ROWS;
  ctx->AieConfigPtr.PartProp = {0};
  ctx->DevInst = {0};
}

int mlir_aie_init_device(aie_libxaie_ctx_t *ctx) {
  AieRC RC = XAIE_OK;

  RC = XAie_CfgInitialize(&(ctx->DevInst), &(ctx->AieConfigPtr));
  if (RC != XAIE_OK) {
    xil_printf("Driver initialization failed.\n\r");
    return -1;
  }

  RC = XAie_PmRequestTiles(&(ctx->DevInst), NULL, 0);
  if (RC != XAIE_OK) {
    xil_printf("Failed to request tiles.\n\r");
    return -1;
  }

  // TODO Extra code to really teardown the segments
  RC = XAie_Finish(&(ctx->DevInst));
  if (RC != XAIE_OK) {
    xil_printf("Failed to finish tiles.\n\r");
    return -1;
  }
  RC = XAie_CfgInitialize(&(ctx->DevInst), &(ctx->AieConfigPtr));
  if (RC != XAIE_OK) {
    xil_printf("Driver initialization failed.\n\r");
    return -1;
  }
  RC = XAie_PmRequestTiles(&(ctx->DevInst), NULL, 0);
  if (RC != XAIE_OK) {
    xil_printf("Failed to request tiles.\n\r");
    return -1;
  }

  return 0;
}

int mlir_aie_reinit_device(aie_libxaie_ctx_t *ctx) {
  AieRC RC = XAIE_OK;

  RC = XAie_Finish(&(ctx->DevInst));
  if (RC != XAIE_OK) {
    xil_printf("Failed to finish tiles.\n\r");
    return -1;
  }
  RC = XAie_CfgInitialize(&(ctx->DevInst), &(ctx->AieConfigPtr));
  if (RC != XAIE_OK) {
    xil_printf("Driver initialization failed.\n\r");
    return -1;
  }
  RC = XAie_PmRequestTiles(&(ctx->DevInst), NULL, 0);
  if (RC != XAIE_OK) {
    xil_printf("Failed to request tiles.\n\r");
    return -1;
  }

  return 0;
}

void mlir_aie_print_dma_status(int col, int row) {
  uint64_t tileAddr = getTileAddr(col, row);

  uint32_t dma_s2mm0_control = in32(tileAddr + 0x0001DE00);
  uint32_t dma_s2mm1_control = in32(tileAddr + 0x0001DE08);
  uint32_t dma_mm2s0_control = in32(tileAddr + 0x0001DE10);
  uint32_t dma_mm2s1_control = in32(tileAddr + 0x0001DE18);
  uint32_t dma_s2mm_status = in32(tileAddr + 0x0001DF00);
  uint32_t dma_mm2s_status = in32(tileAddr + 0x0001DF10);

  uint32_t s2mm_ch0_running = dma_s2mm_status & 0x3;
  uint32_t s2mm_ch1_running = (dma_s2mm_status >> 2) & 0x3;
  uint32_t mm2s_ch0_running = dma_mm2s_status & 0x3;
  uint32_t mm2s_ch1_running = (dma_mm2s_status >> 2) & 0x3;

  xil_printf("DMA [%d, %d] tile addr=0x%lx\r\n", col, row, tileAddr);
  xil_printf("  mm2s (0=%s 1=%s) status=%08X ctrl0=%02X ctrl1=%02X\r\n",
             decode_dma_state(mm2s_ch0_running),
             decode_dma_state(mm2s_ch1_running), dma_mm2s_status,
             dma_mm2s0_control, dma_mm2s1_control);
  xil_printf("  s2mm (0=%s 1=%s) status=%08X ctrl0=%02X ctrl1=%02X\r\n",
             decode_dma_state(s2mm_ch0_running),
             decode_dma_state(s2mm_ch1_running), dma_s2mm_status,
             dma_s2mm0_control, dma_s2mm1_control);

  xil_printf("Descriptors:\r\n");
  for (uint32_t bd = 0; bd < NUM_BD; bd++) {
    uint32_t dma_bd_addr_a = in32(tileAddr + REG_AIE_DMA_BD_ADDR_A(bd));
    uint32_t dma_bd_control = in32(tileAddr + REG_AIE_DMA_BD_CTL(bd));
    if (dma_bd_control & AIE_DMA_BD_CTL_VALID) {
      xil_printf("BD %d valid\n\r", bd);
      uint32_t current_s2mm_ch0 = (dma_s2mm_status >> 16) & 0xf;
      uint32_t current_s2mm_ch1 = (dma_s2mm_status >> 20) & 0xf;
      uint32_t current_mm2s_ch0 = (dma_mm2s_status >> 16) & 0xf;
      uint32_t current_mm2s_ch1 = (dma_mm2s_status >> 20) & 0xf;

      if (s2mm_ch0_running && bd == current_s2mm_ch0) {
        xil_printf(" * Current BD for s2mm channel 0\n\r");
      }
      if (s2mm_ch1_running && bd == current_s2mm_ch1) {
        xil_printf(" * Current BD for s2mm channel 1\n\r");
      }
      if (mm2s_ch0_running && bd == current_mm2s_ch0) {
        xil_printf(" * Current BD for mm2s channel 0\n\r");
      }
      if (mm2s_ch1_running && bd == current_mm2s_ch1) {
        xil_printf(" * Current BD for mm2s channel 1\n\r");
      }

      if (dma_bd_control & AIE_DMA_BD_CTL_ENA_PKT) {
        uint32_t dma_packet = in32(tileAddr + REG_AIE_DMA_BD_PKT(bd));
        xil_printf("   Packet mode: %02X\n\r", dma_packet & AIE_DMA_BD_PKT_ID);
      }
      int words_to_transfer = 1 + (dma_bd_control & AIE_DMA_BD_CTL_LEN);
      int base_address = dma_bd_addr_a & AIE_DMA_BD_ADDR_BASE;
      xil_printf("   Transfering %d 32 bit words to/from %06X\n\r",
                 words_to_transfer, base_address);

      xil_printf("   ");
      for (int w = 0; w < 7; w++) {
        uint32_t tmpd = in32(tileAddr + (base_address << 2) + (w * 4));
        xil_printf("%08X ", tmpd);
      }
      xil_printf("\n\r");
      if (dma_bd_addr_a & AIE_DMA_BD_ADDR_ENA_ACQ) {
        uint32_t lock_id = (dma_bd_addr_a >> 22) & 0xf;
        xil_printf("   Acquires lock %d ", lock_id);
        if (dma_bd_addr_a & 0x10000)
          xil_printf("with value %d ", (dma_bd_addr_a >> 17) & 0x1);

        xil_printf("currently ");
        uint32_t locks = in32(tileAddr + 0x0001EF00);
        uint32_t two_bits = (locks >> (lock_id * 2)) & 0x3;
        if (two_bits) {
          uint32_t acquired = two_bits & 0x1;
          uint32_t value = two_bits & 0x2;
          if (acquired)
            xil_printf("Acquired ");
          xil_printf(value ? "1" : "0");
        } else
          xil_printf("0");
        xil_printf("\n\r");
      }
      if (dma_bd_control & 0x30000000) { // FIFO MODE
        int FIFO = (dma_bd_control >> 28) & 0x3;
        uint32_t dma_fifo_counter = in32(tileAddr + 0x0001DF20);
        xil_printf("   Using FIFO Cnt%d : %08X\n\r", FIFO, dma_fifo_counter);
      }
      uint32_t nextBd = ((dma_bd_control >> 13) & 0xF);
      uint32_t useNextBd = ((dma_bd_control >> 17) & 0x1);
      xil_printf("   Next BD: %d %s\r\n", nextBd,
                 (useNextBd == 0) ? "(unused)" : "(used)");
    }
  }
}

/*
  The shim tile is always row 0
*/
void mlir_aie_print_shimdma_status(uint16_t col) {
  uint64_t tileAddr = getTileAddr(col, 0);
  uint32_t s2mm_status = in32(tileAddr + REG_SHIM_DMA_S2MM_STATUS);
  uint32_t mm2s_status = in32(tileAddr + REG_SHIM_DMA_MM2S_STATUS);

  xil_printf("Shim DMA [%u]\r\n", col);
  xil_printf("S2MM\r\n");
  for (uint8_t channel = 0; channel < NUM_SHIM_DMA_S2MM_CHANNELS; channel++) {
    xil_printf("   [channel %u] start_bd=%u queue_size=%u curr_bd=%u status=%s "
               "stalled=%s\r\n",
               channel, in32(tileAddr + REG_SHIM_DMA_START_QUEUE(channel)),
               GET_SHIM_DMA(QUEUE_SIZE, s2mm_status, channel),
               GET_SHIM_DMA(CURR_BD, s2mm_status, channel),
               GET_SHIM_DMA(STATUS, s2mm_status, channel),
               GET_SHIM_DMA(STALLED, s2mm_status, channel));
  }
  xil_printf("MM2S\r\n");
  for (uint8_t channel = 0; channel < NUM_SHIM_DMA_MM2S_CHANNELS; channel++) {
    xil_printf("   [channel %u] start_bd=%u queue_size=%u curr_bd=%u status=%s "
               "stalled=%s\r\n",
               channel, in32(tileAddr + REG_SHIM_DMA_START_QUEUE(channel)),
               GET_SHIM_DMA(QUEUE_SIZE, mm2s_status, channel),
               GET_SHIM_DMA(CURR_BD, mm2s_status, channel),
               GET_SHIM_DMA(STATUS, mm2s_status, channel),
               GET_SHIM_DMA(STALLED, mm2s_status, channel));
  }

  xil_printf("Descriptors:\r\n");
  for (int bd = 0; bd < 16; bd++) {
    uint64_t bd_addr_a = in32(tileAddr + REG_SHIM_DMA_BD_ADDR(bd));
    uint32_t dma_bd_buffer_length = in32(tileAddr + REG_SHIM_DMA_BD_BUF_LEN(bd));
    uint32_t dma_bd_control = in32(tileAddr + REG_SHIM_DMA_BD_CTRL(bd));

    xil_printf("[%02d] ", bd);
    if (dma_bd_control & SHIM_DMA_BD_CTRL_VALID)
      xil_printf("valid ");

    int words_to_transfer = dma_bd_buffer_length;
    uint64_t base_address =
        (uint64_t)bd_addr_a + ((uint64_t)((dma_bd_control >> 16) & 0xFFFF) << 32);
    xil_printf("   Transferring %d 32 bit words to/from %08lX\n\r",
               words_to_transfer, base_address);

    int use_next_bd = ((dma_bd_control >> 15) & 0x1);
    int next_bd = ((dma_bd_control >> 11) & 0xF);
    int lockID = ((dma_bd_control >> 7) & 0xF);
    int enable_lock_release = ((dma_bd_control >> 6) & 0x1);
    int lock_release_val = ((dma_bd_control >> 5) & 0x1);
    int use_release_val = ((dma_bd_control >> 4) & 0x1);
    int enable_lock_acquire = ((dma_bd_control >> 3) & 0x1);
    int lock_acquire_val = ((dma_bd_control >> 2) & 0x1);
    int use_acquire_val = ((dma_bd_control >> 1) & 0x1);

    xil_printf("next=%d, use_next=%d ", next_bd, use_next_bd);
    xil_printf("lock: %d, acq(en: %d, val: %d, use: %d), rel(en: %d, val: %d, "
               "use: %d)\r\n",
               lockID, enable_lock_acquire, lock_acquire_val, use_acquire_val,
               enable_lock_release, lock_release_val, use_release_val);
  }
}

/// Print the status of a core represented by the given tile, at the given
/// coordinates.
void mlir_aie_print_tile_status(int col, int row) {
  uint32_t trace_status;
  uint32_t status, coreTimerLow, PC, LR, SP, locks, R0, R4;
  uint64_t tileAddr = getTileAddr(col, row);

  status = in32(tileAddr + REG_AIE_CORE_STATUS);
  coreTimerLow = in32(tileAddr + 0x0340F8);
  PC = in32(tileAddr + 0x00030280);
  LR = in32(tileAddr + 0x000302B0);
  SP = in32(tileAddr + 0x000302A0);
  locks = in32(tileAddr + 0x0001EF00);
  trace_status = in32(tileAddr + 0x000140D8);
  R0 = in32(tileAddr + 0x00030000);
  R4 = in32(tileAddr + 0x00030040);

  xil_printf("Core [%d, %d] addr is 0x%08lX\n\r", col, row, tileAddr);
  xil_printf(
      "Core [%d, %d] status is 0x%08X, timer is %u, PC is 0x%08X, locks are "
      "%08X, LR is %08X, SP is %08X, R0 is %08X,R4 is %08X\n\r",
      col, row, status, coreTimerLow, PC, locks, LR, SP, R0, R4);
  xil_printf("Core [%d, %d] trace status is %08X\n\r", col, row, trace_status);

  for (int lock = 0; lock < 16; lock++) {
    uint32_t two_bits = (locks >> (lock * 2)) & 0x3;
    if (two_bits) {
      xil_printf("Lock %d: ", lock);
      uint32_t acquired = two_bits & 0x1;
      uint32_t value = two_bits & 0x2;
      if (acquired)
        xil_printf("Acquired ");
      xil_printf(value ? "1" : "0");
      xil_printf("\n\r");
    }
  }

  const char *core_status_strings[] = {"Enabled",
                                       "In Reset",
                                       "Memory Stall S",
                                       "Memory Stall W",
                                       "Memory Stall N",
                                       "Memory Stall E",
                                       "Lock Stall S",
                                       "Lock Stall W",
                                       "Lock Stall N",
                                       "Lock Stall E",
                                       "Stream Stall S",
                                       "Stream Stall W",
                                       "Stream Stall N",
                                       "Stream Stall E",
                                       "Cascade Stall Master",
                                       "Cascade Stall Slave",
                                       "Debug Halt",
                                       "ECC Error",
                                       "ECC Scrubbing",
                                       "Error Halt",
                                       "Core Done"};
  xil_printf("Core Status: ");
  for (int i = 0; i <= 20; i++) {
    if ((status >> i) & 0x1)
      xil_printf("%s ", core_status_strings[i]);
  }
  xil_printf("\r\n");
}

#endif

int xaie_shim_dma_wait_idle(uint64_t TileAddr, int direction, int channel) {
  uint32_t shimDMAchannel = channel;
  uint32_t status_register_offset;
  uint32_t status_mask_shift;
  if (channel == 0) {
    status_mask_shift = 0;
  } else {
    status_mask_shift = 2;
  }
  if (direction == SHIM_DMA_S2MM) {
    shimDMAchannel += XAIEDMA_SHIM_CHNUM_S2MM0;
    status_register_offset = 0x1d160;
  } else {
    shimDMAchannel += XAIEDMA_SHIM_CHNUM_MM2S0;
    status_register_offset = 0x1d164;
  }

  // Will timeout if shim is busy
  uint32_t timeout_count = 0;
  uint32_t timeout_val = 100;
  while ((in32(TileAddr + status_register_offset) >> status_mask_shift) &
         0b11) {
    if (timeout_count >= timeout_val) {
      air_printf("[WARNING] xaie_shim_dma_wait_idle timed out\r\n");
      return 1;
    }
    timeout_count++;
  }

  return 0;
}

uint32_t xaie_shim_dma_get_outstanding(uint64_t TileAddr, int direction,
                                       int channel) {
  uint32_t shimDMAchannel = channel;
  uint32_t status_register_offset;
  uint32_t start_queue_size_mask_shift;
  if (channel == 0) {
    start_queue_size_mask_shift = 6;
  } else {
    start_queue_size_mask_shift = 9;
  }
  if (direction == SHIM_DMA_S2MM) {
    shimDMAchannel += XAIEDMA_SHIM_CHNUM_S2MM0;
    status_register_offset = 0x1d160;
  } else {
    shimDMAchannel += XAIEDMA_SHIM_CHNUM_MM2S0;
    status_register_offset = 0x1d164;
  }
  uint32_t outstanding =
      (in32(TileAddr + status_register_offset) >> start_queue_size_mask_shift) &
      0b111;
  return outstanding;
}

//// GLOBAL for shim DMAs mapped to the controller
// uint16_t mappedShimDMA[2] = {0};
//// GLOBAL for round-robin bd locations
// uint32_t last_bd[4][2] = {0};
uint32_t last_bd[8] = {0};

int xaie_shim_dma_push_bd(uint64_t TileAddr, int direction, int channel,
                          uint16_t col, uint64_t addr, uint32_t len) {
  uint32_t shimDMAchannel = channel; // Need
  uint32_t status_register_offset;
  uint32_t status_mask_shift;
  uint32_t control_register_offset;
  uint32_t start_queue_register_offset;
  uint32_t start_queue_size_mask_shift;

  if (direction == SHIM_DMA_S2MM) {
    shimDMAchannel += XAIEDMA_SHIM_CHNUM_S2MM0;
    status_register_offset = 0x1d160;
    if (channel == 0) {
      status_mask_shift = 0;
      control_register_offset = 0x1d140;
      start_queue_register_offset = 0x1d144;
      start_queue_size_mask_shift = 6;
    } else {
      status_mask_shift = 2;
      control_register_offset = 0x1d148;
      start_queue_register_offset = 0x1d14c;
      start_queue_size_mask_shift = 9;
    }
    air_printf("\n\r  S2MM Shim DMA %d start channel %d\n\r", col,
               shimDMAchannel);
    // air_printf("\n\r  S2MM Shim DMA %d start channel %d\n\r",
    // mappedShimDMA[dma], shimDMAchannel);
  } else {
    shimDMAchannel += XAIEDMA_SHIM_CHNUM_MM2S0;
    status_register_offset = 0x1d164;
    if (channel == 0) {
      status_mask_shift = 0;
      control_register_offset = 0x1d150;
      start_queue_register_offset = 0x1d154;
      start_queue_size_mask_shift = 6;
    } else {
      status_mask_shift = 2;
      control_register_offset = 0x1d158;
      start_queue_register_offset = 0x1d15c;
      start_queue_size_mask_shift = 9;
    }
    air_printf("\n\r  MM2S Shim DMA %d start channel %d\n\r", col,
               shimDMAchannel);
    // air_printf("\n\r  MM2S Shim DMA %d start channel %d\n\r",
    // mappedShimDMA[dma], shimDMAchannel);
  }

  uint32_t start_bd = 4 * shimDMAchannel; // shimDMAchannel<<2;
  uint32_t outstanding =
      (in32(TileAddr + status_register_offset) >> start_queue_size_mask_shift) &
      0b111;
  // If outstanding >=4, we're in trouble!!!!
  // Theoretically this should never occur due to check in do_packet_nd_memcpy
  if (outstanding >= 4) { // NOTE had this at 3? // What is proper 'stalled'
                          // threshold? if (outstanding >=4)
    air_printf("\n\r *** BD OVERFLOW in shimDMA channel %d *** \n\r",
               shimDMAchannel);
    bool waiting = true;
    while (waiting) {
      outstanding = (in32(TileAddr + status_register_offset) >>
                     start_queue_size_mask_shift) &
                    0b111;
      waiting = (outstanding > 3); // NOTE maybe >= 3
      air_printf("*** Stalled in shimDMA channel %d outstanding = %d *** \n\r",
                 shimDMAchannel, outstanding + 1);
    } // WARNING this can lead to an endless loop
  }
  air_printf("Outstanding pre : %d\n\r", outstanding);
  // uint32_t bd = start_bd+outstanding;// + 0; // HACK
  int slot = channel;
  slot += ((col % 2) == 1) ? 4 : 0;
  if (direction == SHIM_DMA_S2MM)
    slot += XAIEDMA_SHIM_CHNUM_S2MM0;
  else
    slot += XAIEDMA_SHIM_CHNUM_MM2S0;
  uint32_t bd = start_bd + last_bd[slot];
  last_bd[slot] = (last_bd[slot] == 3) ? 0 : last_bd[slot] + 1;

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
  uint32_t precheck_status =
      (in32(TileAddr + status_register_offset) >> status_mask_shift) & 0b11;
  if (precheck_status == 0b00) {
    // Stream traffic can run, we can issue AXI-MM, and the channel is enabled
    air_printf("Enabling shim DMA [%u] channel %u\r\n", col, channel);
    out32(TileAddr + control_register_offset, 0x1);
  }

  // Now push into the queue
  air_printf("Pushing bd %u into 0x%lx\r\n", bd,
             TileAddr + start_queue_register_offset);
  out32(TileAddr + start_queue_register_offset, bd);

#if CHATTY
  outstanding =
      (in32(TileAddr + status_register_offset) >> start_queue_size_mask_shift) &
      0b111;
  air_printf("Outstanding post: %d\n\r", outstanding);
  air_printf("bd pushed as bd %u\r\n", bd);

  if (direction == SHIM_DMA_S2MM) {
    air_printf("  End of S2MM Shim DMA %d start channel %d\n\r", col,
               shimDMAchannel);
  } else {
    air_printf("  End of MM2S Shim DMA %d start channel %d\n\r", col,
               shimDMAchannel);
  }
#endif

  return 1;
}

int xaie_lock_release(uint16_t col, uint16_t row, uint32_t lock_id, uint32_t val) {
  uint64_t Addr = getTileAddr(col, row);
  uint64_t LockOfst = 0x0001E020;
  if (row != 0)
    LockOfst = 0x0001E020 + 0x10 * (val & 0x1);
  else {
    switch (col % 4) {
    case 0:
    case 1:
      LockOfst = 0x00014020 + 0x10 * (val & 0x1);
      break;
    default:
      LockOfst = 0x00014020 + 0x10 * (val & 0x1);
      break;
    }
  }
  maskpoll32(Addr + LockOfst + 0x80 * lock_id, 0x1, 0x1, 0);
  // XAieTile_LockRelease(tile, lock_id, val, 0);
  return 1;
}

int xaie_lock_acquire_nb(uint16_t col, uint16_t row, uint32_t lock_id, uint32_t val) {
  uint64_t Addr = getTileAddr(col, row);
  uint64_t LockOfst = 0x0001E060;
  if (row != 0)
    LockOfst = 0x0001E060 + 0x10 * (val & 0x1);
  else {
    switch (col % 4) {
    case 0:
    case 1:
      LockOfst = 0x00014060 + 0x10 * (val & 0x1);
      break;
    default:
      LockOfst = 0x00014060 + 0x10 * (val & 0x1);
      break;
    }
  }
  uint8_t lock_ret = 0;
  uint32_t loop = 0;
  while ((!lock_ret) && (loop < 512)) {
    lock_ret = maskpoll32(Addr + LockOfst + 0x80 * lock_id, 0x1, 0x1, 100);
    // lock_ret = XAieTile_LockAcquire(tile, lock_id, val, 10000);
    loop++;
  }
  if (loop == 512) {
    air_printf("Acquire [%d, %d, %d] value %d time-out\n\r", col, row, lock_id,
               val);
    return 0;
  }
  return 1;
}

#ifdef ARM_CONTROLLER

void xaie_array_reset() {

  // Getting a pointer to NPI
  auto *npib = (volatile uint32_t *)(NPI_BASE);

  // Performing array reset sequence
  air_printf("Starting array reset\r\n");

  // Unlocking NPI
  npib[NPI_LOCK_REG >> 2] = 0xF9E8D7C6;

  // Performing reset
  npib[NPI_MASK_REG >> 2] = 0x04000000;
  npib[NPI_VAL_REG >> 2] = 0x040381B1;
  npib[NPI_MASK_REG >> 2] = 0x04000000;
  npib[NPI_VAL_REG >> 2] = 0x000381B1;

  // Locking NPI
  npib[NPI_LOCK_REG >> 2] = 0x12341234;
  air_printf("Done with array reset\r\n");
}

// This should be called after enabling the proper
// shims to be reset via the mask
void xaie_strobe_shim_reset() {

  // Getting a pointer to NPI
  auto *npib = (volatile uint32_t *)(NPI_BASE);

  air_printf("Starting shim reset\r\n");

  // Unlocking NPI
  npib[NPI_LOCK_REG >> 2] = 0xF9E8D7C6;

  // Performing reset
  npib[NPI_MASK_REG >> 2] = 0x08000000;
  npib[NPI_VAL_REG >> 2] = 0x080381B1;
  npib[NPI_MASK_REG >> 2] = 0x08000000;
  npib[NPI_VAL_REG >> 2] = 0x000381B1;

  // Locking NPI
  npib[NPI_LOCK_REG >> 2] = 0x12341234;
  air_printf("Done with shim reset\r\n");
}

#endif

/*
  Reset all of the ME tiles in the specified column
*/
static void aie_reset_column(uint16_t col_idx) {
  air_printf("Resetting column %u\r\n", col_idx);
  out32(getTileAddr(col_idx, 0) + REG_AIE_COL_RESET, 1); // 1 == ResetEnable
  out32(getTileAddr(col_idx, 0) + REG_AIE_COL_RESET, 0); // 0 == ResetDisable
}

/*
  Invalidate all BDs by writing to their buffer control register
*/
void xaie_shim_dma_init(uint16_t col) {
  uint64_t tileAddr = getTileAddr(col, 0);
  // Disable all channels
  for (uint8_t ch = 0; ch < 4; ch++) {
    out32(tileAddr + REG_SHIM_DMA_CTRL(ch), 0);
  }
  for (uint8_t bd = 0; bd < SHIM_DMA_NUM_BDS; bd++) {
    out32(tileAddr + REG_SHIM_DMA_BD_CTRL(bd), 0);
  }
}

/*
  Reset a shim tile
*/
void xaie_reset_shim(uint16_t col) {
  air_printf("Resetting shim tile %u\r\n", col);
  for (uint16_t c = 0; c < XAIE_NUM_COLS; c++) {
    uint32_t val = (c == col) ? 1 : 0;
    out32(getTileAddr(c, 0) + REG_SHIM_RESET_ENA, val);
  }

  xaie_strobe_shim_reset();

  out32(getTileAddr(col, 0) + REG_SHIM_RESET_ENA, 0);
}

void xaie_device_init(void) {

  air_printf("Initializing device...\r\n");

  // First, resetting the entire device
  xaie_array_reset();

#ifdef ARM_CONTROLLER
  int err = mlir_aie_reinit_device(_xaie);
  if (err)
    xil_printf("ERROR initializing device.\n\r");
#endif

  for (int c = 0; c < NUM_SHIM_DMAS; c++) {
    xaie_shim_dma_init(shim_dma_cols[c]);
  }

  // Turning the shim_reset_enable bit low for every column so they don't get
  // reset when we perform a global shim reset
  for (int col = 0; col < XAIE_NUM_COLS; col++) {
    out32(getTileAddr(col, 0) + 0x0003604C, 0);
  }
}

// Initialize one segment with lower left corner at (col_start, row_start)
void xaie_segment_init(uint16_t start_col, uint16_t num_cols,
                       uint16_t start_row, uint16_t num_rows) {
  HerdCfgInst.col_start = start_col;
  HerdCfgInst.num_cols = num_cols;
  HerdCfgInst.row_start = start_row;
  HerdCfgInst.num_rows = num_rows;
#ifdef ARM_CONTROLLER

  // Performing the shim reset
  air_printf("Performing shim reset; start_col=%u num_cols=%u\r\n", start_col,
             num_cols);
  for (uint16_t c = start_col; c < start_col + num_cols; c++) {
    out32(getTileAddr(c, 0) + REG_SHIM_RESET_ENA, 1);
  }

  xaie_strobe_shim_reset();

  for (uint16_t c = start_col; c < start_col + num_cols; c++) {
    out32(getTileAddr(c, 0) + REG_SHIM_RESET_ENA, 0);
  }

  // Performing the column reset
  air_printf("Performing col reset\r\n");
  for (uint16_t c = start_col; c < start_col + num_cols; c++)
    aie_reset_column(c);

#endif
}

const uint64_t shmem_base = 0x020100000000ULL;

/*
  Put a tile into reset
*/
void aie_tile_reset(int col, int row) {
  uint64_t tileAddr = getTileAddr(col, row);
  out32(tileAddr + REG_AIE_CORE_CTL, REG_AIE_CORE_CTL_RESET);
}

/*
  Take a tile out of reset
*/
void aie_tile_enable(int col, int row) {
  uint64_t tileAddr = getTileAddr(col, row);
  out32(tileAddr + REG_AIE_CORE_CTL, REG_AIE_CORE_CTL_ENABLE);
}

bool setup;

uint64_t get_base_address(void) { return shmem_base; }

void lock_uart(uint32_t id) {
// ARM has seperate UART so doesn't use lock
#ifndef ARM_CONTROLLER
  XMutex_Lock(xmutex_ptr, XPAR_MUTEX_0_UART_LOCK, id);
#endif
}

void unlock_uart(uint32_t id) {
// ARM has seperate UART so doesn't use lock
#ifndef ARM_CONTROLLER
  XMutex_Unlock(xmutex_ptr, XPAR_MUTEX_0_UART_LOCK, id);
#endif
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

void handle_packet_device_initialize(hsa_agent_dispatch_packet_t *pkt) {
  packet_set_active(pkt, true);
  xaie_device_init();
}

void handle_packet_segment_initialize(hsa_agent_dispatch_packet_t *pkt) {
  setup = true;
  packet_set_active(pkt, true);

  // Address mode here is absolute range
  if (((pkt->arg[0] >> 48) & 0xf) == AIR_ADDRESS_ABSOLUTE_RANGE) {
    uint16_t start_row = (pkt->arg[0] >> 16) & 0xff;
    uint16_t num_rows = (pkt->arg[0] >> 24) & 0xff;
    uint16_t start_col = (pkt->arg[0] >> 32) & 0xff;
    uint16_t num_cols = (pkt->arg[0] >> 40) & 0xff;

    uint16_t segment_id = pkt->arg[1] & 0xffff;

    // TODO more checks on segment dimensions
    if (start_row == 0)
      start_row++;
    xaie_segment_init(start_col, num_cols, start_row, num_rows);
    air_printf("Initialized segment %d at (%d, %d) of size (%d,%d)\r\n",
               segment_id, start_col, start_row, num_cols, num_rows);
  } else {
    air_printf("Unsupported address type 0x%04X for segment initialize\r\n",
               (pkt->arg[0] >> 48) & 0xf);
  }
}

void handle_packet_get_capabilities(hsa_agent_dispatch_packet_t *pkt, uint32_t mb_id) {
  // packet is in active phase
  packet_set_active(pkt, true);
  uint64_t *addr = (uint64_t *)(pkt->return_address);

  lock_uart(mb_id);
  air_printf("Writing to 0x%llx\n\r", (uint64_t)addr);
  unlock_uart(mb_id);
  // We now write a capabilities structure to the address we were just passed
  // We've already done this once - should we just cache the results?
#if defined(ARM_CONTROLLER)
  int user1 = 1;
  int user2 = 0;

#else
  pvr_t pvr;
  microblaze_get_pvr(&pvr);
  int user1 = MICROBLAZE_PVR_USER1(pvr);
  int user2 = MICROBLAZE_PVR_USER2(pvr);
#endif
  addr[0] = (uint64_t)mb_id;        // region id
  addr[1] = (uint64_t)user1;        // num regions
  addr[2] = (uint64_t)(user2 >> 8); // region controller firmware version
  addr[3] = 16L;                    // cores per region
  addr[4] = 32768L;                 // Total L1 data memory per core
  addr[5] = 8L;                     // Number of L1 data memory banks
  addr[6] = 16384L;                 // L1 program memory per core
  addr[7] = 0L;                     // L2 data memory per region
}

void handle_packet_get_info(hsa_agent_dispatch_packet_t *pkt, uint32_t mb_id) {
  // packet is in active phase
  packet_set_active(pkt, true);
  uint64_t attribute = (pkt->arg[0]);
  uint64_t *addr =
      (uint64_t *)(&pkt->return_address); // FIXME when we can use a VA

#if defined(ARM_CONTROLLER)
  int user1 = 1;
  int user2 = 0;
#else
  pvr_t pvr;
  microblaze_get_pvr(&pvr);
  int user1 = MICROBLAZE_PVR_USER1(pvr);
  int user2 = MICROBLAZE_PVR_USER2(pvr);
#endif
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
    *addr = (uint64_t)mb_id; // region id
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

#ifdef ARM_CONTROLLER

/* Hardcoded . If the platform memory map changes 
these will have to change */
uint64_t ernic_0_base = 0x0000020100080000UL;
uint64_t ernic_1_base = 0x00000201000C0000UL;

/* Used for the device controller to poll on an 
incoming RDMA SEND, and copy the payload to some 
buffer in memory */
void handle_packet_rdma_post_recv(hsa_agent_dispatch_packet_t *pkt) {

  // Need to do this before processing the packet
  packet_set_active(pkt,true);

  // Parsing the packet
  uint64_t local_physical_address = pkt->arg[0];
  uint8_t  qpid                 = pkt->arg[1] & 0xFF;
  //uint8_t  tag                  = (pkt->arg[1] >> 8)  & 0xFF; // Currently don't use tag
  uint32_t length               = (pkt->arg[1] >> 16) & 0xFFFF;
  uint8_t  ernic_sel            = (pkt->arg[1] >> 48) & 0xFF;

  // Pointing to the proper ERNIC 
  volatile uint32_t *ernic_csr = NULL;
  if(ernic_sel == 0) {
    ernic_csr = (volatile uint32_t *)ernic_0_base;
  }
  else {
    ernic_csr = (volatile uint32_t *)ernic_1_base;
  }

  // Can print without grabbing the lock because running on the ARM in the ARM
  // which has sole use of the UART (BPs use JTAG UART)
  air_printf("Packet:\r\n");
  air_printf("\tlocal_physical_address: 0x%lx\r\n", local_physical_address);
  air_printf("\tlength: 0x%x\r\n", length);
  air_printf("\tqpid: 0x%x\r\n", qpid);
  air_printf("\ternic_sel: 0x%x\r\n", ernic_sel);

  // Determine base address of the RQ to read the RQE
  uint32_t rq_base_address_low    = ernic_csr[ERNIC_QP_ADDR(qpid, RQBAi)];
  uint32_t rq_base_address_high   = ernic_csr[ERNIC_QP_ADDR(qpid, RQBAMSBi)];
  uint64_t rq_base_address        = (((uint64_t)rq_base_address_high) << 32) | ((uint64_t)rq_base_address_low);
  air_printf("\trq_base_address: 0x%lx\r\n", rq_base_address);

  // Wait for RQPIDB to be greater than RQCIDB
  uint32_t rq_ci_db = ernic_csr[ERNIC_QP_ADDR(qpid, RQCIi)];
  uint32_t rq_pi_db = ernic_csr[ERNIC_QP_ADDR(qpid, STATRQPIDBi)];
  air_printf("Polling onon rq_pi_db to be greater than 0x%x. Read: 0x%x\r\n", rq_ci_db, rq_pi_db);
  while(rq_pi_db <= rq_ci_db) {
      rq_pi_db = ernic_csr[ERNIC_QP_ADDR(qpid, STATRQPIDBi)];
  }
  air_printf("Observed aSEND. Copying to local buffer\r\n");

  // Copy what RQ PIDB is pointing at to local_physical_address
  void *rqe = (void *)(rq_base_address + (rq_pi_db - 1) * RQE_SIZE);
  air_printf("rqe is at %p and copying to 0x%lx\r\n", rqe, local_physical_address);
  memcpy((size_t *)local_physical_address, (size_t *)rqe, length);

  // Increment RQ CIDB so it knows that it can overwrite it
  ernic_csr[ERNIC_QP_ADDR(qpid, RQCIi)] = rq_ci_db + 1;
}

/* Used for the device controller to post an RDMA WQE
to the ERNIC. This can be used to initiate either
one-sided or two-sided communication. */
void handle_packet_rdma_post_wqe(hsa_agent_dispatch_packet_t *pkt) {

  // Need to do this before processing the packet
  packet_set_active(pkt, true);

  // Parsing the packet
  uint64_t remote_virtual_address = pkt->arg[0];
  uint64_t local_physical_address = pkt->arg[1];
  uint32_t length                 = pkt->arg[2] & 0xFFFF;
  uint8_t  op                     = (pkt->arg[2] >> 32) & 0xFF;
  uint8_t  key                    = (pkt->arg[2] >> 40) & 0xFF;
  uint8_t  qpid                   = (pkt->arg[2] >> 48) & 0xFF;
  uint8_t  ernic_sel              = (pkt->arg[2] >> 56) & 0xFF;

  // Pointing to the proper ERNIC 
  volatile uint32_t *ernic_csr = NULL;
  if(ernic_sel == 0) {
    ernic_csr = (volatile uint32_t *)ernic_0_base;
  }
  else {
    ernic_csr = (volatile uint32_t *)ernic_1_base;
  }

  // Can print without grabbing the lock because running on the ARM in the ARM
  // which has sole use of the UART (BPs use JTAG UART)
  air_printf("Packet:\r\n");
  air_printf("\tremote_virtual_address: 0x%lx\r\n", remote_virtual_address);
  air_printf("\tlocal_physical_address: 0x%lx\r\n", local_physical_address);
  air_printf("\tlength: 0x%x\r\n", length);
  air_printf("\top: 0x%x\r\n", op);
  air_printf("\tkey: 0x%x\r\n", key);
  air_printf("\tqpid: 0x%x\r\n", qpid);
  air_printf("\ternic_sel: 0x%x\r\n", ernic_sel);

  uint32_t sq_base_address_low    = ernic_csr[ERNIC_QP_ADDR(qpid, SQBAi)];
  uint32_t sq_base_address_high   = ernic_csr[ERNIC_QP_ADDR(qpid, SQBAMSBi)];
  uint64_t sq_base_address        = (((uint64_t)sq_base_address_high) << 32) | ((uint64_t)sq_base_address_low);
  air_printf("\tsq_base_address: 0x%lx\r\n", sq_base_address);

  // Read the doorbell to determine where to put the WQE
  uint32_t sq_pi_db = ernic_csr[ERNIC_QP_ADDR(qpid, SQPIi)];
  air_printf("\tsq_pi_db: 0x%x\r\n", sq_pi_db);

  // Write the WQE to the SQ
  struct pcie_ernic_wqe *wqe = &(((struct pcie_ernic_wqe *)(sq_base_address))[sq_pi_db]);
  air_printf("Starting writing WQE to %p\r\n", wqe);
  wqe->wrid           = 0xe0a6 & 0x0000FFFF; // Just hardcoding the ID for now
  wqe->laddr_lo       = (uint32_t)(local_physical_address & 0x00000000FFFFFFFF);
  wqe->laddr_hi       = (uint32_t)(local_physical_address >> 32);
  wqe->length         = length;
  wqe->op             = op & 0x000000FF;
  wqe->offset_lo      = (uint32_t)(remote_virtual_address & 0x00000000FFFFFFFF);
  wqe->offset_hi      = (uint32_t)(remote_virtual_address >> 32);
  wqe->rtag           = key;
  wqe->send_data_dw_0 = 0;
  wqe->send_data_dw_1 = 0;
  wqe->send_data_dw_2 = 0;
  wqe->send_data_dw_3 = 0;
  wqe->immdt_data     = 0;
  wqe->reserved_1     = 0;
  wqe->reserved_2     = 0;
  wqe->reserved_3     = 0;
  air_printf("Done writing WQE\r\n");

  // Ring the doorbell
  ernic_csr[ERNIC_QP_ADDR(qpid, SQPIi)] = sq_pi_db + 1;

  // Poll on the completion
  uint32_t cq_ci_db = ernic_csr[ERNIC_QP_ADDR(qpid, CQHEADi)];
  while(cq_ci_db != (sq_pi_db + 1) ) {
      air_printf("Polling on on CQHEADi to be 0x%x. Read: 0x%x\r\n", sq_pi_db + 1, cq_ci_db);
      cq_ci_db = ernic_csr[ERNIC_QP_ADDR(qpid, CQHEADi)];
  }
}
#endif

void handle_packet_read_write_aie_reg32(hsa_agent_dispatch_packet_t *pkt, bool is_write) {

  packet_set_active(pkt, true);
  uint64_t address = pkt->arg[0];
  uint32_t value = pkt->arg[1] & 0xFFFFFFFF;

  if (address > AIE_CSR_SIZE) {
    printf("[ERROR] read32/write32 packets provided address of size 0x%lx. "
           "Window is only 4GB\n",
           address);
  }

  if (is_write) {
    out32(AIE_BASE + address, value);
  } else {
    pkt->arg[2] = in32(AIE_BASE + address);
  }
  packet_set_active(pkt, false);
  --pkt->completion_signal.handle;
}

/*
 Load an AIRBIN from DRAM into tiles

 The AIRBIN is loaded into device memory (usually from the host)
 arg[0]: a table specifying an offset and length of each section. The parameter
 is a user virtual address.  The offsets in the table are relative to the start
 of the table.

 A CDMA descriptor chain is created to load these sections into tile
 memory. This is necessary because the tile memory is not directly accessible
 by the host.
*/
void handle_packet_load_airbin(hsa_agent_dispatch_packet_t *pkt) {
  uint32_t idx = 0;
  uint64_t src, dest, tile;
  uint16_t col, row, starting_col;
  uint16_t start_col = 0xFFFF;
  uint16_t start_row = 0xFFFF;
  uint16_t end_col = 0;
  uint16_t end_row = 0;
  uint64_t table_va = pkt->arg[0];
  uint16_t target_col = pkt->arg[1] & 0xFFFF;
  uint64_t entry_iter = 0;
  airbin_table_entry *entry =
      (airbin_table_entry *)translate_virt_to_phys(table_va);

  air_printf("Loading AIRBIN to col %u from 0x%lx\r\n", target_col,
             (uint64_t)entry);
  packet_set_active(pkt, true);

  // reset our shim DMA
  xaie_shim_dma_init(target_col);

  cdma_sg_init();

  // parse airbin table and create CDMA descriptor chain to load the data
  while (entry->size) {
    col = GET_COLUMN(entry->addr);

    // Marking the first column
    if (entry_iter == 0) {
      starting_col = col;
      entry_iter++;
    }

    // Shifting the design over to the target column + offset
    col = col - starting_col + target_col;

    row = GET_ROW(entry->addr);
    tile = getTileAddr(col, row);
    src = translate_virt_to_phys(table_va + entry->offset);
    dest = tile | (entry->addr & 0xFFFFFF);
    air_printf("Entry: src=0x%lx dest=0x%lx size=%x\r\n", src, dest,
               entry->size);
    cdma_sg_set(idx++, dest, src, entry->size);

    if (col < start_col)
      start_col = col;
    if (row < start_row)
      start_row = row;
    if (col > end_col)
      end_col = col;
    if (row > end_row)
      end_row = row;

    entry++;
  }

  // roll index back so it refers to the last valid descriptor
  idx--;

  // reset shim tile
  xaie_reset_shim(start_col);

  // put AIE cores in reset
  for (uint16_t c = start_col; c <= end_col; c++) {
    for (uint16_t r = start_row; r <= end_row; r++) {
      // row 0 is reserved for addressing components within the shim
      if (r == 0)
        continue;
      air_printf("Putting core (%u, %u) in reset\r\n", c, r);
      out32(getTileAddr(c, r) + REG_AIE_CORE_CTL, REG_AIE_CORE_CTL_RESET);
    }

    // reset the column
    aie_reset_column(c);
  }
  
  // copy
  uint32_t ret = cdma_sg_start_sync(0, idx);
  if (ret) {
    printf("Error 0x%x in CDMA\r\n", ret);
    return;
  }
  air_printf("DMA done\r\n");

  // start the AIE cores
  for (uint16_t c = start_col; c <= end_col; c++) {
    for (uint16_t r = start_row; r <= end_row; r++) {
      // row 0 is reserved for addressing components within the shim
      if (r == 0)
        continue;

      // reset locks
      // TODO: this can be parallelized
      for (int l = 0; l < 16; l++) {
        maskpoll32(getTileAddr(c, r) + REG_AIE_LOCK_RELEASE_0(l), 0x1, 0x1, 0);
      }

      // bring cores out of reset
      air_printf("Enabling core (%u, %u)\r\n", c, r);
      out32(getTileAddr(c, r) + REG_AIE_CORE_CTL, REG_AIE_CORE_CTL_ENABLE);
    }
  }

  air_printf("AIE started\r\n");
}

void handle_packet_sg_cdma(hsa_agent_dispatch_packet_t *pkt) {
  // packet is in active phase
  packet_set_active(pkt, true);
  volatile uint32_t *cdmab = (volatile uint32_t *)(CDMA_BASE);
  uint32_t start_row = (pkt->arg[3] >> 0) & 0xff;
  uint32_t num_rows = (pkt->arg[3] >> 8) & 0xff;
  uint32_t start_col = (pkt->arg[3] >> 16) & 0xff;
  uint32_t num_cols = (pkt->arg[3] >> 24) & 0xff;
  for (uint c = start_col; c < start_col + num_cols; c++) {
    for (uint r = start_row; r < start_row + num_rows; r++) {
      out32(getTileAddr(c, r) + 0x00032000, 0x2);
      air_printf("Done resetting col %d row %d.\n\r", c, r);
    }
    air_printf("Resetting column %d.\n\r", c);
    aie_reset_column(c);
  }
  air_printf("CDMA reset.\n\r");
  cdmab[0] |= 0x4;
  cdmab[0] &= 0x4;
  while (cdmab[0] & 0x4)
    ;
  air_printf("CDMA start.\n\r");
  uint64_t daddr = (pkt->arg[0]);
  uint64_t saddr = (pkt->arg[1]);
  uint32_t bytes = (pkt->arg[2]);
  air_printf("CMDA daddr 0x%016lx saddr 0x%016lx\n\r", daddr, saddr);
  cdmab[0] = 0x0;          // unset SG mode
  if (bytes >= 0xffffff) { // SG
    cdmab[0] = 0x8;        // set SG mode
    cdmab[2] = saddr & 0xffffffff;
    cdmab[3] = saddr >> 32;
    cdmab[5] = daddr >> 32;
    cdmab[4] = daddr & 0xffffffff;
  } else {
    cdmab[6] = saddr & 0xffffffff;
    cdmab[7] = saddr >> 32;
    cdmab[8] = daddr & 0xffffffff;
    cdmab[9] = daddr >> 32;
    cdmab[10] = bytes;
  }
  int cnt = 100;
  while (!(cdmab[1] & 2) && cnt--)
    air_printf("SG CDMA wait... %x\n\r", cdmab[1]);
  for (uint c = start_col; c < start_col + num_cols; c++) {
    for (uint r = start_row; r <= start_row + num_rows; r++) {
      for (int l = 0; l < 16; l++)
        maskpoll32(getTileAddr(c, r) + REG_AIE_LOCK_RELEASE_0(l), 0x1, 0x1, 0);
      out32(getTileAddr(c, r) + REG_AIE_CORE_CTL, REG_AIE_CORE_CTL_ENABLE);
    }
  }
  air_printf("CDMA done!\n\r");
}

void handle_packet_cdma(hsa_agent_dispatch_packet_t *pkt) {
  // packet is in active phase
  packet_set_active(pkt, true);
  uint32_t start_row = (pkt->arg[3] >> 0) & 0xff;
  uint32_t num_rows = (pkt->arg[3] >> 8) & 0xff;
  uint32_t start_col = (pkt->arg[3] >> 16) & 0xff;
  uint32_t num_cols = (pkt->arg[3] >> 24) & 0xff;
  uint32_t op = (pkt->arg[3] >> 32) & 0xff;
  if (op == 2) {
    for (uint c = start_col; c < start_col + num_cols; c++) {
      for (uint r = start_row; r < start_row + num_rows; r++) {
        int st = in32(getTileAddr(c, r) + REG_AIE_CORE_STATUS);
        air_printf("Status col %d row %d. 0x%x\n\r", c, r, st & 0x3);
        if ((0x3 & st) != 0x2) {
          out32(getTileAddr(c, r) + REG_AIE_CORE_CTL, REG_AIE_CORE_CTL_RESET);
          air_printf("Done resetting col %d row %d.\n\r", c, r);
        }
      }
    }
  }
  if (op == 1) {
    for (uint8_t c = start_col; c < start_col + num_cols; c++) {
      air_printf("Resetting column %u.\n\r", c);
      aie_reset_column(c);
      air_printf("Done resetting column %u.\n\r", c);
    }
  }
  volatile uint32_t *cdmab = (volatile uint32_t *)(CDMA_BASE);
  uint32_t status = cdmab[1];
  air_printf("CMDA raw %x idle %x\n\r", status, status & 2);
  uint64_t daddr = (pkt->arg[0]);
  uint64_t saddr = (pkt->arg[1]);
  uint32_t bytes = (pkt->arg[2]);
  air_printf("CMDA dst %lx src %lx\n\r", daddr, saddr);
  cdmab[0] = 0x0; // unset SG mode
  cdmab[6] = saddr & 0xffffffff;
  cdmab[7] = saddr >> 32;
  cdmab[8] = daddr & 0xffffffff;
  cdmab[9] = daddr >> 32;
  cdmab[10] = bytes;
  while (!(cdmab[1] & 2))
    air_printf("CMDA wait...\n\r");
  if (op == 2) {
    for (uint c = start_col; c < start_col + num_cols; c++) {
      for (uint r = start_row; r <= start_row + num_rows; r++) {
        for (int l = 0; l < 16; l++)
          maskpoll32(getTileAddr(c, r) + REG_AIE_LOCK_RELEASE_0(l), 0x1, 0x1,
                     0);
        out32(getTileAddr(c, r) + REG_AIE_CORE_CTL, REG_AIE_CORE_CTL_ENABLE);
      }
    }
  }
}

void handle_packet_xaie_lock(hsa_agent_dispatch_packet_t *pkt) {
  // packet is in active phase
  packet_set_active(pkt, true);

  uint32_t num_cols =
      (((pkt->arg[0] >> 48) & 0xf) == AIR_ADDRESS_HERD_RELATIVE_RANGE)
          ? ((pkt->arg[0] >> 40) & 0xff)
          : 1;
  uint32_t num_rows =
      (((pkt->arg[0] >> 48) & 0xf) == AIR_ADDRESS_HERD_RELATIVE_RANGE)
          ? ((pkt->arg[0] >> 24) & 0xff)
          : 1;
  uint32_t start_col = (pkt->arg[0] >> 32) & 0xff;
  uint32_t start_row = (pkt->arg[0] >> 16) & 0xff;
  uint32_t lock_id = pkt->arg[1];
  uint32_t acqrel = pkt->arg[2];
  uint32_t val = pkt->arg[3];
  for (uint32_t col = 0; col < num_cols; col++) {
    for (uint32_t row = 0; row < num_rows; row++) {
      if (acqrel == 0)
        xaie_lock_acquire_nb(HerdCfgInst.col_start + start_col + col,
                             HerdCfgInst.row_start + start_row + row, lock_id,
                             val);
      else
        xaie_lock_release(HerdCfgInst.col_start + start_col + col,
                          HerdCfgInst.row_start + start_row + row, lock_id,
                          val);
    }
  }
}

#ifdef ARM_CONTROLLER
void handle_packet_xaie_status(hsa_agent_dispatch_packet_t *pkt, uint32_t type) {
  xil_printf("Reading status! %d %d %d\n\r", type, pkt->arg[0], pkt->arg[1]);
  if (type == 1) {
    mlir_aie_print_shimdma_status(pkt->arg[0]);
  } else if (type == 2) {
    mlir_aie_print_dma_status(pkt->arg[0], pkt->arg[1]);
  } else if (type == 3) {
    mlir_aie_print_tile_status(pkt->arg[0], pkt->arg[1]);
  }
}
#endif

void handle_packet_hello(hsa_agent_dispatch_packet_t *pkt, uint32_t mb_id) {
  packet_set_active(pkt, true);

  uint64_t say_what = pkt->arg[0];
  lock_uart(mb_id);
  xil_printf("MB %d : HELLO %08X\n\r", mb_id, (uint32_t)say_what);
  unlock_uart(mb_id);
}

typedef struct staged_nd_memcpy_s {
  uint32_t valid;
  hsa_agent_dispatch_packet_t *pkt;
  uint64_t paddr[3];
  uint32_t index[3];
} staged_nd_memcpy_t; // about 48B therefore @ 64 slots ~3kB

uint32_t get_slot(uint16_t col, uint16_t space) {
  if (space == 2) {
    for (uint16_t i = 0; i < NUM_SHIM_DMAS; i++) {
      if (col == shim_dma_cols[i]) {
        return i * 4;
      }
    }
  } else if (space == 1) {
    for (uint16_t i = 0; i < NUM_COL_DMAS; i++) {
      if (col == col_dma_cols[i]) {
        return i * 4 + NUM_SHIM_DMAS * 4;
      }
    }
  }
  return 0;
}

// GLOBAL storage for 'in progress' ND memcpy work
// NOTE 4 slots per shim DMA
staged_nd_memcpy_t staged_nd_slot[NUM_DMAS * 4];

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

int do_packet_nd_memcpy(uint32_t slot) {
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

  air_printf(
      "%s: col=%u dir=%u chan=%u paddr=0x%llx 4d stride=%u length=%u\r\n",
      __func__, col, direction, channel, paddr_1d, stride_4d, length_4d);
  air_printf(
      "  3d stride=%u length=%u, 2d stride=%u length=%u, 1d length=%u\r\n",
      stride_3d, length_3d, stride_2d, length_2d, length_1d);

  for (; index_4d < length_4d; index_4d++) {
    for (; index_3d < length_3d; index_3d++) {
      for (; index_2d < length_2d; index_2d++) {
        outstanding = xaie_shim_dma_get_outstanding(getTileAddr(col, 0),
                                                    direction, channel);
        air_printf("\n\rND start shim DMA %u %u [%u][%u][%u] paddr=0x%llx\r\n",
                   direction, channel, index_4d, index_3d, index_2d, paddr_1d);
        if (outstanding >= 4) { // NOTE What is proper 'stalled' threshold?
          nd_dma_put_checkpoint(&a_pkt, slot, index_4d, index_3d, index_2d,
                                paddr_3d, paddr_2d, paddr_1d);
          return 1;
        } else {
          xaie_shim_dma_push_bd(getTileAddr(col, 0), direction, channel, col,
                                paddr_1d, length_1d);
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

  // Wait check idle
  int wait_idle_ret =
      xaie_shim_dma_wait_idle(getTileAddr(col, 0), direction, channel);

  // If return 1 we timed out, BDs waiting on other BDs. Put checkpoint and
  // return 1
  if (wait_idle_ret) {
    nd_dma_put_checkpoint(&a_pkt, slot, index_4d, index_3d, index_2d, paddr_3d,
                          paddr_2d, paddr_1d);
  }

  return wait_idle_ret;
}

int do_packet_memcpy(uint32_t slot) {
  if (slot >= NUM_SHIM_DMAS * 4) {
    return 0;
  } else {
    return do_packet_nd_memcpy(slot);
  }
}

int stage_packet_nd_memcpy(hsa_agent_dispatch_packet_t *pkt, uint32_t slot,
                           uint32_t memory_space) {
  air_printf("stage_packet_nd_memcpy %d\n\r", slot);
  if (staged_nd_slot[slot].valid) {
    air_printf("STALL: ND Memcpy Slot %d Busy!\n\r", slot);
    return 2;
  }
  packet_set_active(pkt, true);

  uint64_t paddr = translate_virt_to_phys(pkt->arg[1]);
  air_printf("ND_MEMCPY: Got physical address 0x%lx\r\n", paddr);

  if (memory_space == 2) {
    nd_dma_put_checkpoint(&pkt, slot, 0, 0, 0, paddr, paddr, paddr);
    staged_nd_slot[slot].valid = 1;
    return 0;
  } else {
    air_printf("NOT SUPPORTED: Cannot program memory space %d DMAs\n\r",
               memory_space);
    return 1;
  }
}

void handle_agent_dispatch_packet(amd_queue_t *amd_queue, uint32_t mb_id, int queue_id) {
  volatile uint64_t *rd_id(&amd_queue->read_dispatch_id);
  uint64_t local_read_index = amd_queue->read_dispatch_id;
  hsa_agent_dispatch_packet_t *pkt_buf(
      reinterpret_cast<hsa_agent_dispatch_packet_t*>(hsa_csr->queue_bufs[queue_id]));
  hsa_agent_dispatch_packet_t *pkt(
      &pkt_buf[local_read_index % amd_queue->hsa_queue.size]);

  int last_slot = 0;
  int max_slot = 4 * NUM_DMAS - 1;

  int num_active_packets = 1;
  int packets_processed = 0;

  do {
    // Looped back because ND memcpy failed to finish on the first try.
    // No other packet type will not finish on first try.
    if (num_active_packets > 1) {
      // NOTE assume we are coming from a stall, that's why we RR.

      // INFO:
      // 1)  check for valid staged packets that aren't the previous
      // 2a)  FOUND process packet here
      // 2b) !FOUND get next packet && check invalid
      // 3b) goto packet_op
      int slot = last_slot;
      bool stalled = true;
      bool active = false;
      do {
        slot = (slot == max_slot) ? 0 : slot + 1; // TODO better heuristic
        if (slot == last_slot)
          break;
        air_printf("RR check slot: %d\n\r", slot);
        if (staged_nd_slot[slot].valid) {
          hsa_agent_dispatch_packet_t *a_pkt = staged_nd_slot[slot].pkt;
          uint16_t channel = (a_pkt->arg[0] >> 24) & 0x00ff;
          uint16_t col = (a_pkt->arg[0] >> 32) & 0x00ff;
          // uint16_t logical_col  = (a_pkt->arg[0] >> 32) & 0x00ff;
          uint16_t direction = (a_pkt->arg[0] >> 60) & 0x000f;
          // uint16_t col          = mappedShimDMA[logical_col];
          stalled = (xaie_shim_dma_get_outstanding(getTileAddr(col, 0),
                                                   direction, channel) >= 4);
          active = packet_get_active(a_pkt);
        } else {
          stalled = true;
          active = false;
        }
        air_printf("RR slot: %d - valid %d stalled %d active %d\n\r", slot,
                   staged_nd_slot[slot].valid, stalled, active);
      } while (!staged_nd_slot[slot].valid || stalled || !active);

      if (slot == last_slot) { // Begin get next packet
        local_read_index++;
        pkt = &pkt_buf[local_read_index % amd_queue->hsa_queue.size];
        air_printf("HELLO NEW PACKET IN FLIGHT!\n\r");
        if (((pkt->header) & 0xff) != HSA_PACKET_TYPE_AGENT_DISPATCH) {
          local_read_index--;
          pkt = &pkt_buf[local_read_index % amd_queue->hsa_queue.size];
          air_printf("WARN: Found invalid HSA packet inside peek loop!\n\r");
          // TRICKY weird state where we didn't find a new packet but RR won't
          // let us retry. So advance last_slot.
          last_slot =
              (slot == max_slot) ? 0 : slot + 1; // TODO better heuristic
          continue;
        } else
          goto packet_op;
      } // End get next packet

      // FOUND ND packet process here
      last_slot = slot;
      int ret = do_packet_memcpy(slot);
      if (ret)
        continue;
      else {
        num_active_packets--;
        staged_nd_slot[slot].valid = 0;
        complete_agent_dispatch_packet(staged_nd_slot[slot].pkt);
        packets_processed++;
        continue;
      }
    }

  packet_op:
    auto op = pkt->type & 0xffff;
    // air_printf("Op is %04X\n\r",op);
    switch (op) {
    case AIR_PKT_TYPE_INVALID:
    default:
      air_printf("WARN: invalid air pkt type\n\r");
      complete_agent_dispatch_packet(pkt);
      packets_processed++;
      break;

    case AIR_PKT_TYPE_DEVICE_INITIALIZE:
      handle_packet_device_initialize(pkt);
      complete_agent_dispatch_packet(pkt);
      packets_processed++;
      break;
    case AIR_PKT_TYPE_SEGMENT_INITIALIZE:
      handle_packet_segment_initialize(pkt);
      complete_agent_dispatch_packet(pkt);
      packets_processed++;
      break;

    case AIR_PKT_TYPE_CONFIGURE:
      handle_packet_sg_cdma(pkt);
      complete_agent_dispatch_packet(pkt);
      packets_processed++;
      break;

#ifdef ARM_CONTROLLER
    case AIR_PKT_TYPE_POST_RDMA_WQE:
      handle_packet_rdma_post_wqe(pkt);
      complete_agent_dispatch_packet(pkt);
      packets_processed++;
    case AIR_PKT_TYPE_READ_AIE_REG32:
      handle_packet_read_write_aie_reg32(pkt, false);
      packets_processed++;
      break;
    case AIR_PKT_TYPE_WRITE_AIE_REG32:
      handle_packet_read_write_aie_reg32(pkt, true);
      packets_processed++;
      break;
    case AIR_PKT_TYPE_POST_RDMA_RECV:
      handle_packet_rdma_post_recv(pkt);
      complete_agent_dispatch_packet(pkt);
      packets_processed++;
      break;
#endif

    case AIR_PKT_TYPE_AIRBIN:
      // hard-coded column number for now
      handle_packet_load_airbin(pkt);
      complete_agent_dispatch_packet(pkt);
      packets_processed++;
      break;

    case AIR_PKT_TYPE_HELLO:
      handle_packet_hello(pkt, mb_id);
      complete_agent_dispatch_packet(pkt);
      packets_processed++;
      break;
    case AIR_PKT_TYPE_GET_CAPABILITIES:
      handle_packet_get_capabilities(pkt, mb_id);
      complete_agent_dispatch_packet(pkt);
      packets_processed++;
      break;
    case AIR_PKT_TYPE_GET_INFO:
      handle_packet_get_info(pkt, mb_id);
      complete_agent_dispatch_packet(pkt);
      packets_processed++;
      break;
    case AIR_PKT_TYPE_XAIE_LOCK:
      handle_packet_xaie_lock(pkt);
      complete_agent_dispatch_packet(pkt);
      packets_processed++;
      break;

    case AIR_PKT_TYPE_ND_MEMCPY: // Only arrive here the first try.
      uint16_t memory_space = (pkt->arg[0] >> 16) & 0x00ff;
      uint16_t channel = (pkt->arg[0] >> 24) & 0x00ff;
      uint16_t direction = (pkt->arg[0] >> 60) & 0x000f;
      uint16_t col = (pkt->arg[0] >> 32) & 0x00ff;
      uint32_t slot = channel + get_slot(col, memory_space);
      if (direction == SHIM_DMA_S2MM)
        slot += XAIEDMA_SHIM_CHNUM_S2MM0;
      else
        slot += XAIEDMA_SHIM_CHNUM_MM2S0;
      int ret = stage_packet_nd_memcpy(pkt, slot, memory_space);
      if (ret == 0) {
        last_slot = slot;
        if (do_packet_memcpy(slot)) {
          num_active_packets++;
          break;
        } // else completed the packet in the first try
      } else if (ret == 2)
        break; // slot busy, retry.
      staged_nd_slot[slot].valid = 0;
      complete_agent_dispatch_packet(
          pkt); // this is correct for the first try or invalid stage
      packets_processed++;
      break;

    } // switch
  } while (num_active_packets > 1);
  lock_uart(mb_id);
  air_printf("Completing: %d packets processed.\n\r", packets_processed);
  unlock_uart(mb_id);
  *rd_id += packets_processed;
}

void handle_barrier_and_packet(amd_queue_t *amd_queue, uint32_t mb_id, int queue_id) {

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

  // lock_uart(mb_id);
  // for (int i = 0; i < 5; i++)
  //  air_printf("MB %d : dep_signal[%d] @ %p\n\r",mb_id,i,(uint64_t
  //  *)(pkt->dep_signal[i]));
  // unlock_uart(mb_id);

  while ( hsa_signal_wait_scacquire(s0, HSA_SIGNAL_CONDITION_EQ, 0, 0x80000, HSA_WAIT_STATE_ACTIVE) != 0 ||
          hsa_signal_wait_scacquire(s1, HSA_SIGNAL_CONDITION_EQ, 0, 0x80000, HSA_WAIT_STATE_ACTIVE) != 0 ||
          hsa_signal_wait_scacquire(s2, HSA_SIGNAL_CONDITION_EQ, 0, 0x80000, HSA_WAIT_STATE_ACTIVE) != 0 ||
          hsa_signal_wait_scacquire(s3, HSA_SIGNAL_CONDITION_EQ, 0, 0x80000, HSA_WAIT_STATE_ACTIVE) != 0 ||
          hsa_signal_wait_scacquire(s4, HSA_SIGNAL_CONDITION_EQ, 0, 0x80000, HSA_WAIT_STATE_ACTIVE) != 0) {
    lock_uart(mb_id);
    air_printf("MB %d : barrier AND packet completion signal timeout!\n\r",
               mb_id);
    for (int i = 0; i < 5; i++)
      air_printf("MB %d : dep_signal[%d] = %d\n\r", mb_id, i,
                 pkt->dep_signal[i]);
    unlock_uart(mb_id);
  }

  complete_barrier_packet(pkt);
  *rd_id += 1;

}

void handle_barrier_or_packet(amd_queue_t *amd_queue, uint32_t mb_id, int queue_id) {

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

  // lock_uart(mb_id);
  // for (int i = 0; i < 5; i++)
  //  air_printf("MB %d : dep_signal[%d] @ %p\n\r",mb_id,i,(uint64_t
  //  *)(pkt->dep_signal[i]));
  // unlock_uart(mb_id);

  while ( hsa_signal_wait_scacquire(s0, HSA_SIGNAL_CONDITION_EQ, 0, 0x80000, HSA_WAIT_STATE_ACTIVE) != 0 &&
          hsa_signal_wait_scacquire(s1, HSA_SIGNAL_CONDITION_EQ, 0, 0x80000, HSA_WAIT_STATE_ACTIVE) != 0 &&
          hsa_signal_wait_scacquire(s2, HSA_SIGNAL_CONDITION_EQ, 0, 0x80000, HSA_WAIT_STATE_ACTIVE) != 0 &&
          hsa_signal_wait_scacquire(s3, HSA_SIGNAL_CONDITION_EQ, 0, 0x80000, HSA_WAIT_STATE_ACTIVE) != 0 &&
          hsa_signal_wait_scacquire(s4, HSA_SIGNAL_CONDITION_EQ, 0, 0x80000, HSA_WAIT_STATE_ACTIVE) != 0) {
 
    lock_uart(mb_id);
    air_printf("MB %d : barrier OR packet completion signal timeout!\n\r",
               mb_id);
    for (int i = 0; i < 5; i++)
      air_printf("MB %d : dep_signal[%d] = %d\n\r", mb_id, i,
                 pkt->dep_signal[i]);
    unlock_uart(mb_id);
  }

  complete_barrier_packet(pkt);
  *rd_id += 1;
}

int main() {

  // Initializing our platform
  init_platform();

  hsa_csr_init();
  hsa_csr_print();

#if defined(ARM_CONTROLLER)
  Xil_DCacheDisable();

  aie_libxaie_ctx_t ctx;
  _xaie = &ctx;
  mlir_aie_init_libxaie(_xaie);
  int err = mlir_aie_init_device(_xaie);
  if (err)
    xil_printf("ERROR initializing device.\n\r");

  // Setting the number of agents in the system
  int user1 = 1; // number of controllers
  int user2 = 0;

  int mb_id = user2 & 0xff;
  int maj = (user2 >> 24) & 0xff;
  int min = (user2 >> 16) & 0xff;
  int ver = (user2 >> 8) & 0xff;
#else
  pvr_t pvr;
  microblaze_get_pvr(&pvr);
  uint32_t user2 = MICROBLAZE_PVR_USER2(pvr);
  uint32_t mb_id = user2 & 0xff;
  uint32_t maj = (user2 >> 24) & 0xff;
  uint32_t min = (user2 >> 16) & 0xff;
  uint32_t ver = (user2 >> 8) & 0xff;
#endif

  lock_uart(mb_id);
#if defined(ARM_CONTROLLER)
  xil_printf("ARM %d of %d firmware %d.%d.%d created on %s at %s GMT\n\r",
             mb_id + 1, hsa_csr->num_aql_queues, maj, min, ver, __DATE__,
             __TIME__);
#else
  xil_printf("MB %d of %d firmware %d.%d.%d created on %s at %s GMT\n\r",
             mb_id + 1, hsa_csr->num_aql_queues, maj, min, ver, __DATE__,
             __TIME__);
#endif
  xil_printf("(c) Copyright 2020-2022 AMD, Inc. All rights reserved.\n\r");
  unlock_uart(mb_id);

  setup = false;
  lock_uart(mb_id);
  unlock_uart(mb_id);

  bool done(false);

  int admin_queue_id(0);
  amd_queue_t *admin_queue(hsa_csr->amd_aql_queues[admin_queue_id]);
  admin_queue->hsa_queue.size = 64;
  volatile uint64_t *admin_doorbell(reinterpret_cast<uint64_t*>(
      hsa_csr->doorbells[admin_queue_id]));
  volatile uint64_t *admin_rd_id(&admin_queue->read_dispatch_id);
  volatile uint64_t *admin_wr_id(&admin_queue->write_dispatch_id);
  hsa_agent_dispatch_packet_t *admin_queue_buf(
      reinterpret_cast<hsa_agent_dispatch_packet_t*>(
          hsa_csr->queue_bufs[admin_queue_id]));
  hsa_agent_dispatch_packet_t *admin_pkt(nullptr);
  uint64_t admin_last_doorbell(std::numeric_limits<uint64_t>::max());

  int hqd_id(1);
  amd_queue_t *amd_queue(hsa_csr->amd_aql_queues[hqd_id]);
  volatile uint64_t *doorbell(reinterpret_cast<uint64_t*>(
      hsa_csr->doorbells[hqd_id]));
  volatile uint64_t *rd_id(&amd_queue->read_dispatch_id);
  volatile uint64_t *wr_id(&amd_queue->write_dispatch_id);
  hsa_agent_dispatch_packet_t *queue_buf(
      reinterpret_cast<hsa_agent_dispatch_packet_t*>(
          hsa_csr->queue_bufs[hqd_id]));
  hsa_agent_dispatch_packet_t *aql_pkt(nullptr);
  uint64_t last_doorbell(std::numeric_limits<uint64_t>::max());

  *doorbell = std::numeric_limits<uint64_t>::max();
  *rd_id = 0;
  *wr_id = 0;

  *admin_doorbell = std::numeric_limits<uint64_t>::max();
  *admin_rd_id = 0;
  *admin_wr_id = 0;

  air_printf("Starting packet processing loop\n\r");
  while (!done) {
    if (*admin_doorbell + 1 > *admin_rd_id) {
      ++admin_last_doorbell;
      admin_pkt = &admin_queue_buf[*admin_rd_id % 64];
      uint32_t type(static_cast<uint32_t>(admin_pkt->header) & 0xffU);

      switch (type) {
        case HSA_PACKET_TYPE_AGENT_DISPATCH:
          handle_agent_dispatch_packet(admin_queue, mb_id, admin_queue_id);
          break;
        default:
          air_printf("Unsupported admin queue packet type: %u\n\r", type);
          ++(*admin_rd_id);
          break;
      }
    }

    if (*doorbell + 1 > *rd_id) {
      ++last_doorbell;
      aql_pkt = &queue_buf[*rd_id % amd_queue->hsa_queue.size];
      uint32_t type(static_cast<uint32_t>(aql_pkt->header) & 0xffU);
      uint32_t func(static_cast<uint32_t>(aql_pkt->type) & 0xffffU);

      air_printf("Doorbell rung %llu\n\r", *doorbell);
      air_printf("Packet type %u, func type %u, pkt data %llx\n\r", type, func,
                 aql_pkt->arg[0]);
      air_printf("queue heap addr %llx\n\r", hsa_csr->queue_dram_cpu_va[hqd_id]);

      uint32_t invalid_count = 0;
      while (type == HSA_PACKET_TYPE_INVALID) {
          aql_pkt = &queue_buf[*rd_id % amd_queue->hsa_queue.size];
          type = aql_pkt->header & 0xff;
          type = static_cast<uint32_t>(aql_pkt->header) & 0xffU;
          func = static_cast<uint32_t>(aql_pkt->type) & 0xffffU;

          // TODO: Come back to this for the multi-prodcer queue as we can hit this
          invalid_count++;
          if(invalid_count > INVLD_COUNT_TIMEOUT) {
            xil_printf("[WARNING] We are stuck in an invalid packet and timed out. Breaking\r\n");
            xil_printf("\theader: 0x%x\r\n", aql_pkt->header);
            xil_printf("\ttype: 0x%x\r\n", type);
            xil_printf("\tfunc: 0x%x\r\n", func);
            xil_printf("\trd_id: 0x%x\r\n", *rd_id);
            xil_printf("\tdoorbell: 0x%x\r\n", *doorbell);
            break;
          }
      }

      switch (type) {
        case HSA_PACKET_TYPE_AGENT_DISPATCH:
          air_printf("Dispatching agent dispatch packet\n\r");
          handle_agent_dispatch_packet(amd_queue, mb_id, hqd_id);
          break;
        case HSA_PACKET_TYPE_BARRIER_AND:
          air_printf("Executing barrier and packet\r\n");
          handle_barrier_and_packet(amd_queue, mb_id, hqd_id);
          break;
        case HSA_PACKET_TYPE_BARRIER_OR:
          air_printf("Executing barrier or packet\r\n");
          handle_barrier_or_packet(amd_queue, mb_id, hqd_id);
          break;
        // We are already handling the invalid packet above
        case HSA_PACKET_TYPE_INVALID:
          break;
        default:
          air_printf("Unsupported packet type\n\r");
          ++(*rd_id);
          break;
      }
    }
    shell();
  }

  cleanup_platform();

  return 0;
}
