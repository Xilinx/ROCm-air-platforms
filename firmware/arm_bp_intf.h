//===- arm_bp_intf.h --------------------------------------------*- C++ -*-===//
//
// Copyright (C) 2025, Advanced Micro Devices, Inc.
// SPDX-License-Identifier: MIT
//
//===----------------------------------------------------------------------===//

#ifndef __ARM_BP_INTF_H_
#define __ARM_BP_INTF_H_

#include "unistd.h"

#include <cstdint>
#include <cstring>
#include <string>
#include <algorithm>
#include <vector>

#include "amd_hsa.h"

#include "arm_intf.h"

// global (all-BP) GPIO reset
#define BP_GPIO_PADDR             0x90000000000LL
#define BP_GPIO_RESET_OFFSET      0x0LL
#define BP_GPIO_RESET_ON          0x0LL
#define BP_GPIO_RESET_OFF         0x1LL

// BP-specific platform addresses and size
#define BP_BASE_PADDR             0x80000000000LL
#define BP_SIZE                   0x100000000LL

#define BP_DRAM_OFFSET            0x80000000LL
#define BP_DRAM_SIZE              0x200000LL
#define BP_DRAM_HIGH              (BP_DRAM_OFFSET+BP_DRAM_SIZE)

// BP CFG
#define BP_CFG_OFFSET             0x200000LL
#define BP_CFG_FREEZE_OFFSET           0x8LL
#define BP_CFG_NPC_OFFSET             0x10LL
#define BP_CFG_CORE_ID_OFFSET         0x18LL
#define BP_CFG_DID_OFFSET             0x20LL
#define BP_CFG_CORD_OFFSET            0x28LL
#define BP_CFG_HOST_DID_OFFSET        0x30LL
#define BP_CFG_HIO_MASK_OFFSET        0x38LL
#define BP_CFG_GLOBAL_ID_OFFSET       0x40LL
#define BP_CFG_ICACHE_ID_OFFSET      0x200LL
#define BP_CFG_ICACHE_MODE_OFFSET    0x208LL
#define BP_CFG_DCACHE_ID_OFFSET      0x400LL
#define BP_CFG_DCACHE_MODE_OFFSET    0x408LL
#define BP_CFG_CCE_ID_OFFSET         0x600LL
#define BP_CFG_CCE_MODE_OFFSET       0x608LL

// BP Freeze - active high
#define BP_CFG_FREEZE_ON  0x1LL
#define BP_CFG_FREEZE_OFF 0x0LL

// BP cache modes
#define BP_CFG_CACHE_UNCACHED 0x0LL
#define BP_CFG_CACHE_NORMAL 0x1LL
#define BP_CFG_CACHE_NONSPEC 0x2LL

// BP cce mode
#define BP_CFG_CCE_UNCACHED 0x0LL
#define BP_CFG_CCE_NORMAL 0x1LL

// High I/O space mask
#define BP_CFG_HIO_MASK_ALL 0xFFFFFFFFLL

// BP CLINT (Core Local Interrupt Controller)
#define BP_CLINT_OFFSET 0x300000LL
#define BP_CLINT_SIZE 0x10000LL
#define BP_CLINT_MIPI_OFFSET 0x0LL
#define BP_CLINT_MTIMECMP_OFFSET 0x4000LL
#define BP_CLINT_MTIMESEL_OFFSET 0x8000LL
#define BP_CLINT_MTIME_OFFSET 0xBFF8LL
#define BP_CLINT_PLIC_OFFSET 0xB000LL
#define BP_CLINT_DEBUG_OFFSET 0xC000LL

// BP MDM UART
// RX is from JTAG to MDM (to PL)
// TX is from PL to MDM (to JTAG)
#define BP_UART_BASE_ADDR (BP_BASE_PADDR + 0x40600000ULL)
#define BP_UART_RX_FIFO 0x00ULL
#define BP_UART_TX_FIFO 0x04ULL
#define BP_UART_STATUS 0x08ULL
#define BP_UART_CONTROL 0x0CULL

// BP Mutex block
// The ARM core needs to reset all mutexes at startup, but otherwise does not use the mutexes
#define BP_MUTEX_BASE_ADDR (0x020200000000ULL)
#define BP_MUTEX_NUM 32
#define BP_MUTEX_SIZE 256
#define BP_MUTEX_LOCK_OFFSET 0
#define BP_MUTEX_LOCK_MASK 0x1U
#define BP_MUTEX_CPUID_OFFSET 1
#define BP_MUTEX_CPUID_MASK 0xFFU

// forward declarations
class BPCoreMgr;

// x86 to ARM packet processing
void handle_packet_prog_firmware(hsa_agent_dispatch_packet_t *pkt, std::vector<BPCoreMgr*> &bp_core_mgrs);
void handle_packet_bp_control(hsa_agent_dispatch_packet_t *pkt, std::vector<BPCoreMgr*> &bp_core_mgrs);

// all-BP operations
void freeze_bps(std::vector<BPCoreMgr*> &bp_core_mgrs);
void start_bps(std::vector<BPCoreMgr*> &bp_core_mgrs);
void bp_strobe_reset();
void heartbeat_bps(std::vector<BPCoreMgr*> &bp_core_mgrs);
void configure_bps(std::vector<BPCoreMgr*> &bp_core_mgrs);
void cleanup_bps(std::vector<BPCoreMgr*> &bp_core_mgrs);

// UART operations
void uart_clear_fifos();
void uart_check_fifos();

// Mutex operations
void mutex_reset();

// per-BP manager
// implements per-BP operations
class BPCoreMgr
{
  public:
    BPCoreMgr(uint64_t _global_id = 0, uint64_t _bp_id = 0);
    ~BPCoreMgr();

    // getters
    uint64_t getGlobalID();
    uint64_t getBPID();
    uint64_t getBaseAddr();

    // config device management
    void freeze();
    void unfreeze();
    void set_icache_mode(uint32_t mode);
    void set_dcache_mode(uint32_t mode);
    void set_cce_mode(uint32_t mode);
    void set_hio_mask(uint32_t mask);
    void set_global_id();
    void set_npc(uint64_t npc);
    void dump_cfg();
    void configure();

    // DRAM management
    void dump_dram(int length);
    void clear_dram();
    void read_dram(off_t offset, size_t size, void* buffer);
    void write_dram(off_t offset, size_t size, const void* buffer);

    // firmware management
    void load_fw(uint64_t phys_addr, uint32_t file_num_lines);
    void run_fw(uint64_t phys_addr, uint32_t file_num_lines);

    // map BP into memory
    void map_bp_cfg();
    void map_bp_clint();
    void map_bp_dram();
    void map_all();

    // unmap BP from memory
    void unmap_bp_cfg();
    void unmap_bp_clint();
    void unmap_bp_dram();
    void unmap_all();

  private:
    // global controller ID of this BP
    // usually bp_id + N, where N is the number of other controllers in the system
    uint64_t global_id = 0;
    // BP core's ID within only the BP complex
    uint64_t bp_id = 0;
    off_t base_paddr = 0;

    void* bp_cfg = nullptr;
    void* bp_clint = nullptr;
    void* bp_dram = nullptr;
};

#endif
