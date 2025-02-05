//===- arm_bp_intf.cpp ------------------------------------------*- C++ -*-===//
//
// Copyright (C) 2025, Advanced Micro Devices, Inc.
// SPDX-License-Identifier: MIT
//
//===----------------------------------------------------------------------===//

#include "arm_bp_intf.h"
#include "memory.h"
#include "xil_printf.h"

using namespace std;

namespace {
bool packet_get_active(hsa_agent_dispatch_packet_t *pkt) {
  return pkt->reserved2 & 0x1;
}

void packet_set_active(hsa_agent_dispatch_packet_t *pkt, bool b) {
  pkt->reserved2 = (pkt->reserved2 & ~0x1) | b;
}
}

// Strobes the reset to all of the BPs
void bp_strobe_reset() {
  void *bp_gpio = (void *)(BP_GPIO_PADDR + BP_GPIO_RESET_OFFSET);
  mmio_write<uint32_t>(bp_gpio, BP_GPIO_RESET_OFFSET, BP_GPIO_RESET_ON);
  sleep(1);
  mmio_write<uint32_t>(bp_gpio, BP_GPIO_RESET_OFFSET, BP_GPIO_RESET_OFF);
}

void handle_packet_prog_firmware(hsa_agent_dispatch_packet_t *pkt, vector<BPCoreMgr*> &bp_core_mgrs) {
  xil_printf("[ARM] handling BP program firmware packet\n\r");
  packet_set_active(pkt, true);

  // must translate from VA in arg[0] to a PA accessible by ARM
  // the VA is a host VA pointing to device DRAM
  uint64_t  phys_addr       = translate_virt_to_phys(pkt->arg[0]);
  uint32_t  file_num_lines  = pkt->arg[1] & 0xFFFFFFFF;
  uint32_t  num_bp          = pkt->arg[2] & 0xFFFFFFFF;

  xil_printf("[ARM] freezing all BP cores\n\r");
  freeze_bps(bp_core_mgrs);

  xil_printf("[ARM] programming %d BP cores from 0x%lx\n\r", num_bp, phys_addr);
  // resize the core managers array
  bp_core_mgrs.resize(num_bp);
  for (int i = 0; i < bp_core_mgrs.size(); i++) {
    bp_core_mgrs.at(i) = new BPCoreMgr(i+1, i);
  }

  // strobe GPIO reset for BP
  bp_strobe_reset();
  sleep(1);

  // freeze all the cores and clear the private memories
  for (auto core_mgr : bp_core_mgrs) {
    core_mgr->freeze();
    core_mgr->clear_dram();
  }
  // configure each core and load the program to its memory
  for (auto core_mgr : bp_core_mgrs) {
    core_mgr->configure();
    core_mgr->dump_cfg();
    core_mgr->load_fw(phys_addr, file_num_lines);
  }
}

void handle_packet_bp_control(hsa_agent_dispatch_packet_t *pkt, vector<BPCoreMgr*> &bp_core_mgrs) {
  xil_printf("[ARM] handling BP Control packet\n\r");
  packet_set_active(pkt, true);

  uint64_t command = pkt->arg[0];
  uint64_t stalls[BP_CFG_DBG_STALL_REGS] = {0};
  uint64_t pc_history[BP_CFG_DBG_PC_LEN] = {0};
  uint64_t arg1 = pkt->arg[1];
  uint64_t arg2 = pkt->arg[2];
  uint64_t arg3 = pkt->arg[3];
  switch (command) {
    case 0x0:
      xil_printf("[ARM] resetting Mutex\n\r");
      mutex_reset();
      break;
    case 0x1:
      xil_printf("[ARM] checking UART FIFOs\n\r");
      uart_check_fifos();
      break;
    case 0x2:
      xil_printf("[ARM] clearing UART FIFOs\n\r");
      uart_clear_fifos();
      break;
    case 0x3:
      xil_printf("[ARM] strobing BP GPIO reset\n\r");
      bp_strobe_reset();
      sleep(1);
      break;
    case 0x4:
      xil_printf("[ARM] freezing BP cores\n\r");
      freeze_bps(bp_core_mgrs);
      break;
    case 0x5:
      break;
    case 0x6:
      xil_printf("[ARM] clearing BP memories\n\r");
      for (auto core_mgr : bp_core_mgrs) {
        core_mgr->clear_dram();
      }
      break;
    case 0x7:
      if (!arg2) {
        arg2 = 1;
      }
      if (arg1 < bp_core_mgrs.size()) {
        xil_printf("[ARM] dumping BP[%d] DRAM\n\r", arg1);
        bp_core_mgrs.at(arg1)->dump_dram(arg2);
      } else {
        xil_printf("[ARM] BP[%d] does not exist\n\r", arg1);
      }
      break;
    case 0x8:
      xil_printf("[ARM] unfreezing BP cores\n\r");
      for (auto core_mgr : bp_core_mgrs) {
        core_mgr->unfreeze();
      }
      break;
    case 0x9:
      xil_printf("[ARM] configuring BP cores\n\r");
      configure_bps(bp_core_mgrs);
      heartbeat_bps(bp_core_mgrs);
      break;
    case 0xA:
      xil_printf("[ARM] dumping BP cores\n\r");
      heartbeat_bps(bp_core_mgrs);
      break;
    default:
      break;
  }
}

void start_bps(vector<BPCoreMgr*> &bp_core_mgrs) {
  for (auto core_mgr : bp_core_mgrs) {
    core_mgr->unfreeze();
  }
}

void freeze_bps(vector<BPCoreMgr*> &bp_core_mgrs) {
  for (auto core_mgr : bp_core_mgrs) {
    core_mgr->freeze();
  }
}

void heartbeat_bps(vector<BPCoreMgr*> &bp_core_mgrs) {
  for (auto core_mgr : bp_core_mgrs) {
    core_mgr->dump_cfg();
  }
}

void configure_bps(vector<BPCoreMgr*> &bp_core_mgrs) {
  for (auto core_mgr : bp_core_mgrs) {
    core_mgr->configure();
  }
}

void cleanup_bps(vector<BPCoreMgr*> &bp_core_mgrs) {
  for (int i = 0; i < bp_core_mgrs.size(); i++) {
    delete bp_core_mgrs.at(i);
  }
}

void uart_clear_fifos() {
  volatile uint32_t *uart_control = (volatile uint32_t*)(BP_UART_BASE_ADDR + BP_UART_CONTROL);
  *uart_control = 0x3;
  xil_printf("[UART] fifos cleared\n\r");
}

void uart_check_fifos() {
  volatile uint32_t *uart_status = (volatile uint32_t*)(BP_UART_BASE_ADDR + BP_UART_STATUS);
  uint32_t status = *uart_status;
  xil_printf("[UART] Status:   %08x\n\r", status);
  xil_printf("[UART] Intr EN:  %1x\n\r", (status >> 4) & 0x1);
  xil_printf("[UART] TX FULL:  %1x\n\r", (status >> 3) & 0x1);
  xil_printf("[UART] TX EMPTY: %1x\n\r", (status >> 2) & 0x1);
  xil_printf("[UART] RX FULL:  %1x\n\r", (status >> 1) & 0x1);
  xil_printf("[UART] RX VALID: %1x\n\r", status & 0x1);
}

void mutex_reset() {
  for (uint64_t i = 0; i < BP_MUTEX_NUM; i++) {
    volatile uint32_t *mutex = (volatile uint32_t *)(BP_MUTEX_BASE_ADDR + (i*BP_MUTEX_SIZE));
    uint32_t val = *mutex;
    uint32_t cpuid = (val >> BP_MUTEX_CPUID_OFFSET) & BP_MUTEX_CPUID_MASK;
    if (val & BP_MUTEX_LOCK_MASK) {
      *mutex = (cpuid << BP_MUTEX_CPUID_OFFSET);
    }
  }
}

// BPCoreMgr

BPCoreMgr::BPCoreMgr(uint64_t _global_id, uint64_t _bp_id) {
  global_id = _global_id;
  bp_id = _bp_id;
  // physical address offset is determined by the BP complex ID
  base_paddr = BP_BASE_PADDR + (bp_id*BP_SIZE);
  map_all();
}

BPCoreMgr::~BPCoreMgr() {
  unmap_all();
}

// get the core's global ID
uint64_t
BPCoreMgr::getGlobalID() {
  return global_id;
}

// get the core's ID within the BP complex
uint64_t
BPCoreMgr::getBPID() {
  return bp_id;
}

// get the core's base physical address
uint64_t
BPCoreMgr::getBaseAddr() {
  return base_paddr;
}

// place the core into a frozen state (halt instruction execution)
void
BPCoreMgr::freeze() {
  xil_printf("[BP:%0d] freezing core\n\r", global_id);
  mmio_write<uint32_t>(bp_cfg, BP_CFG_FREEZE_OFFSET, BP_CFG_FREEZE_ON);
}

// release the core for execution
// note: freezing and then unfreezing a BP core during execution results in
// undefined behavior. Unfreeze should only be called once after reset, to
// start execution after the core has been configured and a program loaded to its memory.
void
BPCoreMgr::unfreeze() {
  xil_printf("[BP:%0d] unfreezing core\n\r", global_id);
  mmio_write<uint32_t>(bp_cfg, BP_CFG_FREEZE_OFFSET, BP_CFG_FREEZE_OFF);
}

void
BPCoreMgr::set_icache_mode(uint32_t mode) {
  mmio_write<uint32_t>(bp_cfg, BP_CFG_ICACHE_MODE_OFFSET, mode);
}

void
BPCoreMgr::set_dcache_mode(uint32_t mode) {
  mmio_write<uint32_t>(bp_cfg, BP_CFG_DCACHE_MODE_OFFSET, mode);
}

void
BPCoreMgr::set_cce_mode(uint32_t mode) {
  mmio_write<uint32_t>(bp_cfg, BP_CFG_CCE_MODE_OFFSET, mode);
}

// set the hio or "High I/O" mask in the core
// this mask determines which addresses in I/O space can be accessed by BP
// see the BP RTL and documentation for more details
void
BPCoreMgr::set_hio_mask(uint32_t mask) {
  mmio_write<uint32_t>(bp_cfg, BP_CFG_HIO_MASK_OFFSET, mask);
}

// write this cores global ID into the core's config device
// this config register is not used directly by the hardware, but can be accessed
// by software, which is useful when the core is one of many "cores" in the system
// and needs to perform actions relative to its global ID in the system (e.g., only
// the core with global ID = 0 should perform some system-wide initialization).
void
BPCoreMgr::set_global_id() {
  mmio_write<uint64_t>(bp_cfg, BP_CFG_GLOBAL_ID_OFFSET, global_id);
}

// write the initial PC register for the core, which is captured by HW on unfreeze()
void
BPCoreMgr::set_npc(uint64_t npc) {
  mmio_write<uint64_t>(bp_cfg, BP_CFG_NPC_OFFSET, npc);
}

void
BPCoreMgr::dump_cfg() {
  // Read core info
  uint64_t core_id = mmio_read<uint64_t>(bp_cfg, BP_CFG_CORE_ID_OFFSET);
  uint64_t did = mmio_read<uint64_t>(bp_cfg, BP_CFG_DID_OFFSET);
  uint64_t cord = mmio_read<uint64_t>(bp_cfg, BP_CFG_CORD_OFFSET);
  uint64_t gid = mmio_read<uint64_t>(bp_cfg, BP_CFG_GLOBAL_ID_OFFSET);
  uint32_t hio_mask = mmio_read<uint32_t>(bp_cfg, BP_CFG_HIO_MASK_OFFSET);

  // Read cache info
  uint64_t icache_id = mmio_read<uint64_t>(bp_cfg, BP_CFG_ICACHE_ID_OFFSET);
  uint64_t icache_mode = mmio_read<uint64_t>(bp_cfg, BP_CFG_ICACHE_MODE_OFFSET);
  uint64_t dcache_id = mmio_read<uint64_t>(bp_cfg, BP_CFG_DCACHE_ID_OFFSET);
  uint64_t dcache_mode = mmio_read<uint64_t>(bp_cfg, BP_CFG_DCACHE_MODE_OFFSET);

  // Read freeze bit
  uint64_t freeze = mmio_read<uint64_t>(bp_cfg, BP_CFG_FREEZE_OFFSET);

  // Read NPC register
  uint64_t npc = mmio_read<uint64_t>(bp_cfg, BP_CFG_NPC_OFFSET);

  xil_printf("[BP:%0d] Core ID : %0d\n\r", global_id, core_id);
  xil_printf("[BP:%0d] Device ID : %0d\n\r", global_id, did);
  xil_printf("[BP:%0d] Coordinate : %0d\n\r", global_id, cord);
  xil_printf("[BP:%0d] Global ID : %0d\n\r", global_id, gid);
  xil_printf("[BP:%0d] HIO mask : 0x%X\n\r", global_id, hio_mask);
  xil_printf("[BP:%0d] I$ ID : %0d\n\r", global_id, icache_id);
  xil_printf("[BP:%0d] I$ mode : %0d\n\r", global_id, icache_mode);
  xil_printf("[BP:%0d] D$ ID : %0d\n\r", global_id, dcache_id);
  xil_printf("[BP:%0d] D$ mode : %0d\n\r", global_id, dcache_mode);
  xil_printf("[BP:%0d] Freeze : %0d\n\r", global_id, freeze);
  xil_printf("[BP:%0d] NPC : 0x%lX\n\r", global_id, npc);
}

// configure BP core for normal execution, but leave in a frozen state
void
BPCoreMgr::configure() {
  xil_printf("[BP:%0d] configuring core\n\r", global_id);
  set_icache_mode(BP_CFG_CACHE_NORMAL);
  set_dcache_mode(BP_CFG_CACHE_NORMAL);
  set_cce_mode(BP_CFG_CCE_NORMAL);
  set_hio_mask(BP_CFG_HIO_MASK_ALL);
  set_global_id();
  set_npc(BP_DRAM_OFFSET);
}

// print every dword of BP DRAM to stdout
void
BPCoreMgr::dump_dram(int length) {
  xil_printf("[BP:%0d] dumping DRAM\n\r", global_id);
  for (int i = 0; i < length; i++) {
    uint64_t dword = mmio_read<uint64_t>(bp_dram, i*8);
    xil_printf("0x%08X: 0x%016lX\n\r", i*8, dword);
  }
}

// zero every byte of BP DRAM
void
BPCoreMgr::clear_dram() {
  xil_printf("[BP:%0d] clearing dram\n\r", global_id);
  size_t page_size = 4096;
  void* buffer = calloc(page_size, 1);
  for (off_t offset = 0; offset < BP_DRAM_SIZE; offset = offset + page_size) {
    write_dram(offset, page_size, (const void*)buffer);
  }
  free(buffer);
}

// read arbitrary bytes of BP DRAM
// buffer must be able to store at least size bytes
void
BPCoreMgr::read_dram(off_t offset, size_t size, void* buffer) {
  uintptr_t addr = reinterpret_cast<uintptr_t>(bp_dram) + offset;
  memcpy(buffer, reinterpret_cast<const void*>(addr), size);
}

// write arbitrary bytes of BP DRAM
// buffer must contain at least size bytes of valid data
void
BPCoreMgr::write_dram(off_t offset, size_t size, const void* buffer) {
  uintptr_t addr = reinterpret_cast<uintptr_t>(bp_dram) + offset;
  memcpy(reinterpret_cast<void*>(addr), buffer, size);
}

char*
find_token(char* s, char token) {
  while (*s != token) {
    s++;
  }
  return s;
}

// Load a firmware program into BP
void
BPCoreMgr::load_fw(uint64_t phys_addr, uint32_t file_num_lines) {
  xil_printf("[BP:%0d] loading dram from 0x%lX\n\r", global_id, phys_addr);

  // Getting the physical address of where to write the firmware
  // expected file format:
  // @address
  // b0b1b2b3b4b5b6b7b7
  // b0b1b2b3b4b5b6b7b7
  // ...

  // ignore empty lines
  // expect no comments - each line is address or data
  // each data line is a power of two bytes, in hex

  if(phys_addr == 0 || file_num_lines == 0) return;

  off_t base_offset = BP_DRAM_OFFSET;
  off_t offset = 0;

  char *start_token = (char *)phys_addr;
  char *end_token = find_token(start_token, '\n');

  for(int line_iter = 0; line_iter < file_num_lines; line_iter++) {

    // convert to c++ string
    size_t n = (size_t)end_token - (size_t)start_token;
    string line(start_token, n);

    // Getting rid of the spaces in each line
    line.erase(remove(line.begin(), line.end(), ' '), line.end());

    // remove carriage returns at end of line
    while (line.back() == '\r') {
      line.pop_back();
    }
    // address line
    // parse the address and generate BP DRAM relative offset
    if (line.front() == '@') {
      line.erase(line.begin());
      offset = stoull(line, nullptr, 16) - base_offset;
    }
    // data line
    // write each byte in line to consecutive addresses, starting at (addr+offset)
    else {
      uint64_t val = 0;
      // hex characters on line
      int len = line.size();
      int i = 0;
      while (len > 0) {
        val = 0;
        int sublen = (len >= 16) ? 16 : len;
        // hex string of WWXXYYZZ...TT that needs to become TT...ZZYYXXWW
        // i.e., byte order reversal, but bits in each byte stay ordered
        string sub = line.substr(i,sublen);
        // convert hex string to 64-bit unsigned int
        val = stoull(string(sub.begin(), sub.end()), nullptr, 16);

        i += sublen;
        len -= sublen;

        uint64_t r = 0;
        if (sublen == 16) {
          r = reverse_bytes<uint64_t>(val);
          mmio_write<uint64_t>(bp_dram, offset, r);
          offset += 8;
        } else if (sublen == 8) {
          r = reverse_bytes<uint32_t>(val);
          mmio_write<uint32_t>(bp_dram, offset, r);
          offset += 4;
        } else if (sublen == 4) {
          r = reverse_bytes<uint16_t>(val);
          mmio_write<uint16_t>(bp_dram, offset, r);
          offset += 2;
        } else if (sublen == 2) {
          r = reverse_bytes<uint8_t>(val);
          mmio_write<uint8_t>(bp_dram, offset, r);
          offset += 1;
        }
      }
    }

    // Getting the next line if there is one
    if(line_iter == file_num_lines - 1) break;
    // next start token is character after current end token
    start_token = end_token+1;
    // find the next end token
    end_token = find_token(start_token, '\n');
  }
}

// Load a firmware program into BP and unfreeze the core to start execution.
// Assumes core is already configured properly.
void
BPCoreMgr::run_fw(uint64_t phys_addr, uint32_t file_num_lines) {
  load_fw(phys_addr, file_num_lines);
  unfreeze();
}

// map this core's config device into user-space
void
BPCoreMgr::map_bp_cfg() {
  if (bp_cfg) return;
  bp_cfg = reinterpret_cast<void*>(base_paddr + BP_CFG_OFFSET);
}

// map this core's local interrupt device into user-space
void
BPCoreMgr::map_bp_clint() {
  if (bp_clint) return;
  bp_clint = reinterpret_cast<void*>(base_paddr + BP_CLINT_OFFSET);
}

// map this core's DRAM memory into user-space
void
BPCoreMgr::map_bp_dram() {
  if (bp_dram) return;
  bp_dram = reinterpret_cast<void*>(base_paddr + BP_DRAM_OFFSET);
}

void
BPCoreMgr::map_all() {
  map_bp_cfg();
  map_bp_clint();
  map_bp_dram();
}

void
BPCoreMgr::unmap_bp_cfg() {
  bp_cfg = nullptr;
}

void
BPCoreMgr::unmap_bp_clint() {
  bp_clint = nullptr;
}

void
BPCoreMgr::unmap_bp_dram() {
  bp_dram = nullptr;
}

void
BPCoreMgr::unmap_all() {
  unmap_bp_cfg();
  unmap_bp_clint();
  unmap_bp_dram();
}

