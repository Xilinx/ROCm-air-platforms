// (c) 2023 SAFARI Research Group at ETH Zurich, Gagandeep Singh, D-ITET
// SPDX-License-Identifier: MIT

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <stdlib.h>
#include <sys/mman.h>
#include <thread>
#include <time.h>
#include <unistd.h>
#include <vector>

#include <sys/stat.h>
#include <gelf.h>

#include "hsa/hsa.h"
#include "hsa/hsa_ext_amd.h"

#define TOTAL_B_BLOCK 1 // only 1
#define B_BLOCK_DEPTH 4 // set how many rows
#define HDIFF_COL 3     // columns
#define START_ROW 1
#define INPUT_ROWS 9
#define DMA_COUNT_IN 256 * INPUT_ROWS
#define DMA_COUNT_OUT 256 * 2 * B_BLOCK_DEPTH
#define XAIE_NUM_COLS 10

// TODO: These to the runtime library in their proper location
#define AIR_PKT_TYPE_DEVICE_INITIALIZE 0x0010L
#define AIR_PKT_TYPE_SEGMENT_INITIALIZE 0x0011L
#define AIR_PKT_TYPE_ND_MEMCPY 0x0103L
#define AIR_ADDRESS_ABSOLUTE_RANGE 0x1L
#define AIR_PKT_TYPE_AIRBIN 0x53L

// TODO: Move this to the runtime library
/*
  Each entry describes a loadable section in device memory. The device uses
  this information to load the data into AIE memory. This definition is shared
  with the device firmware.
*/
struct airbin_table_entry {
  uint32_t offset; // offset into allocated device memory
  uint32_t size;   // size of the loadable section
  uint64_t addr;   // base address to load the data
};

// TODO: Figure out what I am going to do with this
hsa_amd_memory_pool_t global_mem_pool;

// TODO: Move these to the runtime library
/*
  'table' is an offset from the beginning of device memory
*/
hsa_status_t air_packet_load_airbin(hsa_agent_dispatch_packet_t *pkt,
                                    uint64_t table) {
  pkt->type = AIR_PKT_TYPE_AIRBIN;
  pkt->header = (HSA_PACKET_TYPE_AGENT_DISPATCH << HSA_PACKET_HEADER_TYPE);
  pkt->arg[0] = table;

  return HSA_STATUS_SUCCESS;
}

// TODO: Move these to the runtime library
/*
  Load an airbin from a file into a device
*/
hsa_status_t air_load_airbin(hsa_agent_t *agent, hsa_queue_t *q,
                             const char *filename, uint8_t column) {
  hsa_status_t ret = HSA_STATUS_SUCCESS;
  int drv_fd = 0, elf_fd = 0;
  uint32_t dram_size;
  uint8_t *dram_ptr = NULL;
  uint8_t *data_ptr = NULL;
  struct timespec ts_start;
  struct timespec ts_end;
  Elf *inelf = NULL;
  GElf_Ehdr *ehdr = NULL;
  GElf_Ehdr ehdr_mem;
  uint64_t wr_idx = 0;
  hsa_agent_dispatch_packet_t pkt;
  size_t shnum;
  uint32_t table_idx = 0;
  airbin_table_entry *airbin_table;
  uint32_t data_offset = 0;
  uint32_t table_size = 0;
  struct stat elf_stat;

  auto time_spec_diff = [](struct timespec &start, struct timespec &end) {
    return (end.tv_sec - start.tv_sec) + 1e-9 * (end.tv_nsec - start.tv_nsec);
  };

  // open the AIRBIN file
  elf_fd = open(filename, O_RDONLY);
  if (elf_fd < 0) {
    printf("Can't open %s\n", filename);
    ret = HSA_STATUS_ERROR_INVALID_FILE;
    goto err_elf_open;
  }

  // calculate the size needed to load
  fstat(elf_fd, &elf_stat);
  //dram_size = elf_stat.st_size;
  dram_size = 6 * 1024 * 1024;
  if(table_size > dram_size) {
    printf("[ERROR] table size is larger than allocated DRAM. Exiting\n");
    ret = HSA_STATUS_ERROR_OUT_OF_RESOURCES;
    goto err_elf_open;
  }

  // get some DRAM from the device
  //dram_ptr = (uint8_t *)air_malloc(dram_size);
  // Doing the thing :D 
  hsa_amd_memory_pool_allocate(global_mem_pool, dram_size, 0, (void **)&dram_ptr);

  if (dram_ptr == MAP_FAILED) {
    printf("Error allocating %u DRAM\n", dram_size);
    ret = HSA_STATUS_ERROR_OUT_OF_RESOURCES;
    goto err_dev_mem_alloc;
  }

  // check the characteristics
  elf_version(EV_CURRENT);
  inelf = elf_begin(elf_fd, ELF_C_READ, NULL);
  ehdr = gelf_getehdr(inelf, &ehdr_mem);
  if (ehdr == NULL) {
    printf("cannot get ELF header: %s\n", elf_errmsg(-1));
    ret = HSA_STATUS_ERROR_INVALID_FILE;
    goto err_elf_read;
  }

  // Read data as 64-bit little endian
  if ((ehdr->e_ident[EI_CLASS] != ELFCLASS64) ||
      (ehdr->e_ident[EI_DATA] != ELFDATA2LSB)) {
    printf("unexpected ELF format\n");
    ret = HSA_STATUS_ERROR_INVALID_FILE;
    goto err_elf_read;
  }

  if (elf_getshdrnum(inelf, &shnum) != 0) {
    printf("cannot get program header count: %s", elf_errmsg(-1));
    ret = HSA_STATUS_ERROR_INVALID_FILE;
    goto err_elf_read;
  }

  /*
    Even though not all sections are loadable, we use the section count as an
    upper bound for how much memory the table will take. We can then safely
    place data after that point and avoid any conflicts. A small amount of
    memory will be wasted but it is usually only two entries (32 bytes) so
    not a big deal. This allows us to do only a single pass on the ELF
    sections so it seems like a good trade-off.
  */
  table_size = shnum * sizeof(airbin_table_entry);
  airbin_table = (airbin_table_entry *)dram_ptr;
  data_offset = table_size; // The data offset starts at the end of the table
  data_ptr = dram_ptr + table_size;

  // Iterate through all sections to create a table in device-readable format.
  for (unsigned int ndx = 0; ndx < shnum; ndx++) {
    GElf_Shdr shdr;
    Elf_Scn *sec = elf_getscn(inelf, ndx);
    if (sec == NULL) {
      printf("cannot get section %d: %s", ndx, elf_errmsg(-1));
      ret = HSA_STATUS_ERROR_INVALID_FILE;
      goto err_elf_read;
    }

    gelf_getshdr(sec, &shdr);

    // for each loadable program header
    if (shdr.sh_type != SHT_PROGBITS || !(shdr.sh_flags & SHF_ALLOC))
      continue;

    // copy the data from into device memory
    Elf_Data *desc;
    desc = elf_getdata(sec, NULL);
    if (!desc) {
      printf("Error reading data for section %u\n", ndx);
      ret = HSA_STATUS_ERROR_INVALID_FILE;
      goto err_elf_read;
    }
    memcpy(data_ptr, desc->d_buf, desc->d_size);

    airbin_table[table_idx].offset = data_offset;
    airbin_table[table_idx].size = shdr.sh_size;
    airbin_table[table_idx].addr = shdr.sh_addr;

    table_idx++;
    data_offset += shdr.sh_size;
    data_ptr += shdr.sh_size;

    if(data_offset > dram_size) {
      printf("[ERROR] Overwriting allocated DRAM size. Exiting\n");
      ret = HSA_STATUS_ERROR_OUT_OF_RESOURCES;
      goto err_elf_read;
    }
  }

  // the last entry must be all 0's
  airbin_table[table_idx].offset = 0;
  airbin_table[table_idx].size = 0;
  airbin_table[table_idx].addr = 0;

  // Send configuration packet
  wr_idx = hsa_queue_add_write_index_relaxed(q, 1);
  air_packet_load_airbin(&pkt, (uint64_t)airbin_table);

  // dispatch and wait has blocking semantics so we can internally create the signal
  hsa_amd_signal_create_on_agent(1, 0, nullptr, agent, 0, &(pkt.completion_signal));

  // Write the packet to the queue
  reinterpret_cast<hsa_agent_dispatch_packet_t *>(q->base_address)[wr_idx % q->size] = pkt;

  // Ringing the doorbell
  hsa_signal_store_screlease(q->doorbell_signal, wr_idx);

  // wait for packet completion
  while (hsa_signal_wait_scacquire(pkt.completion_signal,
                             HSA_SIGNAL_CONDITION_EQ, 0, 0x80000,
                             HSA_WAIT_STATE_ACTIVE) != 0);

  hsa_signal_destroy(pkt.completion_signal);
  // clock_gettime(CLOCK_BOOTTIME, &ts_end);

  // printf("airbin loading time: %0.8f sec\n", time_spec_diff(ts_start,
  // ts_end));

err_elf_read:
  elf_end(inelf);
  close(elf_fd);

err_elf_open:
err_dev_mem_alloc:
  return ret;
}

// TODO: Move this to the runtime library
hsa_status_t
air_packet_nd_memcpy(hsa_agent_dispatch_packet_t *pkt, uint16_t herd_id, uint8_t col,
                     uint8_t direction, uint8_t channel, uint8_t burst_len,
                     uint8_t memory_space, uint64_t phys_addr,
                     uint32_t transfer_length1d, uint32_t transfer_length2d,
                     uint32_t transfer_stride2d, uint32_t transfer_length3d,
                     uint32_t transfer_stride3d, uint32_t transfer_length4d,
                     uint32_t transfer_stride4d) {

  pkt->arg[0] = 0;
  pkt->arg[0] |= ((uint64_t)memory_space) << 16;
  pkt->arg[0] |= ((uint64_t)channel) << 24;
  pkt->arg[0] |= ((uint64_t)col) << 32;
  pkt->arg[0] |= ((uint64_t)burst_len) << 52;
  pkt->arg[0] |= ((uint64_t)direction) << 60;

  pkt->arg[1] = phys_addr;
  pkt->arg[2] = transfer_length1d;
  pkt->arg[2] |= ((uint64_t)transfer_length2d) << 32;
  pkt->arg[2] |= ((uint64_t)transfer_stride2d) << 48;
  pkt->arg[3] = transfer_length3d;
  pkt->arg[3] |= ((uint64_t)transfer_stride3d) << 16;
  pkt->arg[3] |= ((uint64_t)transfer_length4d) << 32;
  pkt->arg[3] |= ((uint64_t)transfer_stride4d) << 48;

  pkt->type = AIR_PKT_TYPE_ND_MEMCPY;
  pkt->header = (HSA_PACKET_TYPE_AGENT_DISPATCH << HSA_PACKET_HEADER_TYPE);

  return HSA_STATUS_SUCCESS;
}

hsa_status_t IterateAgents(hsa_agent_t agent, void *data) {
  hsa_status_t status(HSA_STATUS_SUCCESS);
  hsa_device_type_t device_type;
  std::vector<hsa_agent_t> *aie_agents(nullptr);

  if (!data) {
    status = HSA_STATUS_ERROR_INVALID_ARGUMENT;
    return status;
  }

  aie_agents = static_cast<std::vector<hsa_agent_t>*>(data);
  status = hsa_agent_get_info(agent, HSA_AGENT_INFO_DEVICE, &device_type);

  if (status != HSA_STATUS_SUCCESS) {
    return status;
  }

  if (device_type == HSA_DEVICE_TYPE_AIE) {
    aie_agents->push_back(agent);
  }

  return status;
}

hsa_status_t IterateMemPool(hsa_amd_memory_pool_t pool, void *data) {
  hsa_status_t status(HSA_STATUS_SUCCESS);
  hsa_region_segment_t segment_type;
  status = hsa_amd_memory_pool_get_info(pool, HSA_AMD_MEMORY_POOL_INFO_SEGMENT,
                                        &segment_type);
  if (segment_type == HSA_REGION_SEGMENT_GLOBAL) {
    printf("Runtime: found global segment");
    *reinterpret_cast<hsa_amd_memory_pool_t*>(data) = pool;
  }

  return status;
}

int main(int argc, char *argv[]) {

  // Starting in the first DU of the VCK5000
  uint64_t row = 0;
  uint64_t shim_one_col = 2;
  uint64_t shim_two_col = 3;
  uint8_t num_cols = 8;
  uint8_t num_rows = 8;

  // HSA datastructures
  std::vector<hsa_agent_t> agents;
  std::vector<hsa_queue_t *> queues;
  uint32_t aie_max_queue_size(0);

  // Initializing HSA
  hsa_status_t hsa_ret = hsa_init();
  if (hsa_ret != HSA_STATUS_SUCCESS) {
    std::cerr << "hsa_init failed" << std::endl;
    return -1;
  }
 
  // Finding all AIE HSA agents
  hsa_iterate_agents(&IterateAgents, reinterpret_cast<void*>(&agents));
  printf("Runtime: found %d AIE agents\n", agents.size());

  // Iterating over memory pools to initialize our allocator
  printf("Runtime: initializing memory pools");
  hsa_amd_agent_iterate_memory_pools(agents.front(),
                                     IterateMemPool,
                                     reinterpret_cast<void*>(&global_mem_pool));

  // Getting the size of queue the agent supports
  hsa_agent_get_info(agents[0], HSA_AGENT_INFO_QUEUE_MAX_SIZE, &aie_max_queue_size);
  std::cout << "Max AIE queue size: " << aie_max_queue_size << std::endl;

  // Creating a queue
  hsa_queue_t *q = NULL;
  auto queue_create_status = hsa_queue_create(agents[0], aie_max_queue_size,
                              HSA_QUEUE_TYPE_SINGLE, nullptr, nullptr, 0,
                              0, &q);

  if(queue_create_status != HSA_STATUS_SUCCESS) {
    std::cout << "hsa_queue_create failed" << std::endl;
  }

  // Adding to our vector of queues
  queues.push_back(q);
  assert(queues.size() > 0 && "No queues were sucesfully created!");

  // Configuring the device
  auto airbin_ret = air_load_airbin(&agents[0], queues[0], "airbin.elf", shim_one_col);
  if (airbin_ret != HSA_STATUS_SUCCESS) {
    printf("Loading airbin failed: %d\n", airbin_ret);
    return -1;
  }

  int errors = 0;

  // Allocating some device memory
  uint32_t *ddr_ptr_in_0 = NULL;
  uint32_t *ddr_ptr_in_1 = NULL;
  uint32_t *ddr_ptr_in_2 = NULL;
  uint32_t *ddr_ptr_in_3 = NULL;
  uint32_t *ddr_ptr_out_0 = NULL;
  uint32_t *ddr_ptr_out_1 = NULL;
  uint32_t *ddr_ptr_out_2 = NULL;
  uint32_t *ddr_ptr_out_3 = NULL;
  hsa_amd_memory_pool_allocate(global_mem_pool, DMA_COUNT_IN * sizeof(uint32_t), 0, (void **)&ddr_ptr_in_0);
  hsa_amd_memory_pool_allocate(global_mem_pool, DMA_COUNT_IN * sizeof(uint32_t), 0, (void **)&ddr_ptr_in_1);
  hsa_amd_memory_pool_allocate(global_mem_pool, DMA_COUNT_IN * sizeof(uint32_t), 0, (void **)&ddr_ptr_in_2);
  hsa_amd_memory_pool_allocate(global_mem_pool, DMA_COUNT_IN * sizeof(uint32_t), 0, (void **)&ddr_ptr_in_3);
  hsa_amd_memory_pool_allocate(global_mem_pool, DMA_COUNT_OUT * sizeof(uint32_t), 0, (void **)&ddr_ptr_out_0);
  hsa_amd_memory_pool_allocate(global_mem_pool, DMA_COUNT_OUT * sizeof(uint32_t), 0, (void **)&ddr_ptr_out_1);
  hsa_amd_memory_pool_allocate(global_mem_pool, DMA_COUNT_OUT * sizeof(uint32_t), 0, (void **)&ddr_ptr_out_2);
  hsa_amd_memory_pool_allocate(global_mem_pool, DMA_COUNT_OUT * sizeof(uint32_t), 0, (void **)&ddr_ptr_out_3);

  // initialize the external buffers
  for (int i = 0; i < DMA_COUNT_IN; i++) {
    *(ddr_ptr_in_0 + i) = i; // input
    *(ddr_ptr_in_1 + i) = i; // input
    *(ddr_ptr_in_2 + i) = i; // input
    *(ddr_ptr_in_3 + i) = i; // input
  }

  for (int i = 0; i < DMA_COUNT_OUT; i++) {
    *(ddr_ptr_out_0 + i) = 0; // input
    *(ddr_ptr_out_1 + i) = 0; // input
    *(ddr_ptr_out_2 + i) = 0; // input
    *(ddr_ptr_out_3 + i) = 0; // input
  }

  //////////////////////////////////////// B Block 0
  //
  // send the data
  //
  uint64_t wr_idx = hsa_queue_add_write_index_relaxed(queues[0], 1);
  uint64_t packet_id = wr_idx % queues[0]->size;
  hsa_agent_dispatch_packet_t pkt;
  air_packet_nd_memcpy(&pkt, 0, shim_one_col, 1, 0, 4, 2,
                       reinterpret_cast<uint64_t>(ddr_ptr_in_0),
                       DMA_COUNT_IN * sizeof(float), 1, 0, 1, 0, 1, 0);
  hsa_amd_signal_create_on_agent(1, 0, nullptr, &agents[0], 0, &pkt.completion_signal);
  reinterpret_cast<hsa_agent_dispatch_packet_t *>(queues[0]->base_address)[packet_id] = pkt;

  //
  // read the data
  //

  wr_idx = hsa_queue_add_write_index_relaxed(queues[0], 1);
  packet_id = wr_idx % queues[0]->size;
  hsa_agent_dispatch_packet_t pkt2;
  air_packet_nd_memcpy(&pkt2, 0, shim_one_col, 0, 0, 4, 2,
                       reinterpret_cast<uint64_t>(ddr_ptr_out_0),
                       DMA_COUNT_OUT * sizeof(float), 1, 0, 1, 0, 1, 0);
  hsa_amd_signal_create_on_agent(1, 0, nullptr, &agents[0], 0, &pkt2.completion_signal);
  reinterpret_cast<hsa_agent_dispatch_packet_t *>(queues[0]->base_address)[packet_id] = pkt2;

  //////////////////////////////////////// B Block 1
  //
  // send the data
  //

  wr_idx = hsa_queue_add_write_index_relaxed(queues[0], 1);
  packet_id = wr_idx % queues[0]->size;
  hsa_agent_dispatch_packet_t pkt3;
  air_packet_nd_memcpy(&pkt3, 0, shim_one_col, 1, 1, 4, 2,
                       reinterpret_cast<uint64_t>(ddr_ptr_in_1),
                       DMA_COUNT_IN * sizeof(float), 1, 0, 1, 0, 1, 0);
  hsa_amd_signal_create_on_agent(1, 0, nullptr, &agents[0], 0, &pkt3.completion_signal);
  reinterpret_cast<hsa_agent_dispatch_packet_t *>(queues[0]->base_address)[packet_id] = pkt3;

  //
  // read the data
  //

  wr_idx = hsa_queue_add_write_index_relaxed(queues[0], 1);
  packet_id = wr_idx % queues[0]->size;
  hsa_agent_dispatch_packet_t pkt4;
  air_packet_nd_memcpy(&pkt4, 0, shim_one_col, 0, 1, 4, 2,
                       reinterpret_cast<uint64_t>(ddr_ptr_out_1),
                       DMA_COUNT_OUT * sizeof(float), 1, 0, 1, 0, 1, 0);
  hsa_amd_signal_create_on_agent(1, 0, nullptr, &agents[0], 0, &pkt4.completion_signal);
  reinterpret_cast<hsa_agent_dispatch_packet_t *>(queues[0]->base_address)[packet_id] = pkt4;

  //////////////////////////////////////// B Block 2
  //
  // send the data
  //

  wr_idx = hsa_queue_add_write_index_relaxed(queues[0], 1);
  packet_id = wr_idx % queues[0]->size;
  hsa_agent_dispatch_packet_t pkt5;
  air_packet_nd_memcpy(&pkt5, 0, shim_two_col, 1, 0, 4, 2,
                       reinterpret_cast<uint64_t>(ddr_ptr_in_2),
                       DMA_COUNT_IN * sizeof(float), 1, 0, 1, 0, 1, 0);
  hsa_amd_signal_create_on_agent(1, 0, nullptr, &agents[0], 0, &pkt5.completion_signal);
  reinterpret_cast<hsa_agent_dispatch_packet_t *>(queues[0]->base_address)[packet_id] = pkt5;

  //
  // read the data
  //

  wr_idx = hsa_queue_add_write_index_relaxed(queues[0], 1);
  packet_id = wr_idx % queues[0]->size;
  hsa_agent_dispatch_packet_t pkt6;
  air_packet_nd_memcpy(&pkt6, 0, shim_two_col, 0, 0, 4, 2,
                       reinterpret_cast<uint64_t>(ddr_ptr_out_2),
                       DMA_COUNT_OUT * sizeof(float), 1, 0, 1, 0, 1, 0);
  hsa_amd_signal_create_on_agent(1, 0, nullptr, &agents[0], 0, &pkt6.completion_signal);
  reinterpret_cast<hsa_agent_dispatch_packet_t *>(queues[0]->base_address)[packet_id] = pkt6;

  //////////////////////////////////////// B Block 3
  //
  // send the data
  //

  wr_idx = hsa_queue_add_write_index_relaxed(queues[0], 1);
  packet_id = wr_idx % queues[0]->size;
  hsa_agent_dispatch_packet_t pkt7;
  air_packet_nd_memcpy(&pkt7, 0, shim_two_col, 1, 1, 4, 2,
                       reinterpret_cast<uint64_t>(ddr_ptr_in_3),
                       DMA_COUNT_IN * sizeof(float), 1, 0, 1, 0, 1, 0);
  hsa_amd_signal_create_on_agent(1, 0, nullptr, &agents[0], 0, &pkt7.completion_signal);
  reinterpret_cast<hsa_agent_dispatch_packet_t *>(queues[0]->base_address)[packet_id] = pkt7;

  //
  // read the data
  //

  wr_idx = hsa_queue_add_write_index_relaxed(queues[0], 1);
  packet_id = wr_idx % queues[0]->size;
  hsa_agent_dispatch_packet_t pkt8;
  air_packet_nd_memcpy(&pkt8, 0, shim_two_col, 0, 1, 4, 2,
                       reinterpret_cast<uint64_t>(ddr_ptr_out_3),
                       DMA_COUNT_OUT * sizeof(float), 1, 0, 1, 0, 1, 0);
  hsa_amd_signal_create_on_agent(1, 0, nullptr, &agents[0], 0, &pkt8.completion_signal);
  reinterpret_cast<hsa_agent_dispatch_packet_t *>(queues[0]->base_address)[packet_id] = pkt8;

  // Ringing the doorbell to notify the command processor of the packet
  hsa_signal_store_screlease(queues[0]->doorbell_signal, wr_idx);

  // wait for packet completion
  while (hsa_signal_wait_scacquire(pkt8.completion_signal,
                             HSA_SIGNAL_CONDITION_EQ, 0, 0x80000,
                             HSA_WAIT_STATE_ACTIVE) != 0);

  hsa_signal_destroy(pkt.completion_signal);
  hsa_signal_destroy(pkt2.completion_signal);
  hsa_signal_destroy(pkt3.completion_signal);
  hsa_signal_destroy(pkt4.completion_signal);
  hsa_signal_destroy(pkt5.completion_signal);
  hsa_signal_destroy(pkt6.completion_signal);
  hsa_signal_destroy(pkt7.completion_signal);
  hsa_signal_destroy(pkt8.completion_signal);

  for (int i = 0; i < 512; i++) {

    if(ddr_ptr_out_0[i] != 514 + i) {
      printf("[ERROR] 0x%x != 0x%x\n", ddr_ptr_out_0[i], 514 + i);
      errors++;
    }

    if (ddr_ptr_out_0[i] != ddr_ptr_out_1[i]) {
      printf("[ERROR] ddr_ptr_out_0[%d] (%d) != ddr_ptr_out_1[%d (%d)]\n", i,
             ddr_ptr_out_0[i], i, ddr_ptr_out_1[i]);
      errors++;
    }

    if (ddr_ptr_out_0[i] != ddr_ptr_out_2[i]) {
      printf("[ERROR] ddr_ptr_out_0[%d] (%d) != ddr_ptr_out_2[%d (%d)]\n", i,
             ddr_ptr_out_0[i], i, ddr_ptr_out_2[i]);
      errors++;
    }

    if (ddr_ptr_out_0[i] != ddr_ptr_out_3[i]) {
      printf("[ERROR] ddr_ptr_out_0[%d] (%d) != ddr_ptr_out_3[%d (%d)]\n", i,
             ddr_ptr_out_0[i], i, ddr_ptr_out_3[i]);
      errors++;
    }
  }

  // destroying the queue
  hsa_queue_destroy(queues[0]);
  hsa_amd_memory_pool_free((void *)ddr_ptr_in_0);
  hsa_amd_memory_pool_free((void *)ddr_ptr_in_1);
  hsa_amd_memory_pool_free((void *)ddr_ptr_in_2);
  hsa_amd_memory_pool_free((void *)ddr_ptr_in_3);
  hsa_amd_memory_pool_free((void *)ddr_ptr_out_0);
  hsa_amd_memory_pool_free((void *)ddr_ptr_out_1);
  hsa_amd_memory_pool_free((void *)ddr_ptr_out_2);
  hsa_amd_memory_pool_free((void *)ddr_ptr_out_3);

  // Check for errors
  int res = 0;
  if (!errors) {
    printf("PASS!\n");
    res = 0;
  } else {
    printf("Fail!\n");
    res = -1;
  }

  // Shutting down HSA
  hsa_ret = hsa_shut_down();
  if(hsa_ret != HSA_STATUS_SUCCESS) {
    printf("[ERROR] hsa_shut_down() failed\n");
    return HSA_STATUS_ERROR;
  }

  printf("test done weather predicted = chocolate :D\n");

  return 0;
}
