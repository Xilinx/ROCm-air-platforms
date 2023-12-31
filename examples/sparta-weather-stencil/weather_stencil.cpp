// (C) 2023, Advanced Micro Devices, Inc.
// (c) 2023 SAFARI Research Group at ETH Zurich, Gagandeep Singh, D-ITET
// SPDX-License-Identifier: MIT

#include <algorithm>
#include <cstring>
#include <fcntl.h>
#include <gelf.h>
#include <iostream>
#include <numeric>
#include <set>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

#include "hsa/hsa.h"
#include "hsa/hsa_ext_amd.h"

// Used to define the size of the application data
auto constexpr b_block_depth = 4; // set how many rows
auto constexpr input_rows = 9;
auto constexpr dma_count_in = 256 * input_rows;
auto constexpr dma_count_out = 256 * 2 * b_block_depth;

// NOTE: These will shortly be moved to
// the converged ROCm runtime
namespace air::pkt::type {
auto constexpr nd_memcpy = 0x0103L;
auto constexpr airbin = 0x53L;
} // namespace air::pkt::type


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

// Use a global variable to store the memory pool information
hsa_amd_memory_pool_t global_mem_pool;

/*
  'table' is an offset from the beginning of device memory
*/
hsa_status_t air_packet_load_airbin(hsa_agent_dispatch_packet_t *pkt,
                                    uint64_t table, uint16_t column) {
  pkt->type = air::pkt::type::airbin;
  pkt->header = HSA_PACKET_TYPE_AGENT_DISPATCH << HSA_PACKET_HEADER_TYPE;
  pkt->arg[0] = table;
  pkt->arg[1] = column;

  return HSA_STATUS_SUCCESS;
}

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

  pkt->type = air::pkt::type::nd_memcpy;
  pkt->header = HSA_PACKET_TYPE_AGENT_DISPATCH << HSA_PACKET_HEADER_TYPE;

  return HSA_STATUS_SUCCESS;
}

/*
  Load an airbin from a file into a device
*/
hsa_status_t air_load_airbin(hsa_agent_t agent, hsa_queue_t *q,
                             const char *filename, uint8_t column) {
  uint8_t *dram_ptr = NULL;
  uint8_t *data_ptr = NULL;
  Elf *inelf = NULL;
  GElf_Ehdr *ehdr = NULL;
  GElf_Ehdr ehdr_mem;
  uint64_t wr_idx = 0;
  uint32_t table_idx = 0;
  airbin_table_entry *airbin_table;
  uint32_t data_offset = 0;
  uint32_t table_size = 0;

  // The starting colums of the DUs on the VCK5000 are: 2, 10, 18, 26, 34, and 42
  if (!std::set {2, 10, 18, 26, 34, 42}.contains(column))
    return HSA_STATUS_ERROR_INVALID_ARGUMENT;

  // open the AIRBIN file
  int elf_fd = open(filename, O_RDONLY);
  hsa_status_t ret = HSA_STATUS_SUCCESS;
  if (elf_fd < 0) {
    std::cerr << "Can't open " << filename << std::endl;
    ret = HSA_STATUS_ERROR_INVALID_FILE;
    goto err_elf_open;
  }



  // check the characteristics
  elf_version(EV_CURRENT);
  inelf = elf_begin(elf_fd, ELF_C_READ, NULL);
  ehdr = gelf_getehdr(inelf, &ehdr_mem);
  if (ehdr == NULL) {
    std::cerr << "cannot get ELF header: " <<  elf_errmsg(-1) << std::endl;
    ret = HSA_STATUS_ERROR_INVALID_FILE;
    goto err_elf_read;
  }

  // Read data as 64-bit little endian
  if ((ehdr->e_ident[EI_CLASS] != ELFCLASS64) ||
      (ehdr->e_ident[EI_DATA] != ELFDATA2LSB)) {
    std::cerr << "unexpected ELF format\n" << std::endl;
    ret = HSA_STATUS_ERROR_INVALID_FILE;
    goto err_elf_read;
  }

  size_t shnum;
  if (elf_getshdrnum(inelf, &shnum) != 0) {
    std::cerr << "cannot get program header count: " << elf_errmsg(-1) << std::endl;
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

  // calculate the size needed to load the configuration and allocate that 
  // amount of device memory
  struct stat elf_stat;
  fstat(elf_fd, &elf_stat);
  hsa_amd_memory_pool_allocate(global_mem_pool, elf_stat.st_size + table_size, 0, (void **)&dram_ptr);
  if (dram_ptr == NULL) {
    std::cerr << "Error allocating " << elf_stat.st_size + table_size << " DRAM"<< std::endl;
    ret = HSA_STATUS_ERROR_OUT_OF_RESOURCES;
    goto err_dev_mem_alloc;
  }

  // Now that we know we have enough space for the data and table, can start
  // writing to device memory
  airbin_table = (airbin_table_entry *)dram_ptr;
  data_offset = table_size; // The data offset starts at the end of the table
  data_ptr = dram_ptr + table_size;

  // Iterate through all sections to create a table in device-readable format.
  for (unsigned int ndx = 0; ndx < shnum; ndx++) {
    GElf_Shdr shdr;
    Elf_Scn *sec = elf_getscn(inelf, ndx);
    if (sec == NULL) {
      std::cerr << "cannot get section " <<  ndx << " " << elf_errmsg(-1) << std::endl;
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
      std::cerr << "Error reading data for section" << ndx << std::endl;
      ret = HSA_STATUS_ERROR_INVALID_FILE;
      goto err_elf_read;
    }
    memcpy(data_ptr, desc->d_buf, desc->d_size);

    airbin_table[table_idx] = { data_offset, uint32_t(shdr.sh_size), shdr.sh_addr };

    table_idx++;
    data_offset += shdr.sh_size;
    data_ptr += shdr.sh_size;
  }

  // the last entry must be all 0's
  airbin_table[table_idx]= { 0, 0, 0 };

  // Send configuration packet
  wr_idx = hsa_queue_add_write_index_relaxed(q, 1);
  hsa_agent_dispatch_packet_t pkt;
  air_packet_load_airbin(&pkt, (uint64_t)airbin_table, (uint16_t)column);

  // dispatch and wait has blocking semantics so we can internally create the signal
  hsa_amd_signal_create_on_agent(1, 0, nullptr, &agent, 0, &(pkt.completion_signal));

  // Write the packet to the queue
  reinterpret_cast<hsa_agent_dispatch_packet_t *>(q->base_address)[wr_idx % q->size] = pkt;

  // Ringing the doorbell
  hsa_signal_store_screlease(q->doorbell_signal, wr_idx);

  // wait for packet completion
  while (hsa_signal_wait_scacquire(pkt.completion_signal,
                             HSA_SIGNAL_CONDITION_EQ, 0, 0x80000,
                             HSA_WAIT_STATE_ACTIVE) != 0);

  hsa_signal_destroy(pkt.completion_signal);


err_elf_read:
  elf_end(inelf);
  close(elf_fd);

err_elf_open:
err_dev_mem_alloc:
  return ret;
}

/// Call back function used by agent enumeration
hsa_status_t get_aie_agent(hsa_agent_t agent, void *data) {
  if (!data)
    return HSA_STATUS_ERROR_INVALID_ARGUMENT;

  auto aie_agents = static_cast<std::vector<hsa_agent_t>*>(data);
  hsa_device_type_t device_type;
  if (hsa_status_t status =
          hsa_agent_get_info(agent, HSA_AGENT_INFO_DEVICE, &device_type);
      status != HSA_STATUS_SUCCESS)
    return status;

  // Printing what type of agent we found
  if(device_type == HSA_DEVICE_TYPE_CPU) {
    std::cout << "Found a CPU HSA agent" << std::endl;
  }
  else if(device_type == HSA_DEVICE_TYPE_GPU) {
    std::cout << "Found a GPU HSA agent" << std::endl;
  }
  else if(device_type == HSA_DEVICE_TYPE_AIE) {
    std::cout << "Found an AIE HSA agent" << std::endl;
  }
  else {
    std::cout << "[WARNING] Unknown device type found" << std::endl;
  }

  if (device_type == HSA_DEVICE_TYPE_AIE)
    aie_agents->push_back(agent);

  return HSA_STATUS_SUCCESS;
}

hsa_status_t get_global_mem_pool(hsa_amd_memory_pool_t pool, void *data) {
  hsa_status_t status(HSA_STATUS_SUCCESS);
  hsa_region_segment_t segment_type;
  status = hsa_amd_memory_pool_get_info(pool, HSA_AMD_MEMORY_POOL_INFO_SEGMENT,
                                        &segment_type);
  if (segment_type == HSA_REGION_SEGMENT_GLOBAL) {
    *reinterpret_cast<hsa_amd_memory_pool_t*>(data) = pool;
  }

  return status;
}

void print_weather(bool passed) {
  if (passed) {
    std::cout << "      ;   :   ;       " << std::endl; 
    std::cout << "   .   \\_,!,_/   ,    " << std::endl; 
    std::cout << "    `.,'     `.,'     " << std::endl; 
    std::cout << "     /         \\      " << std::endl; 
    std::cout << "~ -- :  PASS   : -- ~ " << std::endl; 
    std::cout << "     \\         /      " << std::endl; 
    std::cout << "    ,'`._   _.'`.     " << std::endl; 
    std::cout << "   '   / `!` \\   `    " << std::endl; 
    std::cout << "      ;   :   ;       " << std::endl; 
  } else {
    std::cout << "   __(                  ) " << std::endl;
    std::cout << "  (_      FAIL      __))  " << std::endl;
    std::cout << "    ((           __)      " << std::endl;
    std::cout << "      (______)--'         " << std::endl;
    std::cout << "      _/  /               " << std::endl;
    std::cout << "     / __/                " << std::endl;
    std::cout << "    / /                   " << std::endl;
    std::cout << "   //                     " << std::endl;
    std::cout << "  /'                      " << std::endl;
  }
}

int main(int argc, char *argv[]) {

  // Starting in the first DU of the VCK5000
  // DUs start in the following columns:
  // 2, 10, 18, 26, 34, and 42.
  uint64_t starting_col = 2;

  // Initializing HSA
  hsa_status_t hsa_ret = hsa_init();
  if (hsa_ret != HSA_STATUS_SUCCESS) {
    std::cerr << "hsa_init failed!" << std::endl;
    return -1;
  }

  // Finding all AIE HSA agents
  std::vector<hsa_agent_t> agents;
  hsa_iterate_agents(get_aie_agent, reinterpret_cast<void*>(&agents));
  if (agents.empty()) {
    std::cerr << "No AIE HSA agent found!" << std::endl;
    return -1;
  }
  // Use the first agent
  auto agent = agents.front();

  // Iterating over memory pools to initialize our allocator on first agent
  hsa_amd_agent_iterate_memory_pools(agent,
                                     get_global_mem_pool,
                                     reinterpret_cast<void*>(&global_mem_pool));

  // Getting the size of queue the agent supports
  uint32_t aie_max_queue_size;
  hsa_agent_get_info(agent, HSA_AGENT_INFO_QUEUE_MAX_SIZE, &aie_max_queue_size);

  // Creating a queue
  hsa_queue_t *queue;
  auto queue_create_status = hsa_queue_create(agent, aie_max_queue_size,
                              HSA_QUEUE_TYPE_SINGLE, nullptr, nullptr, 0,
                              0, &queue);

  if (queue_create_status != HSA_STATUS_SUCCESS) {
    std::cerr << "hsa_queue_create failed" << std::endl;
    hsa_shut_down();
    return -1;
  }

  // Configuring the device
  auto airbin_ret = air_load_airbin(agent, queue, "sparta-1DU.elf", starting_col);
  if (airbin_ret != HSA_STATUS_SUCCESS) {
    std::cerr << "Loading airbin failed: " << airbin_ret << std::endl;
    hsa_queue_destroy(queue);
    hsa_shut_down();
    return -1;
  }

  // Allocate and initialize the input buffers with some device memory
  auto allocate_and_init_input_buffer = [&] {
    uint32_t *ddr_ptr_in;
    hsa_amd_memory_pool_allocate(global_mem_pool,
                                 dma_count_in * sizeof(uint32_t), 0,
                                 (void **)&ddr_ptr_in);
    // Fill with 0, 1, ..., dma_count_in-1.
    std::iota(ddr_ptr_in, ddr_ptr_in + dma_count_in, 0);
    return ddr_ptr_in;
  };
  auto ddr_ptr_in_0 = allocate_and_init_input_buffer();
  auto ddr_ptr_in_1 = allocate_and_init_input_buffer();
  auto ddr_ptr_in_2 = allocate_and_init_input_buffer();
  auto ddr_ptr_in_3 = allocate_and_init_input_buffer();

  // Make sure data was written to the device correctly
  for (int i = 0; i < dma_count_in; i++) {
    if (ddr_ptr_in_0[i] != i || ddr_ptr_in_1[i] != i || ddr_ptr_in_2[i] != i ||
        ddr_ptr_in_3[i] != i) {
      std::cerr << "[ERROR] Input buffer was not written to correctly\n"
                << std::endl;
      hsa_queue_destroy(queue);
      hsa_shut_down();
      return -1;
    }
  }

  // Allocate and initialize the output buffers with some device memory
  auto allocate_and_init_output_buffer = [&] {
    uint32_t *ddr_ptr_out;
    hsa_amd_memory_pool_allocate(global_mem_pool,
                                 dma_count_in * sizeof(uint32_t), 0,
                                 (void **)&ddr_ptr_out);
    // Fill with 0
    std::fill(ddr_ptr_out, ddr_ptr_out + dma_count_in, 0);
    return ddr_ptr_out;
  };
  auto ddr_ptr_out_0 = allocate_and_init_output_buffer();
  auto ddr_ptr_out_1 = allocate_and_init_output_buffer();
  auto ddr_ptr_out_2 = allocate_and_init_output_buffer();
  auto ddr_ptr_out_3 = allocate_and_init_output_buffer();

  // Creating one signal for all DMA packets.
  // Each packet completion will decrement the signal.
  // Once it reaches zero we will know that all DMAs are complete.
  hsa_signal_t dma_signal;
  hsa_amd_signal_create_on_agent(8, 0, nullptr, &agent, 0, &dma_signal);

  //////////////////////////////////////// B Block 0
  //
  // send the data
  //
  uint64_t wr_idx = hsa_queue_add_write_index_relaxed(queue, 1);
  uint64_t packet_id = wr_idx % queue->size;
  hsa_agent_dispatch_packet_t pkt;
  air_packet_nd_memcpy(&pkt, 0, starting_col, 1, 0, 4, 2,
                       reinterpret_cast<uint64_t>(ddr_ptr_in_0),
                       dma_count_in * sizeof(float), 1, 0, 1, 0, 1, 0);
  pkt.completion_signal = dma_signal;
  reinterpret_cast<hsa_agent_dispatch_packet_t *>(queue->base_address)[packet_id] = pkt;

  //
  // read the data
  //

  wr_idx = hsa_queue_add_write_index_relaxed(queue, 1);
  packet_id = wr_idx % queue->size;
  hsa_agent_dispatch_packet_t pkt2;
  air_packet_nd_memcpy(&pkt2, 0, starting_col, 0, 0, 4, 2,
                       reinterpret_cast<uint64_t>(ddr_ptr_out_0),
                       dma_count_out * sizeof(float), 1, 0, 1, 0, 1, 0);
  pkt2.completion_signal = dma_signal;
  reinterpret_cast<hsa_agent_dispatch_packet_t *>(queue->base_address)[packet_id] = pkt2;

  //////////////////////////////////////// B Block 1
  //
  // send the data
  //

  wr_idx = hsa_queue_add_write_index_relaxed(queue, 1);
  packet_id = wr_idx % queue->size;
  hsa_agent_dispatch_packet_t pkt3;
  air_packet_nd_memcpy(&pkt3, 0, starting_col, 1, 1, 4, 2,
                       reinterpret_cast<uint64_t>(ddr_ptr_in_1),
                       dma_count_in * sizeof(float), 1, 0, 1, 0, 1, 0);
  pkt3.completion_signal = dma_signal;
  reinterpret_cast<hsa_agent_dispatch_packet_t *>(queue->base_address)[packet_id] = pkt3;

  //
  // read the data
  //

  wr_idx = hsa_queue_add_write_index_relaxed(queue, 1);
  packet_id = wr_idx % queue->size;
  hsa_agent_dispatch_packet_t pkt4;
  air_packet_nd_memcpy(&pkt4, 0, starting_col, 0, 1, 4, 2,
                       reinterpret_cast<uint64_t>(ddr_ptr_out_1),
                       dma_count_out * sizeof(float), 1, 0, 1, 0, 1, 0);
  pkt4.completion_signal = dma_signal;
  reinterpret_cast<hsa_agent_dispatch_packet_t *>(queue->base_address)[packet_id] = pkt4;

  //////////////////////////////////////// B Block 2
  //
  // send the data
  //

  wr_idx = hsa_queue_add_write_index_relaxed(queue, 1);
  packet_id = wr_idx % queue->size;
  hsa_agent_dispatch_packet_t pkt5;
  air_packet_nd_memcpy(&pkt5, 0, starting_col+1, 1, 0, 4, 2,
                       reinterpret_cast<uint64_t>(ddr_ptr_in_2),
                       dma_count_in * sizeof(float), 1, 0, 1, 0, 1, 0);
  pkt5.completion_signal = dma_signal;
  reinterpret_cast<hsa_agent_dispatch_packet_t *>(queue->base_address)[packet_id] = pkt5;

  //
  // read the data
  //

  wr_idx = hsa_queue_add_write_index_relaxed(queue, 1);
  packet_id = wr_idx % queue->size;
  hsa_agent_dispatch_packet_t pkt6;
  air_packet_nd_memcpy(&pkt6, 0, starting_col+1, 0, 0, 4, 2,
                       reinterpret_cast<uint64_t>(ddr_ptr_out_2),
                       dma_count_out * sizeof(float), 1, 0, 1, 0, 1, 0);
  pkt6.completion_signal = dma_signal;
  reinterpret_cast<hsa_agent_dispatch_packet_t *>(queue->base_address)[packet_id] = pkt6;

  //////////////////////////////////////// B Block 3
  //
  // send the data
  //

  wr_idx = hsa_queue_add_write_index_relaxed(queue, 1);
  packet_id = wr_idx % queue->size;
  hsa_agent_dispatch_packet_t pkt7;
  air_packet_nd_memcpy(&pkt7, 0, starting_col+1, 1, 1, 4, 2,
                       reinterpret_cast<uint64_t>(ddr_ptr_in_3),
                       dma_count_in * sizeof(float), 1, 0, 1, 0, 1, 0);
  pkt7.completion_signal = dma_signal;
  reinterpret_cast<hsa_agent_dispatch_packet_t *>(queue->base_address)[packet_id] = pkt7;

  //
  // read the data
  //

  wr_idx = hsa_queue_add_write_index_relaxed(queue, 1);
  packet_id = wr_idx % queue->size;
  hsa_agent_dispatch_packet_t pkt8;
  air_packet_nd_memcpy(&pkt8, 0, starting_col+1, 0, 1, 4, 2,
                       reinterpret_cast<uint64_t>(ddr_ptr_out_3),
                       dma_count_out * sizeof(float), 1, 0, 1, 0, 1, 0);
  pkt8.completion_signal = dma_signal;
  reinterpret_cast<hsa_agent_dispatch_packet_t *>(queue->base_address)[packet_id] = pkt8;

  // Ringing the doorbell to notify the command processor of the packet
  hsa_signal_store_screlease(queue->doorbell_signal, wr_idx);

  // wait for packet completion
  while (hsa_signal_wait_scacquire(dma_signal,
                             HSA_SIGNAL_CONDITION_EQ, 0, 0x80000,
                             HSA_WAIT_STATE_ACTIVE) != 0);

  // Destroying the signal
  hsa_signal_destroy(dma_signal);

  int errors = 0;
  for (int i = 0; i < 512; i++) {
    if (ddr_ptr_out_0[i] != 514 + i) {
      std::cerr << "[ERROR] " << ddr_ptr_out_0[i] << " != " << 514 + i << std::endl;
      errors++;
    }

    if (ddr_ptr_out_0[i] != ddr_ptr_out_1[i]) {
      std::cerr << "[ERROR] ddr_ptr_out_0[" << i << "] (" << ddr_ptr_out_0[i] << ")  != ddr_ptr_out_1[" << i << "] (" << ddr_ptr_out_1[i] << ")" << std::endl;
      errors++;
    }

    if (ddr_ptr_out_0[i] != ddr_ptr_out_2[i]) {
      std::cerr << "[ERROR] ddr_ptr_out_0[" << i << "] (" << ddr_ptr_out_0[i] << ")  != ddr_ptr_out_2[" << i << "] (" << ddr_ptr_out_2[i] << ")" << std::endl;
      errors++;
    }

    if (ddr_ptr_out_0[i] != ddr_ptr_out_3[i]) {
      std::cerr << "[ERROR] ddr_ptr_out_0[" << i << "] (" << ddr_ptr_out_0[i] << ")  != ddr_ptr_out_3[" << i << "] (" << ddr_ptr_out_3[i] << ")" << std::endl;
      errors++;
    }
  }

  // destroying the queue
  hsa_queue_destroy(queue);
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
    print_weather(true);
    res = 0;
  } else {
    print_weather(false);
    res = -1;
  }

  // Shutting down HSA
  hsa_ret = hsa_shut_down();
  if (hsa_ret != HSA_STATUS_SUCCESS) {
    std::cerr << "[ERROR] hsa_shut_down() failed\n" << std::endl;
    return HSA_STATUS_ERROR;
  }

  return res;
}
