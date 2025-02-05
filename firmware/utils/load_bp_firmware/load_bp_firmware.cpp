//===- load_bp_firmware.cpp -------------------------------------*- C++ -*-===//
//
// Copyright (C) 2021-2022, Xilinx Inc.
// Copyright (C) 2022, Advanced Micro Devices, Inc.
// SPDX-License-Identifier: MIT
//
//===----------------------------------------------------------------------===//

#include <assert.h>
#include <cstdio>
#include <fcntl.h>
#include <iostream>
#include <stdlib.h>
#include <sys/mman.h>
#include <string.h>
#include <vector>
#include <stdio.h>

#include <sstream>
#include <iomanip>

#include "hsa/hsa.h"
#include "hsa/hsa_ext_amd.h"

#define AIR_PKT_TYPE_PROG_FIRMWARE 0x50L

// Use a global variable to story the memory pool information
hsa_amd_memory_pool_t global_mem_pool;

using namespace std;

hsa_status_t get_aie_agents(hsa_agent_t agent, void *data) {
  hsa_status_t status(HSA_STATUS_SUCCESS);
  hsa_device_type_t device_type;
  vector<hsa_agent_t> *aie_agents(nullptr);

  if (!data) {
    status = HSA_STATUS_ERROR_INVALID_ARGUMENT;
    return status;
  }

  aie_agents = static_cast<vector<hsa_agent_t>*>(data);
  status = hsa_agent_get_info(agent, HSA_AGENT_INFO_DEVICE, &device_type);

  if (status != HSA_STATUS_SUCCESS) {
    return status;
  }

  if (device_type == HSA_DEVICE_TYPE_AIE) {
    aie_agents->push_back(agent);
  }

  return status;
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

hsa_status_t air_packet_program_bp_firmware(hsa_agent_dispatch_packet_t *pkt, uint64_t virt_addr,
                                            uint32_t file_num_lines, uint32_t num_bp,
                                            hsa_signal_t completion_signal) {
  pkt->arg[0] = virt_addr;
  pkt->arg[1] = (uint64_t)file_num_lines;
  pkt->arg[2] = (uint64_t)num_bp;

  pkt->completion_signal = completion_signal;

  pkt->type = AIR_PKT_TYPE_PROG_FIRMWARE;
  pkt->header = (HSA_PACKET_TYPE_AGENT_DISPATCH << HSA_PACKET_HEADER_TYPE);

  return HSA_STATUS_SUCCESS;
}


int main(int argc, char *argv[]) {

  // Getting the binary to load
  string file_name = "main.mem";
  if(argc > 1) {
    file_name = argv[1];
  }

  // get number of BP cores to program
  int num_bp = 0;
  if (argc > 2) {
    num_bp = atoi(argv[2]);
  }

  FILE *mem_fd = fopen(file_name.c_str(), "rb");
  if(mem_fd == NULL) {
    printf("[ERROR] Cannot find file %s\n", file_name.c_str());
    return -1;
  }
  // Need to get the size of the file
  fseek(mem_fd, 0, SEEK_END);
  uint32_t file_size = ftell(mem_fd);
  fseek(mem_fd, 0, SEEK_SET); // Have to reset the ptr

  // Need to get the number of lines of the file
  uint32_t file_num_lines = 0;
  char * num_lines_line = NULL;
  size_t num_lines_len = 0;
  ssize_t num_lines_read;
  while((num_lines_read = getline(&num_lines_line, &num_lines_len, mem_fd)) != -1) {
    file_num_lines++;
  }

  printf("Loading elf from %s of size %d and %d lines into all BPs\n", file_name.c_str(), file_size, file_num_lines);

  // Use mmap to treat the file like an array
  void *elf_host_mem = mmap(NULL, file_size, PROT_READ, MAP_PRIVATE, fileno(mem_fd), 0);

  // HSA datastructures
  vector<hsa_agent_t> agents;
  vector<hsa_queue_t *> queues;
  uint32_t aie_max_queue_size(0);

  // Initializing HSA
  hsa_status_t hsa_ret = hsa_init();
  if (hsa_ret != HSA_STATUS_SUCCESS) {
    cout << "hsa_init failed" << endl;
    return -1;
  }

  // Finding all AIE HSA agents
  hsa_iterate_agents(&get_aie_agents, reinterpret_cast<void*>(&agents));
  if (agents.empty()) {
    cout << "failed to find any AIE agents" << endl;
    return -1;
  }

  // Iterating over memory pools to initialize our allocator
  hsa_amd_agent_iterate_memory_pools(agents.front(),
                                     get_global_mem_pool,
                                     reinterpret_cast<void*>(&global_mem_pool));

  // Getting the size of queue the agent supports
  hsa_agent_get_info(agents.front(), HSA_AGENT_INFO_QUEUE_MAX_SIZE, &aie_max_queue_size);

  // Creating a queue
  hsa_queue_t *q = NULL;
  auto queue_create_status = hsa_queue_create(agents.front(), aie_max_queue_size,
                              HSA_QUEUE_TYPE_SINGLE, nullptr, nullptr, 0,
                              0, &q);

  if(queue_create_status != HSA_STATUS_SUCCESS) {
    cout << "hsa_queue_create failed" << endl;
    hsa_shut_down();
    return -1;
  }

  // Adding to our vector of queues
  queues.push_back(q);
  if(queues.size() == 0) {
    cout << "No queues were sucesfully created!" << endl;
    hsa_queue_destroy(queues.front());
    hsa_shut_down();
    return -1;
  }

  // completion signal
  hsa_signal_t completion_signal;
  hsa_amd_signal_create_on_agent(1, 0, nullptr, agents.data(), 0, &completion_signal);

  // allocate device buffer for BP firmware and copy from elf_host_mem to buffer
  void *elf_dev_mem = nullptr;
  hsa_amd_memory_pool_allocate(global_mem_pool, file_size, 0, &elf_dev_mem);
  cout << "Allocating firmware at VA: 0x" << hex << elf_dev_mem << endl;
  memcpy(elf_dev_mem, elf_host_mem, file_size);

  // packet creation
  uint64_t wr_idx = hsa_queue_add_write_index_relaxed(queues.front(), 1);
  uint64_t packet_id = wr_idx % queues.front()->size;
  hsa_agent_dispatch_packet_t pkt;

  air_packet_program_bp_firmware(&pkt, reinterpret_cast<uint64_t>(elf_dev_mem),
                                 file_num_lines, num_bp, completion_signal);

  // dispatch and wait
  reinterpret_cast<hsa_agent_dispatch_packet_t *>(queues.front()->base_address)[packet_id] = pkt;
  hsa_signal_store_screlease(queues.front()->doorbell_signal, wr_idx);

  cout << "dispatching packet" << endl;

  // wait for packet completion
  while (hsa_signal_wait_scacquire(completion_signal,
                             HSA_SIGNAL_CONDITION_EQ, 0, 0x80000,
                             HSA_WAIT_STATE_ACTIVE) != 0);


  cout << "wait complete" << endl;

  // cleanup
  munmap(elf_host_mem, file_size);
  fclose(mem_fd);
  hsa_signal_destroy(completion_signal);
  hsa_queue_destroy(queues.front());
  hsa_amd_memory_pool_free(elf_dev_mem);

  hsa_ret = hsa_shut_down();
  if (hsa_ret != HSA_STATUS_SUCCESS) {
    cout << "[ERROR] air_shut_down() failed" << endl;
    return -1;
  }

  cout << "PASS!" << endl;
  return 0;
}
