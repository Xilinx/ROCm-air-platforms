//===- control_bp.cpp -------------------------------------------*- C++ -*-===//
//
// Copyright (C) 2021-2022, Xilinx Inc.
// Copyright (C) 2022-2025, Advanced Micro Devices, Inc.
// SPDX-License-Identifier: MIT
//
//===----------------------------------------------------------------------===//

#include <cassert>
#include <cstdio>
#include <iostream>
#include <vector>
#include <cstdlib>

#include "hsa/hsa.h"
#include "hsa/hsa_ext_amd.h"

#define AIR_PKT_TYPE_BP_CONTROL 0x200L
#define AIR_PKT_TYPE_HELLO 0x12L

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

hsa_status_t air_packet_control_bp(hsa_agent_dispatch_packet_t *pkt, uint32_t command,
                                   uint32_t arg1, uint32_t arg2, hsa_signal_t completion_signal) {
  pkt->arg[0] = command;
  pkt->arg[1] = arg1;
  pkt->arg[2] = arg2;

  pkt->completion_signal = completion_signal;

  pkt->type = AIR_PKT_TYPE_BP_CONTROL;
  pkt->header = (HSA_PACKET_TYPE_AGENT_DISPATCH << HSA_PACKET_HEADER_TYPE);

  return HSA_STATUS_SUCCESS;
}

hsa_status_t air_packet_hello(hsa_agent_dispatch_packet_t *pkt) {
  pkt->type = AIR_PKT_TYPE_BP_CONTROL;
  pkt->header = (HSA_PACKET_TYPE_AGENT_DISPATCH << HSA_PACKET_HEADER_TYPE);
  return HSA_STATUS_SUCCESS;
}


int main(int argc, char *argv[]) {

  int command = 0;
  if (argc > 1) {
    command = atoi(argv[1]);
  }

  int arg1 = 0;
  if (argc > 2) {
    arg1 = atoi(argv[2]);
  }

  int arg2 = 0;
  if (argc > 3) {
    arg2 = atoi(argv[3]);
  }

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

  // packet creation
  uint64_t wr_idx = hsa_queue_add_write_index_relaxed(queues.front(), 1);
  uint64_t packet_id = wr_idx % queues.front()->size;
  hsa_agent_dispatch_packet_t pkt;

  air_packet_control_bp(&pkt, command, arg1, arg2, completion_signal);

  // dispatch and wait
  reinterpret_cast<hsa_agent_dispatch_packet_t *>(queues.front()->base_address)[packet_id] = pkt;
  hsa_signal_store_screlease(queues.front()->doorbell_signal, wr_idx);

  cout << "dispatching signal" << endl;

  // wait for packet completion
  while (hsa_signal_wait_scacquire(completion_signal,
                             HSA_SIGNAL_CONDITION_EQ, 0, 0x80000,
                             HSA_WAIT_STATE_ACTIVE) != 0);


  cout << "wait complete" << endl;

  // cleanup
  hsa_signal_destroy(completion_signal);
  hsa_queue_destroy(queues.front());

  hsa_ret = hsa_shut_down();
  if (hsa_ret != HSA_STATUS_SUCCESS) {
    cout << "[ERROR] air_shut_down() failed" << endl;
    return -1;
  }

  cout << "PASS!" << endl;
  return 0;
}
