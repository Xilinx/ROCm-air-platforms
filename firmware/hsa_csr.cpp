#include "hsa_csr.h"

#include "debug.h"
#include "memory.h"

HsaControlStatusRegs *hsa_csr = nullptr;

void hsa_csr_init()
{
  air_printf("Initializing HSA CSRs.\n\r");
  hsa_csr = reinterpret_cast<HsaControlStatusRegs*>(BRAM_BASE);

  hsa_csr->version = 1;
  hsa_csr->num_aql_queues = NUM_AQL_QUEUES;
  hsa_csr->aql_queue_size = AQL_QUEUE_NUM_ENTRIES;
  hsa_csr->aql_queue_size_bytes
      = AQL_QUEUE_NUM_ENTRIES * AQL_QUEUE_BYTES_PER_ENTRY;

  hsa_csr->queue_desc_base = BRAM_BASE + PAGE_SIZE;
  hsa_csr->queue_desc_offset = hsa_csr->queue_desc_base - BRAM_BASE;
  hsa_csr->queue_desc_size = NUM_AQL_QUEUES * PAGE_SIZE;

  hsa_csr->queue_buf_base = hsa_csr->queue_desc_base + hsa_csr->queue_desc_size;
  hsa_csr->queue_buf_offset = hsa_csr->queue_buf_base - BRAM_BASE;
  hsa_csr->queue_buf_size = NUM_AQL_QUEUES * PAGE_SIZE;

  hsa_csr->heap_base = hsa_csr->queue_buf_base + hsa_csr->queue_buf_size;
  hsa_csr->heap_offset = hsa_csr->heap_base - BRAM_BASE;
  hsa_csr->heap_size = NUM_HEAP_PAGES * PAGE_SIZE;

  hsa_csr->doorbell_base = hsa_csr->heap_base + hsa_csr->heap_size;
  hsa_csr->doorbell_offset = hsa_csr->doorbell_base - BRAM_BASE;
  hsa_csr->doorbell_size = NUM_AQL_QUEUES * PAGE_SIZE;

  hsa_csr->global_barrier = 0;

  for (int i = 0; i < hsa_csr->num_aql_queues; ++i) {
    hsa_csr->amd_aql_queues[i] = reinterpret_cast<amd_queue_t*>(
        hsa_csr->queue_desc_base + i * PAGE_SIZE);
    hsa_csr->doorbells[i] = hsa_csr->doorbell_base + i * PAGE_SIZE;
    hsa_csr->queue_bufs[i] = hsa_csr->queue_buf_base + i * PAGE_SIZE;

    hsa_csr->amd_aql_queues[i]->read_dispatch_id = 0;
    hsa_csr->amd_aql_queues[i]->write_dispatch_id = 0;
    hsa_csr->amd_aql_queues[i]->hsa_queue.base_address
        = reinterpret_cast<void*>(hsa_csr->queue_bufs[i]);

    for (int j = 0; j < AQL_QUEUE_NUM_ENTRIES; ++j) {
      reinterpret_cast<hsa_agent_dispatch_packet_t*>(
          hsa_csr->queue_bufs[i])[j].header = HSA_PACKET_TYPE_INVALID;
    }
  }

  for (int i = 0; i < NUM_HEAP_PAGES; ++i) {
    hsa_csr->heap[i] = hsa_csr->heap_base + i * PAGE_SIZE;
  }

  hsa_csr->dna_reg[0] = *reinterpret_cast<uint32_t*>(0xf1250020);
  hsa_csr->dna_reg[1] = *reinterpret_cast<uint32_t*>(0xf1250024);
  hsa_csr->dna_reg[2] = *reinterpret_cast<uint32_t*>(0xf1250028);
  hsa_csr->dna_reg[3] = *reinterpret_cast<uint32_t*>(0xf125002c);
}

void hsa_csr_print()
{
  for (int i = 0; i < hsa_csr->num_aql_queues; ++i) {
    air_printf("queue %d: addr %p, buf addr %p, rd ptr %p, wr ptr %p, rd id %lx, wr id %lx\n\r",
               i, hsa_csr->amd_aql_queues[i],
               hsa_csr->amd_aql_queues[i]->hsa_queue.base_address,
               (void*)&hsa_csr->amd_aql_queues[i]->read_dispatch_id,
               (void*)&hsa_csr->amd_aql_queues[i]->write_dispatch_id,
	       hsa_csr->amd_aql_queues[i]->read_dispatch_id,
	       hsa_csr->amd_aql_queues[i]->write_dispatch_id);
  }

  for (int i = 0; i < 4; ++i) {
    air_printf("DNA reg%d: %x\n\r", i, hsa_csr->dna_reg[i]);
  }
}
