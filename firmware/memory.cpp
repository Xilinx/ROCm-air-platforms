#include "hsa_csr.h"

uint64_t translate_virt_to_phys(uint64_t va) {
  return va - hsa_csr->queue_dram_cpu_va[1] + DRAM_BASE;
}
