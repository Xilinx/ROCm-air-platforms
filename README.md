# ROCm AIR Platforms

This ROCm AIR platforms repository contains a proof of concept implementation demonstrating a future extended ROCr release, unifying the runtime environment between CPUs, GPUs, and dedicated accelerators like the AMD AI Engine. In this experimental release, a converged ROCm runtime provides infrastructure to perform tasks such as system discovery, work dispatch, synchronization, and memory allocation for CPUs, GPUs, and now AMD AI Engine platforms. This lowers the barrier to entry for AMD AI Engines by providing a familiar runtime and user experience similar to that of other popular accelerators. This repository contains the hardware platform configuration, driver, firmware, runtime library and example code to demonstrate this proof of concept. The demonstration relies on an experimental [ROCm-air branch](https://github.com/RadeonOpenCompute/ROCR-Runtime/tree/experimental/rocm-5.6.x-air) containing the extensions to ROCr to support AI Engine devices. 

Both the experimental rocm-5.6.x-air branch and the ROCm-air-platforms repository are experimental prototypes of AMD research and not official AMD products. Both suites of software are provided as-is with no guarantees of support of AMD or AMD Research and Advanced Development.

## Getting Started

## Examples

A high level description of the provided example. The README in the example directory explains how to run it in detail. 

## Platforms

This repository provides a hardware platform configuratons to enable ROCr for AMD AI Engine devices.

### VCK5000 

The [VCK5000](https://www.xilinx.com/products/boards-and-kits/vck5000.html) is a PCIe form factor accelerator card, it contains a VC1902 device with 400 AI Engines capable of offering 145 INT8 TOPs.  In this release, only a fraction of the AI Engines available are used for acceleration. The platform contains a command processor complex implementing the HSA agent abstraction to handle architected queuing language (AQL) packets. 

## Firmware

Handles packets to configure the device and invoke data transfers.

## Driver 

Implements the interface for ROCr on the platform. 

## Runtime Library
