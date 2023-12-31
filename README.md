# ROCm AIR Platforms

This repository provides a proof of concept platform implementation supporting an experimental ROCm runtime release. The experimental release envisions a unified runtime environment between CPUs, GPUs, and dedicated accelerators like the AMD AI Engine. This converged ROCm runtime provides infrastructure to perform tasks such as system discovery, work dispatch, synchronization, and memory allocation for CPUs, GPUs, and now AMD AI Engine platforms. The barrier to entry for AMD AI Engines is substantially lowered by providing a familiar runtime and user experience similar to that of other popular accelerators. This repository contains the hardware platform configuration, driver, firmware, and example code to demonstrate this proof of concept. The demonstration relies on an experimental [rocm-5.6.x-air](https://github.com/RadeonOpenCompute/ROCR-Runtime/tree/experimental/rocm-5.6.x-air) branch containing the extensions to the ROCm runtime to support AMD AI Engine devices. 

Both the experimental [rocm-5.6.x-air](https://github.com/RadeonOpenCompute/ROCR-Runtime/tree/experimental/rocm-5.6.x-air) branch and this ROCm-air-platforms repository are experimental prototypes of AMD research and not official AMD products. Both suites of software are provided as-is with no guarantees of support of AMD or AMD Research and Advanced Development.

## Getting Started

1. **Install the ROCm release:** Follow the following steps to install ROCm and our experimental converged ROCm runtime:
  
    1. Follow the instructions provided to [install a global version of ROCM 5.6](https://rocm.docs.amd.com/en/docs-5.6.0/deploy/linux/os-native/install.html). Note that it is necessary to specifically install the ROCm HIP SDK for 5.6 using `sudo apt install rocm-hip-sdk5.6.0`. If a smaller install is required, one can just install `rocm-hip-runtime-dev5.6.0`.
    3. We will then install local copies of ROCt and our experimental ROCr which utilize this global install. We assume that ROCm is installed in `/opt/rocm`. If it is not, user provided arguments can be applied to the following scripts to point to the correct location.
    4. Locally install ROCt 5.6. We provide a script to clone the repository from GitHub and install the library. Simply run `./utils/clone-build-roct.sh`.
    5. Locally install our [experimental converged ROCm runtime (ROCr) 5.6](https://github.com/RadeonOpenCompute/ROCR-Runtime/tree/experimental/rocm-5.6.x-air). We provide a script to clone the repository from GitHub and install the library. Simply run `./utils/clone-build-rocr.sh`
    6. Now, set the environment variable `${ROCM_ROOT}` to `./rocm`. This environment variable is used to point to the local ROCr runtime install when compiling our provided examples. 
   
2. **Configure the board:** Refer to the [platform documentation](platform/vck5000) on how to load the platform on the VCK5000.
   
3. **Insert the driver:** Refer to the [driver documentation](driver) on how to compile and load the driver.
   
4. **Run the example:** Refer to the [weather stencil application](examples/sparta-weather-stencil) on how to compile and execute the application on the ROCm AIR platform.

## Examples

We provide an [examples directory](examples) which houses examples which utilize the ROCm AIR platform and the experimental rocm-air runtime. Each example contains dedicated README which describes both the computation performed and a guide for users to compile and run it themselves. Below is the current list of examples in the directory: 

* **examples/sparta-weather-stencil:** Real-world climate and weather simulations involve the utilization of complex compound stencil kernels, which are composed of a combination of different stencils. Horizontal diffusion is one such important compound stencil found in many regional global climate and weather prediction models. The horizontal diffusion kernel is mapped across three AI Engine tiles. We carefully hand-tune the code to overlap memory operations with arithmetic operations, to improve performance. We use the AIE data forwarding interfaces to forward the results from the first AIE core (used for Laplacian calculation) to the subsequent AIE cores (used for flux calculation). This approach allows for the concurrent execution of multiple stencil calculations, which can increase the overall performance and throughput of the hdiff design.  The performance of the hdiff implementation can be maximized by scaling it out across as many AIE cores as possible while avoiding data starvation. We develop a bundle or B-block-based design. A B-block is a cluster of AIE cores connected to the same shimDMA input/output channel. In our design, we choose a B-Block to consist of 12-AI Engine tiles, of which we utilize four to consume a total of 48 AI Engine tiles. 

## Platform

This repository provides a hardware platform configuraton to enable ROCm for AMD AI Engine devices. We currently only provide a platform which runs on the AMD Versal VCK5000.  

### VCK5000 

The [VCK5000](https://www.xilinx.com/products/boards-and-kits/vck5000.html) is a PCIe form factor accelerator card, it contains a VC1902 device with 400 AI Engines capable of offering 145 INT8 TOPs.  In this release, only a fraction of the AI Engines available are used for acceleration. The platform contains a command processor complex implementing the HSA agent abstraction to handle architected queuing language (AQL) packets. 

## Firmware

We provide the firmware which runs on the ARM core in the [firmware directory](firmware). The firmware describes how the ARM implements the HSA agent abstraction and handles AQL packets. Most users will not need to touch this as the provided [platform configuration](platform/vck5000) contains the default firmware. For users looking to modify the firmware, please refer to the [firmware documentation](firmware) for more information on how to compile and dynamically load new firmware on the device. 

## Driver 

The amdair driver implements the interface for the ROCm runtime on the platform. Refer to the [driver documentation](driver) for more information.

## Frequently Asked Questions

**Q: What is this release?**

A: This proof of concept envisions a future extended ROCm runtime release, unifying the runtime environment between CPUs, GPUs, and dedicated accelerators like the AMD AI Engine. In this experimental release, a converged ROCm runtime provides infrastructure to perform tasks such as system discovery, work dispatch, synchronization, and memory allocation for CPUs, GPUs, and now AMD AI Engine platforms. The barrier to entry for AMD AI Engines is substantially lowered by providing a familiar runtime and user experience similar to that of other popular accelerators.

**Q: What can I do with this release?**

A: This experimental release allows users to interact with AMD AIE Engines using the same ROCm runtime infrastructure that targets CPUs and GPUs. Along with this functionality, we provide a horizontal diffusion weather stencil HPC application mapped to AMD AI Engines on an AMD VCK5000, which we refer to as [SPARTA](https://arxiv.org/pdf/2303.03509.pdf). This application mapped to AMD AI Engines outperforms state-of-the-art CPU, GPU, and FPGA implementations by 17.1x, 1.2x, and 2.1x respectively with extremely low power consumption. The application utilizes the experimental converged ROCm runtime to perform tasks such as system discovery, work dispatch, synchronization, and memory allocation.

In this proof of concept, we are limiting the scope of the supported use cases to this single application.

**Q: Is this a product or a proof of concept?**

A: This is strictly a proof of concept.

**Q: Tell me more about what is being accelerated?**

A: **[examples/sparta-weather-stencil](examples/sparta-weather-stencil):** Real-world climate and weather simulations involve the utilization of complex compound stencil kernels, which are composed of a combination of different stencils. Horizontal diffusion is one such important compound stencil found in many regional global climate and weather prediction models. The horizontal diffusion kernel is mapped across three AI Engine tiles. We carefully hand-tune the code to overlap memory operations with arithmetic operations, to improve performance. We use the AIE data forwarding interfaces to forward the results from the first AIE core (used for Laplacian calculation) to the subsequent AIE cores (used for flux calculation). This approach allows for the concurrent execution of multiple stencil calculations, which can increase the overall performance and throughput of the hdiff design.  The performance of the hdiff implementation can be maximized by scaling it out across as many AIE cores as possible while avoiding data starvation. We develop a bundle or B-block-based design. A B-block is a cluster of AIE cores connected to the same shimDMA input/output channel. In our design, we choose a B-Block to consist of 12-AI Engine tiles, of which we utilize four to consume a total of 48 AI Engine tiles. 

**Q: Can I use this release to accelerate my own application on AMD AI Engines?**

A: No.  This release is tied to a single pre-developed application kernel.

**Q: How was the accelerator developed?**

A: We leverage the [MLIR](https://mlir.llvm.org) compiler infrastructure in our development process for this example design. Please refer to the [SPARTA](https://arxiv.org/pdf/2303.03509.pdf) paper for details. 

**Q: I am interested in this work, and would like to contribute?**

A: Pull requests to this repository and the experimental ROCm runtime branch, [rocm-5.6.x-air](https://github.com/RadeonOpenCompute/ROCR-Runtime/tree/experimental/rocm-5.6.x-air), are welcome. Please don't hesitate to interact with the repository through issues as well. 

**Q: What is this release?**

This repository and the experimental [rocm-5.6.x-air](https://github.com/RadeonOpenCompute/ROCR-Runtime/tree/experimental/rocm-5.6.x-air) branch are experimental prototypes of AMD research and not official AMD products. Both suites of software are provided as-is with no guarantees of support of AMD or AMD Research and Advanced Development.

**Q: What platforms does this release work on?**

A: The hardware configurations supported by ROCm are listed here.  The only supported AMD AI Engine board in this release is the VCK5000, which is PCIe form factor accelerator card, capable of offering 145 INT8 TOPs. In this release, only a fraction of the AI Engines available are used for acceleration.
