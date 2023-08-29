# VCK5000 ROCm AIR platform

We present an HSA-compatible AMD AI Engine platform implemented on an [AMD VCK5000 board](https://www.xilinx.com/products/boards-and-kits/vck5000.html) which can be targeted through our experimental [ROCm-air branch](https://github.com/RadeonOpenCompute/ROCR-Runtime/tree/experimental/rocm-5.6.x-air). The build flow is compatible with AMD-Xilinx VCK5000 cards with production silicon: "VCK5000-AIE-ADK-G-ED". It is built with 2022.1 Vivado/Vitis tools and consists of a relatively empty Versal design (CIPS, QDMA-PCIe frontend, BRAM 'Queue Memory', NoC configuration, and CDMA 'AIE-Configuration DMA'). The ARM processor acts as an HSA AQL packet processor that manages AIE configurations and affects runtime DMA transfers from external memory into the AIE array.

## Prerequisites
Vivado 2022.1

The pdi is loaded to the card over JTAG, the USB-JTAG cable must be connected to the micro-USB input on the VCK5000 card and a programming machine (this can be the x86 host). The Xilinx Cable drivers must be [installed](https://docs.xilinx.com/r/en-US/ug973-vivado-release-notes-install-license/Installing-Cable-Drivers) on the programming machine. 

## Programming steps
The pdi can be loaded to the card by calling:
```
vivado -mode batch -source program_vck5000.tcl
```
After programming the host should undergo a **warm reboot**.

## Verification
After a warm reboot, you can verify that the card has been programmed properly with the VCK5000 AIR platfrorm by executing this command:
```
sudo lspci -vd 10ee:
```
The output should match the following (perhaps with a different bus ID):
```
21:00.0 Memory controller: Xilinx Corporation Device b034
        Subsystem: Xilinx Corporation Device 0007
        Flags: bus master, fast devsel, latency 0, IRQ 319, NUMA node 0
        Memory at 10200000000 (64-bit, prefetchable) [size=8G]
        Memory at 95200000 (64-bit, non-prefetchable) [size=1M]
        Capabilities: [40] Power Management version 3
        Capabilities: [70] Express Endpoint, MSI 00
        Capabilities: [100] Advanced Error Reporting
        Capabilities: [1c0] Secondary PCI Express
        Capabilities: [1f0] Virtual Channel
```

## Driver Insertion
Refer to the [driver](../../driver) documentation on how to insert the driver which allows the runtime to communicate with the device.

-----

<p align="center">Copyright&copy; 2019-2022 Advanced Micro Devices, Inc.</p>
