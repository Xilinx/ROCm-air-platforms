# PLIO

The platform contained in this repository supports two mechanisms of interfacing with the AIEs. General Memory IO (GMIO) programs the hardened shim DMAs in the tiles within Row 0 of the array to move data between the AIE array and the device DRAM. Programmable Logic IO (PLIO) exposes interfaces to the PL which connect to the AIE array AXI Stream network. There are PLIOs in every column of the devices and GMIOs in certain columns of the device (More details can be found in the [VCK5000 platform documentation](./README.md)). This guide will walk through how to build off this platform to connect IPs within the programmable logic to an AIE design programmed using [mlir-aie](https://github.com/Xilinx/mlir-aie). 

## Platform Building and Programming Flow

To build the platform, simply follow the [VCK5000 platform documentation](./README.md). This will instantiate a AXI DMA that can transfer data to and from the AIEs using the PLIO from a single column. 

# Understanding the PLIO Integration

As mentioned in the [VCK5000 platform documentation](./README.md), building the platform has two steps:

1. Vivado IPI design. This step generates a Vitis extensible hardware platform containing the necessary CIPS configuration and hardware IP components. This step generates an .xsa file for the design. (To run just this step, run `make xsa`)
2. Vitis design compiles a 32 GMIO 1 PLIO AIE design with simple add functionality to enable all NoC NMU/NSU connections so all shimDMAs (used for GMIO) are enabled. This rebuilds the design targeting the platform generated by step 1. (To run just this step, run `make platform`)

This placeholder Vitis design used in step 2 to instantiate the PLIO and GMIO can be found in [./aie\_platform/aie](./aie_platform/aie). In the [graph definition](./aie_platform/aie/graph.h) the parameters `NUM_MM` and `NUM_STREAM` define how many GMIO and PLIO interfaces respectively get instantiated. The PLIOs are connected to a placeholder HLS kernel which can be found in [./aie\_platform/pl/counter/](./aie_platform/pl/counter/) which contains two AXI streaming ports to connect to the input and output of the device. Note this HLS kernel has no function other than to be a placeholder for an AXI streaming interface. See the platform [Makefile](./aie_platform/Makefile) and the system configuration file [Makefile](./aie_platform/system.cfg) for more information on how we use v++ to connect the AIE and the PL. 

After step 2, there is an internal Vivado project created with the PLIO and GMIO interfaces instantiated. This project can be found at `./platform/vck5000/aie_platform/_x/link/vivado/vpl/prj/`. Opening this project will show the initial platform design in IP integrator with teh following modifications:

1. 32 GMIOs connected to the NOC going to the DRAM.
2. 1 PLIO input / output pair going to an AXI DMA IP.

To understand what column of PLIO is being used, open the generated file at `platform/vck5000/aie_platform/Work/reports/graph_mapping_analysis_report.txt`. This will contain information about what columns each PLIO is connected to. For example, the following will show that the input and output PLIO are mapped to column 26:

```
Block:Function Name  CR(x,y)/IO(x)  Schedule  Utilization  Variable Name  Graph Name
 -------------------  -------------  --------  -----------  -------------  ----------
 i0:PLIO              IO(26)                                plioIn0        g
 i32:PLIO             IO(26)                                plioOut0       g
```

To demonstrate how the vivado project can be modified, we provide [fixup\_pl\_design.tcl](./aie_platform/fixup_pl_design.tcl) which removes the placeholder HLS kernel and replaces it with an AXI DMA from the Vivado provided IP catalog. This also demonstrates how a user would extend the platform to connect their IP to the NOC for connectivity to the ARM Processing Subssytem (PS) and to the DRAM. 

# Modifying the platform with your own PL IP

Users can treat the vivado project at `./platform/vck5000/aie_platform/_x/link/vivado/vpl/prj/` as a standard Vivado project. Users can replace the AXI DMA with their own PL IPs and interface with the AIE PLIO using the standard mechanism to connect IPs using the IP Integrator or RTL IP instaniations. Users can also use the Vivado provided Integrated Logic Analyzer (ILA) functionality to get visibility into the design. After modifying the design, users can leverage Vivado to export a block design TCL script to allow version control and portability of changes made to the block design.

When done customizing the design, a `.pdi` can be generated using the standard Vivado flow for running an implementation and generating a device binary. Note that when a PDI is created this way, the ARM firmware is not automatically added to the PDI. See the [firmware](../../firmware/) documentation on how to dynamically load the firmware via `xsdb` or the [platform makefile](./aie_platform/Makefile) on how to use bootgen to embed the firmware ELF into the `.pdi`.

