# AMD AIR Firmware

The ARM processor acts as an HSA AQL packet processor that manages AIE configurations and affects runtime DMA transfers from external memory into the AIE array. In this directory, we provide the firmware which runs on the ARM to perform these operations. Note, most users will not need to touch this as the provided platform configuration contains the default firmware. If a user has a need to reload the firmware on the card, a copy of the compiled firmware binary is provided in the releases of this github repository. 

## Compiling

Simply run `make` to compile the firmware. This will generate an output elf in `build/acdc_agent/Debug/acdc_agent.elf`.


## Loading

The script provided in `utils/load_firmware.tcl` will dynamically load the firmware onto the ARM. Use the following command to download the ELF from the location it is compiled:

```
ELF_FILE=../build/acdc_agent/Debug/acdc_agent.elf xsct load_firmware.tcl
```


## Viewing

The ARM processor can communicate over UART to the host x86. The UART serves two purposes. First, it allows a user on the x86 to view debug information coming from the ARM. The firmware can be compiled with `CHATTY=1` in debug.h to provide verbose debug information. Second, the UART provides a shell functionality which allows sideband access to access the memory and CSRs of the various IP on the device. Typing `help` will show the currently supported commands in the shell.

Here is an example of how we can open a UART connection between the host and the board: `screen /dev/ttyUSB2 115200`. 

-----

<p align="center">Copyright&copy; 2019-2023 Advanced Micro Devices, Inc.</p>
