# Building the BlackParrot Firmware

`Makefile.bp` is used to build the BlackParrot firmware.

## Prerequisites

Before compiling the firmware, you must have a BlackParrot SDK install available. See the instructions
and Makefile in `platform/ip/black-parrot/` for instructions on setting up the
SDK.

The Embedded SW source and a startup file for BlackParrot must also be available.
The Embedded SW repository exists as a git submodule within the `bp/lib` directory.
It can be fetched by running the following command.

```
make -f Makefile.bp checkout
```

## Compile the Firmware Library

The BlackParrot firmware relies on some library code to interact with the Mutex and UartLite devices.
This library must be built before the main firmware file can be compiled. Build the firmware library
with the following commands.

```
make -f Makefile.bp library
```

## Compile the Firmware

The BlackParrot firmware is compiled by running the following commands.

```
make -f Makefile.bp firmware
```

The firmware and library can be compiled in one command by running:

```
make -f Makefile.bp all
```

A custom firmware source file can be set with the `BP_FW_MAIN_SRC` variable.

```
make -f Makefile.bp all BP_FW_MAIN_SRC=path/to/fw.cpp
```

## Cleaning the build

The firmware and its library can be cleaned using the following commands.

```
make -f Makefile.bp clean_lib
make -f Makefile.bp clean_fw
make -f Makefile.bp clean
```

-----

<p align="center">Copyright&copy; 2022-2025 Advanced Micro Devices, Inc.</p>
