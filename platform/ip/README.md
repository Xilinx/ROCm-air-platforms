# Overview

This folder contains the custom IP blocks used in the VCK5000 platforms.

# Building

```
make build_ip # runs Vivado to build and export the IP block for use in IPI
make pack_ip  # creates .tar.gz archives of each IP block
make clean_ip # cleans the IP builds and archives
```

Each of the above commands can be parallelized with the `-j` flag to `make` to
execute the specified operation across all IP blocks concurrently.

-----

<p align="center">Copyright&copy; 2024-2025 Advanced Micro Devices, Inc.</p>
