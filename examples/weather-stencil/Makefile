# Copyright (C) 2022, Advanced Micro Devices, Inc.
# SPDX-License-Identifier: MIT

CC=clang

LDFLAGS = -fuse-ld=lld -rdynamic \
		-lstdc++ \
		-lm \
		-Wl,--no-as-needed -ldl \
    -Wl,-rpath,/scratch/tgutierr/rocm/lib /scratch/tgutierr/rocm/lib/libhsa-runtime64.so.1.9.0 \
    /scratch/eddier/mlir-air-rocr-airbin/mlir-air/elfutils/libelf/libelf.a \

CFLAGS += -I/scratch/tgutierr/rocm/include

all: weather_stencil.exe

%.exe: weather_stencil.cpp
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ weather_stencil.cpp -Wl,--whole-archive -Wl,--no-whole-archive


clean::
	rm -rf *.exe 
