# Copyright (C) 2022, Advanced Micro Devices, Inc.
# SPDX-License-Identifier: MIT

CC=clang

LDFLAGS = -fuse-ld=lld -rdynamic \
	-std=c++20 -lstdc++ \
	-lm \
	-Wl,-rpath,${ROCM_ROOT}/lib ${ROCM_ROOT}/lib/libhsa-runtime64.so.1.9.0 \
	${ELFUTILS_ROOT}/libelf.a \

CFLAGS += -I/${ROCM_ROOT}/include

.PHONY: check_rocm check_elfutils

all: check_rocm check_elfutils weather_stencil.exe

check_rocm:
ifndef ROCM_ROOT
	$(error ROCM_ROOT is not set)
endif

check_elfutils:
ifndef ELFUTILS_ROOT
	$(error ELFUTILS_ROOT is not set)
endif

%.exe: weather_stencil.cpp
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ weather_stencil.cpp


clean::
	rm -rf *.exe
