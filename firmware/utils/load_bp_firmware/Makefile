# Copyright (C) 2022-2025, Advanced Micro Devices, Inc.
# SPDX-License-Identifier: MIT
# General Makefile for tests

TOP := $(shell git rev-parse --show-toplevel)

# set these variables when invoking make to point to non-default locations
ROCM_ROOT ?= /opt/rocm

CC=clang

LDFLAGS = -fuse-ld=lld -rdynamic \
		-lstdc++ \
		-lm \
		-Wl,-rpath,${ROCM_ROOT}/lib ${ROCM_ROOT}/lib/libhsa-runtime64.so

CFLAGS += -g -I${ROCM_ROOT}/include
CFLAGS += -std=c++17

all: load_bp_firmware.exe

%.exe: load_bp_firmware.cpp
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ load_bp_firmware.cpp -Wl,--whole-archive -Wl,--no-whole-archive

clean::
	rm -rf load_bp_firmware.exe
