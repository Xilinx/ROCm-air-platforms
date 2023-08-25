//===- debug.h ---------------------------------------------------*- C++ -*-===//
//
// Copyright (C) 2023, Advanced Micro Devices, Inc.
// SPDX-License-Identifier: MIT
//
//===----------------------------------------------------------------------===//

#ifndef DEBUG_H_
#define DEBUG_H_

#include "xil_printf.h"

#define CHATTY 0

#define air_printf(fmt, ...) \
  do { \
    if (CHATTY) \
      xil_printf(fmt, ##__VA_ARGS__); \
  } while (0)

#endif // DEBUG_H_
