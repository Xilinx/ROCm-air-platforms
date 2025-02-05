/* Copyright (c) 2023 Advanced Micro Devices, Inc. All rights reserved. */
/* SPDX-License-Identifier: MIT */

.section ".text.init"
.globl _start
.type  _start, @function
_start:
.option push
.option norelax
  # global pointer
  la gp, _gp
  # stack pointer
  la   sp, _sp
.option pop
  # Save the stack pointer to mscratch, so it can be used on first trap entry
  csrw mscratch, sp
  # Get 4K and subtract it off to get the program stack pointer
  li   t0, 1
  slli t0, t0, 12
  sub  sp, sp, t0

  # Setup mtvec from libperch exception.S
  la t0, __mtvec_handler
  csrw mtvec, t0

  # Enable FPU
  li t0, 0x00006000 & (0x00006000 >> 1);
  csrs mstatus, t0;
  csrwi fcsr, 0

  # 0 all registers except sp and gp
  addi x1,  x0, 0
  addi x4,  x0, 0
  addi x5,  x0, 0
  addi x6,  x0, 0
  addi x7,  x0, 0
  addi x8,  x0, 0
  addi x9,  x0, 0
  addi x10, x0, 0
  addi x11, x0, 0
  addi x12, x0, 0
  addi x13, x0, 0
  addi x14, x0, 0
  addi x15, x0, 0
  addi x16, x0, 0
  addi x17, x0, 0
  addi x18, x0, 0
  addi x19, x0, 0
  addi x20, x0, 0
  addi x21, x0, 0
  addi x22, x0, 0
  addi x23, x0, 0
  addi x24, x0, 0
  addi x25, x0, 0
  addi x26, x0, 0
  addi x27, x0, 0
  addi x28, x0, 0
  addi x29, x0, 0
  addi x30, x0, 0
  addi x31, x0, 0

  # Clear the bss segment
  la      a0, _edata
  la      a2, _end
  sub     a2, a2, a0
  li      a1, 0
  call    memset

  la   a0, __libc_fini_array # Register global termination functions
  call atexit                #  to be called upon exit
  call __libc_init_array     # Run global initialization functions

  # setup args to main and call
  # BP firmware has no main args, so don't try to use them!
  li a0, 0     # a0 = argc = NULL
  li a1, 0     # a1 = argv = NULL
  li a2, 0     # a2 = envp = NULL
  call main
  # exit when main returns
  tail exit
_bp_terminal:
  j _bp_terminal
  .size  _start, .-_start

