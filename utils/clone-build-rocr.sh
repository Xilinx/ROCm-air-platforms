# Copyright (C) 2023, Advanced Micro Devices, Inc.
# SPDX-License-Identifier: MIT

#!/usr/bin/env bash

SOURCE_BASE=${1:-$(pwd)}
INSTALL_BASE=${2:-$(pwd)}
ROCM_TARGET=${3:-rocm}
ROCR_NAME=${4:-ROCR-Runtime}
ROCM_BRANCH=${5:-experimental/rocm-5.6.x-air}

ROCM_DIR=/opt/rocm
LLVM_DIR=${ROCM_DIR}/llvm
LLVM_BIN_DIR=${LLVM_DIR}/bin
LLVM_LIB_DIR=${LLVM_DIR}/lib

INSTALL_DIR=${INSTALL_BASE}/${ROCM_TARGET}

ROCR_DIR=${SOURCE_BASE}/${ROCR_NAME}
BUILD_DIR=${ROCR_DIR}/src/build

HSAKMT_DIR=${INSTALL_DIR}/lib/cmake/hsakmt

git clone --branch ${ROCM_BRANCH} --depth 1 \
  https://github.com/RadeonOpenCompute/ROCR-Runtime ${ROCR_DIR}

mkdir -p ${BUILD_DIR}

pushd ${BUILD_DIR}

cmake .. \
  -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_C_COMPILER=${LLVM_BIN_DIR}/clang \
  -DCMAKE_CXX_COMPILER=${LLVM_BIN_DIR}/clang++ \
  -DClang_DIR=${LLVM_LIB_DIR}/cmake/clang \
  -DLLVM_DIR=${LLVM_LIB_DIR}/cmake/llvm \
  -Dhsakmt_DIR=${HSAKMT_DIR}

make -j64
make install

popd
