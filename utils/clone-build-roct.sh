# Copyright (C) 2023, Advanced Micro Devices, Inc.
# SPDX-License-Identifier: MIT

#!/usr/bin/env bash

SOURCE_BASE=${1:-$(pwd)}
INSTALL_BASE=${2:-$(pwd)}
ROCM_TARGET=${3:-rocm}
ROCT_NAME=${4:-ROCT-Thunk-Interface}
ROCT_BRANCH=${5:-rocm-5.6.x}

INSTALL_DIR=${INSTALL_BASE}/${ROCM_TARGET}

ROCT_DIR=${SOURCE_BASE}/${ROCT_NAME}
BUILD_DIR=${ROCT_DIR}/build

echo ${ROCT_DIR}
echo ${INSTALL_DIR}

git clone --branch ${ROCT_BRANCH} --depth 1 \
  https://github.com/RadeonOpenCompute/ROCT-Thunk-Interface ${ROCT_DIR}

mkdir -p ${BUILD_DIR}

pushd ${BUILD_DIR}

cmake .. \
  -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} \
  -DCMAKE_BUILD_TYPE=Release

make
make install

popd
