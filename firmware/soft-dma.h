//===- soft-dma.h ---------------------------------------------------*- C++ -*-===//
//
// Copyright (C) 2024, Advanced Micro Devices, Inc.
// SPDX-License-Identifier: MIT
//
//===----------------------------------------------------------------------===//

/*
  Note: The contents of this file has been copied from 
  https://github.com/Xilinx/embeddedsw with the same license.
*/

#ifndef SOFT_DMA_H_
#define SOFT_DMA_H_

#define XAXIDMA_RX_OFFSET 0x00000030 /**< RX channel registers base
               * offset */
#define XAXIDMA_CR_OFFSET  0x00000000   /**< Channel control */
#define XAXIDMA_SR_OFFSET  0x00000004   /**< Status */
#define XAXIDMA_CDESC_OFFSET   0x00000008   /**< Current descriptor pointer */
#define XAXIDMA_CDESC_MSB_OFFSET 0x0000000C   /**< Current descriptor pointer */
#define XAXIDMA_TDESC_OFFSET   0x00000010   /**< Tail descriptor pointer */
#define XAXIDMA_TDESC_MSB_OFFSET 0x00000014   /**< Tail descriptor pointer */
#define XAXIDMA_SRCADDR_OFFSET   0x00000018   /**< Simple mode source address
            pointer */
#define XAXIDMA_SRCADDR_MSB_OFFSET  0x0000001C  /**< Simple mode source address
            pointer */
#define XAXIDMA_DESTADDR_OFFSET   0x00000018   /**< Simple mode destination address pointer */
#define XAXIDMA_DESTADDR_MSB_OFFSET 0x0000001C   /**< Simple mode destination address pointer */
#define XAXIDMA_BUFFLEN_OFFSET    0x00000028   /**< Tail descriptor pointer */
#define XAXIDMA_SGCTL_OFFSET    0x0000002c   /**< SG Control Register */

#define XAXIDMA_HALTED_MASK   0x00000001  /**< DMA channel halted */
#define XAXIDMA_IDLE_MASK   0x00000002  /**< DMA channel idle */
#define XAXIDMA_ERR_INTERNAL_MASK 0x00000010  /**< Datamover internal
                  *  err */
#define XAXIDMA_ERR_SLAVE_MASK    0x00000020  /**< Datamover slave err */
#define XAXIDMA_ERR_DECODE_MASK   0x00000040  /**< Datamover decode
                  *  err */
#define XAXIDMA_ERR_SG_INT_MASK   0x00000100  /**< SG internal err */
#define XAXIDMA_ERR_SG_SLV_MASK   0x00000200  /**< SG slave err */
#define XAXIDMA_ERR_SG_DEC_MASK   0x00000400  /**< SG decode err */
#define XAXIDMA_ERR_ALL_MASK    0x00000770  /**< All errors */
   
/** @name Bitmasks of XAXIDMA_CR_OFFSET register
 * @{
 */
#define XAXIDMA_CR_RUNSTOP_MASK 0x00000001 /**< Start/stop DMA channel */
#define XAXIDMA_CR_RESET_MASK 0x00000004 /**< Reset DMA engine */
#define XAXIDMA_CR_KEYHOLE_MASK 0x00000008 /**< Keyhole feature */
#define XAXIDMA_CR_CYCLIC_MASK  0x00000010 /**< Cyclic Mode */ 

#endif // SOFT_DMA_H
