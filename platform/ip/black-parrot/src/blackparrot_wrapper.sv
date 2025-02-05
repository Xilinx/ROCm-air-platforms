// Copyright(C) 2022-2025 Advanced Micro Devices, Inc. All Rights Reserved.
// SPDX-License-Identifier: MIT
//
// SystemVerilog wrapper for BlackParrot
//

`include "bp_common_defines.svh"
`include "bp_be_defines.svh"
`include "bp_me_defines.svh"

module blackparrot_wrapper
 import bp_common_pkg::*;
 import bp_be_pkg::*;
 import bp_me_pkg::*;
 #(parameter bp_params_e bp_params_p = e_bp_unicore_air_cfg
   `declare_bp_proc_params(bp_params_p)

   // BP I/O
   // physical address of BP in the system
   , parameter integer C_S00_AXI_DATA_WIDTH   = 64
   , parameter integer C_S00_AXI_ADDR_WIDTH   = 64
   , parameter integer C_S00_AXI_ID_WIDTH     = 6
   , parameter [63:0] C_S00_AXI_BASEADDR = 64'h800_0000_0000
   , parameter integer C_M00_AXI_DATA_WIDTH   = 64
   , parameter integer C_M00_AXI_ADDR_WIDTH   = 64
   , parameter integer C_M00_AXI_ID_WIDTH     = 6
   // BP memory
   , parameter integer C_M01_AXI_DATA_WIDTH   = 128
   , parameter integer C_M01_AXI_ADDR_WIDTH   = 32
   , parameter integer C_M01_AXI_ID_WIDTH     = 6
   // Global ID
   , parameter integer C_GLOBAL_ID            = 0
   )
  (input wire                                    core_reset
   , input wire                                  axi_aclk
   , input wire                                  axi_aresetn

   // Incoming I/O
   , input wire [C_S00_AXI_ADDR_WIDTH-1:0]       s00_axi_awaddr
   , input wire                                  s00_axi_awvalid
   , output wire                                 s00_axi_awready
   , input wire [C_S00_AXI_ID_WIDTH-1:0]         s00_axi_awid
   , input wire                                  s00_axi_awlock
   , input wire [3:0]                            s00_axi_awcache
   , input wire [2:0]                            s00_axi_awprot
   , input wire [7:0]                            s00_axi_awlen
   , input wire [2:0]                            s00_axi_awsize
   , input wire [1:0]                            s00_axi_awburst
   , input wire [3:0]                            s00_axi_awqos
   , input wire [3:0]                            s00_axi_awregion

   , input wire [C_S00_AXI_DATA_WIDTH-1:0]       s00_axi_wdata
   , input wire                                  s00_axi_wvalid
   , output wire                                 s00_axi_wready
   , input wire                                  s00_axi_wlast
   , input wire [(C_S00_AXI_DATA_WIDTH/8)-1:0]   s00_axi_wstrb

   , output wire                                 s00_axi_bvalid
   , input wire                                  s00_axi_bready
   , output wire [C_S00_AXI_ID_WIDTH-1:0]        s00_axi_bid
   , output wire [1:0]                           s00_axi_bresp

   , input wire [C_S00_AXI_ADDR_WIDTH-1:0]       s00_axi_araddr
   , input wire                                  s00_axi_arvalid
   , output wire                                 s00_axi_arready
   , input wire [C_S00_AXI_ID_WIDTH-1:0]         s00_axi_arid
   , input wire                                  s00_axi_arlock
   , input wire [3:0]                            s00_axi_arcache
   , input wire [2:0]                            s00_axi_arprot
   , input wire [7:0]                            s00_axi_arlen
   , input wire [2:0]                            s00_axi_arsize
   , input wire [1:0]                            s00_axi_arburst
   , input wire [3:0]                            s00_axi_arqos
   , input wire [3:0]                            s00_axi_arregion

   , output wire [C_S00_AXI_DATA_WIDTH-1:0]      s00_axi_rdata
   , output wire                                 s00_axi_rvalid
   , input wire                                  s00_axi_rready
   , output wire [C_S00_AXI_ID_WIDTH-1:0]        s00_axi_rid
   , output wire                                 s00_axi_rlast
   , output wire [1:0]                           s00_axi_rresp

   // Outgoing I/O
   , output wire [C_M00_AXI_ADDR_WIDTH-1:0]      m00_axi_awaddr
   , output wire                                 m00_axi_awvalid
   , input wire                                  m00_axi_awready
   , output wire [C_M00_AXI_ID_WIDTH-1:0]        m00_axi_awid
   , output wire                                 m00_axi_awlock
   , output wire [3:0]                           m00_axi_awcache
   , output wire [2:0]                           m00_axi_awprot
   , output wire [7:0]                           m00_axi_awlen
   , output wire [2:0]                           m00_axi_awsize
   , output wire [1:0]                           m00_axi_awburst
   , output wire [3:0]                           m00_axi_awqos
   , output wire [3:0]                           m00_axi_awregion

   , output wire [C_M00_AXI_DATA_WIDTH-1:0]      m00_axi_wdata
   , output wire                                 m00_axi_wvalid
   , input wire                                  m00_axi_wready
   , output wire                                 m00_axi_wlast
   , output wire [(C_M00_AXI_DATA_WIDTH/8)-1:0]  m00_axi_wstrb

   , input wire                                  m00_axi_bvalid
   , output wire                                 m00_axi_bready
   , input wire [C_M00_AXI_ID_WIDTH-1:0]         m00_axi_bid
   , input wire [1:0]                            m00_axi_bresp

   , output wire [C_M00_AXI_ADDR_WIDTH-1:0]      m00_axi_araddr
   , output wire                                 m00_axi_arvalid
   , input wire                                  m00_axi_arready
   , output wire [C_M00_AXI_ID_WIDTH-1:0]        m00_axi_arid
   , output wire                                 m00_axi_arlock
   , output wire [3:0]                           m00_axi_arcache
   , output wire [2:0]                           m00_axi_arprot
   , output wire [7:0]                           m00_axi_arlen
   , output wire [2:0]                           m00_axi_arsize
   , output wire [1:0]                           m00_axi_arburst
   , output wire [3:0]                           m00_axi_arqos
   , output wire [3:0]                           m00_axi_arregion

   , input wire [C_M00_AXI_DATA_WIDTH-1:0]       m00_axi_rdata
   , input wire                                  m00_axi_rvalid
   , output wire                                 m00_axi_rready
   , input wire [C_M00_AXI_ID_WIDTH-1:0]         m00_axi_rid
   , input wire                                  m00_axi_rlast
   , input wire [1:0]                            m00_axi_rresp

   // Outgoing Memory
   , output wire [C_M01_AXI_ADDR_WIDTH-1:0]      m01_axi_awaddr
   , output wire                                 m01_axi_awvalid
   , input wire                                  m01_axi_awready
   , output wire [C_M01_AXI_ID_WIDTH-1:0]        m01_axi_awid
   , output wire                                 m01_axi_awlock
   , output wire [3:0]                           m01_axi_awcache
   , output wire [2:0]                           m01_axi_awprot
   , output wire [7:0]                           m01_axi_awlen
   , output wire [2:0]                           m01_axi_awsize
   , output wire [1:0]                           m01_axi_awburst
   , output wire [3:0]                           m01_axi_awqos
   , output wire [3:0]                           m01_axi_awregion

   , output wire [C_M01_AXI_DATA_WIDTH-1:0]      m01_axi_wdata
   , output wire                                 m01_axi_wvalid
   , input wire                                  m01_axi_wready
   , output wire                                 m01_axi_wlast
   , output wire [(C_M01_AXI_DATA_WIDTH/8)-1:0]  m01_axi_wstrb

   , input wire                                  m01_axi_bvalid
   , output wire                                 m01_axi_bready
   , input wire [C_M01_AXI_ID_WIDTH-1:0]         m01_axi_bid
   , input wire [1:0]                            m01_axi_bresp

   , output wire [C_M01_AXI_ADDR_WIDTH-1:0]      m01_axi_araddr
   , output wire                                 m01_axi_arvalid
   , input wire                                  m01_axi_arready
   , output wire [C_M01_AXI_ID_WIDTH-1:0]        m01_axi_arid
   , output wire                                 m01_axi_arlock
   , output wire [3:0]                           m01_axi_arcache
   , output wire [2:0]                           m01_axi_arprot
   , output wire [7:0]                           m01_axi_arlen
   , output wire [2:0]                           m01_axi_arsize
   , output wire [1:0]                           m01_axi_arburst
   , output wire [3:0]                           m01_axi_arqos
   , output wire [3:0]                           m01_axi_arregion

   , input wire [C_M01_AXI_DATA_WIDTH-1:0]       m01_axi_rdata
   , input wire                                  m01_axi_rvalid
   , output wire                                 m01_axi_rready
   , input wire [C_M01_AXI_ID_WIDTH-1:0]         m01_axi_rid
   , input wire                                  m01_axi_rlast
   , input wire [1:0]                            m01_axi_rresp
   );

  // BlackParrot reset signal
  wire bp_reset_li = ~axi_aresetn | core_reset;

  // subtract base address from AXI address to map into BP-local address
  logic [C_S00_AXI_ADDR_WIDTH-1:0] s00_axi_awaddr_li, s00_axi_araddr_li;
  assign s00_axi_awaddr_li = s00_axi_awaddr - C_S00_AXI_BASEADDR;
  assign s00_axi_araddr_li = s00_axi_araddr - C_S00_AXI_BASEADDR;

  wire [did_width_p-1:0] did = C_GLOBAL_ID;
  // Notes:
  // BP runs on a single clock - all axi clocks must be the same
  // the clock used is axi_aclk
  // BP has single reset - currently the OR of all AXI and other resets
  bp_axi4_top #
    (.bp_params_p(bp_params_p)
     ,.m_axi_addr_width_p(C_M00_AXI_ADDR_WIDTH)
     ,.m_axi_data_width_p(C_M00_AXI_DATA_WIDTH)
     ,.m_axi_id_width_p(C_M00_AXI_ID_WIDTH)
     ,.s_axi_addr_width_p(C_S00_AXI_ADDR_WIDTH)
     ,.s_axi_data_width_p(C_S00_AXI_DATA_WIDTH)
     ,.s_axi_id_width_p(C_S00_AXI_ID_WIDTH)
     ,.m01_axi_addr_width_p(C_M01_AXI_ADDR_WIDTH)
     ,.m01_axi_data_width_p(C_M01_AXI_DATA_WIDTH)
     ,.m01_axi_id_width_p(C_M01_AXI_ID_WIDTH)
     )
    blackparrot
    (.clk_i(axi_aclk)
     ,.reset_i(bp_reset_li)
     ,.rt_clk_i(axi_aclk)
     ,.my_did_i(did)
     ,.host_did_i('1)

     // I/O reads/writes from BlackParrot
     ,.m_axi_awaddr_o   (m00_axi_awaddr)
     ,.m_axi_awvalid_o  (m00_axi_awvalid)
     ,.m_axi_awready_i  (m00_axi_awready)
     ,.m_axi_awid_o     (m00_axi_awid)
     ,.m_axi_awlock_o   (m00_axi_awlock)
     ,.m_axi_awcache_o  (m00_axi_awcache)
     ,.m_axi_awprot_o   (m00_axi_awprot)
     ,.m_axi_awlen_o    (m00_axi_awlen)
     ,.m_axi_awsize_o   (m00_axi_awsize)
     ,.m_axi_awburst_o  (m00_axi_awburst)
     ,.m_axi_awqos_o    (m00_axi_awqos)
     ,.m_axi_awregion_o (m00_axi_awregion)

     ,.m_axi_wdata_o    (m00_axi_wdata)
     ,.m_axi_wvalid_o   (m00_axi_wvalid)
     ,.m_axi_wready_i   (m00_axi_wready)
     ,.m_axi_wlast_o    (m00_axi_wlast)
     ,.m_axi_wstrb_o    (m00_axi_wstrb)

     ,.m_axi_bvalid_i   (m00_axi_bvalid)
     ,.m_axi_bready_o   (m00_axi_bready)
     ,.m_axi_bid_i      (m00_axi_bid)
     ,.m_axi_bresp_i    (m00_axi_bresp)

     ,.m_axi_araddr_o   (m00_axi_araddr)
     ,.m_axi_arvalid_o  (m00_axi_arvalid)
     ,.m_axi_arready_i  (m00_axi_arready)
     ,.m_axi_arid_o     (m00_axi_arid)
     ,.m_axi_arlock_o   (m00_axi_arlock)
     ,.m_axi_arcache_o  (m00_axi_arcache)
     ,.m_axi_arprot_o   (m00_axi_arprot)
     ,.m_axi_arlen_o    (m00_axi_arlen)
     ,.m_axi_arsize_o   (m00_axi_arsize)
     ,.m_axi_arburst_o  (m00_axi_arburst)
     ,.m_axi_arqos_o    (m00_axi_arqos)
     ,.m_axi_arregion_o (m00_axi_arregion)

     ,.m_axi_rdata_i    (m00_axi_rdata)
     ,.m_axi_rvalid_i   (m00_axi_rvalid)
     ,.m_axi_rready_o   (m00_axi_rready)
     ,.m_axi_rid_i      (m00_axi_rid)
     ,.m_axi_rlast_i    (m00_axi_rlast)
     ,.m_axi_rresp_i    (m00_axi_rresp)

     // I/O reads/writes into BlackParrot
     ,.s_axi_awaddr_i   (s00_axi_awaddr_li)
     ,.s_axi_awvalid_i  (s00_axi_awvalid)
     ,.s_axi_awready_o  (s00_axi_awready)
     ,.s_axi_awid_i     (s00_axi_awid)
     ,.s_axi_awlock_i   (s00_axi_awlock)
     ,.s_axi_awcache_i  (s00_axi_awcache)
     ,.s_axi_awprot_i   (s00_axi_awprot)
     ,.s_axi_awlen_i    (s00_axi_awlen)
     ,.s_axi_awsize_i   (s00_axi_awsize)
     ,.s_axi_awburst_i  (s00_axi_awburst)
     ,.s_axi_awqos_i    (s00_axi_awqos)
     ,.s_axi_awregion_i (s00_axi_awregion)

     ,.s_axi_wdata_i    (s00_axi_wdata)
     ,.s_axi_wvalid_i   (s00_axi_wvalid)
     ,.s_axi_wready_o   (s00_axi_wready)
     ,.s_axi_wlast_i    (s00_axi_wlast)
     ,.s_axi_wstrb_i    (s00_axi_wstrb)

     ,.s_axi_bvalid_o   (s00_axi_bvalid)
     ,.s_axi_bready_i   (s00_axi_bready)
     ,.s_axi_bid_o      (s00_axi_bid)
     ,.s_axi_bresp_o    (s00_axi_bresp)

     ,.s_axi_araddr_i   (s00_axi_araddr_li)
     ,.s_axi_arvalid_i  (s00_axi_arvalid)
     ,.s_axi_arready_o  (s00_axi_arready)
     ,.s_axi_arid_i     (s00_axi_arid)
     ,.s_axi_arlock_i   (s00_axi_arlock)
     ,.s_axi_arcache_i  (s00_axi_arcache)
     ,.s_axi_arprot_i   (s00_axi_arprot)
     ,.s_axi_arlen_i    (s00_axi_arlen)
     ,.s_axi_arsize_i   (s00_axi_arsize)
     ,.s_axi_arburst_i  (s00_axi_arburst)
     ,.s_axi_arqos_i    (s00_axi_arqos)
     ,.s_axi_arregion_i (s00_axi_arregion)

     ,.s_axi_rdata_o    (s00_axi_rdata)
     ,.s_axi_rvalid_o   (s00_axi_rvalid)
     ,.s_axi_rready_i   (s00_axi_rready)
     ,.s_axi_rid_o      (s00_axi_rid)
     ,.s_axi_rlast_o    (s00_axi_rlast)
     ,.s_axi_rresp_o    (s00_axi_rresp)

     // Memory access from BlackParrot
     ,.m01_axi_awaddr_o   (m01_axi_awaddr)
     ,.m01_axi_awvalid_o  (m01_axi_awvalid)
     ,.m01_axi_awready_i  (m01_axi_awready)
     ,.m01_axi_awid_o     (m01_axi_awid)
     ,.m01_axi_awlock_o   (m01_axi_awlock)
     ,.m01_axi_awcache_o  (m01_axi_awcache)
     ,.m01_axi_awprot_o   (m01_axi_awprot)
     ,.m01_axi_awlen_o    (m01_axi_awlen)
     ,.m01_axi_awsize_o   (m01_axi_awsize)
     ,.m01_axi_awburst_o  (m01_axi_awburst)
     ,.m01_axi_awqos_o    (m01_axi_awqos)
     ,.m01_axi_awregion_o (m01_axi_awregion)

     ,.m01_axi_wdata_o    (m01_axi_wdata)
     ,.m01_axi_wvalid_o   (m01_axi_wvalid)
     ,.m01_axi_wready_i   (m01_axi_wready)
     ,.m01_axi_wlast_o    (m01_axi_wlast)
     ,.m01_axi_wstrb_o    (m01_axi_wstrb)

     ,.m01_axi_bvalid_i   (m01_axi_bvalid)
     ,.m01_axi_bready_o   (m01_axi_bready)
     ,.m01_axi_bid_i      (m01_axi_bid)
     ,.m01_axi_bresp_i    (m01_axi_bresp)

     ,.m01_axi_araddr_o   (m01_axi_araddr)
     ,.m01_axi_arvalid_o  (m01_axi_arvalid)
     ,.m01_axi_arready_i  (m01_axi_arready)
     ,.m01_axi_arid_o     (m01_axi_arid)
     ,.m01_axi_arlock_o   (m01_axi_arlock)
     ,.m01_axi_arcache_o  (m01_axi_arcache)
     ,.m01_axi_arprot_o   (m01_axi_arprot)
     ,.m01_axi_arlen_o    (m01_axi_arlen)
     ,.m01_axi_arsize_o   (m01_axi_arsize)
     ,.m01_axi_arburst_o  (m01_axi_arburst)
     ,.m01_axi_arqos_o    (m01_axi_arqos)
     ,.m01_axi_arregion_o (m01_axi_arregion)

     ,.m01_axi_rdata_i    (m01_axi_rdata)
     ,.m01_axi_rvalid_i   (m01_axi_rvalid)
     ,.m01_axi_rready_o   (m01_axi_rready)
     ,.m01_axi_rid_i      (m01_axi_rid)
     ,.m01_axi_rlast_i    (m01_axi_rlast)
     ,.m01_axi_rresp_i    (m01_axi_rresp)
     );

endmodule

