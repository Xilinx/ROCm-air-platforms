// Copyright(C) 2022-2025 Advanced Micro Devices, Inc. All Rights Reserved.
// SPDX-License-Identifier: MIT
//
// Verilog top-level for Vivado IP block creation
//

module blackparrot
  #(
    // BP I/O
    parameter integer C_S00_AXI_DATA_WIDTH   = 64
    , parameter integer C_S00_AXI_ADDR_WIDTH = 64
    , parameter integer C_S00_AXI_ID_WIDTH   = 6
    , parameter [C_S00_AXI_ADDR_WIDTH-1:0] C_S00_AXI_BASEADDR = 64'h800_0000_0000
    , parameter integer C_M00_AXI_DATA_WIDTH = 64
    , parameter integer C_M00_AXI_ADDR_WIDTH = 64
    , parameter integer C_M00_AXI_ID_WIDTH   = 6
    // BP Memory
    , parameter integer C_M01_AXI_DATA_WIDTH = 128
    , parameter integer C_M01_AXI_ADDR_WIDTH = 32
    , parameter integer C_M01_AXI_ID_WIDTH   = 6
    // Global ID
    , parameter integer C_GLOBAL_ID          = 0
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

   blackparrot_wrapper #
     (.C_S00_AXI_DATA_WIDTH (C_S00_AXI_DATA_WIDTH)
      ,.C_S00_AXI_ADDR_WIDTH(C_S00_AXI_ADDR_WIDTH)
      ,.C_S00_AXI_ID_WIDTH(C_S00_AXI_ID_WIDTH)
      ,.C_S00_AXI_BASEADDR(C_S00_AXI_BASEADDR)
      ,.C_M00_AXI_DATA_WIDTH(C_M00_AXI_DATA_WIDTH)
      ,.C_M00_AXI_ADDR_WIDTH(C_M00_AXI_ADDR_WIDTH)
      ,.C_M00_AXI_ID_WIDTH(C_M00_AXI_ID_WIDTH)
      ,.C_M01_AXI_DATA_WIDTH(C_M01_AXI_DATA_WIDTH)
      ,.C_M01_AXI_ADDR_WIDTH(C_M01_AXI_ADDR_WIDTH)
      ,.C_M01_AXI_ID_WIDTH(C_M01_AXI_ID_WIDTH)
      ,.C_GLOBAL_ID(C_GLOBAL_ID)
      )
     blackparrot_wrapper_inst
     (.*);

endmodule

