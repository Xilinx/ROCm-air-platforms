// Copyright (C) 2022, Advanced Micro Devices, Inc. All rights reserved.
// SPDX-License-Identifier: MIT

`timescale 1ps / 1ps

`include "qdma_stm_defines.svh"
module xilinx_qdma_pcie_ep #
  (
    parameter PL_LINK_CAP_MAX_LINK_WIDTH  = 4,            // 1- X1; 2 - X2; 4 - X4; 8 - X8
    parameter PL_SIM_FAST_LINK_TRAINING   = "FALSE",  // Simulation Speedup
    parameter PL_LINK_CAP_MAX_LINK_SPEED  = 4,             // 1- GEN1; 2 - GEN2; 4 - GEN3
    parameter C_DATA_WIDTH                = 256 ,
    parameter EXT_PIPE_SIM                = "FALSE",  // This Parameter has effect on selecting Enable External PIPE Interface in GUI.
    parameter C_ROOT_PORT                 = "FALSE",  // PCIe block is in root port mode
    parameter C_DEVICE_NUMBER             = 0,        // Device number for Root Port configurations only
    parameter AXIS_CCIX_RX_TDATA_WIDTH    = 256,
    parameter AXIS_CCIX_TX_TDATA_WIDTH    = 256,
    parameter AXIS_CCIX_RX_TUSER_WIDTH    = 46,
    parameter AXIS_CCIX_TX_TUSER_WIDTH    = 46
  )
  (
    output [(PL_LINK_CAP_MAX_LINK_WIDTH - 1) : 0]   pci_exp_txp,
    output [(PL_LINK_CAP_MAX_LINK_WIDTH - 1) : 0]   pci_exp_txn,
    input  [(PL_LINK_CAP_MAX_LINK_WIDTH - 1) : 0]   pci_exp_rxp,
    input  [(PL_LINK_CAP_MAX_LINK_WIDTH - 1) : 0]   pci_exp_rxn,


    // synthesis translate_off
    input [25:0]  common_commands_in,
    input [83:0]  pipe_rx_0_sigs,
    input [83:0]  pipe_rx_1_sigs,
    input [83:0]  pipe_rx_2_sigs,
    input [83:0]  pipe_rx_3_sigs,
    input [83:0]  pipe_rx_4_sigs,
    input [83:0]  pipe_rx_5_sigs,
    input [83:0]  pipe_rx_6_sigs,
    input [83:0]  pipe_rx_7_sigs,
    input [83:0]  pipe_rx_8_sigs,
    input [83:0]  pipe_rx_9_sigs,
    input [83:0]  pipe_rx_10_sigs,
    input [83:0]  pipe_rx_11_sigs,
    input [83:0]  pipe_rx_12_sigs,
    input [83:0]  pipe_rx_13_sigs,
    input [83:0]  pipe_rx_14_sigs,
    input [83:0]  pipe_rx_15_sigs,
    output [25:0]  common_commands_out,
    output [83:0]  pipe_tx_0_sigs,
    output [83:0]  pipe_tx_1_sigs,
    output [83:0]  pipe_tx_2_sigs,
    output [83:0]  pipe_tx_3_sigs,
    output [83:0]  pipe_tx_4_sigs,
    output [83:0]  pipe_tx_5_sigs,
    output [83:0]  pipe_tx_6_sigs,
    output [83:0]  pipe_tx_7_sigs,
    output [83:0]  pipe_tx_8_sigs,
    output [83:0]  pipe_tx_9_sigs,
    output [83:0]  pipe_tx_10_sigs,
    output [83:0]  pipe_tx_11_sigs,
    output [83:0]  pipe_tx_12_sigs,
    output [83:0]  pipe_tx_13_sigs,
    output [83:0]  pipe_tx_14_sigs,
    output [83:0]  pipe_tx_15_sigs,
    // synthesis translate_on

    input ddr4_c0_sysclk_clk_n,
    input ddr4_c0_sysclk_clk_p,
    input ddr4_c1_sysclk_clk_n,
    input ddr4_c1_sysclk_clk_p,
    input ddr4_c2_sysclk_clk_n,
    input ddr4_c2_sysclk_clk_p,
    input ddr4_c3_sysclk_clk_n,
    input ddr4_c3_sysclk_clk_p,
    output ddr4_sdram_c0_act_n,
    output [16:0]ddr4_sdram_c0_adr,
    output [1:0]ddr4_sdram_c0_ba,
    output ddr4_sdram_c0_bg,
    output ddr4_sdram_c0_ck_c,
    output ddr4_sdram_c0_ck_t,
    output ddr4_sdram_c0_cke,
    output ddr4_sdram_c0_cs_n,
    inout [7:0]ddr4_sdram_c0_dm_n,
    inout [63:0]ddr4_sdram_c0_dq,
    inout [7:0]ddr4_sdram_c0_dqs_c,
    inout [7:0]ddr4_sdram_c0_dqs_t,
    output ddr4_sdram_c0_odt,
    output ddr4_sdram_c0_reset_n,
    output ddr4_sdram_c1_act_n,
    output [16:0]ddr4_sdram_c1_adr,
    output [1:0]ddr4_sdram_c1_ba,
    output ddr4_sdram_c1_bg,
    output ddr4_sdram_c1_ck_c,
    output ddr4_sdram_c1_ck_t,
    output ddr4_sdram_c1_cke,
    output ddr4_sdram_c1_cs_n,
    inout [7:0]ddr4_sdram_c1_dm_n,
    inout [63:0]ddr4_sdram_c1_dq,
    inout [7:0]ddr4_sdram_c1_dqs_c,
    inout [7:0]ddr4_sdram_c1_dqs_t,
    output ddr4_sdram_c1_odt,
    output ddr4_sdram_c1_reset_n,
    output ddr4_sdram_c2_act_n,
    output [16:0]ddr4_sdram_c2_adr,
    output [1:0]ddr4_sdram_c2_ba,
    output ddr4_sdram_c2_bg,
    output ddr4_sdram_c2_ck_c,
    output ddr4_sdram_c2_ck_t,
    output ddr4_sdram_c2_cke,
    output ddr4_sdram_c2_cs_n,
    inout [7:0]ddr4_sdram_c2_dm_n,
    inout [63:0]ddr4_sdram_c2_dq,
    inout [7:0]ddr4_sdram_c2_dqs_c,
    inout [7:0]ddr4_sdram_c2_dqs_t,
    output ddr4_sdram_c2_odt,
    output ddr4_sdram_c2_reset_n,
    output ddr4_sdram_c3_act_n,
    output [16:0]ddr4_sdram_c3_adr,
    output [1:0]ddr4_sdram_c3_ba,
    output ddr4_sdram_c3_bg,
    output ddr4_sdram_c3_ck_c,
    output ddr4_sdram_c3_ck_t,
    output ddr4_sdram_c3_cke,
    output ddr4_sdram_c3_cs_n,
    inout [7:0]ddr4_sdram_c3_dm_n,
    inout [63:0]ddr4_sdram_c3_dq,
    inout [7:0]ddr4_sdram_c3_dqs_c,
    inout [7:0]ddr4_sdram_c3_dqs_t,
    output ddr4_sdram_c3_odt,
    output ddr4_sdram_c3_reset_n,

    input   sys_clk_p,
    input   sys_clk_n
 );

   // Local Parameters derived from user selection
   localparam integer USER_CLK_FREQ = ((PL_LINK_CAP_MAX_LINK_SPEED == 3'h4) ? 5 : 4);
   localparam TCQ = 1;
   localparam C_S_AXI_ID_WIDTH   = 4;
   localparam C_M_AXI_ID_WIDTH   = 4;
   localparam C_S_AXI_DATA_WIDTH = C_DATA_WIDTH;
   localparam C_M_AXI_DATA_WIDTH = C_DATA_WIDTH;
   localparam C_S_AXI_ADDR_WIDTH = 64;
   localparam C_M_AXI_ADDR_WIDTH = 64;
   localparam C_NUM_USR_IRQ  = 16;
   localparam CRC_WIDTH          = 32;
   localparam MULTQ_EN = 1;
   localparam C_DSC_MAGIC_EN	= 1;
   localparam C_H2C_NUM_RIDS	= 64;
   localparam C_H2C_NUM_CHNL	= MULTQ_EN ? 4 : 4;
   localparam C_C2H_NUM_CHNL	= MULTQ_EN ? 4 : 4;
   localparam C_C2H_NUM_RIDS	= 32;
   localparam C_NUM_PCIE_TAGS	= 256;
   localparam C_S_AXI_NUM_READ 	= 32;
   localparam C_S_AXI_NUM_WRITE	= 8;
   localparam C_H2C_TUSER_WIDTH	= 55;
   localparam C_C2H_TUSER_WIDTH	= 64;
   localparam C_MDMA_DSC_IN_NUM_CHNL = 3;   // only 2 interface are userd. 0 is for MM and 2 is for ST. 1 is not used
   localparam C_MAX_NUM_QUEUE    = 128;
   localparam TM_DSC_BITS = 16;
   localparam C_S_AXIS_DATA_WIDTH        = C_DATA_WIDTH;
   localparam C_M_AXIS_DATA_WIDTH        = C_DATA_WIDTH;
   localparam C_M_AXIS_RQ_USER_WIDTH     = 137;
   localparam C_S_AXIS_CQP_USER_WIDTH    = 183;
   localparam C_M_AXIS_RC_USER_WIDTH     = 161;
   localparam C_S_AXIS_CC_USER_WIDTH     = 81;
   localparam C_S_KEEP_WIDTH             = C_S_AXI_DATA_WIDTH / 32;
   localparam C_M_KEEP_WIDTH             = C_M_AXI_DATA_WIDTH / 32;
  wire user_lnk_up;

  //----------------------------------------------------------------------------------------------------------------//
  //  AXI Interface                                                                                                 //
  //----------------------------------------------------------------------------------------------------------------//
  wire user_clk;
  wire axi_aclk;
  wire axi_aresetn;
  wire core_ext_start_0;
  wire core_ext_start_1;
  wire user_clk_dma_in;
  wire user_reset_dma_in;
  wire user_reset_dma_out;
  // Wires for Avery HOT/WARM and COLD RESET
  wire avy_sys_rst_n_c;
  wire avy_cfg_hot_reset_out;
  reg  avy_sys_rst_n_g;
  reg  avy_cfg_hot_reset_out_g;

  assign  avy_sys_rst_n_c = avy_sys_rst_n_g;
  assign  avy_cfg_hot_reset_out = avy_cfg_hot_reset_out_g;

  initial begin
    avy_sys_rst_n_g = 1;
    avy_cfg_hot_reset_out_g =0;
  end

  assign user_clk = axi_aclk;

  wire ddr4_c0_sysclk_clk_n;
  wire ddr4_c0_sysclk_clk_p;
  wire ddr4_c1_sysclk_clk_n;
  wire ddr4_c1_sysclk_clk_p;
  wire ddr4_c2_sysclk_clk_n;
  wire ddr4_c2_sysclk_clk_p;
  wire ddr4_c3_sysclk_clk_n;
  wire ddr4_c3_sysclk_clk_p;
  wire ddr4_sdram_c0_act_n;
  wire [16:0]ddr4_sdram_c0_adr;
  wire [1:0]ddr4_sdram_c0_ba;
  wire ddr4_sdram_c0_bg;
  wire ddr4_sdram_c0_ck_c;
  wire ddr4_sdram_c0_ck_t;
  wire ddr4_sdram_c0_cke;
  wire ddr4_sdram_c0_cs_n;
  wire [7:0]ddr4_sdram_c0_dm_n;
  wire [63:0]ddr4_sdram_c0_dq;
  wire [7:0]ddr4_sdram_c0_dqs_c;
  wire [7:0]ddr4_sdram_c0_dqs_t;
  wire ddr4_sdram_c0_odt;
  wire ddr4_sdram_c0_reset_n;
  wire ddr4_sdram_c1_act_n;
  wire [16:0]ddr4_sdram_c1_adr;
  wire [1:0]ddr4_sdram_c1_ba;
  wire ddr4_sdram_c1_bg;
  wire ddr4_sdram_c1_ck_c;
  wire ddr4_sdram_c1_ck_t;
  wire ddr4_sdram_c1_cke;
  wire ddr4_sdram_c1_cs_n;
  wire [7:0]ddr4_sdram_c1_dm_n;
  wire [63:0]ddr4_sdram_c1_dq;
  wire [7:0]ddr4_sdram_c1_dqs_c;
  wire [7:0]ddr4_sdram_c1_dqs_t;
  wire ddr4_sdram_c1_odt;
  wire ddr4_sdram_c1_reset_n;
  wire ddr4_sdram_c2_act_n;
  wire [16:0]ddr4_sdram_c2_adr;
  wire [1:0]ddr4_sdram_c2_ba;
  wire ddr4_sdram_c2_bg;
  wire ddr4_sdram_c2_ck_c;
  wire ddr4_sdram_c2_ck_t;
  wire ddr4_sdram_c2_cke;
  wire ddr4_sdram_c2_cs_n;
  wire [7:0]ddr4_sdram_c2_dm_n;
  wire [63:0]ddr4_sdram_c2_dq;
  wire [7:0]ddr4_sdram_c2_dqs_c;
  wire [7:0]ddr4_sdram_c2_dqs_t;
  wire ddr4_sdram_c2_odt;
  wire ddr4_sdram_c2_reset_n;
  wire ddr4_sdram_c3_act_n;
  wire [16:0]ddr4_sdram_c3_adr;
  wire [1:0]ddr4_sdram_c3_ba;
  wire ddr4_sdram_c3_bg;
  wire ddr4_sdram_c3_ck_c;
  wire ddr4_sdram_c3_ck_t;
  wire ddr4_sdram_c3_cke;
  wire ddr4_sdram_c3_cs_n;
  wire [7:0]ddr4_sdram_c3_dm_n;
  wire [63:0]ddr4_sdram_c3_dq;
  wire [7:0]ddr4_sdram_c3_dqs_c;
  wire [7:0]ddr4_sdram_c3_dqs_t;
  wire ddr4_sdram_c3_odt;
  wire ddr4_sdram_c3_reset_n;

  //----------------------------------------------------------------------------------------------------------------//
  //    System(SYS) Interface                                                                                       //
  //----------------------------------------------------------------------------------------------------------------//

  wire  sys_clk;
  wire  sys_rst_n_c;


  // User Clock LED Heartbeat
  reg [25:0] user_clk_heartbeat;

  //-- AXI Master Write Address Channel
  wire [C_M_AXI_ADDR_WIDTH-1:0]  m_axi_awaddr;
  wire [C_M_AXI_ID_WIDTH-1:0]    m_axi_awid;
  wire [2:0]                     m_axi_awprot;
  wire [1:0]                     m_axi_awburst;
  wire [2:0]                     m_axi_awsize;
  wire [3:0]                     m_axi_awcache;
  wire [7:0]                     m_axi_awlen;
  wire                           m_axi_awlock;
  wire                           m_axi_awvalid;
  wire                           m_axi_awready;

  //-- AXI Master Write Data Channel
  wire [C_M_AXI_DATA_WIDTH-1:0]      m_axi_wdata;
  wire [(C_M_AXI_DATA_WIDTH/8)-1:0]  m_axi_wstrb;
  wire                               m_axi_wlast;
  wire                               m_axi_wvalid;
  wire                               m_axi_wready;

  //-- AXI Master Write Response Channel
  wire                           m_axi_bvalid;
  wire                           m_axi_bready;
  wire [C_M_AXI_ID_WIDTH-1 : 0]  m_axi_bid ;
  wire [1:0]                     m_axi_bresp ;

  //-- AXI Master Read Address Channel
  wire [C_M_AXI_ID_WIDTH-1 : 0]  m_axi_arid;
  wire [C_M_AXI_ADDR_WIDTH-1:0]  m_axi_araddr;
  wire [7:0]                     m_axi_arlen;
  wire [2:0]                     m_axi_arsize;
  wire [1:0]                     m_axi_arburst;
  wire [2:0]                     m_axi_arprot;
  wire                           m_axi_arvalid;
  wire                           m_axi_arready;
  wire                           m_axi_arlock;
  wire [3:0]                     m_axi_arcache;

  //-- AXI Master Read Data Channel
  wire [C_M_AXI_ID_WIDTH-1 : 0]  m_axi_rid;
  wire [C_M_AXI_DATA_WIDTH-1:0]  m_axi_rdata;
  wire [1:0]                     m_axi_rresp;
  wire                           m_axi_rvalid;
  wire                           m_axi_rready;
  wire                           m_axi_rlast;

///////////////////////////////////////////////////////////////////////////////
  // CQ forwarding port to BRAM
  wire [C_M_AXI_ADDR_WIDTH-1:0] m_axib_awaddr;
  wire [C_M_AXI_ID_WIDTH-1:0]   m_axib_awid;
  wire [2:0]                    m_axib_awprot;
  wire [1:0]                    m_axib_awburst;
  wire [2:0]                    m_axib_awsize;
  wire [3:0]                    m_axib_awcache;
  wire [7:0]                    m_axib_awlen;
  wire                          m_axib_awlock;
  wire                          m_axib_awvalid;
  wire                          m_axib_awready;
  //-- AXI Master Write Data Channel
  wire [C_M_AXI_DATA_WIDTH-1:0]     m_axib_wdata;
  wire [(C_M_AXI_DATA_WIDTH/8)-1:0] m_axib_wstrb;
  wire                              m_axib_wlast;
  wire                              m_axib_wvalid;
  wire                              m_axib_wready;
  //-- AXI Master Write Response Channel
  wire                          m_axib_bvalid;
  wire                          m_axib_bready;
  wire [C_M_AXI_ID_WIDTH-1 : 0] m_axib_bid;
  wire [1 : 0]                  m_axib_bresp;

  //-- AXI Master Read Address Channel
  wire [C_M_AXI_ID_WIDTH-1 : 0] m_axib_arid;
  wire [C_M_AXI_ADDR_WIDTH-1:0] m_axib_araddr;
  wire [7:0]                    m_axib_arlen;
  wire [2:0]                    m_axib_arsize;
  wire [1:0]                    m_axib_arburst;
  wire [2:0]                    m_axib_arprot;
  wire                          m_axib_arvalid;
  wire                          m_axib_arready;
  wire                          m_axib_arlock;
  wire [3:0]                    m_axib_arcache;
  ////////////////////////////////////////////////////////////////////////////////
  //-- AXI Master Read Data Channel
  wire [C_M_AXI_ID_WIDTH-1 : 0] m_axib_rid;
  wire [C_M_AXI_DATA_WIDTH-1:0] m_axib_rdata;
  wire [1:0]                    m_axib_rresp;
  wire                          m_axib_rvalid;
  wire                          m_axib_rready;

  //////////////////////////////////////////////////  LITE
  //-- AXI Master Write Address Channel
  wire [31:0] m_axil_awaddr;
  wire [2:0]  m_axil_awprot;
  wire        m_axil_awvalid;
  wire        m_axil_awready;

  //-- AXI Master Write Data Channel
  wire [31:0] m_axil_wdata;
  wire [3:0]  m_axil_wstrb;
  wire        m_axil_wvalid;
  wire        m_axil_wready;

  //-- AXI Master Write Response Channel
  wire        m_axil_bvalid;
  wire        m_axil_bready;

  //-- AXI Master Read Address Channel
  wire [31:0] m_axil_araddr;
  wire [2:0]  m_axil_arprot;
  wire        m_axil_arvalid;
  wire        m_axil_arready;

  //-- AXI Master Read Data Channel
  wire [31:0] m_axil_rdata;
  wire [1:0]  m_axil_rresp;
  wire        m_axil_rvalid;
  wire        m_axil_rready;
  wire [1:0]  m_axil_bresp;

  wire [2:0]  msi_vector_width;
  wire        msi_enable;

  wire [3:0]  leds;

  wire   free_run_clock;

  wire [5:0]  cfg_ltssm_state;
  //******************************************************************
  //New ports for split IP
  //******************************************************************
  wire [C_S_AXIS_DATA_WIDTH-1:0]     s_axis_rq_tdata;
  wire                               s_axis_rq_tlast;
  wire [C_M_AXIS_RQ_USER_WIDTH-1:0]  s_axis_rq_tuser;
  wire [C_S_KEEP_WIDTH-1:0]          s_axis_rq_tkeep;
  wire                               s_axis_rq_tvalid;
  wire [3:0]                         s_axis_rq_tready;

  wire [C_M_AXIS_DATA_WIDTH-1:0]     m_axis_rc_tdata;
  wire [C_M_AXIS_RC_USER_WIDTH-1:0]  m_axis_rc_tuser;
  wire                               m_axis_rc_tlast;
  wire [C_M_KEEP_WIDTH-1:0]          m_axis_rc_tkeep;
  wire                               m_axis_rc_tvalid;
  wire                               m_axis_rc_tready;

  wire [C_M_AXIS_DATA_WIDTH-1:0]     m_axis_cq_tdata;
  wire [C_S_AXIS_CQP_USER_WIDTH-1:0] m_axis_cq_tuser;
  wire                               m_axis_cq_tlast;
  wire [C_M_KEEP_WIDTH-1:0]          m_axis_cq_tkeep;
  wire                               m_axis_cq_tvalid;
  wire                               m_axis_cq_tready;

  wire [C_S_AXIS_DATA_WIDTH-1:0]     s_axis_cc_tdata;
  wire [C_S_AXIS_CC_USER_WIDTH-1:0]  s_axis_cc_tuser;
  wire                               s_axis_cc_tlast;
  wire [C_S_KEEP_WIDTH-1:0]          s_axis_cc_tkeep;
  wire                               s_axis_cc_tvalid;
  wire [3:0]                         s_axis_cc_tready;

  wire        user_reset;
  wire        phy_rdy_out;
  wire [1:0]  pcie_cq_np_req;
  wire [5:0]  pcie_cq_np_req_count;
  wire [3:0]  pcie_tfc_nph_av;
  wire [3:0]  pcie_tfc_npd_av;
  wire        pcie_rq_seq_num_vld0;
  wire [5:0]  pcie_rq_seq_num0;
  wire        pcie_rq_seq_num_vld1;
  wire [5:0]  pcie_rq_seq_num1;

  wire [15:0] cfg_function_status;
  wire [503:0] cfg_vf_status;
  wire [2:0]  cfg_max_read_req;
  wire [1:0]  cfg_max_payload;
  wire [7:0]  cfg_fc_nph;
  wire [7:0]  cfg_fc_ph;
  wire [2:0]  cfg_fc_sel;
  wire        cfg_phy_link_down;
  wire [1:0]  cfg_phy_link_status;
  wire [2:0]  cfg_negotiated_width;
  wire [1:0]  cfg_current_speed;
  wire        cfg_pl_status_change;
  wire        cfg_hot_reset_out;
  wire [7:0]  cfg_ds_port_number;
  wire [7:0]  cfg_ds_bus_number;
  wire [7:0]  cfg_bus_number;
  wire [4:0]  cfg_ds_device_number;
  wire [2:0]  cfg_ds_function_number;
  wire        cfg_dbe;
  wire [63:0] cfg_dsn;
  wire        cfg_err_uncor_in;
  wire        cfg_err_cor_in;
  wire        cfg_link_training_enable;

  // Interrupt Interface Signals
  wire [3:0]  cfg_interrupt_int;
  wire        cfg_interrupt_sent;
  wire [3:0]  cfg_interrupt_pending;

  wire [3:0]  cfg_interrupt_msi_enable;
  wire        cfg_interrupt_msi_mask_update;
  wire [31:0] cfg_interrupt_msi_data;
  wire [31:0] cfg_interrupt_msi_int;
  wire [31:0] cfg_interrupt_msi_pending_status;
  wire        cfg_interrupt_msi_pending_status_data_enable;
  wire [3:0]  cfg_interrupt_msi_pending_status_function_num;
  wire [2:0]  cfg_interrupt_msi_attr;
  wire        cfg_interrupt_msi_tph_present;
  wire [1:0]  cfg_interrupt_msi_tph_type;
  wire [8:0]  cfg_interrupt_msi_tph_st_tag;
  wire [7:0]  cfg_interrupt_msi_function_number;
  wire        cfg_interrupt_msi_sent;
  wire        cfg_interrupt_msi_fail;

  wire          cfg_interrupt_msix_int;       // Configuration Interrupt MSI-X Data Valid.
  wire [31:0]   cfg_interrupt_msix_data;      // Configuration Interrupt MSI-X Data.
  wire [63:0]   cfg_interrupt_msix_address;   // Configuration Interrupt MSI-X Address.
  wire [3:0]    cfg_interrupt_msix_enable;    // Configuration Interrupt MSI-X Function Enabled.
  wire [3:0]    cfg_interrupt_msix_mask;      // Configuration Interrupt MSI-X Function Mask.
  wire [251:0]  cfg_interrupt_msix_vf_enable; // Configuration Interrupt MSI-X on VF Enabled.
  wire [251:0]  cfg_interrupt_msix_vf_mask;   // Configuration Interrupt MSI-X VF Mask.
  wire [1:0]    cfg_interrupt_msix_vec_pending; // Configuration Interrupt MSI-X on VF Enabled.
  wire [0:0]    cfg_interrupt_msix_vec_pending_status;   // Configuration Interrupt MSI-X VF Mask.

  // Error Reporting Interface
  wire          cfg_err_cor_out;
  wire          cfg_err_nonfatal_out;
  wire          cfg_err_fatal_out;
  wire [4:0]    cfg_local_error;
  wire          cfg_req_pm_transition_l23_ready;

  wire          cfg_msg_received;
  wire [7:0]    cfg_msg_received_data;
  wire [4:0]    cfg_msg_received_type;
  wire          cfg_msg_transmit;
  wire [2:0]    cfg_msg_transmit_type;
  wire [31:0]   cfg_msg_transmit_data;
  wire          cfg_msg_transmit_done;
  wire [3:0]    cfg_flr_in_process;
  wire [3:0]    cfg_flr_done;
  wire [251:0]  cfg_vf_flr_in_process;

  wire [7:0]		c2h_sts_0;
  wire [7:0]		h2c_sts_0;
  wire [7:0]		c2h_sts_1;
  wire [7:0]		h2c_sts_1;
  wire [7:0]		c2h_sts_2;
  wire [7:0]		h2c_sts_2;
  wire [7:0]		c2h_sts_3;
  wire [7:0]		h2c_sts_3;

  // MDMA signals
  wire   [C_DATA_WIDTH-1:0]   m_axis_h2c_tdata;
  wire   [CRC_WIDTH-1:0]      m_axis_h2c_tcrc;
  wire   [10:0]               m_axis_h2c_tuser_qid;
  wire   [2:0]                m_axis_h2c_tuser_port_id;
  wire                        m_axis_h2c_tuser_err;
  wire   [31:0]               m_axis_h2c_tuser_mdata;
  wire   [5:0]                m_axis_h2c_tuser_mty;
  wire                        m_axis_h2c_tuser_zero_byte;
  wire                        m_axis_h2c_tvalid;
  wire                        m_axis_h2c_tready;
  wire                        m_axis_h2c_tlast;

  wire                        m_axis_h2c_tready_lpbk;
  wire                        m_axis_h2c_tready_int;

  // AXIS C2H packet wire
  wire [C_DATA_WIDTH-1:0]     s_axis_c2h_tdata;
  wire [CRC_WIDTH-1:0]        s_axis_c2h_tcrc;
  wire                        s_axis_c2h_ctrl_marker;
  wire [6:0]                  s_axis_c2h_ctrl_ecc;
  wire [15:0]                 s_axis_c2h_ctrl_len;
  wire [2:0]                  s_axis_c2h_ctrl_port_id;
  wire [10:0]                 s_axis_c2h_ctrl_qid ;
  wire                        s_axis_c2h_ctrl_has_cmpt ;
  wire [C_DATA_WIDTH-1:0]     s_axis_c2h_tdata_int;
  wire                        s_axis_c2h_ctrl_marker_int;
  wire [15:0]                 s_axis_c2h_ctrl_len_int;
  wire [10:0]                 s_axis_c2h_ctrl_qid_int ;
  wire                        s_axis_c2h_ctrl_has_cmpt_int ;
  wire                        s_axis_c2h_tvalid;
  wire                        s_axis_c2h_tready;
  wire                        s_axis_c2h_tlast;
  wire  [5:0]                 s_axis_c2h_mty;
  wire                        s_axis_c2h_tvalid_lpbk;
  wire                        s_axis_c2h_tlast_lpbk;
  wire  [5:0]                 s_axis_c2h_mty_lpbk;
  wire                        s_axis_c2h_tvalid_int;
  wire                        s_axis_c2h_tlast_int;
  wire  [5:0]                 s_axis_c2h_mty_int;

  // AXIS C2H tuser wire
  wire  [511:0] s_axis_c2h_cmpt_tdata;
  wire  [1:0]   s_axis_c2h_cmpt_size;
  wire  [15:0]  s_axis_c2h_cmpt_dpar;
  wire          s_axis_c2h_cmpt_tvalid;
  wire          s_axis_c2h_cmpt_tvalid_int;
  wire  [511:0] s_axis_c2h_cmpt_tdata_int;
  wire  [1:0]   s_axis_c2h_cmpt_size_int;
  wire  [15:0]  s_axis_c2h_cmpt_dpar_int;
  wire          s_axis_c2h_cmpt_tready_int;
  wire          s_axis_c2h_cmpt_tready;
	wire [10:0]		s_axis_c2h_cmpt_ctrl_qid;
	wire [1:0]		s_axis_c2h_cmpt_ctrl_cmpt_type;
	wire [15:0]		s_axis_c2h_cmpt_ctrl_wait_pld_pkt_id;
	wire 				  s_axis_c2h_cmpt_ctrl_marker;
	wire 				  s_axis_c2h_cmpt_ctrl_user_trig;
	wire [2:0]		s_axis_c2h_cmpt_ctrl_col_idx;
	wire [2:0]		s_axis_c2h_cmpt_ctrl_err_idx;

  // Descriptor Bypass Out for qdma
  wire  [255:0] h2c_byp_out_dsc;
  wire  [3:0]   h2c_byp_out_fmt;
  wire          h2c_byp_out_st_mm;
  wire  [10:0]  h2c_byp_out_qid;
  wire  [1:0]   h2c_byp_out_dsc_sz;
  wire          h2c_byp_out_error;
  wire  [7:0]   h2c_byp_out_func;
  wire  [15:0]  h2c_byp_out_cidx;
  wire  [2:0]   h2c_byp_out_port_id;
  wire          h2c_byp_out_vld;
  wire          h2c_byp_out_rdy;

  wire  [255:0] c2h_byp_out_dsc;
  wire  [3:0]   c2h_byp_out_fmt;
  wire          c2h_byp_out_st_mm;
  wire  [1:0]   c2h_byp_out_dsc_sz;
  wire  [10:0]  c2h_byp_out_qid;
  wire          c2h_byp_out_error;
  wire  [7:0]   c2h_byp_out_func;
  wire  [15:0]  c2h_byp_out_cidx;
  wire  [2:0]   c2h_byp_out_port_id;
  wire  [6:0]   c2h_byp_out_pfch_tag;
  wire          c2h_byp_out_vld;
  wire          c2h_byp_out_rdy;

  // Descriptor Bypass In for qdma MM
  wire  [63:0]  h2c_byp_in_mm_radr;
  wire  [63:0]  h2c_byp_in_mm_wadr;
  wire  [15:0]  h2c_byp_in_mm_len;
  wire          h2c_byp_in_mm_mrkr_req;
  wire          h2c_byp_in_mm_sdi;
  wire  [10:0]  h2c_byp_in_mm_qid;
  wire          h2c_byp_in_mm_error;
  wire  [7:0]   h2c_byp_in_mm_func;
  wire  [15:0]  h2c_byp_in_mm_cidx;
  wire  [2:0]   h2c_byp_in_mm_port_id;
  wire  [1:0]   h2c_byp_in_mm_at;
  wire          h2c_byp_in_mm_no_dma;
  wire          h2c_byp_in_mm_vld;
  wire          h2c_byp_in_mm_rdy;

  wire  [63:0]  c2h_byp_in_mm_radr;
  wire  [63:0]  c2h_byp_in_mm_wadr;
  wire  [15:0]  c2h_byp_in_mm_len;
  wire          c2h_byp_in_mm_mrkr_req;
  wire          c2h_byp_in_mm_sdi;
  wire  [10:0]  c2h_byp_in_mm_qid;
  wire          c2h_byp_in_mm_error;
  wire  [7:0]   c2h_byp_in_mm_func;
  wire  [15:0]  c2h_byp_in_mm_cidx;
  wire  [2:0]   c2h_byp_in_mm_port_id;
  wire  [1:0]   c2h_byp_in_mm_at;
  wire          c2h_byp_in_mm_no_dma;
  wire          c2h_byp_in_mm_vld;
  wire          c2h_byp_in_mm_rdy;

  // Descriptor Bypass In for qdma ST
  wire [63:0]   h2c_byp_in_st_addr;
  wire [15:0]   h2c_byp_in_st_len;
  wire          h2c_byp_in_st_eop;
  wire          h2c_byp_in_st_sop;
  wire          h2c_byp_in_st_mrkr_req;
  wire          h2c_byp_in_st_sdi;
  wire  [10:0]  h2c_byp_in_st_qid;
  wire          h2c_byp_in_st_error;
  wire  [7:0]   h2c_byp_in_st_func;
  wire  [15:0]  h2c_byp_in_st_cidx;
  wire  [2:0]   h2c_byp_in_st_port_id;
  wire  [1:0]   h2c_byp_in_st_at;
  wire          h2c_byp_in_st_no_dma;
  wire          h2c_byp_in_st_vld;
  wire          h2c_byp_in_st_rdy;

  wire  [63:0]  c2h_byp_in_st_csh_addr;
  wire  [10:0]  c2h_byp_in_st_csh_qid;
  wire          c2h_byp_in_st_csh_error;
  wire  [7:0]   c2h_byp_in_st_csh_func;
  wire  [2:0]   c2h_byp_in_st_csh_port_id;
  wire  [6:0]   c2h_byp_in_st_csh_pfch_tag;
  wire  [1:0]   c2h_byp_in_st_csh_at;
  wire          c2h_byp_in_st_csh_vld;
  wire          c2h_byp_in_st_csh_rdy;

  wire          usr_irq_in_vld;
  wire [10 : 0] usr_irq_in_vec;
  wire [7 : 0]  usr_irq_in_fnc;
  wire          usr_irq_out_ack;
  wire          usr_irq_out_fail;

  wire          st_rx_msg_rdy;
  wire          st_rx_msg_valid;
  wire          st_rx_msg_last;
  wire [31:0]   st_rx_msg_data;

  wire          tm_dsc_sts_vld;
  wire          tm_dsc_sts_qen;
  wire          tm_dsc_sts_byp;
  wire          tm_dsc_sts_dir;
  wire          tm_dsc_sts_mm;
  wire          tm_dsc_sts_error;
  wire  [10:0]  tm_dsc_sts_qid;
  wire  [15:0]  tm_dsc_sts_avl;
  wire          tm_dsc_sts_qinv;
  wire          tm_dsc_sts_irq_arm;
  wire          tm_dsc_sts_rdy;

  // Descriptor credit In
  wire          dsc_crdt_in_vld;
  wire          dsc_crdt_in_rdy;
  wire          dsc_crdt_in_dir;
  wire          dsc_crdt_in_fence;
  wire [10:0]   dsc_crdt_in_qid;
  wire [15:0]   dsc_crdt_in_crdt;

  // Report the DROP case
  wire          axis_c2h_status_drop;
  wire          axis_c2h_status_last;
  wire          axis_c2h_status_valid;
  wire          axis_c2h_status_imm_or_marker;
  wire          axis_c2h_status_cmp;
  wire [10:0]   axis_c2h_status_qid;
  wire [7:0]    qsts_out_op;
  wire [63:0]   qsts_out_data;
  wire [2:0]    qsts_out_port_id;
  wire [12:0]   qsts_out_qid;
  wire          qsts_out_vld;
  wire          qsts_out_rdy;

  wire [3:0]		cfg_tph_requester_enable;
  wire [251:0]	cfg_vf_tph_requester_enable;
	wire          soft_reset_n;
	wire					st_loopback;

  wire [10:0]   c2h_num_pkt;
  wire [10:0]   c2h_st_qid;
  wire [15:0]   c2h_st_len;
  wire [31:0]   h2c_count;
  wire          h2c_match;
  wire          clr_h2c_match;
  wire 	        c2h_end;
  wire [31:0]   c2h_control;
  wire [10:0]   h2c_qid;
  wire [31:0]   cmpt_size;
  wire [255:0]  wb_dat;

  wire [TM_DSC_BITS-1:0] credit_out;
  wire [TM_DSC_BITS-1:0] credit_needed;
  wire [TM_DSC_BITS-1:0] credit_perpkt_in;
  wire                   credit_updt;

  wire [15:0] buf_count;
  wire        sys_clk_gt;


  // Ref clock buffer
//  IBUFDS_GTE5 # (.REFCLK_HROW_CK_SEL(2'b00)) refclk_ibuf (.O(sys_clk_gt), .ODIV2(sys_clk), .I(sys_clk_p), .CEB(1'b0), .IB(sys_clk_n));
  // Reset buffer
  IBUF   sys_reset_n_ibuf (.O(sys_rst_n_c), .I(sys_rst_n));

  wire  [25:0]  common_commands_in_i;
  wire  [83:0]  pipe_rx_0_sigs_i;
  wire  [83:0]  pipe_rx_1_sigs_i;
  wire  [83:0]  pipe_rx_2_sigs_i;
  wire  [83:0]  pipe_rx_3_sigs_i;
  wire  [83:0]  pipe_rx_4_sigs_i;
  wire  [83:0]  pipe_rx_5_sigs_i;
  wire  [83:0]  pipe_rx_6_sigs_i;
  wire  [83:0]  pipe_rx_7_sigs_i;
  wire  [83:0]  pipe_rx_8_sigs_i;
  wire  [83:0]  pipe_rx_9_sigs_i;
  wire  [83:0]  pipe_rx_10_sigs_i;
  wire  [83:0]  pipe_rx_11_sigs_i;
  wire  [83:0]  pipe_rx_12_sigs_i;
  wire  [83:0]  pipe_rx_13_sigs_i;
  wire  [83:0]  pipe_rx_14_sigs_i;
  wire  [83:0]  pipe_rx_15_sigs_i;
  wire  [25:0]  common_commands_out_i;
  wire  [83:0]  pipe_tx_0_sigs_i;
  wire  [83:0]  pipe_tx_1_sigs_i;
  wire  [83:0]  pipe_tx_2_sigs_i;
  wire  [83:0]  pipe_tx_3_sigs_i;
  wire  [83:0]  pipe_tx_4_sigs_i;
  wire  [83:0]  pipe_tx_5_sigs_i;
  wire  [83:0]  pipe_tx_6_sigs_i;
  wire  [83:0]  pipe_tx_7_sigs_i;
  wire  [83:0]  pipe_tx_8_sigs_i;
  wire  [83:0]  pipe_tx_9_sigs_i;
  wire  [83:0]  pipe_tx_10_sigs_i;
  wire  [83:0]  pipe_tx_11_sigs_i;
  wire  [83:0]  pipe_tx_12_sigs_i;
  wire  [83:0]  pipe_tx_13_sigs_i;
  wire  [83:0]  pipe_tx_14_sigs_i;
  wire  [83:0]  pipe_tx_15_sigs_i;


// synthesis translate_off
generate if (EXT_PIPE_SIM == "TRUE")
begin
  assign common_commands_in_i = common_commands_in;
  assign pipe_rx_0_sigs_i     = pipe_rx_0_sigs;
  assign pipe_rx_1_sigs_i     = pipe_rx_1_sigs;
  assign pipe_rx_2_sigs_i     = pipe_rx_2_sigs;
  assign pipe_rx_3_sigs_i     = pipe_rx_3_sigs;
  assign pipe_rx_4_sigs_i     = pipe_rx_4_sigs;
  assign pipe_rx_5_sigs_i     = pipe_rx_5_sigs;
  assign pipe_rx_6_sigs_i     = pipe_rx_6_sigs;
  assign pipe_rx_7_sigs_i     = pipe_rx_7_sigs;
  assign pipe_rx_8_sigs_i     = pipe_rx_8_sigs;
  assign pipe_rx_9_sigs_i     = pipe_rx_9_sigs;
  assign pipe_rx_10_sigs_i    = pipe_rx_10_sigs;
  assign pipe_rx_11_sigs_i    = pipe_rx_11_sigs;
  assign pipe_rx_12_sigs_i    = pipe_rx_12_sigs;
  assign pipe_rx_13_sigs_i    = pipe_rx_13_sigs;
  assign pipe_rx_14_sigs_i    = pipe_rx_14_sigs;
  assign pipe_rx_15_sigs_i    = pipe_rx_15_sigs;
  assign common_commands_out  = common_commands_out_i;
  assign pipe_tx_0_sigs       = pipe_tx_0_sigs_i;
  assign pipe_tx_1_sigs       = pipe_tx_1_sigs_i;
  assign pipe_tx_2_sigs       = pipe_tx_2_sigs_i;
  assign pipe_tx_3_sigs       = pipe_tx_3_sigs_i;
  assign pipe_tx_4_sigs       = pipe_tx_4_sigs_i;
  assign pipe_tx_5_sigs       = pipe_tx_5_sigs_i;
  assign pipe_tx_6_sigs       = pipe_tx_6_sigs_i;
  assign pipe_tx_7_sigs       = pipe_tx_7_sigs_i;
  assign pipe_tx_8_sigs       = pipe_tx_8_sigs_i;
  assign pipe_tx_9_sigs       = pipe_tx_9_sigs_i;
  assign pipe_tx_10_sigs      = pipe_tx_10_sigs_i;
  assign pipe_tx_11_sigs      = pipe_tx_11_sigs_i;
  assign pipe_tx_12_sigs      = pipe_tx_12_sigs_i;
  assign pipe_tx_13_sigs      = pipe_tx_13_sigs_i;
  assign pipe_tx_14_sigs      = pipe_tx_14_sigs_i;
  assign pipe_tx_15_sigs      = pipe_tx_15_sigs_i;
 end
endgenerate
// synthesis translate_on

generate if (EXT_PIPE_SIM == "FALSE")
begin
  assign common_commands_in_i = 26'h0;
  assign pipe_rx_0_sigs_i     = 84'h0;
  assign pipe_rx_1_sigs_i     = 84'h0;
  assign pipe_rx_2_sigs_i     = 84'h0;
  assign pipe_rx_3_sigs_i     = 84'h0;
  assign pipe_rx_4_sigs_i     = 84'h0;
  assign pipe_rx_5_sigs_i     = 84'h0;
  assign pipe_rx_6_sigs_i     = 84'h0;
  assign pipe_rx_7_sigs_i     = 84'h0;
  assign pipe_rx_8_sigs_i     = 84'h0;
  assign pipe_rx_9_sigs_i     = 84'h0;
  assign pipe_rx_10_sigs_i    = 84'h0;
  assign pipe_rx_11_sigs_i    = 84'h0;
  assign pipe_rx_12_sigs_i    = 84'h0;
  assign pipe_rx_13_sigs_i    = 84'h0;
  assign pipe_rx_14_sigs_i    = 84'h0;
  assign pipe_rx_15_sigs_i    = 84'h0;
 end
endgenerate

 project_1 qdma_ep_i
  (
    .ddr4_c0_sysclk_clk_n(ddr4_c0_sysclk_clk_n),
    .ddr4_c0_sysclk_clk_p(ddr4_c0_sysclk_clk_p),
    .ddr4_c1_sysclk_clk_n(ddr4_c1_sysclk_clk_n),
    .ddr4_c1_sysclk_clk_p(ddr4_c1_sysclk_clk_p),
    .ddr4_c2_sysclk_clk_n(ddr4_c2_sysclk_clk_n),
    .ddr4_c2_sysclk_clk_p(ddr4_c2_sysclk_clk_p),
    .ddr4_c3_sysclk_clk_n(ddr4_c3_sysclk_clk_n),
    .ddr4_c3_sysclk_clk_p(ddr4_c3_sysclk_clk_p),
    .ddr4_sdram_c0_act_n(ddr4_sdram_c0_act_n),
    .ddr4_sdram_c0_adr(ddr4_sdram_c0_adr),
    .ddr4_sdram_c0_ba(ddr4_sdram_c0_ba),
    .ddr4_sdram_c0_bg(ddr4_sdram_c0_bg),
    .ddr4_sdram_c0_ck_c(ddr4_sdram_c0_ck_c),
    .ddr4_sdram_c0_ck_t(ddr4_sdram_c0_ck_t),
    .ddr4_sdram_c0_cke(ddr4_sdram_c0_cke),
    .ddr4_sdram_c0_cs_n(ddr4_sdram_c0_cs_n),
    .ddr4_sdram_c0_dm_n(ddr4_sdram_c0_dm_n),
    .ddr4_sdram_c0_dq(ddr4_sdram_c0_dq),
    .ddr4_sdram_c0_dqs_c(ddr4_sdram_c0_dqs_c),
    .ddr4_sdram_c0_dqs_t(ddr4_sdram_c0_dqs_t),
    .ddr4_sdram_c0_odt(ddr4_sdram_c0_odt),
    .ddr4_sdram_c0_reset_n(ddr4_sdram_c0_reset_n),
    .ddr4_sdram_c1_act_n(ddr4_sdram_c1_act_n),
    .ddr4_sdram_c1_adr(ddr4_sdram_c1_adr),
    .ddr4_sdram_c1_ba(ddr4_sdram_c1_ba),
    .ddr4_sdram_c1_bg(ddr4_sdram_c1_bg),
    .ddr4_sdram_c1_ck_c(ddr4_sdram_c1_ck_c),
    .ddr4_sdram_c1_ck_t(ddr4_sdram_c1_ck_t),
    .ddr4_sdram_c1_cke(ddr4_sdram_c1_cke),
    .ddr4_sdram_c1_cs_n(ddr4_sdram_c1_cs_n),
    .ddr4_sdram_c1_dm_n(ddr4_sdram_c1_dm_n),
    .ddr4_sdram_c1_dq(ddr4_sdram_c1_dq),
    .ddr4_sdram_c1_dqs_c(ddr4_sdram_c1_dqs_c),
    .ddr4_sdram_c1_dqs_t(ddr4_sdram_c1_dqs_t),
    .ddr4_sdram_c1_odt(ddr4_sdram_c1_odt),
    .ddr4_sdram_c1_reset_n(ddr4_sdram_c1_reset_n),
    .ddr4_sdram_c2_act_n(ddr4_sdram_c2_act_n),
    .ddr4_sdram_c2_adr(ddr4_sdram_c2_adr),
    .ddr4_sdram_c2_ba(ddr4_sdram_c2_ba),
    .ddr4_sdram_c2_bg(ddr4_sdram_c2_bg),
    .ddr4_sdram_c2_ck_c(ddr4_sdram_c2_ck_c),
    .ddr4_sdram_c2_ck_t(ddr4_sdram_c2_ck_t),
    .ddr4_sdram_c2_cke(ddr4_sdram_c2_cke),
    .ddr4_sdram_c2_cs_n(ddr4_sdram_c2_cs_n),
    .ddr4_sdram_c2_dm_n(ddr4_sdram_c2_dm_n),
    .ddr4_sdram_c2_dq(ddr4_sdram_c2_dq),
    .ddr4_sdram_c2_dqs_c(ddr4_sdram_c2_dqs_c),
    .ddr4_sdram_c2_dqs_t(ddr4_sdram_c2_dqs_t),
    .ddr4_sdram_c2_odt(ddr4_sdram_c2_odt),
    .ddr4_sdram_c2_reset_n(ddr4_sdram_c2_reset_n),
    .ddr4_sdram_c3_act_n(ddr4_sdram_c3_act_n),
    .ddr4_sdram_c3_adr(ddr4_sdram_c3_adr),
    .ddr4_sdram_c3_ba(ddr4_sdram_c3_ba),
    .ddr4_sdram_c3_bg(ddr4_sdram_c3_bg),
    .ddr4_sdram_c3_ck_c(ddr4_sdram_c3_ck_c),
    .ddr4_sdram_c3_ck_t(ddr4_sdram_c3_ck_t),
    .ddr4_sdram_c3_cke(ddr4_sdram_c3_cke),
    .ddr4_sdram_c3_cs_n(ddr4_sdram_c3_cs_n),
    .ddr4_sdram_c3_dm_n(ddr4_sdram_c3_dm_n),
    .ddr4_sdram_c3_dq(ddr4_sdram_c3_dq),
    .ddr4_sdram_c3_dqs_c(ddr4_sdram_c3_dqs_c),
    .ddr4_sdram_c3_dqs_t(ddr4_sdram_c3_dqs_t),
    .ddr4_sdram_c3_odt(ddr4_sdram_c3_odt),
    .ddr4_sdram_c3_reset_n(ddr4_sdram_c3_reset_n),

    // sys_reset provided but CIPS output
    //.sys_reset (sys_rst_n_c),
    .pcie_refclk_clk_p(sys_clk_p),
    .pcie_refclk_clk_n(sys_clk_n),
    //---------------------------------------------------//
    //  PCI Express (pci_exp) Interface                  //
    //---------------------------------------------------//
    .pcie_mgt_gtx_n (pci_exp_txn),
    .pcie_mgt_gtx_p (pci_exp_txp),
    .pcie_mgt_grx_n (pci_exp_rxn),
    .pcie_mgt_grx_p (pci_exp_rxp),

    .pipe_ep_commands_out (common_commands_in_i),
    .pipe_ep_tx_0 (pipe_rx_0_sigs_i),
    .pipe_ep_tx_1 (pipe_rx_1_sigs_i),
    .pipe_ep_tx_2 (pipe_rx_2_sigs_i),
    .pipe_ep_tx_3 (pipe_rx_3_sigs_i),
    .pipe_ep_tx_4 (pipe_rx_4_sigs_i),
    .pipe_ep_tx_5 (pipe_rx_5_sigs_i),
    .pipe_ep_tx_6 (pipe_rx_6_sigs_i),
    .pipe_ep_tx_7 (pipe_rx_7_sigs_i),
    .pipe_ep_tx_8 (pipe_rx_8_sigs_i),
    .pipe_ep_tx_9 (pipe_rx_9_sigs_i),
    .pipe_ep_tx_10(pipe_rx_10_sigs_i),
    .pipe_ep_tx_11(pipe_rx_11_sigs_i),
    .pipe_ep_tx_12(pipe_rx_12_sigs_i),
    .pipe_ep_tx_13(pipe_rx_13_sigs_i),
    .pipe_ep_tx_14(pipe_rx_14_sigs_i),
    .pipe_ep_tx_15(pipe_rx_15_sigs_i),

    .pipe_ep_commands_in(common_commands_out_i),
    .pipe_ep_rx_0  (pipe_tx_0_sigs_i),
    .pipe_ep_rx_1  (pipe_tx_1_sigs_i),
    .pipe_ep_rx_2  (pipe_tx_2_sigs_i),
    .pipe_ep_rx_3  (pipe_tx_3_sigs_i),
    .pipe_ep_rx_4  (pipe_tx_4_sigs_i),
    .pipe_ep_rx_5  (pipe_tx_5_sigs_i),
    .pipe_ep_rx_6  (pipe_tx_6_sigs_i),
    .pipe_ep_rx_7  (pipe_tx_7_sigs_i),
    .pipe_ep_rx_8  (pipe_tx_8_sigs_i),
    .pipe_ep_rx_9  (pipe_tx_9_sigs_i),
    .pipe_ep_rx_10 (pipe_tx_10_sigs_i),
    .pipe_ep_rx_11 (pipe_tx_11_sigs_i),
    .pipe_ep_rx_12 (pipe_tx_12_sigs_i),
    .pipe_ep_rx_13 (pipe_tx_13_sigs_i),
    .pipe_ep_rx_14 (pipe_tx_14_sigs_i),
    .pipe_ep_rx_15 (pipe_tx_15_sigs_i)
   );

endmodule

