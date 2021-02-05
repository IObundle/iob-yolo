   //address write
   `OUTPUT(m_axi_awid,    2*1),  //Address write channel ID
   `OUTPUT(m_axi_awaddr,  2*DDR_ADDR_W), //Address write channel address
   `OUTPUT(m_axi_awlen,   2*8),  //Address write channel burst length
   `OUTPUT(m_axi_awsize,  2*3),  //Address write channel burst size. This signal indicates the size of each transfer in the burst
   `OUTPUT(m_axi_awburst, 2*2),  //Address write channel burst type
   `OUTPUT(m_axi_awlock,  2*1),  //Address write channel lock type
   `OUTPUT(m_axi_awcache, 2*4),  //Address write channel memory type. Transactions set with Normal Non-cacheable Modifiable and Bufferable (0011).
   `OUTPUT(m_axi_awprot,  2*3),  //Address write channel protection type. Transactions set with Normal, Secure, and Data attributes (000).
   `OUTPUT(m_axi_awqos,   2*4),  //Address write channel quality of service
   `OUTPUT(m_axi_awvalid, 2*1),  //Address write channel valid
   `INPUT(m_axi_awready,  2*1),  //Address write channel ready

   //write
   `OUTPUT(m_axi_wdata,   2*MIG_BUS_W), //Write channel data
   `OUTPUT(m_axi_wstrb,   2*MIB_BUS_W/8),  //Write channel write strobe
   `OUTPUT(m_axi_wlast,   2*1),  //Write channel last word flag
   `OUTPUT(m_axi_wvalid,  2*1),  //Write channel valid
   `INPUT(m_axi_wready,   2*1),  //Write channel ready

   //write response
   `INPUT(m_axi_bid,      2*1),  //Write response channel ID
   `INPUT(m_axi_bresp,    2*2),  //Write response channel response
   `INPUT(m_axi_bvalid,   2*1),  //Write response channel valid
   `OUTPUT(m_axi_bready,  2*1),  //Write response channel ready
  
   //address read
   `OUTPUT(m_axi_arid,    2*1),  //Address read channel id
   `OUTPUT(m_axi_araddr,  2*MIG_BUS_W), //Address read channel address
   `OUTPUT(m_axi_arlen,   2*8),  //Address read channel burst length
   `OUTPUT(m_axi_arsize,  2*3),  //Address read channel burst size. This signal indicates the size of each transfer in the burst
   `OUTPUT(m_axi_arburst, 2*2),  //Address read channel burst type
   `OUTPUT(m_axi_arlock,  2*1),  //Address read channel lock type
   `OUTPUT(m_axi_arcache, 2*4),  //Address read channel memory type. Transactions set with Normal Non-cacheable Modifiable and Bufferable (0011).
   `OUTPUT(m_axi_arprot,  2*3),  //Address read channel protection type. Transactions set with Normal, Secure, and Data attributes (000).
   `OUTPUT(m_axi_arqos,   2*4),  //Address read channel quality of service
   `OUTPUT(m_axi_arvalid, 2*1),  //Address read channel valid
   `INPUT(m_axi_arready,  2*1),  //Address read channel ready

   //read
   `INPUT(m_axi_rid,      2*1),  //Read channel ID
   `INPUT(m_axi_rdata,    2*MIG_BUS_W), //Read channel data
   `INPUT(m_axi_rresp,    2*2),  //Read channel response
   `INPUT(m_axi_rlast,    2*1),  //Read channel last word
   `INPUT(m_axi_rvalid,   2*1),  //Read channel valid
   `OUTPUT(m_axi_rready,  2*1),  //Read channel ready 
