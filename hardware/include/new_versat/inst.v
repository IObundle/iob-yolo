`ifdef USE_NEW_VERSAT

   //
   // VERSAT
   //

   xversat #(
	     .ADDR_W (ADDR_W-2-$clog2(`N_SLAVES)-2)
	     //-2 to remove E and P bits
	     //-2 from byte to 32 bit addressable
	     ) versat (
		       .clk       (clk),
		       .rst       (reset),
		       
		       //cpu interface
		       .valid     (slaves_req[`valid(`VERSAT)]),
		       .addr      (slaves_req[`address(`VERSAT, ADDR_W-2-$clog2(`N_SLAVES))-2]),
		       .wdata     (slaves_req[`wdata(`VERSAT)]),
		       .wstrb     (|slaves_req[`wstrb(`VERSAT)]),
		       .rdata     (slaves_resp[`rdata(`VERSAT)]),
		       .ready     (slaves_resp[`ready(`VERSAT)]),

		       // AXI Interface
		       // Address write
		       .axi_awid(m_axi_awid[0*1+:1]), 
		       .axi_awaddr(m_axi_awaddr[0*`DDR_ADDR_W+:`DDR_ADDR_W]), 
		       .axi_awlen(m_axi_awlen[0*8+:8]), 
		       .axi_awsize(m_axi_awsize[0*3+:3]), 
		       .axi_awburst(m_axi_awburst[0*2+:2]), 
		       .axi_awlock(m_axi_awlock[0*1+:1]), 
		       .axi_awcache(m_axi_awcache[0*4+:4]), 
		       .axi_awprot(m_axi_awprot[0*3+:3]),
		       .axi_awqos(m_axi_awqos[0*4+:4]), 
		       .axi_awvalid(m_axi_awvalid[0*1+:1]), 
		       .axi_awready(m_axi_awready[0*1+:1]),
		       //write
		       .axi_wdata(m_axi_wdata[0*`MIG_BUS_W+:`MIG_BUS_W]), 
		       .axi_wstrb(m_axi_wstrb[0*`MIG_BUS_W/8+:`MIG_BUS_W/8]), 
		       .axi_wlast(m_axi_wlast[0*1+:1]), 
		       .axi_wvalid(m_axi_wvalid[0*1+:1]), 
		       .axi_wready(m_axi_wready[0*1+:1]), 
		       //write response
		       // .m_axi_bid(axi_bid[0*1+:1]), 
		       .axi_bresp(m_axi_bresp[0*2+:2]), 
		       .axi_bvalid(m_axi_bvalid[0*1+:1]), 
		       .axi_bready(m_axi_bready[0*1+:1]),
		       //address read
		       .axi_arid(m_axi_arid[0*1+:1]), 
		       .axi_araddr(m_axi_araddr[0*`DDR_ADDR_W+:`DDR_ADDR_W]), 
		       .axi_arlen(m_axi_arlen[0*8+:8]), 
		       .axi_arsize(m_axi_arsize[0*3+:3]), 
		       .axi_arburst(m_axi_arburst[0*2+:2]), 
		       .axi_arlock(m_axi_arlock[0*1+:1]), 
		       .axi_arcache(m_axi_arcache[0*4+:4]), 
		       .axi_arprot(m_axi_arprot[0*3+:3]), 
		       .axi_arqos(m_axi_arqos[0*4+:4]), 
		       .axi_arvalid(m_axi_arvalid[0*1+:1]), 
		       .axi_arready(m_axi_arready[0*1+:1]), 
		       //read 
		       // .m_axi_rid(axi_rid[0*1+:1]), 
		       .axi_rdata(m_axi_rdata[0*`MIG_BUS_W+:`MIG_BUS_W]), 
		       .axi_rresp(m_axi_rresp[0*2+:2]), 
		       .axi_rlast(m_axi_rlast[0*1+:1]), 
		       .axi_rvalid(m_axi_rvalid[0*1+:1]),  
		       .axi_rready(m_axi_rready[0*1+:1])
		       );


`else

   //
   // VERSAT
   //

   xversat # (
   	     .ADDR_W (ADDR_W-2)
   	     ) versat (
      .clk (clk),
      .rst (reset),
      
      //cpu interface
      .valid (slaves_req[`valid(`VERSAT)]),
      .addr  (slaves_req[`address(`VERSAT, ADDR_W)-2]),
      .rdata (slaves_req[`wdata(`VERSAT)]),
      .we    (|slaves_req[`wstrb(`VERSAT)]),
      .wdata (slaves_resp[`rdata(`VERSAT)]),
      .ready (slaves_resp[`ready(`VERSAT)])
      );
`endif
