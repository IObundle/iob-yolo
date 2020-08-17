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

		       //vread databus interface
		       .databus_valid (dbus_valid),
		       .databus_addr  (dbus_addr),
		       .databus_wdata (dbus_wdata),
		       .databus_wstrb (dbus_wstrb),
		       .databus_rdata (dbus_rdata),
		       .databus_ready (dbus_ready),

		       // DMA - number of tranfers per burst
		       .dma_len	      (dbus_len),
		       .dma_size      (dbus_size)
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
