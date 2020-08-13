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

		       //vread databus interface
		       .ywrite_databus_valid (ywrite_dbus_valid),
		       .ywrite_databus_addr  (ywrite_dbus_addr),
		       .ywrite_databus_wdata (ywrite_dbus_wdata),
		       .ywrite_databus_wstrb (ywrite_dbus_wstrb),
		       .ywrite_databus_rdata (ywrite_dbus_rdata),
		       .ywrite_databus_ready (ywrite_dbus_ready)
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
