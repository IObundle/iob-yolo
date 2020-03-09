#include "system.h"
#include "iob-uart.h"
#include "iob-eth.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define UART (UART_BASE<<(DATA_W-N_SLAVES_W))
#define SOFT_RESET (SOFT_RESET_BASE<<(ADDR_W-N_SLAVES_W))
#define ETHERNET (ETHERNET_BASE<<(ADDR_W-N_SLAVES_W))

#define ETH_NBYTES (256-18) //minimum ethernet payload excluding FCS

int main() {

  //init UART
  uart_init(UART,UART_CLK_FREQ/UART_BAUD_RATE);

  //send init message
  uart_printf("\nETHERNET TEST\n");
  uart_txwait();

  //init ETHERNET
  eth_init(ETHERNET);
  eth_set_rx_payload_size(ETH_NBYTES);

  //ETHERNET variables
  int rcv_timeout = 5000;
  char data_to_send[ETH_NBYTES] = {0};
  char data_rcv[ETH_NBYTES+18];
  strcpy(data_to_send, "Hello from FPGA\n");

  //receive frame
  uart_puts("Waiting to receive data\n");
  while(eth_rcv_frame(data_rcv, ETH_NBYTES+18, rcv_timeout) !=0);
  uart_printf("Data received: %s\n", &data_rcv[14]);

  //send frame
  uart_printf("Data to be sent: %s\n", data_to_send);
  eth_send_frame (data_to_send, ETH_NBYTES);
  uart_puts("Data Sent\n");

  //end program
  uart_putc(4);
  return 0;
}
