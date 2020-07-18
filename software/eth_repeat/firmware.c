#include "system.h"
#include "periphs.h"

#include "iob-uart.h"
#include "iob-eth.h"

#include <string.h>

#define ETH_NBYTES (256-18) //minimum ethernet payload excluding FCS

int main() {

  //init UART
  uart_init(UART_BASE,FREQ/BAUD);

  //send init message
  uart_printf("\nETHERNET TEST\n");

  //init ETHERNET
  eth_init(ETHERNET_BASE);
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
  uart_putc(ETX);
  return 0;
}
