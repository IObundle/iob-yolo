//import custom libraries
#include "system.h"
#include "iob-uart.h"
#include "iob-eth.h"
#include "iob_timer.h"

//import c libraries
#include <stdint.h>
#include <stdio.h>
#include <string.h>

//define peripheral base addresses
#define UART (UART_BASE<<(DATA_W-N_SLAVES_W))
#define ETHERNET (ETHERNET_BASE<<(ADDR_W-N_SLAVES_W))
#define TIMER (TIMER_BASE<<(ADDR_W-N_SLAVES_W))
#ifdef SIM
  #define DDR_MEM (CACHE_BASE<<(ADDR_W-N_SLAVES_W))
#else
  #define DDR_MEM ((CACHE_BASE<<(ADDR_W-N_SLAVES_W)) + 0x00100000)
#endif

//define constants
#define ETH_NBYTES (1024-18) //minimum ethernet payload excluding FCS
#define INPUT_FILE_SIZE (418*418*3*2) //16 bits per point
#define OUTPUT_FILE_SIZE ((13*13*255+26*26*255)*2) //16 bits per point
#define NUM_INPUT_FRAMES (INPUT_FILE_SIZE/ETH_NBYTES)
#define NUM_OUTPUT_FRAMES (OUTPUT_FILE_SIZE/ETH_NBYTES)

int main() {

  //init UART
  uart_init(UART,UART_CLK_FREQ/UART_BAUD_RATE);

  //send init message
  uart_printf("\nETHERNET and DDR TEST\n");
  uart_txwait();

  //init ETHERNET
  eth_init(ETHERNET);
  eth_set_rx_payload_size(ETH_NBYTES);

  //ETHERNET variables
  int rcv_timeout = 5000;
  char data_to_send[ETH_NBYTES];
  char data_rcv[ETH_NBYTES+18];

  //Local variables
  int i, j;
  volatile char *ddr_p = (volatile char*) (DDR_MEM);
  unsigned int count_errors = 0, bytes_to_send, bytes_to_receive, count_bytes = 0;
  unsigned int start, end;

#ifdef SIM
  uart_puts("\nStarting input.network reception\n");

  //Loop to receive input.network frames
  for(j = 0; j < NUM_INPUT_FRAMES+1; j++) {

     //wait to receive frame
     while(eth_rcv_frame(data_rcv, ETH_NBYTES+18, rcv_timeout) !=0);
     if(j==0) start = timer_get_count_us(TIMER);

     //check if it is last packet (has less data that full payload size)
     if(j == NUM_INPUT_FRAMES) bytes_to_receive = INPUT_FILE_SIZE - count_bytes;
     else bytes_to_receive = ETH_NBYTES;

     //check data received is same in DDR
     for(i = 0; i < bytes_to_receive; i++) {
        if(data_rcv[14+i] != ddr_p[j*ETH_NBYTES + i]) count_errors += 1;
     }

     //update byte counter
     count_bytes += ETH_NBYTES;
  }

  //measure final time for input.network reception
  end = timer_get_count_us(TIMER);
  uart_printf("input.network transferred in %d ms\n", (end-start)/1000);
  uart_printf("input.network transferred with %d errors\n", count_errors);
  uart_puts("Starting output.network transfer\n");

  //new local variables
  int ddr_offset = NUM_INPUT_FRAMES*ETH_NBYTES + bytes_to_receive;
  count_bytes = 0;

  //measure initial time for output.network transmission
  timer_reset(TIMER);
  start = timer_get_count_us(TIMER);

  //Loop to send output.network
  for(j = 0; j < NUM_OUTPUT_FRAMES+1; j++) {

     //check if it is last packet (has less data that full payload size)
     if(j == NUM_OUTPUT_FRAMES) {
        bytes_to_send = OUTPUT_FILE_SIZE - count_bytes;
     } else bytes_to_send = ETH_NBYTES;

     //prepare to send back
     for(i = 0; i < bytes_to_send; i++) data_to_send[i] = ddr_p[ddr_offset + j*ETH_NBYTES + i];

     //send frame back
     eth_send_frame(data_to_send, ETH_NBYTES);

     //update byte counter
     count_bytes += ETH_NBYTES;
  }

  //measure final time for output.network transmission
  end = timer_get_count_us(TIMER);
  uart_printf("output.network transferred in %d ms\n", (end-start)/1000);

#else

  uart_puts("\nStarting output.network reception\n");

  //Loop to receive input.network frames
  for(j = 0; j < NUM_OUTPUT_FRAMES+1; j++) {

     //wait to receive frame
     while(eth_rcv_frame(data_rcv, ETH_NBYTES+18, rcv_timeout) !=0);
     if(j == 0) start = timer_get_count_us(TIMER);

     //check if it is last packet (has less data that full payload size)
     if(j == NUM_OUTPUT_FRAMES) bytes_to_receive = OUTPUT_FILE_SIZE - count_bytes;
     else bytes_to_receive = ETH_NBYTES;

     //store in DDR
     for(i = 0; i < bytes_to_receive; i++) ddr_p[j*ETH_NBYTES + i] = data_rcv[14+i];

     //update byte counter
     count_bytes += ETH_NBYTES;
  }

  //measure final time for data transmission
  end = timer_get_count_us(TIMER);
  uart_printf("output.network received in %d ms\n", (end-start)/1000);
  uart_puts("Starting output.network transfer\n");
  count_bytes = 0;

  //measure initial time for output.network transmission
  timer_reset(TIMER);
  start = timer_get_count_us(TIMER);

  //Loop to send output.network
  for(j = 0; j < NUM_OUTPUT_FRAMES+1; j++) {

     //check if it is last packet (has less data that full payload size)
     if(j == NUM_OUTPUT_FRAMES) bytes_to_send = OUTPUT_FILE_SIZE - count_bytes;
     else bytes_to_send = ETH_NBYTES;

     //prepare to send back
     for(i = 0; i < bytes_to_send; i++) data_to_send[i] = ddr_p[j*ETH_NBYTES + i];

     //send frame back
     eth_send_frame(data_to_send, ETH_NBYTES);

     //update byte counter
     count_bytes += ETH_NBYTES;
  }

  //measure final time for output.network transmission
  end = timer_get_count_us(TIMER);
  uart_printf("output.network transferred in %d ms\n", (end-start)/1000);
#endif

  //end program
  uart_putc(4);
  return 0;
}
