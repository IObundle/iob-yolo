//import custom libraries
#include "system.h"
#include "periphs.h"

#include "iob-uart.h"
#include "iob-eth.h"
#include "iob_timer.h"

//import c libraries
#include <stdint.h>
#include <stdio.h>
#include <string.h>

//define constants
#define ETH_NBYTES (1024-18) //minimum ethernet payload excluding FCS
#define INPUT_FILE_SIZE (418*418*3*2) //16 bits per point
#define OUTPUT_FILE_SIZE ((13*13*255+26*26*255)*2) //16 bits per point
#define NUM_INPUT_FRAMES (INPUT_FILE_SIZE/ETH_NBYTES)
#define NUM_OUTPUT_FRAMES (OUTPUT_FILE_SIZE/ETH_NBYTES)

//set DDR pointer - NOTE that this test only makes sense if USE_DDR==1
#if (RUN_DDR==0)
#define DDR_PTR EXTRA_BASE
#else
#define DDR_PTR (1<<FIRM_ADDR_W)
#endif

int main() {

  //init UART
  uart_init(UART_BASE,FREQ/BAUD);

  //send init message
  uart_printf("\nETHERNET and DDR TEST\n");
  uart_txwait();

  //check for USE_DDR condition
#if (USE_DDR==0)
  uart_printf("This test requires USE_DDR=1\n");
  uart_printf("Ending program...\n");
  //end program
  uart_putc(ETX);
  return 0;  
#endif

  //init ETHERNET
  eth_init(ETHERNET_BASE);
  eth_set_rx_payload_size(ETH_NBYTES);

  //ETHERNET variables
  int rcv_timeout = 5000;
  char data_to_send[ETH_NBYTES];
  char data_rcv[ETH_NBYTES+18];

  //Local variables
  int i, j;
  char *ddr_p = (char*) (DDR_PTR);
  unsigned int count_errors = 0, bytes_to_send, bytes_to_receive, count_bytes = 0;
  unsigned int start, end;

#ifdef SIM
  uart_puts("\nStarting input.network reception\n");

  //Loop to receive input.network frames
  for(j = 0; j < NUM_INPUT_FRAMES+1; j++) {

     //wait to receive frame
     while(eth_rcv_frame(data_rcv, ETH_NBYTES+18, rcv_timeout) !=0);
     if(j==0) start = timer_time_us(TIMER_BASE);

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
  end = timer_time_us(TIMER_BASE);
  uart_printf("input.network transferred in %d ms\n", (end-start)/1000);
  uart_printf("input.network transferred with %d errors\n", count_errors);
  uart_puts("Starting output.network transfer\n");

  //new local variables
  int ddr_offset = NUM_INPUT_FRAMES*ETH_NBYTES + bytes_to_receive;
  count_bytes = 0;

  //measure initial time for output.network transmission
  timer_reset(TIMER_BASE);
  start = timer_time_us(TIMER_BASE);

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
  end = timer_timer_us(TIMER_BASE);
  uart_printf("output.network transferred in %d ms\n", (end-start)/1000);

#else

  uart_puts("\nStarting output.network reception\n");

  //Loop to receive input.network frames
  for(j = 0; j < NUM_OUTPUT_FRAMES+1; j++) {

     //wait to receive frame
     while(eth_rcv_frame(data_rcv, ETH_NBYTES+18, rcv_timeout) !=0);
     if(j == 0) start = timer_time_us(TIMER_BASE);

     //check if it is last packet (has less data that full payload size)
     if(j == NUM_OUTPUT_FRAMES) bytes_to_receive = OUTPUT_FILE_SIZE - count_bytes;
     else bytes_to_receive = ETH_NBYTES;

     //store in DDR
     for(i = 0; i < bytes_to_receive; i++) ddr_p[j*ETH_NBYTES + i] = data_rcv[14+i];

     //update byte counter
     count_bytes += ETH_NBYTES;
  }

  //measure final time for data transmission
  end = timer_time_us(TIMER_BASE);
  uart_printf("output.network received in %d ms\n", (end-start)/1000);
  uart_puts("Starting output.network transfer\n");
  count_bytes = 0;

  //measure initial time for output.network transmission
  timer_reset(TIMER_BASE);
  start = timer_time_us(TIMER_BASE);

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
  end = timer_time_us(TIMER_BASE);
  uart_printf("output.network transferred in %d ms\n", (end-start)/1000);
#endif

  //end program
  uart_putc(ETX);
  return 0;
}
