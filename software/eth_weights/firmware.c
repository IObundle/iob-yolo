#include "system.h"
#include "iob-uart.h"
#include "iob-eth.h"
#include "iob_timer.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define UART (UART_BASE<<(DATA_W-N_SLAVES_W))
#define SOFT_RESET (SOFT_RESET_BASE<<(ADDR_W-N_SLAVES_W))
#define ETHERNET (ETHERNET_BASE<<(ADDR_W-N_SLAVES_W))
#define TIMER (TIMER_BASE<<(ADDR_W-N_SLAVES_W))

#define ETH_NBYTES (1024-18) //minimum ethernet payload excluding FCS
#define DATA_FILE_SIZE (418*418*3*2) //16 bits per input
#define WEIGHTS_FILE_SIZE (17704732) //16 bits per input
#define NUM_DATA_FRAMES (DATA_FILE_SIZE/ETH_NBYTES)
#define NUM_WEIGHT_FRAMES (WEIGHTS_FILE_SIZE/ETH_NBYTES)

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
  char data_to_send[ETH_NBYTES];
  char data_rcv[ETH_NBYTES+18];

  //Local variables
  int i, j;
  int16_t input_data[ETH_NBYTES/2];
  char * input_data_p = (char *) &input_data;
  uart_printf("\n");
  unsigned int start, end;

  //Loop to receive and send back data frames
  for(j = 0; j < NUM_DATA_FRAMES+1; j++) {

     //wait to receive frame
     while(eth_rcv_frame(data_rcv, ETH_NBYTES+18, rcv_timeout) !=0);

     // start timer
     if(j == 0){
       timer_reset(TIMER);
       start = timer_get_count_us(TIMER);
     }

     //save in local mem
     for(i = 0; i < ETH_NBYTES; i++) input_data_p[i] = data_rcv[14+i];

     //prepare to send back
     for(i = 0; i < ETH_NBYTES; i++) data_to_send[i] = input_data_p[i];

     //send frame back
     eth_send_frame(data_to_send, ETH_NBYTES);
  }

  //measure final time for data transmission
  end = timer_get_count_us(TIMER);
  uart_printf("Data transmission done in %d ms\n", (end-start)/1000);

  //Loop to receive and send back weight frames
  for(j = 0; j < NUM_WEIGHT_FRAMES+1; j++) {

     //wait to receive frame
     while(eth_rcv_frame(data_rcv, ETH_NBYTES+18, rcv_timeout) !=0);
     
     // start timer
     if(j == 0){
       timer_reset(TIMER);
       start = timer_get_count_us(TIMER);
     }

     //save in local mem
     for(i = 0; i < ETH_NBYTES; i++) input_data_p[i] = data_rcv[14+i];

     //prepare to send back
     for(i = 0; i < ETH_NBYTES; i++) data_to_send[i] = input_data_p[i];

     //send frame back
     eth_send_frame(data_to_send, ETH_NBYTES);

  }

  //measure final time for weight transmission
  end = timer_get_count_us(TIMER);
  uart_printf("Weight transmission done in %d ms\n", (end-start)/1000);

  //end program
  uart_putc(4);
  return 0;
}
